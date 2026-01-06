// super_local_traj_benchmark.cpp
// Benchmark SUPER (ExpTrajOpt) on precomputed corridors stored in .mysco2 files,
// plus per-sample violation logging (SFC / vel / acc / jerk) at 0.01s.
// ADDITION: Dump each successful trajectory to a per-file CSV for ROS2 RViz2 plotting.

#include <ros/ros.h>
#include <ros/package.h>

#include <Eigen/Dense>
#include <Eigen/StdVector>

#include <boost/filesystem.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

#include <yaml-cpp/yaml.h>

#include <ros_interface/ros_interface.hpp>
#include <traj_opt/exp_traj_optimizer_s4.h>
#include <super_core/super_planner.h>

#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>

// SUPER type aliases live in these namespaces
using geometry_utils::Trajectory;
using super_utils::StatePVAJ;
using super_utils::vec_Vec3f;

static uint32_t readU32(std::ifstream &ifs)
{
    uint32_t v;
    ifs.read(reinterpret_cast<char *>(&v), sizeof(v));
    if (!ifs)
        throw std::runtime_error("Corrupt .mysco2 (u32 read).");
    return v;
}

static double readD(std::ifstream &ifs)
{
    double v;
    ifs.read(reinterpret_cast<char *>(&v), sizeof(v));
    if (!ifs)
        throw std::runtime_error("Corrupt .mysco2 (double read).");
    return v;
}

struct BenchResult
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    std::string file;
    bool success{false};
    bool gurobi_error{false}; // keep schema
    double per_opt_runtime_ms{0.0};
    double total_traj_time_sec{0.0};
    std::string status;

    Eigen::Vector3d start{0, 0, 0};
    Eigen::Vector3d goal{0, 0, 0};

    double cost_value{0.0};

    // Added summary metrics
    double max_sfc_violation{0.0};
    double max_vel_violation{0.0};
    double max_acc_violation{0.0};
    double max_jerk_violation{0.0};

    // Jerk smoothness metrics
    // Sjerk    = ∫ ||j(t)|| dt
    // Sjerk_rms = sqrt( (1/T) ∫ ||j(t)||^2 dt )
    double jerk_smoothness_l1{0.0};
    double jerk_rms{0.0};

    double traj_length_m{0.0}; // ∫ ||v(t)|| dt
};

static inline void fillStatePVAJAtRest(StatePVAJ &s, const Eigen::Vector3d &p)
{
    s.setZero();
    s.col(0) = p;
}

static std::vector<double> toDurations(const std::vector<double> &seg_end_times,
                                       bool seg_times_are_cumulative)
{
    if (!seg_times_are_cumulative)
        return seg_end_times;
    std::vector<double> dt(seg_end_times.size(), 0.0);
    double prev = 0.0;
    for (size_t i = 0; i < seg_end_times.size(); ++i)
    {
        dt[i] = seg_end_times[i] - prev;
        prev = seg_end_times[i];
        if (dt[i] < 1e-6)
            dt[i] = 1e-3;
    }
    return dt;
}

// Load .mysco2 -> start/goal, path points, segment times, sfcs, and store plane matrices for debug/violation
static void loadMysco2ToSuper(
    const std::string &file,
    Eigen::Vector3d &start,
    Eigen::Vector3d &goal,
    std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> &path_pts,
    std::vector<double> &seg_end_times,
    super_planner::PolytopeVec &sfcs,
    std::vector<Eigen::MatrixXd> &sfc_planes_list,
    std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> &init_ps,
    Eigen::VectorXd &init_ts,
    double poly_seed_eps,
    bool seg_times_are_cumulative)
{

    std::ifstream ifs(file, std::ios::binary);
    if (!ifs)
        throw std::runtime_error("Failed to open: " + file);

    char magic[8];
    ifs.read(magic, 8);
    if (!ifs)
        throw std::runtime_error("Corrupt .mysco2 (magic): " + file);

    const std::string m(magic, magic + 8);
    if (m.rfind("MYSCO2", 0) != 0)
    {
        throw std::runtime_error("Bad magic in: " + file);
    }

    const uint32_t version = readU32(ifs);
    if (version != 1)
        throw std::runtime_error("Unsupported .mysco2 version in: " + file);

    start.x() = readD(ifs);
    start.y() = readD(ifs);
    start.z() = readD(ifs);
    goal.x() = readD(ifs);
    goal.y() = readD(ifs);
    goal.z() = readD(ifs);

    const uint32_t num_path_pts = readU32(ifs);
    path_pts.clear();
    path_pts.reserve(num_path_pts);
    for (uint32_t i = 0; i < num_path_pts; ++i)
    {
        Eigen::Vector3d p;
        p.x() = readD(ifs);
        p.y() = readD(ifs);
        p.z() = readD(ifs);
        path_pts.push_back(p);
    }

    const uint32_t num_seg = readU32(ifs);
    seg_end_times.resize(num_seg);
    for (uint32_t i = 0; i < num_seg; ++i)
        seg_end_times[i] = readD(ifs);

    if (path_pts.size() < 2 || (path_pts.size() - 1) != num_seg)
    {
        throw std::runtime_error("File inconsistent: path.size()-1 != num_seg in " + file);
    }

    sfcs.clear();
    sfcs.reserve(num_seg);
    sfc_planes_list.clear();
    sfc_planes_list.reserve(num_seg);

    for (uint32_t si = 0; si < num_seg; ++si)
    {
        const uint32_t mplanes = readU32(ifs);

        Eigen::MatrixXd A(mplanes, 3);
        Eigen::VectorXd b(mplanes);

        for (uint32_t r = 0; r < mplanes; ++r)
            for (int c = 0; c < 3; ++c)
                A(r, c) = readD(ifs);

        for (uint32_t r = 0; r < mplanes; ++r)
            b(r) = readD(ifs);

        // Flip planes if midpoint violates: A x <= b
        const Eigen::Vector3d mid = 0.5 * (path_pts[si] + path_pts[si + 1]);
        for (int r = 0; r < A.rows(); ++r)
        {
            const double v = A.row(r).dot(mid) - b(r);
            if (v > poly_seed_eps)
            {
                A.row(r) *= -1.0;
                b(r) *= -1.0;
            }
        }

        // SUPER planes: n·x + d <= 0, where d = -b for A x <= b
        Eigen::MatrixXd planes(mplanes, 4);
        planes.leftCols<3>() = A;
        planes.col(3) = -b;

        super_planner::Polytope poly;
        poly.SetPlanes(planes);
        sfcs.emplace_back(poly);
        sfc_planes_list.push_back(planes);
    }

    // init_ps: intermediate points excluding endpoints
    init_ps.clear();
    if (path_pts.size() >= 3)
    {
        init_ps.reserve(path_pts.size() - 2);
        for (size_t i = 1; i + 1 < path_pts.size(); ++i)
            init_ps.push_back(path_pts[i]);
    }

    // init_ts: durations per segment
    const std::vector<double> dt = toDurations(seg_end_times, seg_times_are_cumulative);
    init_ts.resize(static_cast<int>(dt.size()));
    for (int i = 0; i < init_ts.size(); ++i)
        init_ts(i) = dt.at(static_cast<size_t>(i));
}

// ---------------- RViz helpers ----------------
static inline geometry_msgs::Point toPointMsg(const Eigen::Vector3d &p)
{
    geometry_msgs::Point q;
    q.x = p.x();
    q.y = p.y();
    q.z = p.z();
    return q;
}

static inline bool isFinite3(const Eigen::Vector3d &p)
{
    return std::isfinite(p.x()) && std::isfinite(p.y()) && std::isfinite(p.z());
}

static visualization_msgs::Marker makeDeleteAllMarker(const std::string &frame_id)
{
    visualization_msgs::Marker m;
    m.header.frame_id = frame_id;
    m.header.stamp = ros::Time::now();
    m.action = visualization_msgs::Marker::DELETEALL;
    return m;
}

static visualization_msgs::MarkerArray
makeTrajectoryMarkers(const geometry_utils::Trajectory &traj,
                      const std::string &frame_id,
                      const std::string &ns_prefix,
                      const double sample_dt = 0.01,
                      const int max_samples = 4000)
{
    visualization_msgs::MarkerArray arr;
    arr.markers.push_back(makeDeleteAllMarker(frame_id));

    const double T = traj.getTotalDuration();
    if (T <= 0.0)
        return arr;

    visualization_msgs::Marker line;
    line.header.frame_id = frame_id;
    line.header.stamp = ros::Time::now();
    line.ns = ns_prefix + "/traj_line";
    line.id = 1;
    line.type = visualization_msgs::Marker::LINE_STRIP;
    line.action = visualization_msgs::Marker::ADD;
    line.pose.orientation.w = 1.0;
    line.scale.x = 0.05;
    line.color.a = 1.0;
    line.color.r = 1.0;
    line.color.g = 0.0;
    line.color.b = 0.0;

    visualization_msgs::Marker pts;
    pts.header = line.header;
    pts.ns = ns_prefix + "/traj_pts";
    pts.id = 2;
    pts.type = visualization_msgs::Marker::SPHERE_LIST;
    pts.action = visualization_msgs::Marker::ADD;
    pts.pose.orientation.w = 1.0;
    pts.scale.x = 0.10;
    pts.scale.y = 0.10;
    pts.scale.z = 0.10;
    pts.color.a = 1.0;
    pts.color.r = 1.0;
    pts.color.g = 0.0;
    pts.color.b = 0.0;

    int count = 0;
    for (double t = 0.0; t <= T + 1e-9; t += sample_dt)
    {
        Eigen::Vector3d p = traj.getPos(t);
        if (!isFinite3(p))
            continue;
        const auto pm = toPointMsg(p);
        line.points.push_back(pm);
        pts.points.push_back(pm);
        if (++count >= max_samples)
            break;
    }

    arr.markers.push_back(line);
    arr.markers.push_back(pts);
    return arr;
}

// (Vertices-from-planes SFC visualization)
static std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>>
computePolyVerticesFromPlanes(const Eigen::MatrixXd &planes,
                              const double inside_eps = 1e-6,
                              const double uniq_eps = 1e-3,
                              const int max_verts = 300)
{
    std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> verts;
    const int m = (int)planes.rows();
    if (m < 4)
        return verts;

    auto isInside = [&](const Eigen::Vector3d &x) -> bool
    {
        for (int r = 0; r < m; ++r)
        {
            const Eigen::Vector3d n = planes.row(r).head<3>();
            const double d = planes(r, 3);
            if (n.dot(x) + d > inside_eps)
                return false;
        }
        return true;
    };

    auto addUnique = [&](const Eigen::Vector3d &x)
    {
        for (const auto &v : verts)
        {
            if ((v - x).norm() < uniq_eps)
                return;
        }
        verts.push_back(x);
    };

    for (int i = 0; i < m; ++i)
    {
        for (int j = i + 1; j < m; ++j)
        {
            for (int k = j + 1; k < m; ++k)
            {
                Eigen::Matrix3d N;
                N.row(0) = planes.row(i).head<3>();
                N.row(1) = planes.row(j).head<3>();
                N.row(2) = planes.row(k).head<3>();

                if (std::abs(N.determinant()) < 1e-10)
                    continue;

                Eigen::Vector3d rhs;
                rhs << -planes(i, 3), -planes(j, 3), -planes(k, 3);

                const Eigen::Vector3d x = N.fullPivLu().solve(rhs);
                if (!isFinite3(x))
                    continue;
                if (!isInside(x))
                    continue;

                addUnique(x);
                if ((int)verts.size() >= max_verts)
                    return verts;
            }
        }
    }
    return verts;
}

static visualization_msgs::MarkerArray
makeSfcMarkersFromPlaneList(const std::vector<Eigen::MatrixXd> &sfc_planes_list,
                            const std::string &frame_id,
                            const std::string &ns_prefix,
                            const double inside_eps = 1e-6)
{
    visualization_msgs::MarkerArray arr;
    arr.markers.push_back(makeDeleteAllMarker(frame_id));

    int mid = 1;
    for (size_t si = 0; si < sfc_planes_list.size(); ++si)
    {
        const Eigen::MatrixXd &planes = sfc_planes_list[si];
        auto verts = computePolyVerticesFromPlanes(planes, inside_eps);

        visualization_msgs::Marker mk;
        mk.header.frame_id = frame_id;
        mk.header.stamp = ros::Time::now();
        mk.ns = ns_prefix + "/sfc_poly";
        mk.id = mid++;
        mk.type = visualization_msgs::Marker::SPHERE_LIST;
        mk.action = visualization_msgs::Marker::ADD;
        mk.pose.orientation.w = 1.0;

        mk.scale.x = 0.10;
        mk.scale.y = 0.10;
        mk.scale.z = 0.10;
        mk.color.a = 0.8;
        mk.color.r = 0.0;
        mk.color.g = 0.6;
        mk.color.b = 0.0;

        mk.points.reserve(verts.size());
        for (const auto &v : verts)
            mk.points.push_back(toPointMsg(v));

        arr.markers.push_back(mk);
    }

    return arr;
}

// ---------------- Violation helpers ----------------
static inline double violationPositive(double x) { return (x > 0.0) ? x : 0.0; }

static inline double maxPlaneValue(const Eigen::MatrixXd &planes_mx4, const Eigen::Vector3d &p)
{
    double maxv = -std::numeric_limits<double>::infinity();
    for (int r = 0; r < planes_mx4.rows(); ++r)
    {
        const Eigen::Vector3d n = planes_mx4.row(r).head<3>();
        const double d = planes_mx4(r, 3);
        const double v = n.dot(p) + d;
        if (v > maxv)
            maxv = v;
    }
    return maxv;
}

static inline int segmentIndexFromTime(const std::vector<double> &guide_t, double t_ref)
{
    // guide_t size = num_pts. segments = num_pts-1.
    if (guide_t.size() < 2)
        return 0;
    if (t_ref <= guide_t.front())
        return 0;
    if (t_ref >= guide_t.back())
        return (int)guide_t.size() - 2;

    auto it = std::upper_bound(guide_t.begin(), guide_t.end(), t_ref);
    int idx = (int)std::distance(guide_t.begin(), it) - 1;
    idx = std::max(0, std::min(idx, (int)guide_t.size() - 2));
    return idx;
}

// Robustly load max_vel/max_acc/max_jerk from click_smooth_ros1.yaml (traj_opt/boundary/*)
static void loadBoundaryLimitsFromYaml(const std::string &yaml_path,
                                       double &max_vel,
                                       double &max_acc,
                                       double &max_jerk)
{
    // Defaults if not found
    max_vel = 999.0;
    max_acc = 999.0;
    max_jerk = 999.0;

    try
    {
        YAML::Node root = YAML::LoadFile(yaml_path);
        YAML::Node n = root["traj_opt"];
        if (!n)
            return;
        n = n["boundary"];
        if (!n)
            return;

        if (n["max_vel"])
            max_vel = n["max_vel"].as<double>();
        if (n["max_acc"])
            max_acc = n["max_acc"].as<double>();
        if (n["max_jerk"])
            max_jerk = n["max_jerk"].as<double>();
    }
    catch (const std::exception &e)
    {
        ROS_WARN("Failed to parse yaml boundary limits (%s): %s. Using defaults.",
                 yaml_path.c_str(), e.what());
    }
}

// Union-of-polytopes corridor violation:
// For point p, compute best = min_i max_r (n_ir · p + d_ir).
// If best <= 0, p is inside at least one polytope.
static inline double corridorUnionValue(const std::vector<Eigen::MatrixXd> &planes_list,
                                        const Eigen::Vector3d &p,
                                        int *best_poly_idx_out = nullptr)
{
    if (best_poly_idx_out)
        *best_poly_idx_out = -1;
    if (planes_list.empty())
        return std::numeric_limits<double>::quiet_NaN();

    double best = std::numeric_limits<double>::infinity();
    int best_idx = -1;

    for (int i = 0; i < (int)planes_list.size(); ++i)
    {
        const double v = maxPlaneValue(planes_list[i], p); // max over planes for poly i
        if (v < best)
        {
            best = v;
            best_idx = i;
        }
    }

    if (best_poly_idx_out)
        *best_poly_idx_out = best_idx;
    return best;
}

// ---------------- Trajectory dump (NEW) ----------------
static inline bool ensureDirectoryExists(const std::string &dir_path)
{
    namespace bfs = boost::filesystem;
    bfs::path p(dir_path);
    if (bfs::exists(p))
        return bfs::is_directory(p);
    return bfs::create_directories(p);
}

static inline std::string sanitizeForFilename(std::string s)
{
    for (char &c : s)
    {
        if (!(std::isalnum(static_cast<unsigned char>(c)) || c == '-' || c == '_' || c == '.'))
            c = '_';
    }
    return s;
}

static void dumpTrajectoryCsvV1(const std::string &out_csv,
                                const std::string &planner_name,
                                const std::string &source_file_basename,
                                const std::string &frame_id,
                                const geometry_utils::Trajectory &traj,
                                double dump_dt)
{
    std::ofstream ofs(out_csv);
    if (!ofs)
        throw std::runtime_error("Failed to open traj dump csv: " + out_csv);

    const double T = traj.getTotalDuration();
    if (T <= 0.0)
    {
        ofs << "# traj_format: super_traj_csv_v1\n";
        ofs << "# planner_name: " << planner_name << "\n";
        ofs << "# source_file: " << source_file_basename << "\n";
        ofs << "# frame_id: " << frame_id << "\n";
        ofs << "# total_duration_sec: 0\n";
        ofs << "t,x,y,z,vx,vy,vz,ax,ay,az,jx,jy,jz\n";
        return;
    }

    if (dump_dt <= 1e-6)
        dump_dt = 0.03;

    ofs << "# traj_format: super_traj_csv_v1\n";
    ofs << "# planner_name: " << planner_name << "\n";
    ofs << "# source_file: " << source_file_basename << "\n";
    ofs << "# frame_id: " << frame_id << "\n";
    ofs << "# total_duration_sec: " << std::fixed << std::setprecision(9) << T << "\n";
    ofs << "# dump_dt_sec: " << std::fixed << std::setprecision(9) << dump_dt << "\n";
    ofs << "t,x,y,z,vx,vy,vz,ax,ay,az,jx,jy,jz\n";

    ofs << std::fixed << std::setprecision(9);

    for (double t = 0.0; t <= T + 1e-9; t += dump_dt)
    {
        const Eigen::Vector3d p = traj.getPos(t);
        const Eigen::Vector3d v = traj.getVel(t);
        const Eigen::Vector3d a = traj.getAcc(t);
        const Eigen::Vector3d j = traj.getJer(t);
        if (!isFinite3(p) || !isFinite3(v) || !isFinite3(a) || !isFinite3(j))
            continue;

        ofs << t << ","
            << p.x() << "," << p.y() << "," << p.z() << ","
            << v.x() << "," << v.y() << "," << v.z() << ","
            << a.x() << "," << a.y() << "," << a.z() << ","
            << j.x() << "," << j.y() << "," << j.z()
            << "\n";
    }
}

// ---------------- main ----------------
int main(int argc, char **argv)
{
    ros::init(argc, argv, "super_local_traj_benchmark");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    std::string sfc_dir, file_ext, csv_out;
    std::string config_path, config_name;

    pnh.param<std::string>("sfc_dir", sfc_dir, std::string("/home/kota/data"));
    pnh.param<std::string>("file_ext", file_ext, std::string(".mysco2"));
    pnh.param<std::string>("csv_out", csv_out, std::string("/home/kota/data/super_local_traj_benchmark.csv"));

    pnh.param<std::string>("config_path", config_path, std::string(""));
    pnh.param<std::string>("config_name", config_name, std::string("click_smooth_ros1.yaml"));

    bool seg_times_are_cumulative = true;
    pnh.param<bool>("seg_times_are_cumulative", seg_times_are_cumulative, true);

    double poly_seed_eps = 1e-6;
    pnh.param<double>("poly_seed_eps", poly_seed_eps, 1e-6);

    // Sampling + logging (violations)
    double sample_dt = 0.01;
    pnh.param<double>("sample_dt", sample_dt, 0.01);

    bool log_samples = true;
    pnh.param<bool>("log_samples", log_samples, true);

    std::string samples_csv_out;
    pnh.param<std::string>("samples_csv_out", samples_csv_out,
                           std::string("/home/kota/data/super_local_traj_samples.csv"));

    // NEW: dump trajectories for ROS2 visualization
    bool traj_dump_enable = true;
    pnh.param<bool>("traj_dump_enable", traj_dump_enable, true);

    std::string traj_dump_dir;
    pnh.param<std::string>("traj_dump_dir", traj_dump_dir, std::string(""));

    double traj_dump_dt = 0.03;
    pnh.param<double>("traj_dump_dt", traj_dump_dt, 0.03);

    if (sfc_dir.empty())
    {
        ROS_ERROR("~sfc_dir is empty. Set it to the folder containing .mysco2 files.");
        return 1;
    }

    if (traj_dump_dir.empty())
    {
        traj_dump_dir = sfc_dir + "/traj_dump_ros1";
    }

    if (traj_dump_enable)
    {
        if (!ensureDirectoryExists(traj_dump_dir))
        {
            ROS_ERROR("Failed to create/validate traj_dump_dir: %s", traj_dump_dir.c_str());
            return 1;
        }
        ROS_INFO("[SUPER BENCH] Trajectory dump enabled. dir=%s dt=%.4f",
                 traj_dump_dir.c_str(), traj_dump_dt);
    }

    if (config_path.empty())
    {
        const std::string pkg_path = ros::package::getPath("super_planner");
        if (pkg_path.empty())
        {
            ROS_ERROR("Could not resolve ROS package path for 'super_planner'. Provide ~config_path explicitly.");
            return 1;
        }
        config_path = pkg_path + "/config/" + config_name;
    }

    // Load boundary limits from the YAML (click_smooth_ros1.yaml)
    double max_vel = 999.0, max_acc = 999.0, max_jerk = 999.0;
    loadBoundaryLimitsFromYaml(config_path, max_vel, max_acc, max_jerk);
    ROS_INFO("[SUPER BENCH] Using limits from %s: max_vel=%.3f max_acc=%.3f max_jerk=%.3f",
             config_path.c_str(), max_vel, max_acc, max_jerk);

    // Create SUPER ros interface (ROS1)
    ros_interface::RosInterface::Ptr ros_ptr = std::make_shared<ros_interface::Ros1Interface>(nh);

    // Load SUPER planner config and extract ExpTrajOpt config
    super_planner::Config sp_cfg(config_path);
    traj_opt::ExpTrajOpt exp_opt(sp_cfg.exp_traj_cfg, ros_ptr);

    // RViz publishers (top_down.rviz)
    const std::string viz_frame = "world";
    const bool viz_enable = true;

    ros::Publisher pub_traj_exp =
        nh.advertise<visualization_msgs::MarkerArray>("/fsm_node/traj_opt/exp/mkr_arr", 1, true);
    ros::Publisher pub_debug_sfc =
        nh.advertise<visualization_msgs::MarkerArray>("/fsm_node/debug", 1, true);
    ros::Publisher pub_vis_exp_traj =
        nh.advertise<visualization_msgs::MarkerArray>("/fsm_node/visualization/exp_traj", 1, true);

    // Collect files
    namespace bfs = boost::filesystem;
    bfs::path dir(sfc_dir);
    if (!bfs::exists(dir) || !bfs::is_directory(dir))
    {
        ROS_ERROR("sfc_dir does not exist or is not a directory: %s", sfc_dir.c_str());
        return 1;
    }

    std::vector<bfs::path> files;
    for (bfs::directory_iterator it(dir); it != bfs::directory_iterator(); ++it)
    {
        if (!bfs::is_regular_file(*it))
            continue;
        const bfs::path p = it->path();
        if (p.extension().string() == file_ext)
            files.push_back(p);
    }
    std::sort(files.begin(), files.end());

    if (files.empty())
    {
        ROS_WARN("No files with extension %s found in %s", file_ext.c_str(), sfc_dir.c_str());
    }

    // Open per-sample CSV once
    std::ofstream ofs_samples;
    if (log_samples)
    {
        ofs_samples.open(samples_csv_out);
        if (!ofs_samples)
        {
            ROS_ERROR("Failed to open samples_csv_out: %s", samples_csv_out.c_str());
            return 1;
        }
        ofs_samples
            << "planner_name,file,t,seg_idx,"
            << "x,y,z,"
            << "vx,vy,vz,v_norm,"
            << "ax,ay,az,a_norm,"
            << "jx,jy,jz,j_norm,"
            << "sfc_best,sfc_violation,"
            << "v_violation,a_violation,j_violation\n";
        ofs_samples.flush();
    }

    std::vector<BenchResult, Eigen::aligned_allocator<BenchResult>> results;
    results.reserve(files.size());

    const std::string planner_name = "super";

    // Run benchmark
    for (const auto &fp : files)
    {
        BenchResult r;
        r.file = fp.string();
        const std::string fname = fp.filename().string();

        try
        {
            Eigen::Vector3d start, goal;
            std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> path_pts;
            std::vector<double> seg_end_times;
            super_planner::PolytopeVec sfcs;
            std::vector<Eigen::MatrixXd> sfc_planes_list;
            std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> init_ps_std;
            Eigen::VectorXd init_ts;

            loadMysco2ToSuper(fp.string(),
                              start, goal,
                              path_pts, seg_end_times,
                              sfcs, sfc_planes_list,
                              init_ps_std, init_ts,
                              poly_seed_eps,
                              seg_times_are_cumulative);

            r.start = start;
            r.goal = goal;

            if (sfcs.size() < 2)
            {
                r.success = false;
                r.status = "SKIP: corridor too short (<2 polytopes)";
            }
            else if ((int)init_ts.size() != (int)sfcs.size())
            {
                r.success = false;
                r.status = "SKIP: init_ts.size != sfcs.size";
            }
            else if ((int)init_ps_std.size() != (int)sfcs.size() - 1)
            {
                r.success = false;
                r.status = "SKIP: init_ps.size != sfcs.size-1";
            }
            else
            {
                // Build head/tail states at rest
                StatePVAJ head, tail;
                fillStatePVAJAtRest(head, start);
                fillStatePVAJAtRest(tail, goal);

                // Build guide path = {start, p1, p2, ..., goal}
                super_utils::vec_Vec3f guide_path;
                guide_path.reserve(path_pts.size());
                for (const auto &p : path_pts)
                    guide_path.emplace_back(p);

                // Build guide timestamps (must match guide_path.size())
                std::vector<double> guide_t;
                guide_t.reserve(path_pts.size());
                guide_t.push_back(0.0);

                const std::vector<double> dt = toDurations(seg_end_times, seg_times_are_cumulative);
                double acc = 0.0;
                for (double dti : dt)
                {
                    acc += std::max(1e-3, dti);
                    guide_t.push_back(acc);
                }

                if (guide_t.size() != guide_path.size())
                {
                    throw std::runtime_error("guide_t.size != guide_path.size (file inconsistent)");
                }

                // Optimize
                geometry_utils::Trajectory out_traj;
                const auto opt0 = std::chrono::steady_clock::now();
                const bool ok = exp_opt.optimize(head, tail, guide_path, guide_t, sfcs, out_traj);
                const auto opt1 = std::chrono::steady_clock::now();

                r.per_opt_runtime_ms = 1e3 * std::chrono::duration<double>(opt1 - opt0).count();

                if (!ok || out_traj.empty())
                {
                    r.success = false;
                    r.status = "NO_SOLUTION";
                }
                else
                {
                    r.success = true;
                    r.status = "OK";
                    r.total_traj_time_sec = out_traj.getTotalDuration();

                    // ---------------- Per-sample violations (0.01s) ----------------
                    r.max_sfc_violation = 0.0;
                    r.max_vel_violation = 0.0;
                    r.max_acc_violation = 0.0;
                    r.max_jerk_violation = 0.0;

                    // --- Smoothness accumulators (use trapezoidal rule over actual trajectory time) ---
                    double jerk_l1_int = 0.0; // ∫ ||j|| dt
                    double jerk_l2_int = 0.0; // ∫ ||j||^2 dt

                    bool have_prev = false;
                    double t_prev = 0.0;
                    double jnorm_prev = 0.0;
                    double jnorm2_prev = 0.0;

                    const double T = out_traj.getTotalDuration();
                    const double T_ref = (guide_t.empty() ? T : guide_t.back());
                    const double time_scale = (T > 1e-9 && T_ref > 1e-9) ? (T_ref / T) : 1.0;

                    // --- Trajectory length accumulator (trapezoidal on speed) ---
                    double length_m = 0.0;
                    bool have_prev_len = false;
                    double t_prev_len = 0.0;
                    double vnorm_prev = 0.0;

                    for (double t = 0.0; t <= T + 1e-9; t += sample_dt)
                    {
                        const double t_ref = t * time_scale; // map to guide timeline

                        Eigen::Vector3d p = out_traj.getPos(t);
                        Eigen::Vector3d v = out_traj.getVel(t);
                        Eigen::Vector3d a = out_traj.getAcc(t);
                        Eigen::Vector3d j = out_traj.getJer(t);

                        if (!isFinite3(p) || !isFinite3(v) || !isFinite3(a) || !isFinite3(j))
                            continue;

                        const double v_norm = v.norm();
                        const double a_norm = a.norm();
                        const double j_norm = j.norm();

                        // Integrate length: L += 0.5*(|v_prev| + |v|) * dt
                        if (!have_prev_len)
                        {
                            have_prev_len = true;
                            t_prev_len = t;
                            vnorm_prev = v_norm;
                        }
                        else
                        {
                            const double dt_i = t - t_prev_len;
                            if (dt_i > 1e-12)
                                length_m += 0.5 * (vnorm_prev + v_norm) * dt_i;

                            t_prev_len = t;
                            vnorm_prev = v_norm;
                        }

                        const double j_norm2 = j_norm * j_norm;

                        // Trapezoidal integration (more accurate than rectangle rule)
                        // Use actual dt between valid samples (important if you skip non-finite points).
                        if (!have_prev)
                        {
                            have_prev = true;
                            t_prev = t;
                            jnorm_prev = j_norm;
                            jnorm2_prev = j_norm2;
                        }
                        else
                        {
                            const double dt_i = t - t_prev;
                            if (dt_i > 1e-12)
                            {
                                jerk_l1_int += 0.5 * (jnorm_prev + j_norm) * dt_i;
                                jerk_l2_int += 0.5 * (jnorm2_prev + j_norm2) * dt_i;
                            }

                            t_prev = t;
                            jnorm_prev = j_norm;
                            jnorm2_prev = j_norm2;
                        }

                        const double v_violation = violationPositive(v_norm - max_vel);
                        const double a_violation = violationPositive(a_norm - max_acc);
                        const double j_violation = violationPositive(j_norm - max_jerk);

                        r.max_vel_violation = std::max(r.max_vel_violation, v_violation);
                        r.max_acc_violation = std::max(r.max_acc_violation, a_violation);
                        r.max_jerk_violation = std::max(r.max_jerk_violation, j_violation);

                        const int seg_idx = segmentIndexFromTime(guide_t, t_ref);

                        // Correct corridor-union check: point is OK if inside at least one polytope.
                        int best_poly_idx = -1;
                        double sfc_best = std::numeric_limits<double>::quiet_NaN();
                        double sfc_violation = 0.0;

                        if (!sfc_planes_list.empty())
                        {
                            // best = min_i maxPlaneValue(poly_i, p)
                            sfc_best = corridorUnionValue(sfc_planes_list, p, &best_poly_idx);
                            sfc_violation = violationPositive(sfc_best);
                            r.max_sfc_violation = std::max(r.max_sfc_violation, sfc_violation);
                        }
                        else
                        {
                            // No corridor info -> treat as huge violation
                            sfc_best = 1e9;
                            sfc_violation = 1e9;
                            r.max_sfc_violation = std::max(r.max_sfc_violation, sfc_violation);
                        }

                        if (log_samples)
                        {
                            ofs_samples
                                << planner_name << ","
                                << fname << ","
                                << std::fixed << std::setprecision(6) << t << ","
                                << seg_idx << ","
                                << p.x() << "," << p.y() << "," << p.z() << ","
                                << v.x() << "," << v.y() << "," << v.z() << "," << v_norm << ","
                                << a.x() << "," << a.y() << "," << a.z() << "," << a_norm << ","
                                << j.x() << "," << j.y() << "," << j.z() << "," << j_norm << ","
                                << sfc_best << "," << sfc_violation << ","
                                << v_violation << "," << a_violation << "," << j_violation
                                << "\n";
                        }
                    }

                    r.jerk_smoothness_l1 = jerk_l1_int;
                    r.jerk_rms = (T > 1e-9) ? std::sqrt(jerk_l2_int / T) : 0.0;
                    r.traj_length_m = length_m;

                    if (log_samples)
                        ofs_samples.flush();

                    // ---------------- Trajectory dump for ROS2 (NEW) ----------------
                    if (traj_dump_enable)
                    {
                        const std::string base = sanitizeForFilename(planner_name + std::string("__") + fname);
                        const std::string out_csv = (bfs::path(traj_dump_dir) / (std::string("traj_") + base + ".csv")).string();

                        dumpTrajectoryCsvV1(out_csv, planner_name, fname, viz_frame, out_traj, traj_dump_dt);
                        ROS_INFO("[SUPER BENCH] Dumped traj CSV: %s", out_csv.c_str());
                    }

                    // ---------------- Visualization (ROS1) ----------------
                    if (viz_enable)
                    {
                        const std::string ns_prefix = std::string("super_bench/") + fname;
                        auto traj_mk = makeTrajectoryMarkers(out_traj, viz_frame, ns_prefix, /*sample_dt=*/0.03);
                        auto sfc_mk = makeSfcMarkersFromPlaneList(sfc_planes_list, viz_frame, ns_prefix);

                        pub_traj_exp.publish(traj_mk);
                        pub_vis_exp_traj.publish(traj_mk);
                        pub_debug_sfc.publish(sfc_mk);

                        ros::spinOnce();
                        ros::Duration(0.03).sleep();
                    }
                }
            }
        }
        catch (const std::exception &e)
        {
            r.success = false;
            r.status = std::string("EXCEPTION: ") + e.what();
        }
        catch (...)
        {
            r.success = false;
            r.status = "EXCEPTION: unknown";
        }

        results.push_back(r);

        ROS_INFO_STREAM("[SUPER BENCH] " << fname
                                         << " success=" << (r.success ? 1 : 0)
                                         << " per_ms=" << std::fixed << std::setprecision(3) << r.per_opt_runtime_ms
                                         << " status=" << r.status
                                         << " maxSFC=" << r.max_sfc_violation
                                         << " maxV=" << r.max_vel_violation
                                         << " maxA=" << r.max_acc_violation
                                         << " maxJ=" << r.max_jerk_violation);
    }

    if (log_samples)
    {
        ofs_samples.flush();
        ofs_samples.close();
        ROS_INFO("Wrote samples CSV: %s", samples_csv_out.c_str());
    }

    // Write summary CSV
    std::ofstream ofs(csv_out);
    if (!ofs)
    {
        ROS_ERROR("Failed to open csv_out: %s", csv_out.c_str());
        return 1;
    }

    ofs << "planner_name,file,success,status,per_opt_runtime_ms,"
           "cost_value,total_traj_time_sec,traj_length_m,"
           "start_x,start_y,start_z,goal_x,goal_y,goal_z,"
           "max_sfc_violation,max_vel_violation,max_acc_violation,max_jerk_violation,"
           "jerk_smoothness_l1,jerk_rms\n";

    for (const auto &r : results)
    {
        ofs << planner_name << ","
            << bfs::path(r.file).filename().string() << ","
            << (r.success ? 1 : 0) << ","
            << "\"" << r.status << "\"" << ","
            << std::fixed << std::setprecision(3)
            << r.per_opt_runtime_ms << ","
            << r.cost_value << ","
            << r.total_traj_time_sec << ","
            << r.traj_length_m << ","
            << r.start.x() << "," << r.start.y() << "," << r.start.z() << ","
            << r.goal.x() << "," << r.goal.y() << "," << r.goal.z() << ","
            << r.max_sfc_violation << ","
            << r.max_vel_violation << ","
            << r.max_acc_violation << ","
            << r.max_jerk_violation << ","
            << r.jerk_smoothness_l1 << ","
            << r.jerk_rms
            << "\n";
    }

    ofs.flush();
    ROS_INFO("Wrote summary CSV: %s", csv_out.c_str());
    return 0;
}
