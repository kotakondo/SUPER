#include <chrono>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>
#include <vector>
#include <algorithm>

#include <ros/ros.h>

#include <Eigen/Dense>

#include <traj_opt/exp_traj_optimizer_s4.h>
#include <ros_interface/null_ros_interface.hpp>

namespace fs = std::filesystem;
using namespace std::chrono;
using namespace super_utils;
using namespace geometry_utils;
using namespace traj_opt;

// ======================== .mysco2 reader ========================

struct Mysco2Data {
    Vec3f start{Vec3f::Zero()};
    Vec3f goal{Vec3f::Zero()};
    vec_Vec3f path;
    std::vector<double> seg_end_times;
    // Per-segment original A/b (for analysis)
    std::vector<Eigen::MatrixXd> seg_A;
    std::vector<Eigen::VectorXd> seg_b;
    // SUPER-format polytopes
    PolytopeVec sfcs;
};

static uint32_t readU32(std::ifstream& ifs) {
    uint32_t v;
    ifs.read(reinterpret_cast<char*>(&v), sizeof(v));
    if (!ifs) throw std::runtime_error("Corrupt .mysco2 (u32).");
    return v;
}

static double readD(std::ifstream& ifs) {
    double v;
    ifs.read(reinterpret_cast<char*>(&v), sizeof(v));
    if (!ifs) throw std::runtime_error("Corrupt .mysco2 (double).");
    return v;
}

static Mysco2Data loadMysco2(const fs::path& file, double poly_seed_eps = 1e-6) {
    std::ifstream ifs(file, std::ios::binary);
    if (!ifs) throw std::runtime_error("Failed to open: " + file.string());

    char magic[8];
    ifs.read(magic, 8);
    if (!ifs) throw std::runtime_error("Corrupt .mysco2 (magic): " + file.string());
    if (std::string(magic, magic + 8).rfind("MYSCO2", 0) != 0)
        throw std::runtime_error("Bad magic in: " + file.string());

    uint32_t version = readU32(ifs);
    if (version != 1) throw std::runtime_error("Unsupported .mysco2 version in: " + file.string());

    Mysco2Data data;
    {
        double sx = readD(ifs), sy = readD(ifs), sz = readD(ifs);
        data.start = Vec3f(sx, sy, sz);
    }
    {
        double gx = readD(ifs), gy = readD(ifs), gz = readD(ifs);
        data.goal = Vec3f(gx, gy, gz);
    }

    uint32_t num_path_pts = readU32(ifs);
    data.path.reserve(num_path_pts);
    for (uint32_t i = 0; i < num_path_pts; ++i) {
        double px = readD(ifs), py = readD(ifs), pz = readD(ifs);
        data.path.push_back(Vec3f(px, py, pz));
    }

    uint32_t num_seg = readU32(ifs);
    data.seg_end_times.resize(num_seg);
    for (uint32_t i = 0; i < num_seg; ++i)
        data.seg_end_times[i] = readD(ifs);

    if (data.path.size() < 2 || (data.path.size() - 1) != num_seg)
        throw std::runtime_error("File inconsistent: path.size()-1 != num_seg in " + file.string());

    data.seg_A.resize(num_seg);
    data.seg_b.resize(num_seg);
    data.sfcs.resize(num_seg);

    for (uint32_t si = 0; si < num_seg; ++si) {
        uint32_t mplanes = readU32(ifs);

        Eigen::MatrixXd A(mplanes, 3);
        Eigen::VectorXd b(mplanes);
        for (uint32_t r = 0; r < mplanes; ++r)
            for (int c = 0; c < 3; ++c)
                A(r, c) = readD(ifs);
        for (uint32_t r = 0; r < mplanes; ++r)
            b(r) = readD(ifs);

        // Seed-consistency fix: ensure midpoint satisfies A*x <= b
        Vec3f pt_inside = (data.path[si] + data.path[si + 1]) / 2.0;
        for (int r = 0; r < A.rows(); ++r) {
            double v = A.row(r).dot(pt_inside.cast<double>()) - b(r);
            if (v > poly_seed_eps) {
                A.row(r) *= -1.0;
                b(r) *= -1.0;
            }
        }

        // Store original A/b for analysis
        data.seg_A[si] = A;
        data.seg_b[si] = b;

        // Convert to SUPER format: [n_x, n_y, n_z, d] where n*x + d <= 0
        // From A*x <= b: A_row * x - b_i <= 0, so plane = [A_row, -b_i]
        MatD4f planes(mplanes, 4);
        for (int r = 0; r < static_cast<int>(mplanes); ++r) {
            double norm = A.row(r).norm();
            if (norm < 1e-12) {
                planes.row(r).setZero();
                continue;
            }
            planes(r, 0) = A(r, 0);
            planes(r, 1) = A(r, 1);
            planes(r, 2) = A(r, 2);
            planes(r, 3) = -b(r);
        }
        data.sfcs[si] = Polytope(planes);
    }

    return data;
}

// ======================== Constraint analysis ========================

struct ConstraintReport {
    double corridor_max_min_violation{0.0};
    double corridor_violation_pct{0.0};  // % of sample points outside all corridors

    double v_max_observed{0.0};
    double v_violation_pct{0.0};   // % of sample points violating v_max

    double a_max_observed{0.0};
    double a_violation_pct{0.0};   // % of sample points violating a_max

    double j_max_observed{0.0};
    double j_violation_pct{0.0};   // % of sample points violating j_max

    double jerk_smoothness_l1{0.0};
    double jerk_rms{0.0};
    double traj_length_m{0.0};
};

static ConstraintReport analyzeTrajectory(
    const Trajectory& traj,
    const std::vector<Eigen::MatrixXd>& seg_A,
    const std::vector<Eigen::VectorXd>& seg_b,
    double v_max, double a_max, double j_max,
    double dc = 0.01,
    double dyn_tol = 1e-3)
{
    ConstraintReport rep;
    double T = traj.getTotalDuration();
    if (T <= 0.0) return rep;

    int num_samples = static_cast<int>(std::ceil(T / dc)) + 1;
    int num_poly = static_cast<int>(seg_A.size());

    auto maxAbsComponent = [](const Eigen::Vector3d& x) -> double {
        return x.cwiseAbs().maxCoeff();
    };

    double worst_corridor = -std::numeric_limits<double>::infinity();
    double vmax_obs = 0.0, amax_obs = 0.0, jmax_obs = 0.0;

    int corr_viol_count = 0;
    int v_viol_count = 0, a_viol_count = 0, j_viol_count = 0;

    double jerk_l1 = 0.0;
    double jerk_l2_int = 0.0;
    double length_m = 0.0;

    for (int k = 0; k < num_samples; ++k) {
        double t = std::min(static_cast<double>(k) * dc, T);

        Eigen::Vector3d pos = traj.getPos(t);
        Eigen::Vector3d vel = traj.getVel(t);
        Eigen::Vector3d acc = traj.getAcc(t);
        Eigen::Vector3d jer = traj.getJer(t);

        // Corridor check: point is safe if ANY polytope contains it.
        // For polytope i: d = A[i]*pos - b[i]; maxCoeff <= 0 means inside.
        // best = min over all polytopes of maxCoeff (most-inside polytope).
        if (num_poly > 0) {
            double best = std::numeric_limits<double>::infinity();
            for (int i = 0; i < num_poly; ++i) {
                Eigen::VectorXd d = seg_A[i] * pos - seg_b[i];
                double violation = d.maxCoeff();
                if (violation < best) best = violation;
            }
            if (best > worst_corridor) worst_corridor = best;
            if (best > dyn_tol) corr_viol_count++;
        }

        // Dynamic limits (L-inf / max-component)
        double vc = maxAbsComponent(vel);
        vmax_obs = std::max(vmax_obs, vc);
        if (vc > v_max + dyn_tol) v_viol_count++;

        double ac = maxAbsComponent(acc);
        amax_obs = std::max(amax_obs, ac);
        if (ac > a_max + dyn_tol) a_viol_count++;

        double jc = maxAbsComponent(jer);
        jmax_obs = std::max(jmax_obs, jc);
        if (jc > j_max + dyn_tol) j_viol_count++;

        // Smoothness (trapezoidal rule, skip k=0)
        if (k > 0) {
            double jnorm = jer.norm();
            jerk_l1 += jnorm * dc;
            jerk_l2_int += (jnorm * jnorm) * dc;

            double vnorm = vel.norm();
            Eigen::Vector3d prev_vel = traj.getVel(std::min(static_cast<double>(k - 1) * dc, T));
            double prev_vnorm = prev_vel.norm();
            length_m += 0.5 * (vnorm + prev_vnorm) * dc;
        }
    }

    rep.corridor_max_min_violation = worst_corridor;
    rep.corridor_violation_pct = (num_poly > 0) ? 100.0 * corr_viol_count / num_samples : 0.0;

    rep.v_max_observed = vmax_obs;
    rep.v_violation_pct = 100.0 * v_viol_count / num_samples;

    rep.a_max_observed = amax_obs;
    rep.a_violation_pct = 100.0 * a_viol_count / num_samples;

    rep.j_max_observed = jmax_obs;
    rep.j_violation_pct = 100.0 * j_viol_count / num_samples;

    rep.jerk_smoothness_l1 = jerk_l1;
    rep.jerk_rms = (T > 0.0) ? std::sqrt(jerk_l2_int / T) : 0.0;
    rep.traj_length_m = length_m;

    return rep;
}

// ======================== Trajectory dump ========================

static void dumpTrajectory(const Trajectory& traj,
                           const std::string& dump_dir,
                           const std::string& case_filename,
                           double dc = 0.01)
{
    fs::path dir(dump_dir);
    fs::create_directories(dir);

    // Replace .mysco2 extension with .csv
    std::string out_name = fs::path(case_filename).stem().string() + ".csv";
    fs::path out_path = dir / out_name;

    std::ofstream ofs(out_path);
    if (!ofs) {
        std::cerr << "WARNING: could not open " << out_path << " for writing\n";
        return;
    }

    ofs << "t,px,py,pz,vx,vy,vz,ax,ay,az,jx,jy,jz\n";

    double T = traj.getTotalDuration();
    int num_samples = static_cast<int>(std::ceil(T / dc)) + 1;

    ofs << std::setprecision(9);
    for (int k = 0; k < num_samples; ++k) {
        double t = std::min(static_cast<double>(k) * dc, T);
        Eigen::Vector3d pos = traj.getPos(t);
        Eigen::Vector3d vel = traj.getVel(t);
        Eigen::Vector3d acc = traj.getAcc(t);
        Eigen::Vector3d jer = traj.getJer(t);

        ofs << t
            << "," << pos.x() << "," << pos.y() << "," << pos.z()
            << "," << vel.x() << "," << vel.y() << "," << vel.z()
            << "," << acc.x() << "," << acc.y() << "," << acc.z()
            << "," << jer.x() << "," << jer.y() << "," << jer.z()
            << "\n";
    }

    ofs.flush();
}

// ======================== CLI argument parsing ========================

struct BenchArgs {
    std::string sfc_dir = "/home/kkondo/code/dynus_ws/src/dynus/data";
    std::string config = "";
    std::string output_csv = "/home/kkondo/code/dynus_ws/src/dynus/benchmark_data/default/super_benchmark.csv";
    std::string dump_dir = "";  // empty = no dump
    double v_max = 1.0;
    double a_max = 2.0;
    double j_max = 3.0;
};

static BenchArgs parseArgs(int argc, char** argv) {
    BenchArgs args;

    // Default config path relative to ROOT_DIR
    args.config = std::string(ROOT_DIR) + "config/benchmark.yaml";

    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--sfc_dir" && i + 1 < argc)      args.sfc_dir = argv[++i];
        else if (arg == "--config" && i + 1 < argc)   args.config = argv[++i];
        else if (arg == "--output_csv" && i + 1 < argc) args.output_csv = argv[++i];
        else if (arg == "--v_max" && i + 1 < argc)    args.v_max = std::stod(argv[++i]);
        else if (arg == "--a_max" && i + 1 < argc)    args.a_max = std::stod(argv[++i]);
        else if (arg == "--j_max" && i + 1 < argc)    args.j_max = std::stod(argv[++i]);
        else if (arg == "--dump_dir" && i + 1 < argc) args.dump_dir = argv[++i];
        else if (arg == "--help" || arg == "-h") {
            std::cout << "Usage: local_traj_benchmark [options]\n"
                      << "  --sfc_dir DIR       Path to .mysco2 files\n"
                      << "  --config FILE       YAML config path\n"
                      << "  --output_csv FILE   Output CSV path\n"
                      << "  --v_max VALUE       Max velocity for analysis\n"
                      << "  --a_max VALUE       Max acceleration for analysis\n"
                      << "  --j_max VALUE       Max jerk for analysis\n"
                      << "  --dump_dir DIR      Dump per-case trajectory CSVs to DIR\n";
            std::exit(0);
        }
    }
    return args;
}

// ======================== CSV writer ========================

struct BenchResult {
    std::string file;
    bool success{false};
    std::string status;
    double per_opt_runtime_ms{0.0};
    double total_opt_runtime_ms{0.0};
    double total_traj_time_sec{0.0};
    double traj_length_m{0.0};
    double jerk_smoothness_l1{0.0};
    double jerk_rms{0.0};
    double corridor_violation_pct{0.0};  // % of sample points outside all corridors
    double v_violation_pct{0.0};   // % of sample points violating v_max
    double a_violation_pct{0.0};   // % of sample points violating a_max
    double j_violation_pct{0.0};   // % of sample points violating j_max
    double corridor_max_min_violation{0.0};
    double v_max_observed{0.0};
    double a_max_observed{0.0};
    double j_max_observed{0.0};
    Vec3f start{Vec3f::Zero()};
    Vec3f goal{Vec3f::Zero()};
};

static void writeCsv(const std::string& path, const std::vector<BenchResult>& results) {
    // Ensure output directory exists
    fs::path out_path(path);
    fs::create_directories(out_path.parent_path());

    std::ofstream ofs(path);
    if (!ofs) {
        std::cerr << "Failed to open CSV for writing: " << path << std::endl;
        return;
    }

    ofs << "planner_name,file,success,status,gurobi_error,per_opt_runtime_ms,total_opt_runtime_ms,"
           "factor_used,cost_value,total_traj_time_sec,"
           "corridor_max_min_violation,corridor_t_at_max,corridor_px,corridor_py,corridor_pz,"
           "corridor_best_poly_idx,corridor_violation_pct,"
           "v_max_observed,v_max_excess,v_t_at_max,v_violation_pct,"
           "a_max_observed,a_max_excess,a_t_at_max,a_violation_pct,"
           "j_max_observed,j_max_excess,j_t_at_max,j_violation_pct,"
           "jerk_smoothness_l1,jerk_rms,"
           "traj_length_m,"
           "start_x,start_y,start_z,goal_x,goal_y,goal_z\n";

    for (const auto& r : results) {
        ofs << "SUPER,"
            << fs::path(r.file).filename().string() << ","
            << (r.success ? 1 : 0) << ","
            << "\"" << r.status << "\","
            << 0 << ","  // gurobi_error (N/A)
            << std::fixed << std::setprecision(3)
            << r.per_opt_runtime_ms << ","
            << r.total_opt_runtime_ms << ","
            << 0.0 << ","   // factor_used (N/A)
            << 0.0 << ","   // cost_value (N/A)
            << r.total_traj_time_sec << ","
            << std::setprecision(9)
            << r.corridor_max_min_violation << ","
            << 0.0 << ","   // corridor_t_at_max
            << 0.0 << "," << 0.0 << "," << 0.0 << ","  // corridor_p
            << -1 << ","    // corridor_best_poly_idx
            << std::setprecision(4)
            << r.corridor_violation_pct << ","
            << r.v_max_observed << ","
            << 0.0 << ","   // v_max_excess
            << 0.0 << ","   // v_t_at_max
            << std::setprecision(4)
            << r.v_violation_pct << ","
            << std::setprecision(9)
            << r.a_max_observed << ","
            << 0.0 << ","   // a_max_excess
            << 0.0 << ","   // a_t_at_max
            << std::setprecision(4)
            << r.a_violation_pct << ","
            << std::setprecision(9)
            << r.j_max_observed << ","
            << 0.0 << ","   // j_max_excess
            << 0.0 << ","   // j_t_at_max
            << std::setprecision(4)
            << r.j_violation_pct << ","
            << std::setprecision(9)
            << r.jerk_smoothness_l1 << ","
            << r.jerk_rms << ","
            << r.traj_length_m << ","
            << std::setprecision(3)
            << r.start.x() << "," << r.start.y() << "," << r.start.z() << ","
            << r.goal.x() << "," << r.goal.y() << "," << r.goal.z()
            << "\n";
    }

    ofs.flush();
    std::cout << "Wrote CSV: " << path << std::endl;
}

// ======================== main ========================

int main(int argc, char** argv) {
    // ros::init needed for link compatibility with the `super` static library
    // (compiled with -DUSE_ROS1). No roscore needed since we use NullRosInterface.
    ros::init(argc, argv, "super_benchmark", ros::init_options::NoSigintHandler);

    BenchArgs args = parseArgs(argc, argv);

    std::cout << "=== SUPER Local Trajectory Benchmark ===" << std::endl;
    std::cout << "  sfc_dir:    " << args.sfc_dir << std::endl;
    std::cout << "  config:     " << args.config << std::endl;
    std::cout << "  output_csv: " << args.output_csv << std::endl;
    std::cout << "  v_max=" << args.v_max << " a_max=" << args.a_max << " j_max=" << args.j_max << std::endl;

    // Load config and create optimizer
    auto ros_ptr = std::make_shared<ros_interface::NullRosInterface>();
    Config cfg(args.config, "exp_traj");
    auto optimizer = std::make_shared<ExpTrajOpt>(cfg, ros_ptr);

    // Discover .mysco2 files
    if (!fs::exists(args.sfc_dir)) {
        std::cerr << "ERROR: sfc_dir does not exist: " << args.sfc_dir << std::endl;
        return 1;
    }

    std::vector<fs::path> files;
    for (const auto& ent : fs::directory_iterator(args.sfc_dir)) {
        if (ent.is_regular_file() && ent.path().extension() == ".mysco2")
            files.push_back(ent.path());
    }
    std::sort(files.begin(), files.end());

    std::cout << "Found " << files.size() << " .mysco2 files" << std::endl;

    // Benchmark loop
    std::vector<BenchResult> results;
    results.reserve(files.size());

    int success_count = 0;

    for (size_t case_idx = 0; case_idx < files.size(); ++case_idx) {
        BenchResult r;
        r.file = files[case_idx].string();
        std::string fname = files[case_idx].filename().string();

        try {
            std::cout << "[" << case_idx + 1 << "/" << files.size() << "] "
                      << fname << ": loading..." << std::flush;
            Mysco2Data data = loadMysco2(files[case_idx]);
            r.start = data.start;
            r.goal = data.goal;

            std::cout << " " << data.sfcs.size() << " segs, "
                      << data.path.size() << " pts... " << std::flush;

            // Build boundary conditions (at rest)
            // Use path endpoints (guaranteed inside corridors) rather than
            // global start/goal which may lie outside the SFC polytopes.
            StatePVAJ headPVAJ = StatePVAJ::Zero();
            headPVAJ.col(0) = data.path.front();
            StatePVAJ tailPVAJ = StatePVAJ::Zero();
            tailPVAJ.col(0) = data.path.back();

            // Build guide trajectory
            vec_E<Vec3f> guide_path(data.path.begin(), data.path.end());
            std::vector<double> guide_t;
            guide_t.push_back(0.0);
            for (double t : data.seg_end_times)
                guide_t.push_back(t);

            // Optimize
            Trajectory out_traj;
            PolytopeVec sfcs = data.sfcs;

            std::cout << "optimizing..." << std::flush;
            auto t0 = steady_clock::now();
            bool ok = optimizer->optimize(headPVAJ, tailPVAJ, guide_path, guide_t, sfcs, out_traj);
            auto t1 = steady_clock::now();

            double elapsed_ms = 1e3 * duration<double>(t1 - t0).count();
            r.per_opt_runtime_ms = elapsed_ms;
            r.total_opt_runtime_ms = elapsed_ms;

            if (ok && !out_traj.empty()) {
                r.success = true;
                r.status = "OK";
                r.total_traj_time_sec = out_traj.getTotalDuration();
                success_count++;

                // Analyze trajectory
                auto rep = analyzeTrajectory(out_traj,
                                             data.seg_A, data.seg_b,
                                             args.v_max, args.a_max, args.j_max);
                r.traj_length_m = rep.traj_length_m;
                r.jerk_smoothness_l1 = rep.jerk_smoothness_l1;
                r.jerk_rms = rep.jerk_rms;
                r.corridor_violation_pct = rep.corridor_violation_pct;
                r.corridor_max_min_violation = rep.corridor_max_min_violation;
                r.v_violation_pct = rep.v_violation_pct;
                r.v_max_observed = rep.v_max_observed;
                r.a_violation_pct = rep.a_violation_pct;
                r.a_max_observed = rep.a_max_observed;
                r.j_violation_pct = rep.j_violation_pct;
                r.j_max_observed = rep.j_max_observed;

                // Dump trajectory samples if requested
                if (!args.dump_dir.empty()) {
                    dumpTrajectory(out_traj, args.dump_dir, fname);
                }

                std::cout << "[" << case_idx + 1 << "/" << files.size() << "] "
                          << fname << ": OK  "
                          << std::fixed << std::setprecision(2)
                          << elapsed_ms << "ms  T=" << r.total_traj_time_sec << "s"
                          << std::endl;
            } else {
                r.success = false;
                r.status = "OPT_FAILED";
                std::cout << "[" << case_idx + 1 << "/" << files.size() << "] "
                          << fname << ": FAILED  "
                          << std::fixed << std::setprecision(2)
                          << elapsed_ms << "ms" << std::endl;
            }
        } catch (const std::exception& e) {
            r.success = false;
            r.status = std::string("EXCEPTION: ") + e.what();
            std::cerr << "[" << case_idx + 1 << "/" << files.size() << "] "
                      << fname << ": EXCEPTION: " << e.what() << std::endl;
        }

        results.push_back(std::move(r));
    }

    // Write CSV
    writeCsv(args.output_csv, results);

    // Print summary
    std::cout << "\n=== Benchmark Summary ===" << std::endl;
    std::cout << "  Total cases: " << results.size() << std::endl;
    std::cout << "  Successes:   " << success_count << " ("
              << std::fixed << std::setprecision(1)
              << (results.empty() ? 0.0 : 100.0 * success_count / results.size()) << "%)" << std::endl;

    // Compute averages over successful cases
    double sum_time = 0, sum_traj_time = 0, sum_length = 0, sum_jerk = 0;
    double sum_corr_viol_pct = 0;
    double sum_v_viol_pct = 0, sum_a_viol_pct = 0, sum_j_viol_pct = 0;
    for (const auto& r : results) {
        if (!r.success) continue;
        sum_time += r.per_opt_runtime_ms;
        sum_traj_time += r.total_traj_time_sec;
        sum_length += r.traj_length_m;
        sum_jerk += r.jerk_smoothness_l1;
        sum_corr_viol_pct += r.corridor_violation_pct;
        sum_v_viol_pct += r.v_violation_pct;
        sum_a_viol_pct += r.a_violation_pct;
        sum_j_viol_pct += r.j_violation_pct;
    }

    if (success_count > 0) {
        std::cout << "  Avg opt time:   " << std::fixed << std::setprecision(2) << sum_time / success_count << " ms" << std::endl;
        std::cout << "  Avg traj time:  " << sum_traj_time / success_count << " s" << std::endl;
        std::cout << "  Avg path length:" << sum_length / success_count << " m" << std::endl;
        std::cout << "  Avg jerk L1:    " << sum_jerk / success_count << std::endl;
        std::cout << "  Corridor viol:  " << std::setprecision(2) << sum_corr_viol_pct / success_count << "% (avg sample-point violation)" << std::endl;
        std::cout << "  Vel viol:       " << sum_v_viol_pct / success_count << "% (avg sample-point violation)" << std::endl;
        std::cout << "  Acc viol:       " << sum_a_viol_pct / success_count << "% (avg sample-point violation)" << std::endl;
        std::cout << "  Jerk viol:      " << sum_j_viol_pct / success_count << "% (avg sample-point violation)" << std::endl;
    }

    std::cout << "\nCSV written to: " << args.output_csv << std::endl;

    return 0;
}
