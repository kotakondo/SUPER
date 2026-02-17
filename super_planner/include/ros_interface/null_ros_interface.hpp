#pragma once

#include <ros_interface/ros_interface.hpp>
#include <chrono>
#include <iostream>

namespace ros_interface {

class NullRosInterface : public RosInterface {
public:
    NullRosInterface() {
        visualization_en_ = false;
    }

    void debug(const std::string&) override {}
    void info(const std::string&) override {}
    void warn(const std::string& msg) override { std::cerr << "[WARN] " << msg << std::endl; }
    void error(const std::string& msg) override { std::cerr << "[ERROR] " << msg << std::endl; }
    void fatal(const std::string& msg) override { std::cerr << "[FATAL] " << msg << std::endl; }

    void setSimTime(const double&) override {}

    double getSimTime() override {
        using namespace std::chrono;
        return duration<double>(steady_clock::now().time_since_epoch()).count();
    }

    void getSimTime(int32_t& sec, uint32_t& nsec) override {
        using namespace std::chrono;
        auto now = steady_clock::now();
        auto dur = now.time_since_epoch();
        auto s = duration_cast<seconds>(dur);
        sec = static_cast<int32_t>(s.count());
        nsec = static_cast<uint32_t>(duration_cast<nanoseconds>(dur - s).count());
    }

    void vizExpTraj(const Trajectory&, const std::string&) override {}
    void vizBackupTraj(const Trajectory&) override {}
    void vizFrontendPath(const vec_Vec3f&) override {}
    void vizExpSfc(const PolytopeVec&) override {}
    void vizBackupSfc(const Polytope&) override {}
    void vizGoalPath(const vec_Vec3f&) override {}
    void vizCommittedTraj(const Trajectory&, const double&) override {}
    void vizYawTraj(const Trajectory&, const Trajectory&) override {}
    void vizAstarBoundingBox(const Vec3f&, const Vec3f&) override {}
    void vizAstarPoints(const Vec3f&, const Color&, const std::string&, const double&, const int&) override {}
    void vizReplanLog(const Trajectory&, const Trajectory&,
                      const Trajectory&, const Trajectory&,
                      const PolytopeVec&, const Polytope&,
                      const vec_Vec3f&, const int&) override {}
    void vizCiriSeedLine(const Vec3f&, const Vec3f&, const double&) override {}
    void vizCiriEllipsoid(const Ellipsoid&) override {}
    void vizCiriInfeasiblePoint(const Vec3f) override {}
    void vizCiriPolytope(const Polytope&, const std::string&) override {}
    void vizCiriPointCloud(const vec_Vec3f&) override {}
};

} // namespace ros_interface
