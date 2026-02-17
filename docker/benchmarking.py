#!/usr/bin/env python3
import subprocess
import sys
import time
import math
import os
import argparse
import yaml
import rospy
import rosgraph
from geometry_msgs.msg import PoseStamped
from quadrotor_msgs.msg import PositionCommand

# CSV file to log simulation outcomes.
CSV_PATH = "/home/kota/data/goal_reached_status_super.csv"

# Base path for the perfect_drone_sim config directory (inside Docker)
DRONE_CONFIG_DIR = "/home/kota/super_ws/src/SUPER/mars_uav_sim/perfect_drone_sim/config"

# Template for the drone sim config (all fields except pcd_name are constant)
DRONE_CONFIG_TEMPLATE = {
    "pcd_name": "easy_forest.pcd",
    "mesh_resource": "package://perfect_drone_sim/meshes/yunque-M.dae",
    "init_position": {"x": 0.0, "y": 0.0, "z": 2.0},
    "init_yaw": 0.0,
    "is_360lidar": True,
    "polar_resolution": 0.4,
    "downsample_res": 0.1,
    "vertical_fov": 178.0,
    "sensing_blind": 0.1,
    "sensing_horizon": 30,
    "sensing_rate": 10,
    "print_time_consumption": False,
    "lidar_type": 2,
    "depth_image_en": False,
}

def run_command(cmd):
    """Launch a command via bash -c and return the Popen handle."""
    return subprocess.Popen(["bash", "-c", cmd])

class SimulationMonitor:
    def __init__(self, goal, threshold):
        """
        Monitors the robot's pose to determine if the goal has been reached.

        goal: tuple (x, y, z) for the desired goal position.
        threshold: distance (in meters) within which the goal is considered reached.

        This implementation subscribes to '"/planning/pos_cmd"'.
        """
        self.goal = goal
        self.threshold = threshold
        self.current_pose = None
        rospy.Subscriber("/planning/pos_cmd", PositionCommand, self.pose_callback)

    def pose_callback(self, msg):
        self.current_pose = (
            msg.position.x,
            msg.position.y,
            msg.position.z
        )
        # rospy.loginfo("Current position: {}".format(self.current_pose))

    def reached_goal(self):
        if self.current_pose is None:
            return False
        dx = self.current_pose[0] - self.goal[0]
        dy = self.current_pose[1] - self.goal[1]
        dz = self.current_pose[2] - self.goal[2]
        dist = math.sqrt(dx*dx + dy*dy + dz*dz)
        rospy.loginfo("Distance to goal: {:.3f}".format(dist))
        return dist <= self.threshold

def wait_for_ros_master(timeout=30):
    """Wait for the ROS master to become available, or exit after timeout."""
    start_time = time.time()
    master = rosgraph.Master('/simulation_monitor_super')
    while True:
        try:
            master.getUri()
            rospy.loginfo("ROS master is available!")
            return True
        except Exception:
            if time.time() - start_time > timeout:
                rospy.logerr("Timeout waiting for ROS master!")
                return False
            rospy.loginfo("Waiting for ROS master...")
            time.sleep(1)

def write_drone_config(env_name):
    """
    Write a per-environment drone config file that points to the correct PCD.

    Returns the config filename (relative name, not full path).
    """
    config = dict(DRONE_CONFIG_TEMPLATE)
    config["pcd_name"] = f"{env_name}.pcd"

    config_filename = f"static_forest_{env_name}.yaml"
    config_path = os.path.join(DRONE_CONFIG_DIR, config_filename)

    with open(config_path, "w") as f:
        yaml.dump(config, f, default_flow_style=False)

    rospy.loginfo(f"Wrote drone config: {config_path} (pcd: {env_name}.pcd)")
    return config_filename

def launch_simulation(sim_num, env_source, goal_coords, launch_file="click_demo.launch",
                      drone_config=None, planner_config=None, data_dir="/home/kota/data"):
    """
    Launch simulation processes (excluding roscore) for super.

    Commands are defined according to your super setup.
    """
    launch_args = f"simulation_number:={sim_num}"
    if drone_config:
        launch_args += f" drone_config:={drone_config}"
    if planner_config:
        launch_args += f" planner_config:={planner_config}"

    mission_planner_cmd = (
        f"{env_source} && roslaunch --wait mission_planner {launch_file} {launch_args}"
    )
    goal_pub_cmd = (
        f"{env_source} && sleep 10 && "
        "rostopic pub /goal geometry_msgs/PoseStamped "
        "'{header: {frame_id: \"world\"}, pose: {position: "
        f"{{x: {goal_coords[0]}, y: {goal_coords[1]}, z: {goal_coords[2]}}}, "
        "orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}' -1"
    )
    bag_file = f"{data_dir}/super_num_{sim_num}.bag"
    rosbag_cmd = f"{env_source} && rosbag record -a -O {bag_file}"

    processes = []

    rospy.loginfo(f"Launching {launch_file}...")
    processes.append(run_command(mission_planner_cmd))
    time.sleep(2)

    rospy.loginfo(f"Launching rosbag recording to {bag_file}...")
    processes.append(run_command(rosbag_cmd))
    time.sleep(10)

    rospy.loginfo("Launching goal publisher (after 10s delay)...")
    processes.append(run_command(goal_pub_cmd))
    time.sleep(2)

    return processes

def kill_processes(processes):
    for p in processes:
        p.terminate()
    for p in processes:
        try:
            p.wait(timeout=5)
        except subprocess.TimeoutExpired:
            p.kill()

def run_trials(total_sim_runs, max_duration, goal_coords, threshold,
               env_source, launch_file, drone_config, data_dir, csv_path,
               planner_config=None):
    """Run a series of trials, logging results to csv_path."""
    os.makedirs(data_dir, exist_ok=True)

    with open(csv_path, "w") as f:
        f.write("sim_num,status\n")

    sim_num = 0
    while not rospy.is_shutdown() and sim_num < total_sim_runs:
        rospy.loginfo("\n==== Starting simulation number {} ====".format(sim_num))
        monitor = SimulationMonitor(goal=goal_coords, threshold=threshold)
        sim_start_time = time.time()
        processes = launch_simulation(
            sim_num, env_source, goal_coords,
            launch_file=launch_file,
            drone_config=drone_config,
            planner_config=planner_config,
            data_dir=data_dir,
        )
        goal_reached = False

        # Monitor simulation until the goal is reached or max duration is exceeded.
        while time.time() - sim_start_time < max_duration and not rospy.is_shutdown():
            if monitor.reached_goal():
                rospy.loginfo("Goal reached!")
                goal_reached = True
                break
            time.sleep(1)

        travel_time = time.time() - sim_start_time
        if goal_reached:
            status = "reached"
            rospy.loginfo("Simulation {} ended successfully in {:.1f} seconds.".format(sim_num, travel_time))
        else:
            status = "timeout"
            rospy.loginfo("Simulation {} timed out after {:.1f} seconds.".format(sim_num, travel_time))

        # Log the simulation result.
        with open(csv_path, "a") as csv_file:
            csv_file.write(f"{sim_num},{status}\n")

        # Terminate all simulation processes (note: roscore is assumed to run externally).
        kill_processes(processes)
        rospy.loginfo("Simulation processes terminated.")

        # Move time_consuming CSV from the root data dir to the per-env data dir.
        # The planner writes to /home/kota/data/time_consuming_num_{sim_num}.csv
        tc_src = f"/home/kota/data/time_consuming_num_{sim_num}.csv"
        tc_dst = os.path.join(data_dir, f"time_consuming_num_{sim_num}.csv")
        if os.path.exists(tc_src) and os.path.abspath(tc_src) != os.path.abspath(tc_dst):
            import shutil
            shutil.move(tc_src, tc_dst)
            rospy.loginfo(f"Moved {tc_src} -> {tc_dst}")

        sim_num += 1
        rospy.loginfo("Restarting simulation in 5 seconds...")
        time.sleep(5)


def parse_args():
    parser = argparse.ArgumentParser(
        description="SUPER benchmark runner",
        usage="%(prog)s <total_sim_runs> <max_duration> <goal_x,y,z> <threshold> [options]"
    )
    parser.add_argument("total_sim_runs", type=int, help="Number of trials per environment")
    parser.add_argument("max_duration", type=float, help="Max seconds per trial")
    parser.add_argument("goal", type=str, help="Goal coordinates as x,y,z")
    parser.add_argument("threshold", type=float, help="Goal reached distance threshold")
    parser.add_argument("--envs", type=str, default=None,
                        help="Comma-separated environment names (e.g. easy_forest,medium_forest,hard_forest)")
    parser.add_argument("--launch_file", type=str, default="click_demo.launch",
                        help="Launch file to use (default: click_demo.launch)")
    parser.add_argument("--world_dir", type=str, default=None,
                        help="Directory containing .world files (for reference)")
    parser.add_argument("--planners", type=str, default=None,
                        help="Comma-separated planner_name:config_file pairs "
                             "(e.g. super_l2:static_forest_l2.yaml,super_linf:static_forest.yaml)")
    return parser.parse_args()


if __name__ == "__main__":
    args = parse_args()

    total_sim_runs = args.total_sim_runs
    max_duration = args.max_duration
    goal_coords = tuple(map(float, args.goal.split(',')))
    threshold = args.threshold

    # Use the setup files for super (Noetic-based)
    env_source = "source /home/kota/super_ws/devel/setup.bash && source /home/kota/mid360_ws/devel/setup.bash"

    # Ensure ROS master is available.
    if not wait_for_ros_master():
        sys.exit(1)

    rospy.init_node("simulation_monitor_super", anonymous=True)

    # Parse planner variants if specified
    planners = None
    if args.planners:
        planners = []
        for entry in args.planners.split(","):
            entry = entry.strip()
            if ":" in entry:
                name, config = entry.split(":", 1)
                planners.append((name.strip(), config.strip()))
            else:
                rospy.logerr(f"Invalid planner entry '{entry}', expected name:config")
                sys.exit(1)
        rospy.loginfo(f"Planner variants: {planners}")

    if args.envs:
        # Multi-environment mode: loop over each environment
        env_names = [e.strip() for e in args.envs.split(",")]

        if planners:
            # Multi-planner + multi-environment mode
            rospy.loginfo(f"Running {total_sim_runs} trials for {len(planners)} planners x {len(env_names)} environments")

            for planner_name, planner_config in planners:
                if rospy.is_shutdown():
                    break

                rospy.loginfo(f"\n{'#'*60}")
                rospy.loginfo(f"  PLANNER: {planner_name} (config: {planner_config})")
                rospy.loginfo(f"{'#'*60}")

                for env_name in env_names:
                    if rospy.is_shutdown():
                        break

                    rospy.loginfo(f"\n{'='*60}")
                    rospy.loginfo(f"  PLANNER: {planner_name} | ENVIRONMENT: {env_name}")
                    rospy.loginfo(f"{'='*60}")

                    drone_config = write_drone_config(env_name)

                    # Per-planner/per-environment data subdirectory
                    planner_env_data_dir = f"/home/kota/data/{planner_name}/{env_name}"
                    csv_path = os.path.join(planner_env_data_dir, "goal_reached_status_super.csv")

                    run_trials(
                        total_sim_runs, max_duration, goal_coords, threshold,
                        env_source, args.launch_file, drone_config,
                        planner_env_data_dir, csv_path,
                        planner_config=planner_config
                    )

                    rospy.loginfo(f"Completed all trials for {planner_name}/{env_name}")

        else:
            # Single-planner + multi-environment mode (original behavior)
            rospy.loginfo(f"Running {total_sim_runs} trials for each of {len(env_names)} environments: {env_names}")

            for env_name in env_names:
                if rospy.is_shutdown():
                    break

                rospy.loginfo(f"\n{'='*60}")
                rospy.loginfo(f"  ENVIRONMENT: {env_name}")
                rospy.loginfo(f"{'='*60}")

                # Write a per-environment drone config with the correct PCD name
                drone_config = write_drone_config(env_name)

                # Per-environment data subdirectory
                env_data_dir = f"/home/kota/data/{env_name}"
                csv_path = os.path.join(env_data_dir, "goal_reached_status_super.csv")

                run_trials(
                    total_sim_runs, max_duration, goal_coords, threshold,
                    env_source, args.launch_file, drone_config, env_data_dir, csv_path
                )

                rospy.loginfo(f"Completed all trials for {env_name}")

    else:
        # Legacy single-environment mode (backwards compatible)
        csv_path = CSV_PATH
        with open(csv_path, "w") as f:
            f.write("sim_num,status\n")

        sim_num = 0
        while not rospy.is_shutdown() and sim_num < total_sim_runs:
            rospy.loginfo("\n==== Starting simulation number {} ====".format(sim_num))
            monitor = SimulationMonitor(goal=goal_coords, threshold=threshold)
            sim_start_time = time.time()
            processes = launch_simulation(sim_num, env_source, goal_coords,
                                          launch_file=args.launch_file)
            goal_reached = False

            while time.time() - sim_start_time < max_duration and not rospy.is_shutdown():
                if monitor.reached_goal():
                    rospy.loginfo("Goal reached!")
                    goal_reached = True
                    break
                time.sleep(1)

            travel_time = time.time() - sim_start_time
            if goal_reached:
                status = "reached"
                rospy.loginfo("Simulation {} ended successfully in {:.1f} seconds.".format(sim_num, travel_time))
            else:
                status = "timeout"
                rospy.loginfo("Simulation {} timed out after {:.1f} seconds.".format(sim_num, travel_time))

            with open(csv_path, "a") as csv_file:
                csv_file.write(f"{sim_num},{status}\n")

            kill_processes(processes)
            rospy.loginfo("Simulation processes terminated.")

            sim_num += 1
            rospy.loginfo("Restarting simulation in 5 seconds...")
            time.sleep(5)

    rospy.loginfo("All simulation runs complete. Simulation monitor terminated.")
