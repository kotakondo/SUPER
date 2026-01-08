#!/usr/bin/env python3
import subprocess
import sys
import time
import math
import os
import rospy
import rosgraph
from geometry_msgs.msg import PoseStamped
from quadrotor_msgs.msg import PositionCommand

# CSV file to log simulation outcomes.
CSV_PATH = "/home/kota/data/goal_reached_status_super.csv"

# Topics to record (explicit allowlist to keep bags small)
TOPICS_TO_RECORD = [
    "/clock",
    "/parameter_events",
    "/rosout",
    "/tf",
    "/tf_static",
    "/livox/lidar",
    "/fsm_node/traj_opt/back/mkr_arr",
    "/fsm_node/traj_opt/exp/mkr_arr",
    "/fsm_node/visualization/yaw_traj",
    "/astar_debug/mkr_arr",
    "/fsm_node/debug",
    "/fsm_node/visualization/astar_debug",
    "/fsm_node/rog_map/map_bound",
    "/fsm_node/visualization/replan_log_mkr",
    "/fsm_node/visualization/replan_log_pc",
    "/perfect_drone/vel_text",
    "/waypoint_mission/mkr",
    "/waypoint_mission",
    "/goal",
    "/planning/pos_cmd",
]

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

def launch_simulation(sim_num, env_source):
    """
    Launch simulation processes (excluding roscore) for super.
    
    Commands are defined according to your super setup.
    """
    start_world_cmd = f"{env_source} && roslaunch --wait acl_sim start_world.launch"
    perfect_tracker_cmd = f"{env_source} && roslaunch --wait acl_sim perfect_tracker_and_sim.launch x:=0.0 y:=0.0 z:=3.0 yaw:=0.0"
    mission_planner_cmd = f"{env_source} && roslaunch --wait mission_planner click_demo.launch simulation_number:={sim_num}"
    goal_pub_cmd = (
        f"{env_source} && sleep 10 && "
        "rostopic pub /goal geometry_msgs/PoseStamped "
        "'{header: {frame_id: \"world\"}, pose: {position: {x: 105.0, y: 0.0, z: 3.0}, "
        "orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}' -1"
    )
    bag_file = f"/home/kota/data/super_num_{sim_num}.bag"
    topics_str = " ".join(TOPICS_TO_RECORD)
    rosbag_cmd = f"{env_source} && rosbag record --lz4 -O {bag_file} {topics_str}"

    processes = []

    rospy.loginfo("Launching start_world.launch...")
    processes.append(run_command(start_world_cmd))
    time.sleep(2)

    rospy.loginfo("Launching perfect_tracker_and_sim.launch...")
    processes.append(run_command(perfect_tracker_cmd))
    time.sleep(2)

    rospy.loginfo("Launching click_demo.launch...")
    processes.append(run_command(mission_planner_cmd))
    time.sleep(2)

    rospy.loginfo("Launching goal publisher (after 20s delay)...")
    processes.append(run_command(goal_pub_cmd))
    time.sleep(2)

    rospy.loginfo(f"Launching rosbag recording to {bag_file}...")
    processes.append(run_command(rosbag_cmd))
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

if __name__ == "__main__":
    if len(sys.argv) < 5:
        print("Usage: {} <total_sim_runs> <max_duration_seconds> <goal_x,goal_y,goal_z> <goal_threshold>".format(sys.argv[0]))
        sys.exit(1)

    total_sim_runs = int(sys.argv[1])
    max_duration = float(sys.argv[2])
    goal_coords = tuple(map(float, sys.argv[3].split(',')))
    threshold = float(sys.argv[4])

    # Use the setup files for super (Noetic-based)
    env_source = "source /home/kota/super_ws/devel/setup.bash && source /home/kota/mid360_ws/devel/setup.bash"

    # Ensure ROS master is available.
    if not wait_for_ros_master():
        sys.exit(1)

    # Prepare the CSV log file.
    if not os.path.exists(CSV_PATH):
        with open(CSV_PATH, "w") as f:
            f.write("sim_num,status\n")

    rospy.init_node("simulation_monitor_super", anonymous=True)

    sim_num = 0
    while not rospy.is_shutdown() and sim_num < total_sim_runs:
        rospy.loginfo("\n==== Starting simulation number {} ====".format(sim_num))
        monitor = SimulationMonitor(goal=goal_coords, threshold=threshold)
        sim_start_time = time.time()
        processes = launch_simulation(sim_num, env_source)
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
        with open(CSV_PATH, "a") as csv_file:
            csv_file.write(f"{sim_num},{status}\n")

        # Terminate all simulation processes (note: roscore is assumed to run externally).
        kill_processes(processes)
        rospy.loginfo("Simulation processes terminated.")

        sim_num += 1
        rospy.loginfo("Restarting simulation in 5 seconds...")
        time.sleep(5)

    rospy.loginfo("All simulation runs complete. Simulation monitor terminated.")
