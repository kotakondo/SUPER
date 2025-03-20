#!/usr/bin/env python3
import subprocess
import sys
import time

# run ./benchmarking.py <simulation_number> to start the simulation like ./benchmarking.py 1

def run_command(cmd):
    """Launch a command via bash -c and return the Popen handle."""
    return subprocess.Popen(["bash", "-c", cmd])

def main():
    if len(sys.argv) < 2:
        print("Usage: {} <simulation_number>".format(sys.argv[0]))
        sys.exit(1)

    sim_num = sys.argv[1]
    # Build a command string to source the three setup files.
    env_source = (
        "source /home/kota/super_ws/devel/setup.bash && "
        # "source /home/kota/livox_ros_ws/devel/setup.bash && "
        "source /home/kota/mid360_ws/devel/setup.bash"
    )

    # Define each command.
    roscore_cmd = f"{env_source} && roscore"
    start_world_cmd = f"{env_source} && roslaunch --wait acl_sim start_world.launch"
    perfect_tracker_cmd = f"{env_source} && roslaunch --wait acl_sim perfect_tracker_and_sim.launch x:=0.0 y:=0.0 z:=3.0 yaw:=0.0"
    mission_planner_cmd = f"{env_source} && roslaunch --wait mission_planner click_demo.launch"
    goal_pub_cmd = (
        f"{env_source} && sleep 20 && "
        "rostopic pub /goal geometry_msgs/PoseStamped "
        "'{header: {frame_id: \"world\"}, pose: {position: {x: 105.0, y: 0.0, z: 3.0}, "
        "orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}' -1"
    )
    bag_file = f"/home/kota/data/super_num_{sim_num}.bag"
    rosbag_cmd = f"{env_source} && rosbag record -a -O {bag_file}"

    processes = []

    print("Launching roscore...")
    processes.append(run_command(roscore_cmd))
    # Wait a few seconds for roscore to come up.
    time.sleep(5)

    print("Launching start_world.launch...")
    processes.append(run_command(start_world_cmd))
    time.sleep(2)

    print("Launching perfect_tracker_and_sim.launch...")
    processes.append(run_command(perfect_tracker_cmd))
    time.sleep(2)

    print("Launching click_demo.launch...")
    processes.append(run_command(mission_planner_cmd))
    time.sleep(2)

    print("Launching goal publisher (after 20s delay)...")
    processes.append(run_command(goal_pub_cmd))
    time.sleep(2)

    print(f"Launching rosbag recording to {bag_file} ...")
    processes.append(run_command(rosbag_cmd))

    print("All processes launched. Press Ctrl+C to terminate.")

    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("Terminating all processes...")
        for p in processes:
            p.terminate()
        for p in processes:
            p.wait()
        print("Done.")

if __name__ == "__main__":
    main()
