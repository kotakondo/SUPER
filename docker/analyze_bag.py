#!/usr/bin/env python3
import rosbag
import sys
import rospy
import numpy as np
import matplotlib.pyplot as plt

# Import custom PositionCommand message.
from quadrotor_msgs.msg import PositionCommand
from geometry_msgs.msg import PoseStamped

def compute_distance(p1, p2):
    return np.linalg.norm(np.array(p1) - np.array(p2))

def main():
    if len(sys.argv) < 2:
        print("Usage: {} <bag_file> [tolerance (m)]".format(sys.argv[0]))
        sys.exit(1)

    bag_file = sys.argv[1]
    tol = 0.5  # tolerance in meters for reaching the goal
    if len(sys.argv) > 2:
        tol = float(sys.argv[2])

    bag = rosbag.Bag(bag_file)

    goal_time = None
    goal_position = None
    travel_end_time = None

    pos_cmd_times = []
    velocities = []
    accelerations = []
    jerks = []

    # Dynamic constraints
    v_constraint = 10.0    # m/s
    a_constraint = 20.0    # m/s^2
    j_constraint = 30.0    # m/s^3

    # For reporting violations, we simply count the number of samples above the limits.
    vel_violations = 0
    acc_violations = 0
    jerk_violations = 0

    print("Processing bag: {}".format(bag_file))

    # Iterate over messages in time order.
    # We assume that the /goal message is published once before any /planning/pos_cmd messages.
    for topic, msg, t in bag.read_messages(topics=["/goal", "/planning/pos_cmd"]):

        if topic == "/goal" and goal_time is None:
            # Use msg.header.stamp if available; otherwise, use the bag timestamp.
            goal_time = t.to_sec()
            # Get goal position from the PoseStamped message.
            goal_position = (msg.pose.position.x, msg.pose.position.y, msg.pose.position.z)
            print("Found /goal at time {:.3f}, goal_position = {}".format(goal_time, goal_position))
        elif topic == "/planning/pos_cmd" and goal_time is not None:
            # Process only messages after the goal is published.
            pos_time = t.to_sec()
            if pos_time < goal_time:
                continue

            pos_cmd_times.append(pos_time)
            # Use the provided norm fields if available; otherwise compute from the vectors.
            vel = msg.vel_norm if hasattr(msg, "vel_norm") else np.linalg.norm(
                [msg.velocity.x, msg.velocity.y, msg.velocity.z])
            acc = msg.acc_norm if hasattr(msg, "acc_norm") else np.linalg.norm(
                [msg.acceleration.x, msg.acceleration.y, msg.acceleration.z])
            jrk = np.linalg.norm([msg.jerk.x, msg.jerk.y, msg.jerk.z])

            velocities.append(vel)
            accelerations.append(acc)
            jerks.append(jrk)

            # Count any constraint violations.
            if vel > v_constraint:
                vel_violations += 1
            if acc > a_constraint:
                acc_violations += 1
            if jrk > j_constraint:
                jerk_violations += 1

            # Check if the commanded position has reached the goal.
            pos = (msg.position.x, msg.position.y, msg.position.z)
            print("distance to goal: {:.3f}".format(compute_distance(pos, goal_position)))
            if compute_distance(pos, goal_position) <= tol:
                travel_end_time = pos_time
                print("Goal reached at time {:.3f}".format(travel_end_time))
                # We break once the goal is reached.
                break

    bag.close()

    if goal_time is None or travel_end_time is None:
        print("Could not compute travel time. Either /goal or goal-reached event was not found.")
        sys.exit(1)
    travel_time = travel_end_time - goal_time
    print("Total travel time: {:.3f} seconds".format(travel_time))
    print("Dynamic constraint violations:")
    print("  Velocity (> {} m/s): {} samples".format(v_constraint, vel_violations))
    print("  Acceleration (> {} m/s^2): {} samples".format(a_constraint, acc_violations))
    print("  Jerk (> {} m/s^3): {} samples".format(j_constraint, jerk_violations))

    # Convert lists to numpy arrays for plotting.
    pos_cmd_times = np.array(pos_cmd_times)
    velocities = np.array(velocities)
    accelerations = np.array(accelerations)
    jerks = np.array(jerks)

    # Plot time series for velocity, acceleration, and jerk.
    plt.figure(figsize=(12, 10))
    plt.subplot(3, 1, 1)
    plt.plot(pos_cmd_times, velocities, label="Velocity (m/s)")
    plt.axhline(y=v_constraint, color="red", linestyle="--", label="v constraint = {} m/s".format(v_constraint))
    plt.xlabel("Time (s)")
    plt.ylabel("Velocity (m/s)")
    plt.legend()
    plt.grid(True)

    plt.subplot(3, 1, 2)
    plt.plot(pos_cmd_times, accelerations, label="Acceleration (m/s²)")
    plt.axhline(y=a_constraint, color="red", linestyle="--", label="a constraint = {} m/s²".format(a_constraint))
    plt.xlabel("Time (s)")
    plt.ylabel("Acceleration (m/s²)")
    plt.legend()
    plt.grid(True)

    plt.subplot(3, 1, 3)
    plt.plot(pos_cmd_times, jerks, label="Jerk (m/s³)")
    plt.axhline(y=j_constraint, color="red", linestyle="--", label="j constraint = {} m/s³".format(j_constraint))
    plt.xlabel("Time (s)")
    plt.ylabel("Jerk (m/s³)")
    plt.legend()
    plt.grid(True)

    plt.tight_layout()
    plt.show()

    # Generate a histogram of velocity.
    plt.figure()
    plt.hist(velocities, bins=20, edgecolor="black")
    plt.xlabel("Velocity (m/s)")
    plt.ylabel("Frequency")
    plt.title("Velocity Profile Histogram")
    plt.axvline(x=v_constraint, color="red", linestyle="--", label="v constraint = {} m/s".format(v_constraint))
    plt.legend()
    plt.grid(True)
    plt.show()

if __name__ == "__main__":
    main()
