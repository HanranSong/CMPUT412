#!/usr/bin/env python3
"""
vis.py

Offline visualization of a robot path integrated from the wheels command topic
in a ROS bag file. This version is aware that when reversing a compensation is
added to the right wheel. It also allows scaling of the commanded speeds via a
parameter so that the integrated distance matches the actual distance traveled.

Usage:
    ./vis.py <bag_file.bag> [vehicle_name] [reverse_compensation] [tolerance] [speed_scale]

    - bag_file.bag:         Path to the ROS bag file.
    - vehicle_name:         (Optional) Vehicle name (default "duckiebot").
    - reverse_compensation: (Optional) The compensation added during reverse (default 0.02).
    - tolerance:            (Optional) Tolerance for detecting compensation (default 0.005).
    - speed_scale:          (Optional) A scaling factor for the wheel commands (default 1.0).

The script assumes that the wheels command topic is:
    /<vehicle_name>/wheels_driver_node/wheels_cmd
and uses differential-drive kinematics:
    v     = (v_left + v_right) / 2
    omega = (v_right - v_left) / wheel_base

When in reverse (i.e. both wheel speeds negative), if the measured bias
(v_right - v_left) is within tolerance of the known reverse_compensation value,
the bias is removed (by replacing v_right with v_left) before further processing.
Finally, the wheel commands are multiplied by the speed_scale factor.
"""

import sys
import math
import rosbag
import matplotlib.pyplot as plt


def process_bag(
    bag_file, wheels_topic, wheel_base, reverse_compensation, tolerance, speed_scale
):
    """
    Processes the bag file and integrates the robot pose using wheels_cmd messages.

    For reverse motion, if the measured bias is within tolerance of the known
    reverse_compensation value, the bias is removed. Then, a scaling factor is
    applied to the (possibly corrected) wheel commands.

    Args:
        bag_file (str): Path to the ROS bag file.
        wheels_topic (str): The topic to use for wheel commands.
        wheel_base (float): Distance between wheels in meters.
        reverse_compensation (float): The reverse compensation value to remove.
        tolerance (float): Tolerance for deciding whether to remove the bias.
        speed_scale (float): Scale factor applied to the wheel commands.

    Returns:
        list of tuple: List of (x, y) poses representing the integrated path.
    """
    bag = rosbag.Bag(bag_file, "r")
    x, y, theta = 0.0, 0.0, 0.0
    last_time = None
    path = [(x, y)]
    msg_count = 0

    for topic, msg, t in bag.read_messages(topics=[wheels_topic]):
        # Use the header timestamp in the message.
        current_time = msg.header.stamp.to_sec()
        if last_time is None:
            last_time = current_time
            continue
        dt = current_time - last_time
        last_time = current_time

        # Get the raw commanded wheel speeds.
        raw_v_left = msg.vel_left
        raw_v_right = msg.vel_right

        # Check if the robot is reversing.
        if raw_v_left < 0 and raw_v_right < 0:
            measured_bias = raw_v_right - raw_v_left
            # If the measured bias is close to the expected reverse_compensation, remove it.
            if abs(measured_bias - reverse_compensation) < tolerance:
                raw_v_right = raw_v_left

        # Apply the scaling factor to get the effective wheel speeds.
        v_left = raw_v_left * speed_scale
        v_right = raw_v_right * speed_scale

        # Compute linear and angular velocities using differential-drive kinematics.
        v = (v_left + v_right) / 2.0
        omega = (v_right - v_left) / wheel_base

        # Integrate the pose using simple Euler integration.
        x += v * math.cos(theta) * dt
        y += v * math.sin(theta) * dt
        theta += omega * dt

        path.append((x, y))
        msg_count += 1

    bag.close()
    print("Processed {} messages from topic: {}".format(msg_count, wheels_topic))
    return path


def main():

    bag_file = "/home/tina/Desktop/exercise-3/packages/camera_package/src/pid.bag"
    vehicle_name = "csc22919"

    reverse_compensation = 0.02
    tolerance = 0.005
    speed_scale = 0.44

    wheels_topic = f"/{vehicle_name}/wheels_driver_node/wheels_cmd"
    wheel_base = 0.1  # meters (adjust if needed)

    print("Reading bag: {}".format(bag_file))
    print("Using wheels command topic: {}".format(wheels_topic))
    print(
        "Reverse compensation: {} (tolerance {})".format(
            reverse_compensation, tolerance
        )
    )
    print("Speed scale: {}".format(speed_scale))

    # Process the bag file to obtain the integrated path.
    path = process_bag(
        bag_file, wheels_topic, wheel_base, reverse_compensation, tolerance, speed_scale
    )
    if not path:
        print("No data found on topic:", wheels_topic)
        sys.exit(1)

    # Unzip the path into x and y coordinates.
    xs, ys = zip(*path)

    # Plot the integrated robot path.
    plt.figure()
    plt.plot(xs, ys, "b-", lw=2, label="Integrated Path")
    plt.xlabel("x [m]")
    plt.ylabel("y [m]")
    plt.title("Robot Path from wheels_cmd (Bag: {})".format(bag_file))
    plt.legend()
    plt.grid(True)
    plt.axis("equal")
    plt.show()


if __name__ == "__main__":
    main()