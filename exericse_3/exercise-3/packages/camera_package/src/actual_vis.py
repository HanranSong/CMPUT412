#!/usr/bin/env python3

import rosbag
import matplotlib.pyplot as plt
import math


def main():
    """
    Reconstruct the path based on ticks
    Almost the opposite from how we calculate and send command to motors
    """
    bag_file = "./packages/motion_package/src/D_good.bag"
    # bag_file = "./packages/motion_package/src/straight_good.bag"
    # bag_file = "./packages/motion_package/src/rotation_good.bag"

    WHEEL_RADIUS = 0.0318  # meters
    WHEEL_BASE = 0.1  # meters

    # Encoder topics
    # May need to update vehicle name
    left_topic = "/csc22919/left_wheel_encoder_node/tick"
    right_topic = "/csc22919/right_wheel_encoder_node/tick"

    # Read encoder messages from the bag file
    left_msgs = []
    right_msgs = []

    bag = rosbag.Bag(bag_file, "r")
    for topic, msg, t in bag.read_messages(topics=[left_topic, right_topic]):
        if topic == left_topic:
            left_msgs.append((t.to_sec(), msg.data, msg.resolution))
        elif topic == right_topic:
            right_msgs.append((t.to_sec(), msg.data, msg.resolution))
    bag.close()

    # Avoid looseing data at very end
    n = min(len(left_msgs), len(right_msgs))

    # Initialize
    # starting at origin facing forward
    x, y, theta = 0.0, 0.0, 0.0
    xs = [x]
    ys = [y]

    for i in range(1, n):
        # dt by average left and right dt
        left_t_prev, _, _ = left_msgs[i - 1]
        left_t_curr, _, _ = left_msgs[i]
        left_dt = left_t_curr - left_t_prev

        right_t_prev, _, _ = right_msgs[i - 1]
        right_t_curr, _, _ = right_msgs[i]
        right_dt = right_t_curr - right_t_prev

        dt = (left_dt + right_dt) / 2.0

        # Delta left wheel
        _, tick_left_prev, res_left = left_msgs[i - 1]
        _, tick_left, _ = left_msgs[i]
        delta_tick_left = tick_left - tick_left_prev
        d_left = 2 * math.pi * WHEEL_RADIUS * (delta_tick_left / res_left)
        v_left = d_left / dt

        # Delta right wheel
        _, tick_right_prev, res_right = right_msgs[i - 1]
        _, tick_right, _ = right_msgs[i]
        delta_tick_right = tick_right - tick_right_prev
        d_right = 2 * math.pi * WHEEL_RADIUS * (delta_tick_right / res_right)
        v_right = d_right / dt

        # Velocity and omega
        v = (v_left + v_right) / 2.0
        omega = (v_right - v_left) / WHEEL_BASE

        # Update x, y, theta
        x += v * math.cos(theta + (omega * dt / 2.0)) * dt
        y += v * math.sin(theta + (omega * dt / 2.0)) * dt
        theta += omega * dt

        xs.append(x)
        ys.append(y)

    print(f"Final computed pose: x = {x:.3f} m, y = {y:.3f} m, theta = {theta:.3f} rad")

    # Plotting
    plt.figure()
    plt.plot(xs, ys, marker="o", linestyle="-")
    plt.xlabel("x (m)")
    plt.ylabel("y (m)")
    # plt.title("Straight")
    # plt.title("Rotation")
    plt.title("Actual robot path")
    plt.axis("equal")
    plt.grid(True)
    plt.show()


if __name__ == "__main__":
    main()