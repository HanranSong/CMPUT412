#!/usr/bin/env python3

import os
import rospy
from duckietown.dtros import DTROS, NodeType
from duckietown_msgs.msg import WheelsCmdStamped, BoolStamped
from std_msgs.msg import Float32
import math
import time


# Simple PID Controller
class PIDController:
    def __init__(self, Kp, Ki, Kd):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.prev_error = 0.0
        self.integral = 0.0
        self.prev_time = rospy.Time.now()

    def compute(self, error):
        current_time = rospy.Time.now()
        dt = (current_time - self.prev_time).to_sec()
        if dt <= 0.0:
            dt = 1e-3
        self.prev_time = current_time

        self.integral += error * dt
        derivative = (error - self.prev_error) / dt
        self.prev_error = error

        return self.Kp * error + self.Ki * self.integral + self.Kd * derivative

    def reset(self):
        self.prev_error = 0.0
        self.integral = 0.0
        self.prev_time = rospy.Time.now()


class LaneDrivingNode(DTROS):
    def __init__(self, node_name):
        super(LaneDrivingNode, self).__init__(
            node_name=node_name, node_type=NodeType.CONTROL
        )
        self._vehicle_name = os.environ["VEHICLE_NAME"]

        # Subscribe to the error published by lane_detection_node
        self.error_topic = f"/{self._vehicle_name}/lane/error"
        self.sub_error = rospy.Subscriber(
            self.error_topic, Float32, self.error_callback, queue_size=1
        )

        # Publisher for wheel commands
        self.wheels_topic = f"/{self._vehicle_name}/wheels_driver_node/wheels_cmd"
        self.pub_wheels = rospy.Publisher(
            self.wheels_topic, WheelsCmdStamped, queue_size=1
        )

        # Subscribe to the stop flag published by vehicle_detection_node
        self.stop_flag_topic = f"/{self._vehicle_name}/stop_flag"
        self.sub_stop_flag = rospy.Subscriber(
            self.stop_flag_topic, BoolStamped, self.stop_flag_callback, queue_size=1
        )

        # PID for steering
        self.pid = PIDController(Kp=0.005, Ki=0.0001, Kd=0.0001)
        self.forward_speed = 0.3  # base forward speed
        self.max_correction = 0.5  # limit how strong the steering correction can be

        # New parameter for maneuver mode bias (force error to a constant value)
        # For example, a positive value here forces a left turn.
        self.maneuver_bias = 100.0  # adjust as needed

        # State machine for lane driving:
        # "NORMAL" - normal driving with original color mapping
        # "STOPPED" - stop for 3 seconds
        # "MANEUVER" - force a bias for 10 seconds to steer into the desired orientation
        self.mode = "NORMAL"
        self.mode_start_time = rospy.Time.now()

        self.last_error = 0.0

        rospy.loginfo(f"[{node_name}] Lane driving node initialized.")

    def stop_flag_callback(self, msg):
        # When receiving a stop flag and we're in NORMAL mode, transition to STOPPED mode.
        if msg.data and self.mode == "NORMAL":
            rospy.loginfo("Received stop flag. Transitioning to STOPPED mode.")
            self.mode = "STOPPED"
            self.mode_start_time = rospy.Time.now()

    def error_callback(self, msg):
        # The error is how far from center the lane is (in pixels).
        error = msg.data
        self.last_error = error
        current_time = rospy.Time.now()

        # State machine handling:
        if self.mode == "STOPPED":
            # In STOPPED mode, hold for 3 seconds.
            if (current_time - self.mode_start_time).to_sec() < 3.0:
                wheels_cmd = WheelsCmdStamped()
                wheels_cmd.header.stamp = rospy.Time.now()
                wheels_cmd.vel_left = 0.0
                wheels_cmd.vel_right = 0.0
                self.pub_wheels.publish(wheels_cmd)
                return
            else:
                rospy.loginfo("Transitioning to MANEUVER mode.")
                self.mode = "MANEUVER"
                self.mode_start_time = rospy.Time.now()
                self.pid.reset()
        elif self.mode == "MANEUVER":
            # In MANEUVER mode, if 10 seconds have elapsed, go back to NORMAL.
            if (current_time - self.mode_start_time).to_sec() >= 2.0:
                rospy.loginfo("Transitioning back to NORMAL mode.")
                self.mode = "NORMAL"
                self.pid.reset()

        # Compute correction based on current mode.
        if self.mode == "NORMAL":
            # Normal mapping: use the error from lane detection.
            correction = self.pid.compute(error)
        elif self.mode == "MANEUVER":
            # In maneuver mode, ignore the detected error and force a desired orientation using maneuver_bias.
            # A positive bias forces a left turn (adjust sign as needed for your vehicle and track).
            correction = self.pid.compute(self.maneuver_bias)
        else:
            correction = 0.0

        # Limit correction to avoid saturating the wheels.
        correction = max(min(correction, self.max_correction), -self.max_correction)

        # Prepare wheels command
        wheels_cmd = WheelsCmdStamped()
        wheels_cmd.header.stamp = rospy.Time.now()

        # Steering: left wheel = base speed - correction, right wheel = base speed + correction
        wheels_cmd.vel_left = self.forward_speed - correction
        wheels_cmd.vel_right = self.forward_speed + correction

        self.pub_wheels.publish(wheels_cmd)

    def on_shutdown(self):
        rospy.loginfo("Shutting down lane driving node. Stopping wheels.")
        stop_msg = WheelsCmdStamped()
        stop_msg.header.stamp = rospy.Time.now()
        stop_msg.vel_left = 0.0
        stop_msg.vel_right = 0.0
        self.pub_wheels.publish(stop_msg)
        rospy.sleep(0.2)
        super().on_shutdown()


if __name__ == "__main__":
    node = LaneDrivingNode("lane_driving_node")
    rospy.on_shutdown(node.on_shutdown)
    rospy.spin()
