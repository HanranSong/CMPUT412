#!/usr/bin/env python3

import os
import rospy
from duckietown.dtros import DTROS, NodeType
from duckietown_msgs.msg import WheelsCmdStamped
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
        # Vehicle name
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

        # PID for steering
        # Tune these gains for stable lane following
        self.pid = PIDController(Kp=0.005, Ki=0.0001, Kd=0.0001)  # 0.005, 0, 0

        # Parameters
        self.forward_speed = 0.3  # base forward speed
        self.max_correction = 0.5  # limit how strong the steering correction can be

        # Last known error
        self.last_error = 0.0

        rospy.loginfo(f"[{node_name}] Lane driving node initialized.")

    def error_callback(self, msg):
        # The error is how far from center the lane is in pixels.
        # A positive error means we need to steer left; negative means steer right.
        error = msg.data
        self.last_error = error

        # Compute correction from PID
        correction = self.pid.compute(error)

        # Limit correction to avoid saturating the wheels
        correction = max(min(correction, self.max_correction), -self.max_correction)

        # Prepare wheels command
        wheels_cmd = WheelsCmdStamped()
        wheels_cmd.header.stamp = rospy.Time.now()

        # Steering:
        # left wheel = base speed - correction
        # right wheel = base speed + correction
        wheels_cmd.vel_left = self.forward_speed - correction
        wheels_cmd.vel_right = self.forward_speed + correction

        self.pub_wheels.publish(wheels_cmd)

    def on_shutdown(self):
        rospy.loginfo("Shutting down lane driving node. Stopping wheels.")
        # Publish a stop command to ensure the wheels stop
        stop_msg = WheelsCmdStamped()
        stop_msg.header.stamp = rospy.Time.now()
        stop_msg.vel_left = 0.0
        stop_msg.vel_right = 0.0
        self.pub_wheels.publish(stop_msg)
        # Sleep a little to ensure the stop message is sent before shutdown
        rospy.sleep(0.2)
        super().on_shutdown()


if __name__ == "__main__":
    node = LaneDrivingNode("lane_driving_node")
    # Register the shutdown callback explicitly
    rospy.on_shutdown(node.on_shutdown)
    rospy.spin()
