#!/usr/bin/env python3
import os
import rospy
import numpy as np
import cv2

from duckietown.dtros import DTROS, NodeType
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Float32, String
from duckietown_msgs.msg import WheelsCmdStamped
from cv_bridge import CvBridge

class PIDController:
    def __init__(self, Kp, Ki, Kd):
        """Initialize PID controller parameters and state."""
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.prev_error = 0.0
        self.integral = 0.0
        self.prev_time = rospy.Time.now()
        
    def reset(self):
        """Reset the PID controller state."""
        self.prev_error = 0.0
        self.integral = 0.0
        self.prev_time = rospy.Time.now()
        
    def compute(self, error):
        """Compute the PID control signal."""
        current_time = rospy.Time.now()
        dt = (current_time - self.prev_time).to_sec()
        if dt <= 0.0:
            dt = 1e-3
        self.prev_time = current_time
        
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt
        self.prev_error = error
        
        return self.Kp * error + self.Ki * self.integral + self.Kd * derivative

class CrosswalkDrivingNode(DTROS):
    def __init__(self, node_name):
        super(CrosswalkDrivingNode, self).__init__(
            node_name=node_name, node_type=NodeType.CONTROL
        )
        
        self._vehicle_name = os.environ["VEHICLE_NAME"]
        
        # Topics
        self.error_topic = f"/{self._vehicle_name}/lane/error"
        self.state_topic = f"/{self._vehicle_name}/crosswalk/state"
        self.wheels_topic = f"/{self._vehicle_name}/wheels_driver_node/wheels_cmd"
        
        # Subscribers and Publishers
        self.sub_error = rospy.Subscriber(
            self.error_topic, Float32, self.error_callback, queue_size=1
        )
        self.sub_state = rospy.Subscriber(
            self.state_topic, String, self.state_callback, queue_size=1
        )
        self.pub_wheels = rospy.Publisher(
            self.wheels_topic, WheelsCmdStamped, queue_size=1
        )

        self.pid_controller = PIDController(0.005, 0.0, 0.0)
        self.forward_velocity = 0.3 # m/s
        self.max_correct = 0.5
        self.state = "normal"
        
    def state_callback(self, msg):
        self.state = msg.data
        rospy.loginfo(f"Current state: {self.state}")
        
    def error_callback(self, msg):
        error = msg.data 
        correction = self.pid_controller.compute(error)
        correction = max(min(correction, self.max_correct), -self.max_correct)
        
        if self.state in ["waiting", "stopped"]:
            left_velocity = 0.0
            right_velocity = 0.0
            rospy.loginfo("State '%s': Stopping robot.", self.state)
        else:
            left_velocity = self.forward_velocity - correction
            right_velocity = self.forward_velocity + correction
            rospy.loginfo(f"State '{self.state}': Correcting error by {correction:.3f}.")
        
        msg = WheelsCmdStamped()
        msg.header.stamp = rospy.Time.now()
        msg.vel_left = left_velocity
        msg.vel_right = right_velocity
        self.pub_wheels.publish(msg)
    
    def send_stop_command(self):
        """
        Publish a wheels command that stops the robot.
        """
        stop_msg = WheelsCmdStamped()
        stop_msg.header.stamp = rospy.Time.now()
        stop_msg.vel_left = 0.0
        stop_msg.vel_right = 0.0
        self.pub_wheels.publish(stop_msg)

    def on_shutdown(self):
        rospy.loginfo("Shutting down apriltag driving node. Stopping wheels.")
        self.send_stop_command()
        rospy.sleep(0.2)
        super().on_shutdown()
        
if __name__ == "__main__":
    node = CrosswalkDrivingNode(node_name="crosswalk_driving_node")
    rospy.on_shutdown(node.on_shutdown)
    rospy.spin()