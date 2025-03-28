#!/usr/bin/env python3
import os
import rospy
from duckietown.dtros import DTROS, NodeType
from std_msgs.msg import String, Float32
from duckietown_msgs.msg import WheelsCmdStamped

class NavigationDrivingNode(DTROS):
    def __init__(self, node_name):
        super(NavigationDrivingNode, self).__init__(
            node_name=node_name, node_type=NodeType.CONTROL
        )

        self._vehicle_name = os.environ["VEHICLE_NAME"]

        self.state_topic = f"/{self._vehicle_name}/navigation/state"
        self.error_topic = f"/{self._vehicle_name}/lane/error"
        self.wheels_topic = f"/{self._vehicle_name}/wheels_driver_node/wheels_cmd"

        self.sub_state = rospy.Subscriber(
            self.state_topic, String, self.state_callback, queue_size=1
        )
        self.sub_error = rospy.Subscriber(
            self.error_topic, Float32, self.error_callback, queue_size=1
        )
        self.pub_wheels = rospy.Publisher(
            self.wheels_topic, WheelsCmdStamped, queue_size=1
        )

        self.forward_velocity = 0.3
        self.pid_controller = PIDController(0.005, 0.0, 0.0, max_integral=0.5)
        self.state = "normal"

    def state_callback(self, msg):
        self.state = msg.data

    def error_callback(self, msg):
        error = msg.data
        correction = self.pid_controller.compute(error)
        correction = max(min(correction, 0.4), -0.4)

        if self.state in ["normal", "pid_follow_in_opposite_lane"]:
            left_velocity = self.forward_velocity - correction
            right_velocity = self.forward_velocity + correction
        elif self.state == "turn_45_left":
            left_velocity = -abs(self.forward_velocity)
            right_velocity = abs(self.forward_velocity)
        elif self.state == "turn_45_right":
            left_velocity = abs(0.2)
            right_velocity = -abs(0.2)
        elif self.state in ["go_straight_angled", "go_straight_return"]:
            left_velocity = self.forward_velocity
            right_velocity = self.forward_velocity
        elif self.state == "turn_back_right":
            left_velocity = abs(self.forward_velocity)
            right_velocity = -abs(self.forward_velocity)
        elif self.state == "turn_back_left":
            left_velocity = -abs(self.forward_velocity)
            right_velocity = abs(self.forward_velocity)
        else:
            left_velocity = 0.0
            right_velocity = 0.0

        wheels_cmd = WheelsCmdStamped()
        wheels_cmd.header.stamp = rospy.Time.now()
        wheels_cmd.vel_left = left_velocity
        wheels_cmd.vel_right = right_velocity
        self.pub_wheels.publish(wheels_cmd)

    def send_stop_command(self):
        stop_msg = WheelsCmdStamped()
        stop_msg.header.stamp = rospy.Time.now()
        stop_msg.vel_left = 0.0
        stop_msg.vel_right = 0.0
        self.pub_wheels.publish(stop_msg)

    def on_shutdown(self):
        rospy.loginfo("Shutting down navigation driving node. Stopping wheels.")
        self.send_stop_command()
        rospy.sleep(0.2)
        super().on_shutdown()

class PIDController:
    def __init__(self, Kp, Ki, Kd, max_integral=1.0):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.prev_error = 0.0
        self.integral = 0.0
        self.max_integral = max_integral
        self.prev_time = rospy.Time.now()

    def reset(self):
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
        self.integral = max(min(self.integral, self.max_integral), -self.max_integral)  # Prevent windup
        derivative = (error - self.prev_error) / dt
        self.prev_error = error

        return self.Kp * error + self.Ki * self.integral + self.Kd * derivative

if __name__ == "__main__":
    node = NavigationDrivingNode(node_name="navigation_driving_node")
    rospy.on_shutdown(node.on_shutdown)
    rospy.spin()
