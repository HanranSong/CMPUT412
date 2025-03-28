#!/usr/bin/env python3

import os
import rospy
import math

from duckietown.dtros import DTROS, NodeType
from duckietown_msgs.msg import WheelsCmdStamped, LEDPattern
from std_msgs.msg import Float32, Int32, ColorRGBA, Bool


# Simple PID Controller for lane following
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
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt
        self.prev_error = error
        self.prev_time = current_time
        return self.Kp * error + self.Ki * self.integral + self.Kd * derivative

    def reset(self):
        self.prev_error = 0.0
        self.integral = 0.0
        self.prev_time = rospy.Time.now()


class ApriltagDrivingNode(DTROS):
    def __init__(self, node_name):
        super(ApriltagDrivingNode, self).__init__(
            node_name=node_name, node_type=NodeType.CONTROL
        )
        self._vehicle_name = os.environ["VEHICLE_NAME"]

        # Subscribers:
        self.sub_error = rospy.Subscriber(
            f"/{self._vehicle_name}/lane/error",
            Float32,
            self.error_callback,
            queue_size=1,
        )
        self.sub_apriltag = rospy.Subscriber(
            f"/{self._vehicle_name}/apriltag/active_tag",
            Int32,
            self.apriltag_callback,
            queue_size=1,
        )
        self.sub_color = rospy.Subscriber(
            f"/{self._vehicle_name}/color/line_detected",
            Bool,
            self.color_callback,
            queue_size=1,
        )

        # Publishers:
        self.pub_wheels = rospy.Publisher(
            f"/{self._vehicle_name}/wheels_driver_node/wheels_cmd",
            WheelsCmdStamped,
            queue_size=1,
        )
        # self.pub_led = rospy.Publisher(
        #     f"/{self._vehicle_name}/led_cmd", LEDPattern, queue_size=1
        # )
        self.pub_led = rospy.Publisher(
            f"/{self._vehicle_name}/led_emitter_node/led_pattern",
            LEDPattern,
            queue_size=1,
        )

        # PID controller for lane following.
        self.pid = PIDController(Kp=0.005, Ki=0.0001, Kd=0.0001)
        self.forward_speed = 0.3
        self.max_correction = 0.5

        # State variables:
        # "driving" : Normal lane following.
        # "stopping": Currently holding a stop.
        self.state = "driving"
        # recorded_tag_id is used for determining stop duration.
        self.recorded_tag_id = None
        # current_apriltag_id is used to update the LED display continuously.
        self.current_apriltag_id = None
        self.stop_start_time = None
        self.stop_duration = 0.0

        # Cooldown: after a stop, ignore new stop triggers for a short period.
        self.cooldown_end_time = rospy.Time(0)

        # Mapping from apriltag id to LED color and stop duration.
        self.tag_to_led = {
            21: "red",
            133: "blue",
            94: "green",
            22: "red",
            50: "blue",
            93: "green",
            163: "red",
            15: "blue",
        }
        self.tag_to_stop_duration = {
            21: 3.0,
            133: 2.0,
            94: 1.0,
            22: 3.0,
            50: 2.0,
            93: 1.0,
            163: 3.0,
            15: 2.0,
        }

        # For avoiding redundant LED commands.
        self.last_led_tag_id = None

        rospy.loginfo("Apriltag driving node initialized.")

    def error_callback(self, msg):
        """
        Lane error callback.
        In "driving" state, compute PID correction and drive normally.
        In "stopping" state, hold the stop until the stop duration has passed,
        then resume driving and clear the recorded apriltag.
        """
        error = msg.data
        current_time = rospy.Time.now()

        if self.state == "driving":
            correction = self.pid.compute(error)
            correction = max(min(correction, self.max_correction), -self.max_correction)
            wheels_cmd = WheelsCmdStamped()
            wheels_cmd.header.stamp = current_time
            wheels_cmd.vel_left = self.forward_speed - correction
            wheels_cmd.vel_right = self.forward_speed + correction
            self.pub_wheels.publish(wheels_cmd)

        elif self.state == "stopping":
            if (current_time - self.stop_start_time).to_sec() < self.stop_duration:
                self.send_stop_command()
            else:
                self.state = "driving"
                self.cooldown_end_time = current_time + rospy.Duration(1.0)
                rospy.loginfo(
                    "Stop complete. Resuming lane following. Cooldown until {:.2f}".format(
                        self.cooldown_end_time.to_sec()
                    )
                )
                # Clear the recorded tag after using it for a stop.
                self.recorded_tag_id = None
                self.update_led(-1)
                self.pid.reset()
                correction = self.pid.compute(error)
                correction = max(
                    min(correction, self.max_correction), -self.max_correction
                )
                wheels_cmd = WheelsCmdStamped()
                wheels_cmd.header.stamp = current_time
                wheels_cmd.vel_left = self.forward_speed - correction
                wheels_cmd.vel_right = self.forward_speed + correction
                self.pub_wheels.publish(wheels_cmd)

    def color_callback(self, msg):
        """
        Callback for colored line detection.
        If a colored line is detected and we're in "driving" state (and not in cooldown),
        trigger an immediate stop.
        Use the recorded_tag_id if available and valid; if not, use current_apriltag_id; otherwise, default to 0.5 sec.
        """
        if not msg.data:
            return  # No line detected.
        current_time = rospy.Time.now()
        if self.state != "driving":
            return  # Already processing a stop.
        if current_time < self.cooldown_end_time:
            return  # In cooldown period; ignore trigger.

        # Determine stop duration using recorded_tag_id if valid; otherwise, try current_apriltag_id.
        if (
            self.recorded_tag_id is not None
            and self.recorded_tag_id in self.tag_to_stop_duration
        ):
            stop_duration = self.tag_to_stop_duration[self.recorded_tag_id]
        elif (
            self.current_apriltag_id is not None
            and self.current_apriltag_id in self.tag_to_stop_duration
        ):
            stop_duration = self.tag_to_stop_duration[self.current_apriltag_id]
        else:
            rospy.loginfo(
                "Colored line detected but no valid apriltag; using default stop duration 0.5 sec."
            )
            stop_duration = 0.5

        # Trigger the stop and clear the recorded tag.
        self.state = "stopping"
        self.stop_start_time = current_time
        self.stop_duration = stop_duration
        rospy.loginfo(
            "Colored line detected. Stopping for {} seconds.".format(self.stop_duration)
        )
        self.send_stop_command()
        self.recorded_tag_id = None

    def apriltag_callback(self, msg):
        tag_id = msg.data
        current_time = rospy.Time.now()

        if current_time < self.cooldown_end_time:
            return  # Ignore updates during cooldown.

        # If we've already recorded a tag, freeze the LED color.
        if self.recorded_tag_id is not None:
            return

        if tag_id != -1:
            self.current_apriltag_id = tag_id
            self.update_led(tag_id)
            # Record the tag so we freeze its LED color.
            if self.recorded_tag_id is None:
                self.recorded_tag_id = tag_id
        else:
            # Only update LED to white if no tag is recorded.
            if self.state == "driving":
                self.current_apriltag_id = None
                self.update_led(-1)

    def update_led(self, tag_id):
        """
        Publish an LEDPattern message to change the LED color.
        Only publishes if the tag id has changed.
        """
        if self.last_led_tag_id == tag_id:
            return  # No change.
        self.last_led_tag_id = tag_id

        led_msg = LEDPattern()
        led_msg.header.stamp = rospy.Time.now()
        num_leds = 5
        led_msg.frequency = 0.0
        led_msg.frequency_mask = [0] * num_leds
        led_msg.color_mask = [1] * num_leds

        if tag_id == -1 or tag_id is None:
            color_str = "0"
            rgb = [1.0, 1.0, 1.0]
        else:
            color_str = self.tag_to_led.get(tag_id, "white")
            if color_str == "red":
                rgb = [1.0, 0.0, 0.0]
            elif color_str == "blue":
                rgb = [0.0, 0.0, 1.0]
            elif color_str == "green":
                rgb = [0.0, 1.0, 0.0]
            else:
                rgb = [1.0, 1.0, 1.0]
        led_msg.color_list = [color_str] * num_leds
        led_msg.rgb_vals = [
            ColorRGBA(rgb[0], rgb[1], rgb[2], 1.0) for _ in range(num_leds)
        ]
        self.pub_led.publish(led_msg)
        rospy.loginfo("Published LED pattern: " + color_str)

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
    node = ApriltagDrivingNode("apriltag_driving_node")
    rospy.on_shutdown(node.on_shutdown)
    rospy.spin()
