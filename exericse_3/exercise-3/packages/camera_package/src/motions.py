#!/usr/bin/env python3

import math
import rospy
import numpy as np
from duckietown_msgs.msg import WheelsCmdStamped, WheelEncoderStamped, LEDPattern
from std_msgs.msg import ColorRGBA


# https://en.wikipedia.org/wiki/Proportional%E2%80%93integral%E2%80%93derivative_controller
# https://softinery.com/blog/implementation-of-pid-controller-in-python/
class PIDController:
    def __init__(self, Kp, Ki, Kd):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.prev_error = 0.0
        self.integral = 0.0

    def compute(self, error, dt):
        if dt <= 0:  # avoid divide by 0
            dt = 1e-3
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt
        self.prev_error = error
        return (
            (self.Kp * error) + (self.Ki * self.integral) + (self.Kd * derivative)
        )  # PID equation

    def reset(self):
        # Clear current error after one operation is done
        self.prev_error = 0.0
        self.integral = 0.0


class Motions:
    def __init__(self, vehicle_name):
        self.vehicle_name = vehicle_name

        self.interrupt = False
        
        # Topics for wheel commands and encoders.
        self.wheels_topic = f"/{vehicle_name}/wheels_driver_node/wheels_cmd"
        self.encoder_left_topic = f"/{vehicle_name}/left_wheel_encoder_node/tick"
        self.encoder_right_topic = f"/{vehicle_name}/right_wheel_encoder_node/tick"

        # Topic for LED control
        self.led_topic = f"/{vehicle_name}/led_emitter_node/led_pattern"

        # For all publisher, I use queue size of 1 to make sure the processing data are up to date
        # Publisher for wheels
        self.pub = rospy.Publisher(self.wheels_topic, WheelsCmdStamped, queue_size=1)

        # Subscribers for wheels
        self.sub_left = rospy.Subscriber(
            self.encoder_left_topic, WheelEncoderStamped, self.callback_left
        )
        self.sub_right = rospy.Subscriber(
            self.encoder_right_topic, WheelEncoderStamped, self.callback_right
        )

        # Publisher for LED
        self.led_pub = rospy.Publisher(self.led_topic, LEDPattern, queue_size=1)

        # tick: one pulse from encoder
        # resolution: ticks per revolution
        self.ticks_left = None
        self.ticks_right = None
        self.res_left = None
        self.res_right = None

        # Wheel parameters
        self.WHEEL_RADIUS = 0.0318
        self.WHEEL_BASE = 0.1

        # PID controller for straight driving
        # Only use P value
        # The PID controller doen't work well with rotation and curve
        self.pid_straight = PIDController(Kp=0.5, Ki=0, Kd=0.001)

        # Define a simple color dictionary.
        self.colors = {
            "red": [255, 0, 0],
            "green": [0, 255, 0],
            "blue": [0, 0, 255],
            "yellow": [255, 255, 0],
            "magenta": [255, 0, 255],
            "cyan": [0, 255, 255],
            "white": [255, 255, 255],
            "off": [0, 0, 0]
        }

    def callback_left(self, msg):
        self.ticks_left = msg.data
        if self.res_left is None:
            self.res_left = msg.resolution
            rospy.loginfo_once(f"Left encoder resolution: {self.res_left}")

    def callback_right(self, msg):
        self.ticks_right = msg.data
        if self.res_right is None:
            self.res_right = msg.resolution
            rospy.loginfo_once(f"Right encoder resolution: {self.res_right}")

    def wait_for_encoders(self):
        rate = rospy.Rate(30)
        rospy.loginfo("Waiting for encoder...")
        while (
            self.ticks_left is None or self.ticks_right is None
        ) and not rospy.is_shutdown():
            rate.sleep()

    def stop_robot(self):
        stop_msg = WheelsCmdStamped()
        stop_msg.header.stamp = rospy.Time.now()
        # Set velocity to 0
        stop_msg.vel_left = 0.0
        stop_msg.vel_right = 0.0
        self.pub.publish(stop_msg)

    def get_color_RGBA(self, rgb):
        normalized = np.array(rgb) / 255.0
        rgb_list = normalized.tolist() + [1.0]  # add alpha channel
        return ColorRGBA(r=rgb_list[0], g=rgb_list[1], b=rgb_list[2], a=rgb_list[3])

    def set_led_status(self, color, indices = [0, 1, 2, 3, 4]):           
        # retrieve the color and rgb values
        if isinstance(color, str):
            color = color.lower()
            rgb = self.colors.get(color, self.colors["off"])
        elif isinstance(color, (list, tuple)) and len(color) == 3:
            innput_rgb = list(color)
            found_color = None
            for key, value in self.colors.items():
                if list(value) == innput_rgb: # check if the rgb value is defined in the color dictionary
                    found_color = key
                    break
                
            if found_color is not None:
                color = found_color
            else:
                color = "off"
            rgb = self.colors.get(color)
        else:
            color = "off"
            rgb = self.colors["off"]
        
        # define the LED pattern
        led_pattern = LEDPattern()
        led_pattern.header.stamp = rospy.Time.now()
        led_pattern.color_list = [color if i in indices else "off" for i in range(5)]
        led_pattern.color_mask = [1 if i in indices else 0 for i in range(5)]
        led_pattern.rgb_vals = [self.get_color_RGBA(rgb) if i in indices else 
                                self.get_color_RGBA(self.colors.get("off")) for i in range(5)]
        led_pattern.frequency = 0.0
        led_pattern.frequency_mask = [0] * 5
        
        self.led_pub.publish(led_pattern)
        rospy.loginfo(f"LED status set to {color}")        

    def move_straight(self, target_distance, speed):
        """
        Straight driving
        """
        self.wait_for_encoders()
        self.pid_straight.reset()

        init_left = self.ticks_left
        init_right = self.ticks_right
        prev_distance_left = 0.0
        prev_distance_right = 0.0
        x, y, theta = 0.0, 0.0, 0.0

        rate = rospy.Rate(30)
        last_time = rospy.Time.now()
        cumulative_distance = 0.0

        while not rospy.is_shutdown():
            current_time = rospy.Time.now()
            dt = (current_time - last_time).to_sec()
            last_time = current_time

            # Tick differences
            delta_left_ticks = self.ticks_left - init_left
            delta_right_ticks = self.ticks_right - init_right

            # current_dist = circumference * (delta_tick / res)
            current_distance_left = (2 * math.pi * self.WHEEL_RADIUS) * (
                delta_left_ticks / self.res_left
            )
            current_distance_right = (2 * math.pi * self.WHEEL_RADIUS) * (
                delta_right_ticks / self.res_right
            )

            # delt_dist
            d_left = current_distance_left - prev_distance_left
            d_right = current_distance_right - prev_distance_right

            prev_distance_left = current_distance_left
            prev_distance_right = current_distance_right

            # delta_center & delta theta
            d_center = (d_left + d_right) / 2.0
            d_theta = (d_right - d_left) / self.WHEEL_BASE

            # update x, y, theta
            x += d_center * math.cos(theta)
            y += d_center * math.sin(theta)
            theta += d_theta

            cumulative_distance += abs(d_center)

            # PID controller
            error_y = 0.0 - y
            correction = self.pid_straight.compute(error_y, dt)
            if speed < 0:  # correct correction direction for reverse
                correction = -correction

            cmd_msg = WheelsCmdStamped()
            cmd_msg.header.stamp = rospy.Time.now()
            cmd_msg.vel_left = speed - correction
            cmd_msg.vel_right = speed + correction
            self.pub.publish(cmd_msg)

            rospy.loginfo(
                f"Distance: {cumulative_distance:.3f} m, y: {y:.3f}, error: {error_y:.3f}, correction: {correction:.3f}"
            )

            if cumulative_distance >= abs(target_distance):
                rospy.loginfo("Target distance reached.")
                break

            rate.sleep()

        self.stop_robot()
        rospy.sleep(1.0)
        
    def curve(self, radius, velocity, angle_span, direction = "right"):
        """
        Curve driving
        """
        self.wait_for_encoders()

        if direction == "right":
            vel_left = velocity * (radius + self.WHEEL_BASE / 2.0) / radius
            vel_right = velocity * (radius - self.WHEEL_BASE / 2.0) / radius
            target_angle = -abs(angle_span)
        else:
            vel_left = velocity * (radius - self.WHEEL_BASE / 2.0) / radius
            vel_right = velocity * (radius + self.WHEEL_BASE / 2.0) / radius
            target_angle = abs(angle_span)

        init_left = self.ticks_left
        init_right = self.ticks_right

        prev_distance_left = 0.0
        prev_distance_right = 0.0
        cumulative_angle = 0.0
        rate = rospy.Rate(30)

        while not rospy.is_shutdown():
            # tick differences
            delta_left_ticks = self.ticks_left - init_left
            delta_right_ticks = self.ticks_right - init_right

            # current_dist = circumference * (delta_tick / res)
            current_distance_left = (2 * math.pi * self.WHEEL_RADIUS) * (
                delta_left_ticks / self.res_left
            )
            current_distance_right = (2 * math.pi * self.WHEEL_RADIUS) * (
                delta_right_ticks / self.res_right
            )

            # delt_dist
            d_left = current_distance_left - prev_distance_left
            d_right = current_distance_right - prev_distance_right

            prev_distance_left = current_distance_left
            prev_distance_right = current_distance_right

            # get current delta angle
            d_theta = (d_right - d_left) / self.WHEEL_BASE

            # Update angle
            cumulative_angle += d_theta

            rospy.loginfo(
                "Cumulative angle: {:.3f} rad, Target: {:.3f} rad".format(
                    cumulative_angle, target_angle
                )
            )

            cmd_msg = WheelsCmdStamped()
            cmd_msg.header.stamp = rospy.Time.now()
            cmd_msg.vel_left = vel_left
            cmd_msg.vel_right = vel_right
            self.pub.publish(cmd_msg)

            if abs(cumulative_angle) >= abs(target_angle):
                rospy.loginfo("Curve target reached.")
                break

            rate.sleep()

        self.stop_robot()
        rospy.sleep(1.0)

    def rotate(self, target_angle, speed):
        """
        Rotate robot
        """
        self.wait_for_encoders()
        
        init_left = self.ticks_left
        init_right = self.ticks_right
        rate = rospy.Rate(30)
        direction = 1 if target_angle >= 0 else -1
        cmd_vel_left = -direction * speed
        cmd_vel_right = direction * speed

        while not rospy.is_shutdown():
            # tick differences
            delta_left = abs(self.ticks_left - init_left)
            delta_right = abs(self.ticks_right - init_right)

            # dist per tick
            dist_per_tick_left = (2 * math.pi * self.WHEEL_RADIUS) / self.res_left
            dist_per_tick_right = (2 * math.pi * self.WHEEL_RADIUS) / self.res_right

            # perform the rotation
            distance_left = (
                delta_left * dist_per_tick_left * (1 if cmd_vel_left >= 0 else -1)
            )
            distance_right = (
                delta_right * dist_per_tick_right * (1 if cmd_vel_right >= 0 else -1)
            )

            # rotation angle
            theta_rot = (distance_right - distance_left) / self.WHEEL_BASE

            rospy.loginfo("Rotated angle: {:.4f} rad".format(theta_rot))

            if abs(theta_rot) >= abs(target_angle):
                rospy.loginfo("Target rotation reached.")
                break

            cmd_msg = WheelsCmdStamped()
            cmd_msg.header.stamp = rospy.Time.now()
            cmd_msg.vel_left = cmd_vel_left
            cmd_msg.vel_right = cmd_vel_right
            self.pub.publish(cmd_msg)
            rate.sleep()

        self.stop_robot()
        rospy.sleep(1.0)

    def stop_and_hold(self, hold_time=5, led_color=None, indices=[0, 1, 2, 3, 4]):
        self.wait_for_encoders()
        if led_color is not None:
            self.set_led_status(led_color, indices)
        self.stop_robot()
        rospy.sleep(hold_time)