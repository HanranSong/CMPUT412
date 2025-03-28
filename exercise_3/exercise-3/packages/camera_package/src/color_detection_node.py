#!/usr/bin/env python3

import os
import rospy
from duckietown.dtros import DTROS, NodeType
from sensor_msgs.msg import CompressedImage
from duckietown_msgs.msg import Twist2DStamped
from motions import Motions
import math

import cv2
import numpy as np
from cv_bridge import CvBridge

class ColorDetectionNode(DTROS):
    def __init__(self, node_name):
        super(ColorDetectionNode, self).__init__(
            node_name=node_name, node_type=NodeType.VISUALIZATION
        )

        # Get the vehicle name from environment
        self._vehicle_name = os.environ["VEHICLE_NAME"]

        self._controller = Motions(self._vehicle_name)
        self._controller.wait_for_encoders()
        self._controller.set_led_status(color="yellow")

        # Subscribe directly to the raw, undistorted camera topic
        self._camera_topic = (
            f"/{self._vehicle_name}/camera_node/image/undistorted/compressed"
        )
        # Publish the final image with color detection annotations
        self._processed_topic = (
            f"/{self._vehicle_name}/camera_node/image/color/compressed"
        )
        
        self._motion_topic = (
            f"/{self._vehicle_name}/car_cmd_switch_node/cmd"
        )
        
        # Define the thresholds
        self._threshold = 500
        self._y_threshold = 190
        self._robot_stopped = False

        self._bridge = CvBridge()

        # Single subscriber and publisher
        self.sub = rospy.Subscriber(
            self._camera_topic, CompressedImage, self.callback, queue_size=1
        )
        self.pub = rospy.Publisher(self._processed_topic, CompressedImage, queue_size=1)
        self.pub_motion = rospy.Publisher(self._motion_topic, Twist2DStamped, queue_size=1)

    def callback(self, msg):
        # Convert incoming compressed image to OpenCV BGR image
        image = self._bridge.compressed_imgmsg_to_cv2(msg, desired_encoding="bgr8")
        h, w = image.shape[:2]
        rospy.loginfo_once(f"Image size: {h} x {w}")

        cropped_img = image[120:h, 0:w]
        # Apply Gaussian blur to reduce noise
        blurred_img = cv2.GaussianBlur(cropped_img, (5, 5), 0)

        # ----- Color Detection -----
        # Convert the pre-processed image from BGR to HSV color space
        hsv = cv2.cvtColor(blurred_img, cv2.COLOR_BGR2HSV)

        # Define HSV thresholds for red, green, and blue
        lower_red = np.array([0 * 0.95, 117 * 0.95, 193 * 0.95])
        upper_red = np.array([24 * 1.01, 157 * 1.01, 233 * 1.01])
        lower_green = np.array([56 * 0.9, 54 * 0.9, 168 * 0.9])
        upper_green = np.array([96 * 1.1, 94 * 1.1, 208 * 1.1])
        lower_blue = np.array([91, 129, 132])
        upper_blue = np.array([131, 169, 172])

        # Define a kernel for dilation to reduce noise in the masks
        kernel = np.ones((5, 5), np.uint8)

        # Define the rgb of each color
        red_rgb = (255, 0, 0)
        green_rgb = (0, 255, 0)
        blue_rgb = (0, 0, 255)

        # Process red color - take only the largest contour
        red_box = self.process_color(hsv, lower_red, upper_red, kernel)
        green_box = self.process_color(hsv, lower_green, upper_green, kernel)
        blue_box = self.process_color(hsv, lower_blue, upper_blue, kernel)
        
        box = None
        if red_box is not None:
            box, color = red_box, red_rgb
            self._y_threshold = 190
        elif green_box is not None:
            box, color = green_box, green_rgb
            self._y_threshold = 155
        elif blue_box is not None:
            box, color = blue_box, blue_rgb
            self._y_threshold = 160

        if box is not None:
            blurred_img = self.draw_image(box, blurred_img, color)
            xb, yb, wb, hb = box
            rospy.loginfo(f"Logged y {yb} and with width {wb}; total {yb + wb}")
        
        # ----- Publish the final annotated image -----
        processed_msg = self._bridge.cv2_to_compressed_imgmsg(blurred_img)
        self.pub.publish(processed_msg)
        
        if box is not None:
            if (yb > self._y_threshold):
                rospy.loginfo("Y threshold reached, stopping robot.")
                self.stop_robot()
                self._robot_stopped = True
                if color == red_rgb:
                    self.execute_red()
                elif color == green_rgb:
                    self.execute_green(color)
                    rospy.loginfo("Detecting green")
                elif color == blue_rgb:
                    self.execute_blue(color)

            elif (yb <= self._y_threshold and self._robot_stopped is False):
                self.move_forward()
        
        elif box is None and self._robot_stopped is False:
            self.move_forward()
    
    def draw_image(self, coordinates, blurred_img, color_rgb):
        x, y, w, h = coordinates
        return cv2.rectangle(blurred_img, (x, y), (x + w, y + h), color_rgb, 2)

    def process_color(self, hsv, lower_range, upper_range, kernel):
        mask = cv2.inRange(hsv, lower_range, upper_range)
        mask = cv2.dilate(mask, kernel, iterations = 1)
        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )

        largest = None
        max_area = 0
        for contour in contours:
            area = cv2.contourArea(contour)
            if area > self._threshold and area > max_area:
                max_area = area
                largest = contour

        if largest is not None:
            return cv2.boundingRect(largest)
        else:
            return None

    def move_forward(self, velocity = 0.15, correction = 0.0):
        twist = Twist2DStamped()
        twist.header.stamp = rospy.Time.now()
        twist.v = velocity
        twist.omega = correction
        self.pub_motion.publish(twist)
    
    def stop_robot(self):
        twist = Twist2DStamped()
        twist.header.stamp = rospy.Time.now()
        twist.v = 0.0
        twist.omega = 0.0
        self.pub_motion.publish(twist)

    def rotate_robot(self, direction):
        if direction == "left":
            self._controller.rotate(target_angle=0.9 * (math.pi / 2), speed=0.4)
        elif direction == "right":
            self._controller.rotate(target_angle=-0.9 * (math.pi / 2), speed=0.4)
        else:
            rospy.logwarn("Invalid direction. No rotation performed.")

    def drive_curve(self, radius=0.092, velocity=0.4, angle_span=0.9 * np.pi / 2, direction="right"):
        self._controller.curve(radius=radius, velocity=velocity, angle_span=angle_span, direction=direction)

    def execute_red(self, color="red"):
        rospy.loginfo("Executing red behaviour")
        self._controller.stop_and_hold(hold_time=3, led_color=color)
        self._controller.move_straight(target_distance=0.4, speed=0.4)
        self.shutdown()

    def execute_blue(self, color):
        rospy.loginfo("Executing blue behaviour")
        self._controller.stop_and_hold(hold_time=3, led_color=color, indices=[1, 4])
        self.drive_curve(radius = 0.075, direction="right")
        self.shutdown()

    def execute_green(self, color):
        rospy.loginfo("Executing green haviour")
        self._controller.stop_and_hold(hold_time=3, led_color=color, indices=[0, 3])
        self._controller.move_straight(speed = 0.2, target_distance=0.3)
        rospy.loginfo("Executing stop and hold")
        self.drive_curve(direction="left")
        self.shutdown()

    def shutdown(self):
        rospy.loginfo("Shutting down color detection node.")
        rospy.signal_shutdown("Color detection node has completed its task.")

if __name__ == "__main__":
    node = ColorDetectionNode(node_name="color_detection_node")
    rospy.spin()