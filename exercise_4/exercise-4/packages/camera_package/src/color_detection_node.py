#!/usr/bin/env python3
import os
import rospy
from duckietown.dtros import DTROS, NodeType
from sensor_msgs.msg import CompressedImage
import cv2
import numpy as np
from cv_bridge import CvBridge
from std_msgs.msg import Bool


class ColorDetectionNode(DTROS):
    def __init__(self, node_name):
        super(ColorDetectionNode, self).__init__(
            node_name=node_name, node_type=NodeType.VISUALIZATION
        )
        self._vehicle_name = os.environ["VEHICLE_NAME"]

        self._camera_topic = (
            f"/{self._vehicle_name}/camera_node/image/undistorted/compressed"
        )
        self._processed_topic = (
            f"/{self._vehicle_name}/camera_node/image/color/compressed"
        )
        self._line_detected_topic = f"/{self._vehicle_name}/color/line_detected"

        self._bridge = CvBridge()
        self.sub = rospy.Subscriber(
            self._camera_topic, CompressedImage, self.callback, queue_size=1
        )
        self.pub = rospy.Publisher(self._processed_topic, CompressedImage, queue_size=1)
        self.line_pub = rospy.Publisher(self._line_detected_topic, Bool, queue_size=1)

    def callback(self, msg):
        image = self._bridge.compressed_imgmsg_to_cv2(msg, desired_encoding="bgr8")
        h, w = image.shape[:2]
        rospy.loginfo_once("Image size: {} x {}".format(h, w))
        cropped_img = image[250:h, 0:w]  # 300
        blurred_img = cv2.GaussianBlur(cropped_img, (5, 5), 0)
        hsv = cv2.cvtColor(blurred_img, cv2.COLOR_BGR2HSV)

        lower_red = np.array([0 * 0.95, 117 * 0.95, 193 * 0.95])
        upper_red = np.array([24 * 1.01, 157 * 1.01, 233 * 1.01])
        # lower_green = np.array([56 * 0.9, 54 * 0.9, 168 * 0.9])
        # upper_green = np.array([96 * 1.1, 94 * 1.1, 208 * 1.1])
        lower_blue = np.array([91, 129, 132])
        upper_blue = np.array([131, 169, 172])
        kernel = np.ones((5, 5), np.uint8)

        mask_red = cv2.inRange(hsv, lower_red, upper_red)
        mask_red = cv2.dilate(mask_red, kernel, iterations=1)
        contours_red, _ = cv2.findContours(
            mask_red, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )
        max_area_red = max([cv2.contourArea(c) for c in contours_red] or [0])

        # mask_green = cv2.inRange(hsv, lower_green, upper_green)
        # mask_green = cv2.dilate(mask_green, kernel, iterations=1)
        # contours_green, _ = cv2.findContours(
        #     mask_green, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        # )
        # max_area_green = max([cv2.contourArea(c) for c in contours_green] or [0])

        mask_blue = cv2.inRange(hsv, lower_blue, upper_blue)
        mask_blue = cv2.dilate(mask_blue, kernel, iterations=1)
        contours_blue, _ = cv2.findContours(
            mask_blue, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )
        max_area_blue = max([cv2.contourArea(c) for c in contours_blue] or [0])

        processed_msg = self._bridge.cv2_to_compressed_imgmsg(blurred_img)
        self.pub.publish(processed_msg)

        # If any color area is large enough, assume a stop line is detected.
        area_threshold = 7500
        # line_detected = (
        #     max_area_red > area_threshold
        #     or max_area_green > area_threshold
        #     or max_area_blue > area_threshold
        # )
        # line_detected = max_area_red > area_threshold
        line_detected = max_area_red > area_threshold or max_area_blue > area_threshold
        self.line_pub.publish(line_detected)


if __name__ == "__main__":
    node = ColorDetectionNode(node_name="color_detection_node")
    rospy.spin()
