#!/usr/bin/env python3
import os
import rospy
import numpy as np
import cv2

from duckietown.dtros import DTROS, NodeType
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Float32
from cv_bridge import CvBridge


class LaneDetectionNode(DTROS):
    def __init__(self, node_name):
        super(LaneDetectionNode, self).__init__(
            node_name=node_name, node_type=NodeType.VISUALIZATION
        )
        self._vehicle_name = os.environ["VEHICLE_NAME"]

        self.camera_topic = (
            f"/{self._vehicle_name}/camera_node/image/undistorted/compressed"
        )
        self.overlay_topic = f"/{self._vehicle_name}/camera_node/image/lane/compressed"
        self.error_topic = f"/{self._vehicle_name}/lane/error"

        self.bridge = CvBridge()
        self.sub_img = rospy.Subscriber(
            self.camera_topic, CompressedImage, self.callback, queue_size=1
        )
        self.pub_overlay = rospy.Publisher(
            self.overlay_topic, CompressedImage, queue_size=1
        )
        self.pub_error = rospy.Publisher(self.error_topic, Float32, queue_size=1)

        self.min_area_threshold = 100

        rospy.loginfo(f"[{node_name}] Initialized lane detection node.")

    def callback(self, msg):
        cv_image = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding="bgr8")
        h, w = cv_image.shape[:2]
        roi_top = 200  # 200
        cropped = cv_image[roi_top:h, 0:w]
        blurred = cv2.GaussianBlur(cropped, (5, 5), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

        # lower_yellow = np.array([15, 116 * 0.9, 189 * 0.9])  # 0.75
        # upper_yellow = np.array([55 * 1.1, 176 * 1.1, 249 * 1.1])  # 1.25
        lower_yellow = np.array([5, 116 * 0.8, 189 * 0.8])
        upper_yellow = np.array([55, 176 * 1.2, 249 * 1.2])

        lower_white = np.array([97 * 0.75, 0 * 0.75, 221 * 0.75])
        upper_white = np.array([157 * 1.25, 40 * 1.25, 255 * 1.25])

        mask_yellow = cv2.inRange(hsv, lower_yellow, upper_yellow)
        mask_white = cv2.inRange(hsv, lower_white, upper_white)

        kernel = np.ones((5, 5), np.uint8)
        mask_yellow = cv2.dilate(mask_yellow, kernel, iterations=10)
        mask_yellow = cv2.morphologyEx(mask_yellow, cv2.MORPH_CLOSE, kernel)
        mask_white = cv2.dilate(mask_white, kernel, iterations=1)
        mask_white = cv2.morphologyEx(mask_white, cv2.MORPH_CLOSE, kernel)

        lane_center_x = self.compute_lane_center(mask_yellow, mask_white, w)
        desired_center = w / 2.0
        error = desired_center - lane_center_x

        self.pub_error.publish(error)

        overlay = self.create_overlay(blurred, mask_yellow, mask_white)
        overlay_msg = self.bridge.cv2_to_compressed_imgmsg(overlay)
        self.pub_overlay.publish(overlay_msg)

    def compute_lane_center(self, mask_yellow, mask_white, image_width):
        yel_idx = np.where(mask_yellow > 0)
        whi_idx = np.where(mask_white > 0)
        yel_x_mean = (
            np.mean(yel_idx[1]) if len(yel_idx[0]) > self.min_area_threshold else None
        )
        whi_x_mean = (
            np.mean(whi_idx[1]) if len(whi_idx[0]) > self.min_area_threshold else None
        )

        if yel_x_mean is not None and whi_x_mean is not None:
            lane_center = (yel_x_mean + whi_x_mean) / 2.0  # - 20
        elif yel_x_mean is not None:
            lane_center = yel_x_mean + 200  # 300 is too much
        elif whi_x_mean is not None:
            lane_center = whi_x_mean - 200
        else:
            lane_center = image_width / 2.0
        return lane_center

    def create_overlay(self, base_bgr, mask_yellow, mask_white):
        overlay = base_bgr.copy()
        overlay[mask_yellow > 0] = (0, 255, 255)
        overlay[mask_white > 0] = (255, 255, 255)
        return overlay


if __name__ == "__main__":
    node = LaneDetectionNode("lane_detection_node")
    rospy.spin()