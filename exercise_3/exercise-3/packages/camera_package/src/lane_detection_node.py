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
        # Get vehicle name from environment
        self._vehicle_name = os.environ["VEHICLE_NAME"]

        # Topics for image input, overlay output, and lane error output
        self.camera_topic = (
            f"/{self._vehicle_name}/camera_node/image/undistorted/compressed"
        )
        self.overlay_topic = f"/{self._vehicle_name}/camera_node/image/lane/compressed"
        self.error_topic = f"/{self._vehicle_name}/lane/error"

        # Publishers and subscriber
        self.bridge = CvBridge()
        self.sub_img = rospy.Subscriber(
            self.camera_topic, CompressedImage, self.callback, queue_size=1
        )
        self.pub_overlay = rospy.Publisher(
            self.overlay_topic, CompressedImage, queue_size=1
        )
        self.pub_error = rospy.Publisher(self.error_topic, Float32, queue_size=1)

        # Minimum pixel count for a valid detection
        self.min_area_threshold = 100

        rospy.loginfo(
            f"[{node_name}] Initialized lane detection node with morphological processing."
        )

    def callback(self, msg):
        # Convert incoming image to OpenCV BGR image
        cv_image = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding="bgr8")
        h, w = cv_image.shape[:2]

        # Crop the image (e.g., remove the top part that is less relevant)
        roi_top = 200  # 240
        cropped = cv_image[roi_top:h, 0:w]

        # Apply Gaussian blur to reduce noise
        blurred = cv2.GaussianBlur(cropped, (5, 5), 0)

        # Convert to HSV color space
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

        # Define HSV thresholds for yellow and white lanes.
        # These values may need tuning based on your lighting conditions.
        lower_yellow = np.array([0 * 0.75, 116 * 0.75, 189 * 0.75])
        upper_yellow = np.array([55 * 1.25, 176 * 1.25, 249 * 1.25])
        lower_white = np.array([97 * 0.75, 0 * 0.75, 221 * 0.75])
        upper_white = np.array([157 * 1.25, 40 * 1.25, 255 * 1.25])

        # Create masks for yellow and white lanes
        mask_yellow = cv2.inRange(hsv, lower_yellow, upper_yellow)
        mask_white = cv2.inRange(hsv, lower_white, upper_white)

        # Define a kernel for morphological operations
        kernel = np.ones((5, 5), np.uint8)

        # Process yellow mask:
        # Dilate to merge nearby regions and then close gaps with morphological closing
        mask_yellow = cv2.dilate(mask_yellow, kernel, iterations=10)
        mask_yellow = cv2.morphologyEx(mask_yellow, cv2.MORPH_CLOSE, kernel)

        # Process white mask similarly
        mask_white = cv2.dilate(mask_white, kernel, iterations=1)
        mask_white = cv2.morphologyEx(mask_white, cv2.MORPH_CLOSE, kernel)

        # Compute lane center from the two masks
        lane_center_x = self.compute_lane_center(mask_yellow, mask_white, w)
        desired_center = w / 2.0
        error = desired_center - lane_center_x

        # Publish the error for use by a separate driving node
        self.pub_error.publish(error)

        # Create an overlay for visualization: paint yellow and white on the real image
        overlay = self.create_overlay(blurred, mask_yellow, mask_white)
        overlay_msg = self.bridge.cv2_to_compressed_imgmsg(overlay)
        self.pub_overlay.publish(overlay_msg)

    def compute_lane_center(self, mask_yellow, mask_white, image_width):
        """Compute the x-coordinate of the lane center based on yellow and white masks."""
        # Get x-coordinates where each mask is non-zero
        yel_idx = np.where(mask_yellow > 0)
        whi_idx = np.where(mask_white > 0)

        yel_x_mean = (
            np.mean(yel_idx[1]) if len(yel_idx[0]) > self.min_area_threshold else None
        )
        whi_x_mean = (
            np.mean(whi_idx[1]) if len(whi_idx[0]) > self.min_area_threshold else None
        )

        # If both lane lines are detected, take the midpoint.
        if yel_x_mean is not None and whi_x_mean is not None:
            lane_center = (yel_x_mean + whi_x_mean) / 2.0
        elif yel_x_mean is not None:
            # Only yellow detected; assume it's the left lane line.
            lane_center = yel_x_mean + 250  # Adjust offset based on lane width.
        elif whi_x_mean is not None:
            # Only white detected; assume it's the right lane line.
            lane_center = whi_x_mean - 250
        else:
            # Fallback to the image center if no lane lines are detected.
            lane_center = image_width / 2.0

        return lane_center

    def create_overlay(self, base_bgr, mask_yellow, mask_white):
        """
        Create an overlay image by painting the detected yellow areas in yellow and
        white areas in white over the original cropped image.
        """
        overlay = base_bgr.copy()
        # Paint yellow areas (BGR: (0, 255, 255))
        overlay[mask_yellow > 0] = (0, 255, 255)
        # Paint white areas (BGR: (255, 255, 255))
        overlay[mask_white > 0] = (255, 255, 255)
        return overlay


if __name__ == "__main__":
    node = LaneDetectionNode("lane_detection_node")
    rospy.spin()
