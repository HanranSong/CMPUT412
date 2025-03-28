#!/usr/bin/env python3
import os
import rospy
from duckietown.dtros import DTROS, NodeType
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Int32, ColorRGBA
from duckietown_msgs.msg import LEDPattern

import cv2
import numpy as np
from cv_bridge import CvBridge
import dt_apriltags as apriltag  # pip install dt-apriltags


class ApriltagDetectionNode(DTROS):
    def __init__(self, node_name):
        super(ApriltagDetectionNode, self).__init__(
            node_name=node_name, node_type=NodeType.VISUALIZATION
        )
        self._vehicle_name = os.environ["VEHICLE_NAME"]

        self._camera_topic = (
            f"/{self._vehicle_name}/camera_node/image/undistorted/compressed"
        )
        self._processed_topic = (
            f"/{self._vehicle_name}/camera_node/image/apriltag/compressed"
        )
        self._active_tag_topic = f"/{self._vehicle_name}/apriltag/active_tag"

        self._bridge = CvBridge()

        self.sub = rospy.Subscriber(
            self._camera_topic, CompressedImage, self.callback, queue_size=1
        )
        self.pub = rospy.Publisher(self._processed_topic, CompressedImage, queue_size=1)
        self.active_tag_pub = rospy.Publisher(
            self._active_tag_topic, Int32, queue_size=1
        )

        self.detector = apriltag.Detector(
            searchpath=["apriltags"],
            families="tag36h11",
            nthreads=1,
            quad_decimate=1.0,
            quad_sigma=0.0,
            refine_edges=1,
            decode_sharpening=0.25,
            debug=0,
        )
        self.last_detection_time = rospy.Time.now()
        self.detection_interval = rospy.Duration(0.33)  # adjust as needed

        # self.tag_to_led = {21: "red", 133: "blue", 94: "green"}
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
        self.num_leds = 5
        self.last_led_color = None
        self.active_tag_id = None
        self.missed_detection_count = 0
        self.missed_detection_threshold = 3

        self.minimum_area = 500  # pixel area threshold; adjust as needed

    def set_led_status(self, color):
        """(Optional) Publishes an LEDPattern message with the chosen color."""
        color = color.lower()
        self.last_led_color = color
        rgb = {
            "red": [1.0, 0.0, 0.0],
            "blue": [0.0, 0.0, 1.0],
            "green": [0.0, 1.0, 0.0],
            "white": [1.0, 1.0, 1.0],
        }.get(color, [1.0, 1.0, 1.0])
        pattern_msg = LEDPattern()
        pattern_msg.header.stamp = rospy.Time.now()
        pattern_msg.color_list = [color] * self.num_leds
        pattern_msg.color_mask = [1] * self.num_leds
        pattern_msg.frequency = 0.0
        pattern_msg.frequency_mask = [0] * self.num_leds
        pattern_msg.rgb_vals = [
            ColorRGBA(rgb[0], rgb[1], rgb[2], 1.0) for _ in range(self.num_leds)
        ]
        self.active_tag_pub.publish(pattern_msg)  # (or publish elsewhere if needed)
        rospy.loginfo("Published LED pattern: " + color)

    def callback(self, msg):
        try:
            image = self._bridge.compressed_imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            rospy.logerr("Image conversion failed: " + str(e))
            return

        # Crop the image to focus on the middle:
        height, width, _ = image.shape
        # For example, remove 20% from left and right:
        left_crop = int(width * 0.2)
        right_crop = int(width * 1.0)
        cropped_image = image[:, left_crop:right_crop]

        current_time = rospy.Time.now()
        if current_time - self.last_detection_time < self.detection_interval:
            processed_msg = self._bridge.cv2_to_compressed_imgmsg(image)
            self.pub.publish(processed_msg)
            return
        self.last_detection_time = current_time

        # Convert the cropped image to grayscale for detection.
        gray = cv2.cvtColor(cropped_image, cv2.COLOR_BGR2GRAY)
        detections = self.detector.detect(gray)

        # Process each detection:
        for detection in detections:
            # The detection coordinates are relative to the cropped image;
            # add the left crop offset to map them to the original image.
            pts = detection.corners.astype(int)
            pts[:, 0] += left_crop
            cv2.polylines(image, [pts], isClosed=True, color=(0, 255, 0), thickness=2)
            center_x = int(np.mean(pts[:, 0]))
            center_y = int(np.mean(pts[:, 1]))
            cv2.putText(
                image,
                str(detection.tag_id),
                (center_x, center_y),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (255, 0, 0),
                2,
            )

        processed_msg = self._bridge.cv2_to_compressed_imgmsg(image)
        self.pub.publish(processed_msg)

        # Filter valid detections: only consider tags with desired IDs and that are big enough.
        valid_detections = [
            d
            for d in detections
            if d.tag_id in self.tag_to_led
            and cv2.contourArea(d.corners.astype(int)) >= self.minimum_area
        ]

        if valid_detections:
            self.missed_detection_count = 0
            largest_detection = max(
                valid_detections, key=lambda d: cv2.contourArea(d.corners.astype(int))
            )
            desired_active_id = largest_detection.tag_id
            self.active_tag_id = desired_active_id
            self.active_tag_pub.publish(desired_active_id)
            rospy.loginfo("Published active tag: " + str(desired_active_id))
        else:
            self.missed_detection_count += 1
            if self.missed_detection_count >= self.missed_detection_threshold:
                self.active_tag_id = None
                self.active_tag_pub.publish(-1)
                rospy.loginfo("No valid tag detected; published -1 as active tag.")


if __name__ == "__main__":
    node = ApriltagDetectionNode(node_name="apriltag_detection_node")
    rospy.spin()
