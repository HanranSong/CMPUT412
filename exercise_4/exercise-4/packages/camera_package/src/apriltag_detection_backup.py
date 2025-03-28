#!/usr/bin/env python3

import os
import rospy
from duckietown.dtros import DTROS, NodeType
from sensor_msgs.msg import CompressedImage
from duckietown_msgs.msg import LEDPattern
from std_msgs.msg import ColorRGBA

import cv2
import numpy as np
from cv_bridge import CvBridge
import dt_apriltags as apriltag  # pip install dt-apriltags


class ApriltagDetectionNode(DTROS):
    def __init__(self, node_name):
        super(ApriltagDetectionNode, self).__init__(
            node_name=node_name, node_type=NodeType.VISUALIZATION
        )

        # Get the vehicle name from environment variables.
        self._vehicle_name = os.environ["VEHICLE_NAME"]

        # Topics for camera image and LED control.
        self._camera_topic = (
            f"/{self._vehicle_name}/camera_node/image/undistorted/compressed"
        )
        self._processed_topic = (
            f"/{self._vehicle_name}/camera_node/image/apriltag/compressed"
        )
        self._led_topic = f"/{self._vehicle_name}/led_emitter_node/led_pattern"

        self._bridge = CvBridge()

        self.sub = rospy.Subscriber(
            self._camera_topic, CompressedImage, self.callback, queue_size=1
        )
        self.pub = rospy.Publisher(self._processed_topic, CompressedImage, queue_size=1)
        self.led_pub = rospy.Publisher(self._led_topic, LEDPattern, queue_size=1)

        # Initialize dt_apriltags detector for the tag36h11 family.
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

        # Set detection rate.
        self.last_detection_time = rospy.Time.now()
        self.detection_interval = rospy.Duration(
            0.33
        )  # 0.25 sec (4 Hz); adjust as needed

        # Mapping from tag id to LED color.
        # self.tag_to_led = {21: "red", 133: "blue", 94: "green"}
        self.tag_to_led = {8: "red", 59: "blue", 21: "green"}

        self.led_colors = {
            "red": [1.0, 0.0, 0.0],
            "blue": [0.0, 0.0, 1.0],
            "green": [0.0, 1.0, 0.0],
            "white": [1.0, 1.0, 1.0],
        }
        self.num_leds = 5  # Adjust if your hardware has a different number

        # Track the last published LED color and the currently active tag.
        self.last_led_color = None
        self.active_tag_id = None

        # For redundancy: count consecutive frames with no valid detection.
        self.missed_detection_count = 0
        self.missed_detection_threshold = (
            3  # e.g. allow 2 consecutive misses before clearing active tag
        )

    def set_led_status(self, color):
        """Publishes an LEDPattern message with the chosen color."""
        color = color.lower()
        self.last_led_color = color  # Update immediately once we decide to change
        rgb = self.led_colors.get(color, self.led_colors["white"])

        pattern_msg = LEDPattern()
        pattern_msg.header.stamp = rospy.Time.now()
        pattern_msg.color_list = [color] * self.num_leds
        pattern_msg.color_mask = [1] * self.num_leds
        pattern_msg.frequency = 0.0
        pattern_msg.frequency_mask = [0] * self.num_leds
        pattern_msg.rgb_vals = [
            ColorRGBA(rgb[0], rgb[1], rgb[2], 1.0) for _ in range(self.num_leds)
        ]
        self.led_pub.publish(pattern_msg)
        rospy.loginfo(f"Published LED pattern: {color}")

    def callback(self, msg):
        try:
            image = self._bridge.compressed_imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            rospy.logerr(f"Image conversion failed: {e}")
            return

        current_time = rospy.Time.now()
        if current_time - self.last_detection_time < self.detection_interval:
            processed_msg = self._bridge.cv2_to_compressed_imgmsg(image)
            self.pub.publish(processed_msg)
            return
        self.last_detection_time = current_time

        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        detections = self.detector.detect(gray)

        # Annotate detections on the image.
        for detection in detections:
            pts = detection.corners.astype(int)
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

        # Build list of valid tag ids.
        valid_tags = [d.tag_id for d in detections if d.tag_id in self.tag_to_led]

        if valid_tags:
            # Reset missed detection counter on valid detection.
            self.missed_detection_count = 0
            # Maintain current active tag if still valid.
            if self.active_tag_id is not None and self.active_tag_id in valid_tags:
                desired_color = self.tag_to_led[self.active_tag_id]
            else:
                # Choose the first valid tag.
                self.active_tag_id = valid_tags[0]
                desired_color = self.tag_to_led[self.active_tag_id]
            rospy.loginfo(
                f"Detected {len(detections)} tag(s); active tag: {self.active_tag_id}, desired LED: {desired_color}"
            )
        else:
            self.missed_detection_count += 1
            if (
                self.missed_detection_count < self.missed_detection_threshold
                and self.active_tag_id is not None
            ):
                # Maintain previous active tag if misses are few.
                desired_color = self.tag_to_led[self.active_tag_id]
                rospy.loginfo(
                    f"No valid tag (missed {self.missed_detection_count} frames); keeping active tag {self.active_tag_id}"
                )
            else:
                # Clear active tag and set LED to white.
                self.active_tag_id = None
                desired_color = "white"
                rospy.loginfo("Clearing active tag; setting LED to white")

        # Update LED only if desired state has changed.
        if desired_color != self.last_led_color:
            self.set_led_status(desired_color)


if __name__ == "__main__":
    node = ApriltagDetectionNode(node_name="apriltag_detection_node")
    rospy.spin()
