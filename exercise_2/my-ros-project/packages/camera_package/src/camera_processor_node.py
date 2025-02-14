#!/usr/bin/env python3

import os
import rospy
from duckietown.dtros import DTROS, NodeType
from sensor_msgs.msg import CompressedImage

import cv2
from cv_bridge import CvBridge


class CameraProcessorNode(DTROS):

    def __init__(self, node_name):
        # initialize the DTROS parent class
        super(CameraProcessorNode, self).__init__(
            node_name=node_name, node_type=NodeType.VISUALIZATION
        )

        # Vehicle name
        self._vehicle_name = os.environ["VEHICLE_NAME"]

        # Camera topic
        self._camera_topic = f"/{self._vehicle_name}/camera_node/image/compressed"
        self._processed_topic = (
            f"/{self._vehicle_name}/camera_node/image/processed/compressed"
        )

        # bridge between OpenCV and ROS
        self._bridge = CvBridge()

        # construct subscriber and publisher
        self.sub = rospy.Subscriber(self._camera_topic, CompressedImage, self.callback)
        self.pub = rospy.Publisher(self._processed_topic, CompressedImage, queue_size=1)

    def callback(self, msg):
        # convert JPEG bytes to CV image
        image = self._bridge.compressed_imgmsg_to_cv2(msg)

        # Get image dimensions
        height, width = image.shape[:2]

        # Convert to grayscale
        gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        # Convert grayscale to 3-channel image (for annotation)
        annotated_image = cv2.cvtColor(gray_image, cv2.COLOR_GRAY2BGR)

        # Add annotation text
        text = f"Duck {self._vehicle_name} says, 'Cheese! Capturing {width}x{height} - quack-tastic!'"
        text_position = (10, height - 10)  # Bottom left corner
        cv2.putText(
            annotated_image,
            text,
            text_position,
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (0, 0, 255),
        )

        # Convert back to ROS message and publish
        processed_msg = self._bridge.cv2_to_compressed_imgmsg(annotated_image)
        self.pub.publish(processed_msg)


if __name__ == "__main__":
    # create the node
    node = CameraProcessorNode(node_name="camera_reader_node")
    # keep spinning
    rospy.spin()
