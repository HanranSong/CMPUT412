#!/usr/bin/env python3
import os
import rospy
import numpy as np
from duckietown.dtros import DTROS, NodeType
from sensor_msgs.msg import CompressedImage

import cv2
from cv_bridge import CvBridge
import yaml


class CameraDistortionNode(DTROS):
    def __init__(self, node_name):
        super(CameraDistortionNode, self).__init__(
            node_name=node_name, node_type=NodeType.VISUALIZATION
        )

        self._vehicle_name = os.environ["VEHICLE_NAME"]

        # topics
        self._camera_topic = f"/{self._vehicle_name}/camera_node/image/compressed"
        self._processed_topic = (
            f"/{self._vehicle_name}/camera_node/image/undistorted/compressed"
        )

        self._bridge = CvBridge()

        # subscriber and publisher
        self.sub = rospy.Subscriber(
            self._camera_topic, CompressedImage, self.callback, queue_size=1
        )
        self.pub = rospy.Publisher(self._processed_topic, CompressedImage, queue_size=1)

        calibration_path = f"./packages/camera_package/src/intrinsic.yaml"
        self._camera_matrix, self._dist_coeffs, self.size = self.load_calibration(
            calibration_path
        )

    def callback(self, msg):
        image = self._bridge.compressed_imgmsg_to_cv2(msg)
        h, w = image.shape[:2]

        newcameramatrix, roi = cv2.getOptimalNewCameraMatrix(
            self._camera_matrix, self._dist_coeffs, (w, h), alpha=0.5, newImgSize=(w, h)
        )

        dst = cv2.undistort(
            image, self._camera_matrix, self._dist_coeffs, None, newcameramatrix
        )
        x, y, w, h = roi
        dst = dst[y : y + h, x : x + w]

        msg_out = self._bridge.cv2_to_compressed_imgmsg(dst)
        self.pub.publish(msg_out)

    def load_calibration(self, path):
        with open(path, "r") as file:
            data = yaml.load(file, Loader=yaml.FullLoader)

        camera_matrix = np.array(data["camera_matrix"]["data"]).reshape((3, 3))
        dist_coeffs = np.array(data["distortion_coefficients"]["data"]).reshape((1, 5))
        size = data["image_height"], data["image_width"]

        return camera_matrix, dist_coeffs, size


if __name__ == "__main__":
    node = CameraDistortionNode(node_name="camera_distortion_node")
    rospy.spin()
