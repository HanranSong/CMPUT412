#!/usr/bin/env python3

import os
import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge
from duckietown.dtros import DTROS, NodeType, DTParam, ParamType
from duckietown_msgs.msg import VehicleCorners, BoolStamped
from sensor_msgs.msg import CompressedImage, CameraInfo
from geometry_msgs.msg import Point32


class VehicleDetectionNode(DTROS):
    """
    This node detects a Duckiebot by looking for a circle grid pattern arranged in 7 columns x 3 rows.
    It subscribes to an undistorted image topic (published by your camera_distortion_node) and uses
    OpenCV's findCirclesGrid to detect the pattern, publishing the detected circle centers in a
    VehicleCorners message.

    Additionally, when using a 2x6 grid detection (12 circles) on the front car, it estimates the grid's
    bounding box width as a proxy for distance. If the grid appears too large (i.e. the front car is too close),
    a stop flag is published. (The threshold value is relative and adjustable.)

    Configuration parameters:
      ~process_frequency (float): Frequency to process incoming images.
      ~circlepattern_dims (list): Grid dimensions [columns, rows]. Default [7, 3] for 7 columns x 3 rows.
      ~blobdetector_min_area (float): Minimum area for blob detection.
      ~blobdetector_min_dist_between_blobs (float): Minimum distance between blobs.
      ~distance_between_centers (float): Physical spacing between circles (meters).
      ~grid_size_threshold (float): Relative threshold for grid width (in pixels) beyond which the car is too close.

    Publishers:
      ~vehicle_detection (VehicleCorners): Detected circle centers.
      ~debug/detection_image/compressed (CompressedImage): Debug image with drawn detections.
      ~stop_flag (BoolStamped): Stop command flag.

    Subscribers:
      /<vehicle_name>/camera_node/image/undistorted/compressed (CompressedImage): Undistorted image.
      ~camera_info (CameraInfo): Camera calibration information.
    """

    def __init__(self, node_name):
        super(VehicleDetectionNode, self).__init__(
            node_name=node_name, node_type=NodeType.PERCEPTION
        )
        self._vehicle_name = os.environ.get("VEHICLE_NAME", "duckiebot")

        # Initialize critical attributes before subscribers
        self.last_stamp = rospy.Time.now()
        self.bridge = CvBridge()
        self.camera_model = None

        # Initialize DTParam parameters
        self.process_frequency = DTParam(
            "~process_frequency", param_type=ParamType.FLOAT, default=5.0
        )
        # Still using the 7x3 pattern for back detection (can be changed if needed)
        self.circlepattern_dims = DTParam(
            "~circlepattern_dims", param_type=ParamType.LIST, default=[4, 3]
        )
        self.blobdetector_min_area = DTParam(
            "~blobdetector_min_area", param_type=ParamType.FLOAT, default=1
        )
        self.blobdetector_min_dist_between_blobs = DTParam(
            "~blobdetector_min_dist_between_blobs",
            param_type=ParamType.FLOAT,
            default=1,
        )
        self.distance_between_centers = DTParam(
            "~distance_between_centers", param_type=ParamType.FLOAT, default=0.001
        )
        # New parameter: if the detected grid width (in pixels) exceeds this threshold, the front car is too close.
        self.grid_size_threshold = DTParam(
            "~grid_size_threshold", param_type=ParamType.FLOAT, default=60.0
        )

        # Configure blob detector for findCirclesGrid with tuned parameters
        params = cv2.SimpleBlobDetector_Params()
        # (Additional parameter tuning can be done here if needed)
        self.blob_detector = cv2.SimpleBlobDetector_create(params)

        # Publisher setup
        self.pub_detection = rospy.Publisher(
            f"/{self._vehicle_name}/vehicle_detection", VehicleCorners, queue_size=1
        )
        self.pub_debug_image = rospy.Publisher(
            f"/{self._vehicle_name}/camera_node/image/vehicle/compressed",
            CompressedImage,
            queue_size=1,
        )
        self.pub_stop_flag = rospy.Publisher(
            f"/{self._vehicle_name}/stop_flag", BoolStamped, queue_size=1
        )

        # Variables for computing the physical circle pattern
        self.circlepattern = None
        self.last_circlepattern_dims = None

        # Set up subscribers after parameters are ready
        undistorted_topic = (
            f"/{self._vehicle_name}/camera_node/image/undistorted/compressed"
        )
        self.sub_image = rospy.Subscriber(
            undistorted_topic, CompressedImage, self.cb_image, queue_size=1
        )

    def compute_circle_pattern(self):
        """
        Builds a model of the circle grid in 3D space (useful if you later want to do pose estimation).
        """
        dims = self.circlepattern_dims.value
        if self.circlepattern is None or self.last_circlepattern_dims != dims:
            rows = dims[1]
            cols = dims[0]
            dist = self.distance_between_centers.value
            self.circlepattern = np.zeros((rows * cols, 3), np.float32)
            for j in range(rows):
                for i in range(cols):
                    index = i + j * cols
                    self.circlepattern[index, 0] = dist * i - dist * (cols - 1) / 2
                    self.circlepattern[index, 1] = dist * j - dist * (rows - 1) / 2
                    self.circlepattern[index, 2] = 0
            self.last_circlepattern_dims = dims

    def cb_image(self, image_msg):
        now = rospy.Time.now()
        if now - self.last_stamp < rospy.Duration.from_sec(
            1.0 / self.process_frequency.value
        ):
            return
        self.last_stamp = now

        try:
            image_cv = self.bridge.compressed_imgmsg_to_cv2(image_msg, "bgr8")
        except Exception as e:
            self.log("Error converting image: " + str(e))
            return

        # Preprocessing to improve detection
        gray = cv2.cvtColor(image_cv, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (3, 3), 0)
        clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
        enhanced = clahe.apply(blurred)
        image_preprocessed = cv2.cvtColor(enhanced, cv2.COLOR_GRAY2BGR)

        # Compute the physical circle pattern if needed
        self.compute_circle_pattern()
        dims = tuple(self.circlepattern_dims.value)

        # For the stopping condition, we use a 2x6 grid detection
        detection, centers = cv2.findCirclesGrid(
            image_preprocessed,
            (6, 2),
            flags=cv2.CALIB_CB_SYMMETRIC_GRID,
            blobDetector=self.blob_detector,
        )

        detection_msg = VehicleCorners()
        detection_msg.header = image_msg.header
        detection_msg.detection.data = detection > 0

        if detection:
            # self.log("Detected {} circle centers.".format(len(centers)))
            points = []
            centers_arr = centers.reshape(-1, 2)
            for pt in centers_arr:
                p = Point32()
                p.x = pt[0]
                p.y = pt[1]
                p.z = 0
                points.append(p)
            detection_msg.corners = points
            detection_msg.H = dims[1]  # Number of rows (from original dims)
            detection_msg.W = dims[0]  # Number of columns (from original dims)

            # Compute the bounding box width as a relative measure for distance
            x_min = np.min(centers_arr[:, 0])
            x_max = np.max(centers_arr[:, 0])
            grid_width = x_max - x_min
            # self.log("Grid width (in pixels): {:.2f}".format(grid_width))

            # Check if the grid appears too large (front car is too close)
            stop_flag = BoolStamped()
            stop_flag.header.stamp = rospy.Time.now()
            if grid_width > 60.0:
                self.log(
                    "Grid width above threshold. Front car is too close. Stopping!"
                )
                stop_flag.data = True
            else:
                stop_flag.data = False
            self.pub_stop_flag.publish(stop_flag)
        else:
            # self.log("No circles detected.")
            detection_msg.corners = []

        self.pub_detection.publish(detection_msg)

        if self.pub_debug_image.get_num_connections() > 0:
            if detection:
                cv2.drawChessboardCorners(
                    image_preprocessed, (6, 2), centers, detection
                )
            try:
                debug_msg = self.bridge.cv2_to_compressed_imgmsg(image_preprocessed)
                self.pub_debug_image.publish(debug_msg)
            except Exception as e:
                self.log("Error publishing debug image: " + str(e))


if __name__ == "__main__":
    node = VehicleDetectionNode(node_name="vehicle_detection_node")
    rospy.spin()
