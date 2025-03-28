#!/usr/bin/env python3

import os
import rospy
import cv2
import numpy as np

from duckietown.dtros import DTROS, NodeType
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String
from cv_bridge import CvBridge

class CrosswalkDetectionNode(DTROS):
    def __init__(self, node_name):
        super(CrosswalkDetectionNode, self).__init__(
            node_name=node_name, node_type=NodeType.LOCALIZATION
        )
        self._vehicle_name = os.environ.get("VEHICLE_NAME", "csc22919")
        
        # Topics
        self.camera_topic = f"/{self._vehicle_name}/camera_node/image/undistorted/compressed"
        self.duckie_topic = f"/{self._vehicle_name}/camera_node/image/duckie/compressed"
        self.line_topic = f"/{self._vehicle_name}/camera_node/image/line/compressed"
        self.state_topic = f"/{self._vehicle_name}/crosswalk/state"
        
        # Parameters
        self.line_threshold = 500   # Minimum contour area for a valid blue line
        self.y_threshold = 135      # Pixel threshold for the blue line to be considered "close"
        self.publish_debug = True   # Toggle debug image publishing
        
        self._bridge = CvBridge()
        
        # Subscribers and Publishers
        self.sub = rospy.Subscriber(self.camera_topic, CompressedImage, self.image_callback, queue_size=1)
        self.pub_duckie = rospy.Publisher(self.duckie_topic, CompressedImage, queue_size=1)
        self.pub_line = rospy.Publisher(self.line_topic, CompressedImage, queue_size=1)
        self.pub_state = rospy.Publisher(self.state_topic, String, queue_size=1)
        
        # State machine variables
        self.crosswalk_state = "normal"
        self.wait_start_time = None
        self.passed_start_time = None
        
        rospy.loginfo(f"[{node_name}] Crosswalk Detection Node Initialized.")
    
    def process_blue(self, hsv):
        """
        Finds and groups blue contours that are roughly in the same horizontal row,
        then returns a merged bounding box for that group.
        Here, we use a grouping by y-coordinate with a tolerance.
        """
        kernel = np.ones((5, 5), np.uint8)
        lower_blue = np.array([91, 129, 132])
        upper_blue = np.array([131, 169, 172])
        mask = cv2.inRange(hsv, lower_blue, upper_blue)
        mask = cv2.dilate(mask, kernel, iterations=1)

        # Find all contours
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        bboxes = []
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area >= self.line_threshold:
                x, y, w, h = cv2.boundingRect(cnt)
                bboxes.append((x, y, w, h))

        if not bboxes:
            return None

        # Sort bounding boxes by y descending (bottom-most first)
        bboxes.sort(key=lambda box: box[1], reverse=True)

        # Group bounding boxes that are close in y
        groups = []
        y_tolerance = 50
        while bboxes:
            current = bboxes.pop(0)
            group = [current]
            i = 0
            while i < len(bboxes):
                (_, y2, _, _) = bboxes[i]
                if abs(y2 - current[1]) < y_tolerance:
                    group.append(bboxes.pop(i))
                else:
                    i += 1
            groups.append(group)
        
        # For each group, merge them into a single bounding box
        merged_bboxes = []
        for grp in groups:
            min_x = min(b[0] for b in grp)
            min_y = min(b[1] for b in grp)
            max_x = max(b[0] + b[2] for b in grp)
            max_y = max(b[1] + b[3] for b in grp)
            merged_bboxes.append((min_x, min_y, max_x - min_x, max_y - min_y))
        
        # Return the bottom-most merged bounding box (first group, as we sorted descending)
        return merged_bboxes[0]

    def draw_image(self, coordinates, img, color_rgb):
        x, y, w, h = coordinates
        return cv2.rectangle(img, (x, y), (x + w, y + h), color_rgb, 2)
    
    def detect_line(self, image):
        """
        Crop the image to focus on the lower part,
        then detect the blue lane and draw its bounding box.
        Returns the annotated image, a flag if a line was detected,
        and a flag (send_state) if the bounding box is close enough.
        """
        height, width, _ = image.shape
        # Crop to lower half of the image for the closest blue lane
        roi = image[int(height * 0.35):, :]
        hsv_roi = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        bbox = self.process_blue(hsv_roi)
        line_detected = False
        send_state = False
        if bbox is not None:
            x, y, w, h = bbox
            # Adjust y coordinate relative to original image
            y_adjusted = y + int(height * 0.35)
            cv2.rectangle(image, (x, y_adjusted), (x + w, y_adjusted + h), (255, 0, 0), 2)
            line_detected = True
            # If the bottom of the box is below our y threshold, mark as ready to change state
            if y_adjusted + h >= self.y_threshold:
                send_state = True
        return image, line_detected, send_state

    def detect_ducks(self, image):
        """
        Detect red-colored duckies (peDuckstrians) in the image.
        Returns the image annotated with bounding boxes and a flag if any duck was detected.
        """
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        lower_red1 = np.array([0, 70, 50])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([170, 70, 50])
        upper_red2 = np.array([180, 255, 255])
        mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
        mask = cv2.bitwise_or(mask1, mask2)
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        duck_detected = False
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < 100:  # Ignore small noise
                continue
            duck_detected = True
            x, y, w, h = cv2.boundingRect(cnt)
            cv2.rectangle(image, (x, y), (x + w, y + h), (0, 255, 0), 2)
        return image, duck_detected

    def image_callback(self, msg):
        try:
            image = self._bridge.compressed_imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            rospy.logerr("Failed to convert image: %s", e)
            return

        # Crop image to focus on lower part
        height, width, _ = image.shape
        image_cropped = image[int(height * 0.35):, :]
        
        # Detect blue lane and duckies
        line_image, line_detected, send_state = self.detect_line(image_cropped.copy())
        duck_image, duck_detected = self.detect_ducks(image_cropped.copy())
        
        # --- State Machine Logic ---
        current_time = rospy.Time.now()
        if self.crosswalk_state == "normal":
            if line_detected and send_state:
                rospy.loginfo("Blue line detected and close: entering waiting state.")
                self.crosswalk_state = "waiting"
                self.wait_start_time = current_time
        elif self.crosswalk_state == "waiting":
            if (current_time - self.wait_start_time).to_sec() >= 1.0:
                if duck_detected:
                    rospy.loginfo("Duck detected during waiting: entering stopped state.")
                    self.crosswalk_state = "stopped"
                else:
                    rospy.loginfo("No duck detected after waiting: entering passed state.")
                    self.crosswalk_state = "passed"
                    self.passed_start_time = current_time
        elif self.crosswalk_state == "stopped":
            if not duck_detected:
                rospy.loginfo("Duck cleared: entering passed state.")
                self.crosswalk_state = "passed"
                self.passed_start_time = current_time
        elif self.crosswalk_state == "passed":
            if (current_time - self.passed_start_time).to_sec() >= 2.0:
                rospy.loginfo("Crosswalk event complete: returning to normal state.")
                self.crosswalk_state = "normal"
        
        # Publish state
        state_msg = String()
        state_msg.data = self.crosswalk_state
        self.pub_state.publish(state_msg)
        
        # Publish debug images if enabled
        if self.publish_debug:
            # Annotate with state text
            state_text = f"State: {self.crosswalk_state}"
            cv2.putText(line_image, state_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,255,255), 2)
            cv2.putText(duck_image, state_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,255,255), 2)
            try:
                line_msg = self._bridge.cv2_to_compressed_imgmsg(line_image)
                duck_msg = self._bridge.cv2_to_compressed_imgmsg(duck_image)
                self.pub_line.publish(line_msg)
                self.pub_duckie.publish(duck_msg)
            except Exception as e:
                rospy.logerr("Failed to publish debug images: %s", e)

if __name__ == '__main__':
    node = CrosswalkDetectionNode(node_name='crosswalk_detection_node')
    rospy.spin()
