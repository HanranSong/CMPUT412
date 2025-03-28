#!/usr/bin/env python3
import os
import rospy
import numpy as np
import cv2

from duckietown.dtros import DTROS, NodeType
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String
from cv_bridge import CvBridge

class SafeNavigationNode(DTROS):
    def __init__(self, node_name):
        super(SafeNavigationNode, self).__init__(
            node_name=node_name, node_type=NodeType.PERCEPTION
        )
        self._vehicle_name = os.environ["VEHICLE_NAME"]

        self._bridge = CvBridge()

        # Topics
        self.camera_topic = f"/{self._vehicle_name}/camera_node/image/undistorted/compressed"
        self.bot_debug_topic = f"/{self._vehicle_name}/camera_node/image/bot/compressed"
        self.state_topic = f"/{self._vehicle_name}/navigation/state"

        # Parameters
        self.y_threshold = 115
        self.state = "normal"
        self.state_start_time = rospy.Time.now()

        # Subscribers and Publishers
        self.sub_image = rospy.Subscriber(self.camera_topic, CompressedImage, self.image_callback, queue_size=1)
        self.pub_state = rospy.Publisher(self.state_topic, String, queue_size=1)
        self.pub_bot_debug = rospy.Publisher(self.bot_debug_topic, CompressedImage, queue_size=1)

    def detect_bot(self, image):
        # Crop image along the x-axis to remove background interference
        height, width, _ = image.shape
        x_min = int(width * 0.2)
        x_max = int(width * 0.8)
        cropped = image[:, x_min:x_max]

        hsv = cv2.cvtColor(cropped, cv2.COLOR_BGR2HSV)
        lower_blue = np.array([100, 150, 0])
        upper_blue = np.array([130, 255, 100])
        mask = cv2.inRange(hsv, lower_blue, upper_blue)
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        bboxes = [cv2.boundingRect(cnt) for cnt in contours if cv2.contourArea(cnt) >= 500]

        annotated = cropped.copy()
        if bboxes:
            closest_bot = min(bboxes, key=lambda box: box[1])
            x, y, w, h = closest_bot
            cv2.rectangle(annotated, (x, y), (x+w, y+h), (0, 0, 255), 3)
            if y > self.y_threshold and self.state == "normal":
                self.state = "detected"
                self.state_start_time = rospy.Time.now()
        cv2.putText(annotated, self.state, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,255,0), 2)
        debug_img = self._bridge.cv2_to_compressed_imgmsg(annotated)
        self.pub_bot_debug.publish(debug_img)

        msg = String()
        msg.data = self.state
        self.pub_state.publish(msg)

    def manuvering_timeout(self):
        elapsed = (rospy.Time.now() - self.state_start_time).to_sec()
        
        if self.state == "detected" and elapsed > 3.0:
            self.state = "turn_45_left"
            self.state_start_time = rospy.Time.now()
            rospy.loginfo("Transition: detected -> turn_45_left")
        elif self.state == "turn_45_left" and elapsed > 0.5:
            self.state = "go_straight_angled"
            self.state_start_time = rospy.Time.now()
            rospy.loginfo("Transition: turn_45_left -> go_straight_angled")
        elif self.state == "go_straight_angled" and elapsed > 1.5:
            self.state = "pid_follow_in_opposite_lane"
            self.state_start_time = rospy.Time.now()
            rospy.loginfo("Transition: go_straight_angled -> turn_45_right")
        # elif self.state == "turn_45_right" and elapsed > 0.5:
        #     self.state = "pid_follow_in_opposite_lane"
        #     self.state_start_time = rospy.Time.now()
            rospy.loginfo("Transition: turn_45_right -> pid_follow_in_opposite_lane")
        elif self.state == "pid_follow_in_opposite_lane" and elapsed > 4:
            self.state = "turn_back_right"
            self.state_start_time = rospy.Time.now()
            rospy.loginfo("Transition: pid_follow_in_opposite_lane -> turn_back_right")
        elif self.state == "turn_back_right" and elapsed > 0.4:
            self.state = "go_straight_return"
            self.state_start_time = rospy.Time.now()
            rospy.loginfo("Transition: turn_back_right -> go_straight_return")
        elif self.state == "go_straight_return" and elapsed > 2:
            self.state = "turn_back_left"
            self.state_start_time = rospy.Time.now()
            rospy.loginfo("Transition: go_straight_return -> turn_back_left")
        elif self.state == "turn_back_left" and elapsed > 0.4:
            self.state = "normal"
            self.state_start_time = rospy.Time.now()
        #     rospy.loginfo("Transition: turn_back_left -> normal")


    def image_callback(self, msg):
        try:
            cv_image = self._bridge.compressed_imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            rospy.logerr(f"Image conversion error: {e}")
            return

        self.detect_bot(cv_image)
        self.manuvering_timeout()

    def on_shutdown(self):
        rospy.loginfo("Shutting down. Stopping Duckiebot.")
        # Stop wheels if needed; implement as desired.
        rospy.sleep(0.2)
        super().on_shutdown()

if __name__ == "__main__":
    node = SafeNavigationNode(node_name='safe_navigation_debugging_node')
    rospy.spin()