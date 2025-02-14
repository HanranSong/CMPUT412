#!/usr/bin/env python3

import os
import rospy
from duckietown.dtros import DTROS, NodeType
from motions_node import Motions
import math

import numpy as np


class CompositeMotionNode(DTROS):
    def __init__(self, node_name):
        # Initialize this node using DTROS.
        super(CompositeMotionNode, self).__init__(
            node_name=node_name, node_type=NodeType.GENERIC
        )
        vehicle_name = os.environ["VEHICLE_NAME"]
        # Instantiate the helper motion controller.
        self.controller = Motions(vehicle_name)

    def run(self):
        # Wait for encoder messages before starting any motion.
        self.controller.wait_for_encoders()

        self.controller.drive_curve(radius=0.1, velocity=0.4, angle_span=2 * np.pi)

        self.controller.drive_curve(radius=-0.1, velocity=0.4, angle_span=2 * np.pi)

        rospy.loginfo("Composite motion sequence complete.")
        # rospy.spin()

    def on_shutdown(self):
        rospy.loginfo("Shutting down CompositeMotionNode, stopping robot.")
        self.controller.stop_robot()


if __name__ == "__main__":
    node = CompositeMotionNode("composite_motion_node")
    node.run()
