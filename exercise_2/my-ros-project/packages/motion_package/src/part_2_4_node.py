#!/usr/bin/env python3

import os
import math
import rospy
from duckietown.dtros import DTROS, NodeType
from motions_node import Motions


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
        # Wait for encoder
        self.controller.wait_for_encoders()

        rospy.loginfo("Rotating ...")
        self.controller.rotate_robot(target_angle=-math.pi / 2, speed=0.3)

        rospy.loginfo("Rotating ...")
        self.controller.rotate_robot(target_angle=math.pi / 2, speed=0.175)

        rospy.loginfo("Composite motion sequence complete.")
        # rospy.spin()

    def on_shutdown(self):
        rospy.loginfo("Shutting down CompositeMotionNode, stopping robot.")
        self.controller.stop_robot()


if __name__ == "__main__":
    node = CompositeMotionNode("composite_motion_node")
    node.run()
