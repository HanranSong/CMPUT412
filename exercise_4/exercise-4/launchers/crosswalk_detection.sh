#!/bin/bash

source /environment.sh

# initialize launch file
dt-launchfile-init

# launch subscriber
rosrun crosswalk_package camera_distortion_node.py &
rosrun crosswalk_package crosswalk_detection_node.py &
rosrun crosswalk_package crosswalk_driving_node.py &
rosrun crosswalk_package lane_detection_node.py

# wait for app to end
dt-launchfile-join