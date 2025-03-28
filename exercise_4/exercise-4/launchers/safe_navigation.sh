#!/bin/bash

source /environment.sh

# initialize launch file
dt-launchfile-init

# launch subscriber
rosrun navigation_package lane_detection_node.py &
rosrun navigation_package navigation_driving_node.py &
rosrun navigation_package camera_distortion_node.py &
rosrun navigation_package safe_navigation_node.py 

# wait for app to end
dt-launchfile-join