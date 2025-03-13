#!/bin/bash

source /environment.sh

# initialize launch file
dt-launchfile-init

# launch subscriber
rosrun camera_package camera_distortion_node.py &

rosrun camera_package color_detection_node.py

# wait for app to end
dt-launchfile-join