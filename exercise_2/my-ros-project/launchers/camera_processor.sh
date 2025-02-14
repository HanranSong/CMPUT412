#!/bin/bash

source /environment.sh

# initialize launch file
dt-launchfile-init

# launch subscriber
rosrun camera_package camera_processor_node.py

# wait for app to end
dt-launchfile-join