#!/bin/bash

source /environment.sh

# initialize launch file
dt-launchfile-init

# launch subscriber
rosrun motion_package part_3_6_node.py

# wait for app to end
dt-launchfile-join