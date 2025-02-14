#!/bin/bash

source /environment.sh

# initialize launch file
dt-launchfile-init

# launch subscriber
rosrun motion_package part_2_4_node.py

# wait for app to end
dt-launchfile-join