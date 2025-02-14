#!/bin/bash

source /environment.sh

# initialize launch file
dt-launchfile-init

# launch subscriber
rosrun motion_package eight_node.py

# wait for app to end
dt-launchfile-join