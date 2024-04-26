#!/bin/bash

cd /home/sdpteam12/Desktop/sdp-team-12/navphy_ws

source /opt/ros/humble/setup.bash
source install/setup.bash
sleep 3

./scripts/system_init.sh

sleep 2
obs --startrecording &

ros2 run gui gui

# `ros2 run gui gui` &
# $SHELL
