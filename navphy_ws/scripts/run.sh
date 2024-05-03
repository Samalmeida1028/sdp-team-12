#!/bin/bash

cd /home/sdpteam12/Desktop/sdp-team-12/navphy_ws

source /opt/ros/humble/setup.bash
source install/setup.bash
sleep 3

./scripts/system_init.sh

ros2 run gui gui
