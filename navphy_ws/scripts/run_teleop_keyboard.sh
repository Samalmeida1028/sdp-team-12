#!/bin/bash

#cd /home/sdpteam12/Desktop/sdp-team-12/navphy_ws
cd /home/adam/Desktop/sdp-team-12/navphy_ws

source /opt/ros/humble/setup.bash
source install/setup.bash
sleep 3

ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap /cmd_vel:=/teleop_vel
