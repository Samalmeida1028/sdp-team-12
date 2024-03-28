#!/bin/bash

source ~/.bashrc
colcon build
ros2 launch basic_mobile_robot nav_physical.launch.py rviz:=False
