#!/bin/bash

source ~/.bashrc
colcon build
ros2 launch basic_mobile_robot teleop.launch.py
