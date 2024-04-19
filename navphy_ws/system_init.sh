#!/bin/bash

colcon build
sudo chmod ao+rwx /dev/ttyACM*
sudo chmod ao+rwx /dev/video*
sudo chmod ao+rwx *py
sudo chmod ao+rwx *.sh
ls -l /dev/ttyACM*
ls -l /dev/video*
ls -l *.py
ls -l *.sh

#./split_video.sh &
#sleep 1
#./start_cloned_streams.sh &
