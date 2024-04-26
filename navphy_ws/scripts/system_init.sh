#!/bin/bash

colcon build
sudo chmod ao+rwx /dev/ttyACM*
sudo chmod ao+rwx /dev/video*
sudo chmod ao+rwx scripts/*py
sudo chmod ao+rwx scripts/*.sh
ls -l /dev/ttyACM*
ls -l /dev/video*
ls -l scripts/*.py
ls -l scripts/*.sh

#./scripts/split_video.sh 
#sleep 5
#sudo chmod 777 /dev/video*
#sleep 2
#./scripts/start_cloned_streams.sh &
