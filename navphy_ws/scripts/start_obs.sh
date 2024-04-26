#!/bin/bash

./scripts/split_video.sh 
sleep 5
sudo chmod 777 /dev/video*
sleep 2
./scripts/start_cloned_streams.sh &
sleep 3

obs --startstreaming