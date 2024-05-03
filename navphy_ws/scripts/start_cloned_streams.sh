#!/bin/bash

sudo rmmod v4l2loopback
sleep 2
sudo modprobe v4l2loopback devices=2

sudo chmod 777 /dev/video*

ffmpeg -f v4l2 -video_size 1920x1080 -r 30 -input_format rawvideo -i /dev/video0 \
    -c:v copy -f v4l2 /dev/video2 \
    -c:v copy -f v4l2 /dev/video3 
    > /dev/null 2&>1