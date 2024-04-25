#!/bin/bash

# ffmpeg -f v4l2 -i /dev/video0 -codec:v copy -f v4l2 /dev/video2 -codec:v copy -f v4l2 /dev/video3

ffmpeg -f v4l2 -video_size 1920x1080 -input_format rawvideo -i /dev/video0 \
    -c:v copy -f v4l2 /dev/video2 \
    -c:v copy -f v4l2 /dev/video3 