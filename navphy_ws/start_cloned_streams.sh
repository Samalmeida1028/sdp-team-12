#!/bin/bash

ffmpeg -f v4l2 -i /dev/video0 -codec:v copy -f v4l2 /dev/video2 -codec:v copy -f v4l2 /dev/video3
