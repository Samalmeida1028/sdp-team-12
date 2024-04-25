#!/bin/bash
# sudo v4l2loopback-ctl set-caps /dev/video0 "MJPG:1920x1080"
sudo rmmod v4l2loopback
sleep 2
sudo modprobe v4l2loopback devices=2
# sudo v4l2loopback-ctl set-caps /dev/video2 "MJPG:1920x1080" 
# sudo modprobe v4l2loopback-ctl set-caps "MJPG:1920x1080" /dev/video3 
