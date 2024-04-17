import time
import argparse
import cv2
from cv2 import aruco
import scipy.io as sio
# import pyrealsense2 as rs
import numpy as np
from imutils.video import FileVideoStream

# Import ROS specific packages
import rclpy
from rclpy.node import Node
import std_msgs.msg as std_m
from std_msgs.msg import String
from std_msgs.msg import Int32, Float32
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Int32MultiArray
import serial
import json
import time
from datetime import datetime
import threading
import pygame
import sys

class CameraController(Node):


    def __init__(self):
        super().__init__('camera_teleop')
        self.control_publisher = self.create_publisher(Float32MultiArray,"/marker_position",1)
        timer_period = .05
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.vector = [0,0]
        decay_and_print = threading.Thread(target=self.keyboard_to_vector)
        decay_and_print.daemon = True
        decay_and_print.start()
        self.message = Float32MultiArray()

   
    def keyboard_to_vector(self):
        while True:
            control = sys.stdin.readline()
            if control == 'w\n':
                self.vector[1] -=.05*1920
            if control == 'a\n':
                self.vector[0] -=.05*1080
            if control == 's\n':
                self.vector[1] +=.05*1920
            if control == 'd\n':
                self.vector[0] +=.05*1080

          
  
    def timer_callback(self):
        self.vector[0] = self.vector[0]*.9
        self.vector[1] = self.vector[1]*.9
        self.vector[0] = round(self.vector[0],3)
        self.vector[1] = round(self.vector[1],3)
        self.message.data = self.vector
        self.control_publisher.publish(self.message)
        # print(self.vector)
        # Display the message on the console
        # self.get_logger().info('Publishing video frame')
    
def main(args=None):
  
  # Initialize the rclpy library
  rclpy.init(args=args)
  
  # Create the node
  camera_controller = CameraController()
  
  # Spin the node so the callback function is called.
  rclpy.spin(camera_controller)
  
  camera_controller.destroy_node()

  rclpy.shutdown()
  
if __name__ == '__main__':
  main()