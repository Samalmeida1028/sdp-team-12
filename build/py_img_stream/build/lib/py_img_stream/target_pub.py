import time
import argparse
import cv2
from cv2 import aruco
import scipy.io as sio
# import pyrealsense2 as rs
import numpy as np

# Import ROS specific packages
import rclpy
from rclpy.node import Node
import std_msgs.msg as std_m
from std_msgs.msg import String
from std_msgs.msg import Int32
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Int32MultiArray

class TargetPublisher(Node):

    def __init__(self):
        super().__init__('target_pub')
        self.publisher = self.create_publisher(Int32, "target_id", 10)
        timer_period = .25
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.declare_parameter('marker_id', 0)
        self.target_id = Int32()
        self.target_id.data = self.get_parameter('marker_id').get_parameter_value().integer_value

    def timer_callback(self):
        self.target_id.data = self.get_parameter('marker_id').get_parameter_value().integer_value
        self.publisher.publish(self.target_id)
        # self.get_logger().info("Published %s" %self.target_id)



def main():
    rclpy.init()
    node = TargetPublisher()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
  