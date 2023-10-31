import time
import argparse
import cv2
import scipy.io as sio
# import pyrealsense2 as rs
import numpy as np


import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class TargetDetector(Node):

    def __init__(self):
        super().__init__('target_detector')
        self.subscription = self.create_subscription(String,'topic',self.listener_callback,10)
        self.subscription


        # self.declare_parameter('camera_type', 'None')
        # self.declare_parameter('marker_length', 100)
        # self.declare_parameter('timer_period', 10)
        # self.declare_parameter('aruco_matrix', 'aruco.DICT_5X5_1000')
        # self.declare_parameter("image_width", 640)
        # self.declare_parameter('image_height', 480)
        # self.declare_parameter('reinitialize', False)

        # if self.get_parameter('camera_type').get_parameter_value().string_value == 'webcam':
        #     camParams = sio.loadmat("arjunPC_camParams.mat")
        #     # self.cam = cv2.VideoCapture(0)
        # else:
        #     self.get_logger().info("starting without calibration")
        self.cam = cv2.VideoCapture(0)

    
    def listener_callback(self, msg):
        self.get_logger().info("here")
        # success, img = self.cam.read()
        # cv2.imshow(img,'carma')




def main(args=None):
    rclpy.init(args=args)

    node = TargetDetector()

    rclpy.spin(node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()
        

        