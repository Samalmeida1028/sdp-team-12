#!/usr/bin/env python3
# Author: Arjun Viswanathan
# Date created: 11/15/23
# Date last modified: 11/15/23
# Description: Obtain odometry information from robot and publish it to odom topic

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import serial

class OdomPub(Node):
    def __init__(self):
        super().__init__('odom_pub')

        self.pub = self.create_publisher(Odometry, '/odom', 10)

        time_period = 0.5
        self.timer = self.create_timer(time_period, self.timer_callback)

    def timer_callback(self):
        msg = Odometry()

        self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)

    cpub = OdomPub()

    rclpy.spin(cpub)

    cpub.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()