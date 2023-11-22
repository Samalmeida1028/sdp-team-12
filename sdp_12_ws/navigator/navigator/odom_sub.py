#!/usr/bin/env python3
# Author: Arjun Viswanathan
# Date created: 11/15/23
# Date last modified: 11/15/23
# Description: Subscribe to odom topic and print out the values

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry

class OdomSub(Node):
    def __init__(self):
        super().__init__('odom_sub')

        self.pub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

    def odom_callback(self, msg):
        print(msg.header)
        print(msg.child_frame_id)
        print(msg.pose.pose)
        print(msg.twist.twist)

def main(args=None):
    rclpy.init(args=args)

    csub = OdomSub()

    rclpy.spin(csub)

    csub.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()