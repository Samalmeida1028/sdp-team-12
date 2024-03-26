#!/usr/bin/env python3
# Author: Arjun Viswanathan
# Date created: 3/25/24
# Date last modified: 3/25/24
# Description: Filters LiDAR scans under some threshold. 
# This is needed because our third platform gets in the way of the LiDAR and needs filtering for nav

from sensor_msgs.msg import LaserScan
import rclpy
from rclpy.node import Node
import numpy as np
import math

class FilterLiDAR(Node):
    def __init__(self):
        super().__init__('filter_lidar')

        self.scansub = self.create_subscription(LaserScan, '/scan', self.filter_scans, 10)
        self.filteredscanpub = self.create_publisher(LaserScan, '/scan_filtered', 10)

        self.threshold = 0.2286 # m
        self.filteredmsg = LaserScan()

        self.get_logger().info('Filter LiDAR Node Ready!')

    def filter_scans(self, scanmsg : LaserScan):
        self.filteredmsg = scanmsg

        filterinds = [i for i in range(len(list(scanmsg.ranges))) if list(scanmsg.ranges)[i] <= self.threshold]

        for i in filterinds:
            # self.get_logger().info('Setting range {} to inf'.format(i))
            self.filteredmsg.ranges[i] = math.inf

        self.filteredscanpub.publish(self.filteredmsg)

def main(args=None):
    rclpy.init(args=args)

    filter = FilterLiDAR()

    rclpy.spin(filter)

    filter.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
