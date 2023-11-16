#!usr/bin/env python3
# Author: Arjun Viswanathan
# Date created: 11/15/23
# Date last modified: 11/15/23
# Description: Convert Odometry messages to TF messages and send

import rclpy
from rclpy.node import Node
import tf2_ros
import geometry_msgs.msg
from nav_msgs.msg import Odometry

class TFBroadcast(Node):
    def __init__(self):
        super().__init__('tf_broadcaster')

        self.br = tf2_ros.TransformBroadcaster(self)
        self.t = geometry_msgs.msg.TransformStamped()

        self.odomsub = self.create_subscription(Odometry, '/odom', self.odom2tf_callback, 10)

    def odom2tf_callback(self, msg):
        self.t.header.stamp = msg.header.stamp
        self.t.header.frame_id = msg.header.frame_id # odom
        self.t.child_frame_id = "base_footprint"

        self.t.transform.translation.x = msg.pose.pose.position.x
        self.t.transform.translation.y = msg.pose.pose.position.y
        self.t.transform.translation.z = msg.pose.pose.position.z

        self.t.transform.rotation.x = msg.pose.pose.orientation.x
        self.t.transform.rotation.y = msg.pose.pose.orientation.y
        self.t.transform.rotation.z = msg.pose.pose.orientation.z
        self.t.transform.rotation.w = msg.pose.pose.orientation.w

        self.br.sendTransform(self.t)

def main(args=None):
    rclpy.init(args=args)

    tfbr = TFBroadcast()

    rclpy.spin(tfbr)

    tfbr.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()