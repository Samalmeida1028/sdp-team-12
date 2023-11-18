#!/usr/bin/env python3
# Author: Arjun Viswanathan
# Date created: 11/17/23
# Date last modified: 11/17/23
# Description: Use encoder values and turn into Odometry messages

from std_msgs.msg import Float32MultiArray
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
import math
import serial
import json
from scipy.spatial.transform import Rotation

class Encoder2Odom(Node):
    def __init__(self):
        super().__init__('enc2odom')

        self.robot_radius = 0.18 # m
        self.wheel_radius = 0.0508 # m
        self.phi = 0

        self.s = serial.Serial("/dev/ttyACM1", 115200)
        self.odompub = self.create_publisher(Odometry, '/wheel/odom', 10)

        time_period = 0.5
        self.timer = self.create_timer(time_period, self.timer_callback)

    def timer_callback(self):
        smsg = self.s.readline()
        if smsg:
            msg = Odometry()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "odom"

            encoder_array = json.loads(smsg.decode('utf-8'))
            print(encoder_array)

            lf_rev = encoder_array[2]
            rf_rev = encoder_array[1]

            lb_rev = lf_rev
            rb_rev = encoder_array[0]

            l_dist = (lf_rev + lb_rev) / 2
            r_dist = (rf_rev + rb_rev) / 2
            d = (l_dist + r_dist) / 2

            self.phi += ((r_dist - l_dist) / (2*self.robot_radius))
            rot = Rotation.from_euler('xyz', [0, 0, self.phi], degrees=False)
            rot_quat = rot.as_quat() # convert angle to quaternion format

            msg.pose.pose.position.x += d*math.cos(self.phi)
            msg.pose.pose.position.y += d*math.sin(self.phi)

            msg.pose.pose.orientation.x = rot_quat[0]
            msg.pose.pose.orientation.y = rot_quat[1]
            msg.pose.pose.orientation.z = rot_quat[2]
            msg.pose.pose.orientation.w = rot_quat[3]

            self.odompub.publish(msg)

def main(args=None):
    rclpy.init(args=args)

    encsub = Encoder2Odom()

    rclpy.spin(encsub)

    encsub.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()