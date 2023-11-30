#!/usr/bin/env python3
# Author: Arjun Viswanathan, Samuel Almeida
# Date created: 11/17/23
# Date last modified: 11/29/23
# Description: Use encoder values and turn into Odometry messages

from nav_msgs.msg import Odometry
from std_msgs.msg import Float32MultiArray
import rclpy
from rclpy.node import Node
import math
import serial
from scipy.spatial.transform import Rotation
from geometry_msgs.msg import Quaternion

class Encoder2Odom(Node):
    def __init__(self):
        self.count = 0
        super().__init__('enc2odom')
        self.current_time = self.get_clock().now().to_msg().sec
        self.last_time = 0

        self.robot_radius = 0.18 # m
        self.wheel_radius = 0.0508 # m
        self.phi = 0

        self.s = serial.Serial("/dev/ttyACM1", 115200)
        self.odompub = self.create_publisher(Odometry, '/wheel/odom', 10)
        self.encoder_sub = self.create_subscription(Float32MultiArray, "encoder_data", self.encoder_to_odom, 10)

        time_period = 0.16
        self.timer = self.create_timer(time_period, self.timer_callback)
        self.msg = Odometry()
        self.msg.header.frame_id = "odom"
        self.msg.child_frame_id = "base_footprint"


    def timer_callback(self):
        # print("Trying serial"
        self.odompub.publish(self.msg)

    def encoder_to_odom(self,msg):
        dt = (self.current_time-self.last_time)
        encoder_array = msg.data

        lf_rev = encoder_array[2]
        rf_rev = encoder_array[1]

        lb_rev = encoder_array[0]
        rb_rev = rf_rev

        l_dist = (lf_rev + lb_rev) / 2
        r_dist = (rf_rev + rb_rev) / 2
        d = (l_dist + r_dist) / 2

        self.phi += ((r_dist - l_dist) / (2*self.robot_radius))
        rot = Rotation.from_euler('xyz', [0, 0, self.phi], degrees=False)
        rot_quat = rot.as_quat() # convert angle to quaternion format

        dx = d*math.cos(self.phi)*dt
        dy = d*math.sin(self.phi)*dt
        self.count+=1

        self.msg.pose.pose.position.x += dx
        self.msg.pose.pose.position.y += dy
        self.msg.pose.pose.position.z = 0.0
        self.msg.pose.pose.orientation = Quaternion(x=rot_quat[0],y=rot_quat[1],z=rot_quat[2],w=rot_quat[3])
        # print(self.msg.pose)
        self.msg.header.stamp = self.get_clock().now().to_msg()

def main(args=None):
    rclpy.init(args=args)

    encsub = Encoder2Odom()

    rclpy.spin(encsub)

    encsub.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()