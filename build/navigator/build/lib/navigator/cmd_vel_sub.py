#!/usr/bin/env python3
# Author: Arjun Viswanathan, Samuel Almeida
# Date created: 11/9/23
# Date last modified: 11/28/23
# Description: Publish the motor control values for pico after reading cmd vel nav values from nav stack

from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray
import rclpy
from rclpy.node import Node

class CmdVelSub(Node):
    def __init__(self):
        super().__init__('cmdvelsub')
        # self.s = serial.Serial("/dev/ttyACM1", 115200,timeout=.01)
        self.poseSub = self.create_subscription(Twist, '/cmd_vel_nav', self.cmdvel_callback, 10)
        self.vector_publisher = self.create_publisher(Float32MultiArray, "cmd_vel_vectors", 1)
        timer_period = .01
        self.timer = self.create_timer(timer_period,self.vector_pub)
        self.v = Float32MultiArray()

    def cmdvel_callback(self, msg):
        print(msg)
        tvel = msg.linear.x
        rvel = msg.angular.z

        self.v.data = [tvel, rvel]
    
    def vector_pub(self):
        self.vector_publisher.publish(self.v)

def main(args=None):
    rclpy.init(args=args)

    cmvsub = CmdVelSub()

    rclpy.spin(cmvsub)

    cmvsub.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()