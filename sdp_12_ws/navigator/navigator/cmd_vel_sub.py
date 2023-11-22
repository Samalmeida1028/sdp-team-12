#!/usr/bin/env python3
# Author: Arjun Viswanathan
# Date created: 11/9/23
# Date last modified: 11/9/23
# Description: Send cmd vel values from the nav stack over serial to the pico

from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
import serial

class CmdVelSub(Node):
    def __init__(self):
        super().__init__('cmdvelsub')

        self.robot_radius = 0.15

        #self.s = serial.Serial("/dev/ttyUSB0", 115200)
        self.poseSub = self.create_subscription(Twist, '/cmd_vel_nav', self.cmdvel_callback, 10)

    def cmdvel_callback(self, msg):
        tvel = msg.linear.x
        rvel = msg.angular.z

        t_rvel = self.robot_radius * rvel

        #print("Linear Velocity: {}, Angular Velocity: {}".format(tvel, rvel))

        left_speed = tvel - t_rvel
        right_speed = tvel + t_rvel

        print("Left motor speed: {}, Right motor speed: {}".format(left_speed, right_speed))

        #self.s.write(bytes(str(tvel) + "," + str(rvel), 'ascii'))

def main(args=None):
    rclpy.init(args=args)

    cmvsub = CmdVelSub()

    rclpy.spin(cmvsub)

    cmvsub.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()