#!/usr/bin/env python3
# Author: Arjun Viswanathan
# Date created: 11/9/23
# Date last modified: 11/26/23
# Description: Send cmd vel values from the nav stack over serial to the pico

from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
import math
import serial
import json

class CmdVelSub(Node):
    def __init__(self):
        super().__init__('cmdvelsub')
        #self.s = serial.Serial("/dev/ttyACM1", 115200)
        self.poseSub = self.create_subscription(Twist, '/cmd_vel_nav', self.cmdvel_callback, 10)

    def cmdvel_callback(self, msg):
        tvel = msg.linear.x
        rvel = msg.angular.z
        v = [tvel * math.sin(rvel), tvel * math.cos(rvel)]

        print("Vx: {}, Vy: {}".format(v[0], v[1]))

        #self.s.write(bytearray(json.dumps(v) + '\n', 'utf-8'))

def main(args=None):
    rclpy.init(args=args)

    cmvsub = CmdVelSub()

    rclpy.spin(cmvsub)

    cmvsub.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()