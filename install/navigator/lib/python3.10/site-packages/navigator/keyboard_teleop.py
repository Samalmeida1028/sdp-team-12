#!/usr/bin/env python3
# Author: Arjun Viswanathan
# Date created: 11/5/23
# Date last modified: 11/5/23
# Description: a keyboard teleop node that publisher twist messages to cmd_vel topic

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
# import keyboard as kb
import sys, tty, termios

class KeyboardTeleop(Node):
    def __init__(self):
        super().__init__('keyboard_teleop')
        self.twistpub = self.create_publisher(Twist, '/cmd_vel', 10)
        time_period = 0.5
        self.timer = self.create_timer(time_period, self.publisher_callback)
        self.key = self.create_timer(time_period, self.getkeystroke)

        print("Initialized")

    def publisher_callback(self):
        command = Twist()

        # command.linear.x = 0.0
        # command.angular.z = 0.0

        # if kb.is_pressed('w'):
        #     if kb.is_pressed('a'):
        #         print("Curving left...")
        #         command.linear.x = 0.25
        #         command.angular.z = 0.25
        #     elif kb.is_pressed('d'):
        #         print("Curving right...")
        #         command.linear.x = 0.25
        #         command.angular.z = -0.25
        #     else:
        #         print("Moving forward...")
        #         command.linear.x = 0.25
        # elif kb.is_pressed('s'):
        #     if kb.is_pressed('a'):
        #         print("Curving right...")
        #         command.linear.x = -0.25
        #         command.angular.z = 0.25
        #     elif kb.is_pressed('d'):
        #         print("Curving left...")
        #         command.linear.x = -0.25
        #         command.angular.z = -0.25
        #     else:
        #         print("Moving back...")
        #         command.linear.x = -0.25
        # elif kb.is_pressed('a'):
        #     print("Rotating left...")
        #     command.angular.z = 0.25
        # elif kb.is_pressed('d'):
        #     print("Rotating right...")
        #     command.angular.z = -0.25

        if self.key == 'w' or self.key == 'W':
            print("Moving forward...")
            command.linear.x = 0.25
        elif self.key == 'a' or self.key == 'A':
            print("Rotating left...")
            command.angular.z = 0.25
        elif self.key == 'd' or self.key == 'D':
            print("Rotating right...")
            command.angular.z = -0.25
        elif self.key == 's' or self.key == 'S':
            print("Moving back...")
            command.linear.x = -0.25

        self.twistpub.publish(command)

    def getkeystroke(self):
        fd=sys.stdin.fileno()
        old_settings=termios.tcgetattr(fd)

        try:
            tty.setraw(sys.stdin.fileno())
            ch=sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd,termios.TCSADRAIN,old_settings)

        return ch

def main(args=None):
    rclpy.init(args=args)

    keyboard_teleop = KeyboardTeleop()
    
    rclpy.spin(keyboard_teleop)

    keyboard_teleop.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()