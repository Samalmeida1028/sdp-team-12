#!/usr/bin/env python3
# Author: Arjun Viswanathan
# Date created: 11/5/23
# Date last modified: 11/9/23
# Description: a keyboard teleop node that publisher twist messages to cmd_vel topic

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from pynput import keyboard as kb

class KeyboardTeleop(Node):
    def __init__(self):
        super().__init__('keyboard_teleop')

        print("Initializing publisher...")
        self.twistpub = self.create_publisher(Twist, '/cmd_vel', 10)

        print("Initializing keyboard listener...")
        self.key = ''
        self.listener = kb.Listener(
            on_press=self.on_press,
            on_release=self.on_release
        )
        self.listener.start()

        print("Initializing timer...")
        time_period = 0.5
        self.timer = self.create_timer(time_period, self.publisher_callback)

        print("Ready!")

    def on_press(self, key):
        print("Pressed {}".format(key.char))
        self.key = key.char

    def on_release(self, key):
        self.key = ''

    def publisher_callback(self):
        command = Twist()

        if self.key == 'w':
            if self.key == 'a':
                print("Curving left...")
                command.linear.x = 0.25
                command.angular.z = 0.25            
            elif self.key == 'd':
                print("Curving right...")
                command.linear.x = 0.25
                command.angular.z = -0.25
            else:
                print("Moving forward...")
                command.linear.x = 0.25
        elif self.key == 's':
            if self.key == 'a':
                print("Curving right...")
                command.linear.x = -0.25
                command.angular.z = 0.25
            elif self.key == 'd':
                print("Curving left...")
                command.linear.x = -0.25
                command.angular.z = -0.25
            else:
                print("Moving back...")
                command.linear.x = -0.25
        elif self.key == 'a':
            print("Rotating left...")
            command.angular.z = 0.25
        elif self.key == 'd':
            print("Rotating right...")
            command.angular.z = -0.25

        self.twistpub.publish(command)

def main(args=None):
    rclpy.init(args=args)

    keyboard_teleop = KeyboardTeleop()
    
    rclpy.spin(keyboard_teleop)

    keyboard_teleop.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()