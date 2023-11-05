# Author: Arjun Viswanathan
# Date created: 11/5/23
# Date last modified: 11/5/23
# Description: a keyboard teleop node that publisher twist messages to cmd_vel topic

import rclpy
import rclpy.Node as Node
from geometry_msgs.msg import Twist
import pygame

class KeyboardTeleop(Node):
    def __init__(self):
        super().__init__('keyboard_teleop')
        self.twistpub = self.create_publisher(Twist, '/cmd_vel', 10)
        time_period = 0.5
        self.timer = self.create_timer(time_period, self.publisher_callback)

    def publisher_callback(self):
        keys = pygame.key.get_pressed()
        command = Twist()

        if keys[pygame.K_w]:
            if keys[pygame.K_a]:
                command.linear.x = 0.25
                command.angular.z = 0.25
            elif keys[pygame.K_d]:
                command.linear.x = 0.25
                command.angular.z = -0.25
            else:
                command.linear.x = 0.25
        elif keys[pygame.K_s]:
            if keys[pygame.K_a]:
                command.linear.x = -0.25
                command.angular.z = 0.25
            elif keys[pygame.K_d]:
                command.linear.x = -0.25
                command.angular.z = -0.25
            else:
                command.linear.x = -0.25
        elif keys[pygame.K_a]:
            command.angular.z = 0.25
        elif keys[pygame.K_d]:
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