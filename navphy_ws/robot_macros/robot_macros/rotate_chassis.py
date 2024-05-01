# Author: Arjun Viswanathan, Samuel Almeida
# Date created: 4/26/24
# Date last modified: 4/26/24
# Rotate macro: rotates the robot 360 degrees fast

import rclpy
from rclpy.node import Node
import time
from geometry_msgs.msg import Twist

class RotateChassis(Node):
    def __init__(self):
        super().__init__('rotate_chassis')

        self.velpub = self.create_publisher(Twist, "/teleop_vel", 1)
        self.timer = self.create_timer(10, self.send_velocity_command)

        self.velmsg = Twist()
        self.direction = 1

        self.get_logger().info("Running rotate chassis macro")

    def send_velocity_command(self):
        self.velmsg.angular.z = 2.0 * self.direction
        self.velpub.publish(self.velmsg)

        time.sleep(3)

        self.velmsg.angular.z = 0.0
        self.velpub.publish(self.velmsg)
        
        self.direction *= -1

def main(args=None):
    rclpy.init(args=args)

    rotchas = RotateChassis()

    rclpy.spin(rotchas)

    rotchas.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()