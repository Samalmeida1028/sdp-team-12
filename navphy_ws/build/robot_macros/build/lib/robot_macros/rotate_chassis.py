# Author: Arjun Viswanathan
# Date created: 4/29/24
# Date last modified: 4/29/24
# Description: rotate the chassis every now and then

import rclpy
from rclpy.node import Node
import time
from geometry_msgs.msg import Twist

class RotateChassis(Node):
    def __init__(self):
        super().__init__('rotate_chassis')

        self.velmsg = Twist()

        self.velpub = self.create_publisher(Twist, "/teleop_vel", 1)
        self.veltimer = self.create_timer(10, self.send_twist_messages)

    def send_twist_messages(self):
        self.velmsg.angular.z = 2.0
        self.velpub.publish(self.velmsg)

def main(args=None):
    rclpy.init(args=args)
    rot_node = RotateChassis()
    rclpy.spin(rot_node)
    rot_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()