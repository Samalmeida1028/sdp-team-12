# Author: Arjun Viswanathan
# Date created: 4/29/24
# Date last modified: 4/29/24

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from scipy.spatial.transform import Rotation

class RotateDance(Node):
    def __init__(self):
        super().__init__('rotate_dance')

        self.velmsg = Twist()
        self.orientation = 0.0

        self.value = 0.0
        self.can_start_macro = 0
        self.state = 1

        self.canstartsub = self.create_subscription(Int32, "/can_start_macros", self.update_start, 10)
        self.odomsub = self.create_subscription(Odometry, "/odometry/filtered", self.set_orientation, 10)

        self.velpub = self.create_publisher(Twist, "/teleop_vel", 1)
        self.decay_vel_timer = self.create_timer(0.5, self.decay_vel)
        self.publish_vel_timer = self.create_timer(0.05, self.publish_vel)

    def update_start(self, msg : Int32):
        self.can_start_macro = msg.data

    def publish_vel(self):
        if self.can_start_macro:
            match self.state:
                case 1:
                    if self.orientation >= 0.5: self.state = 1
                    else: self.state = -1
                case -1:
                    if self.orientation <= -0.5: self.state = -1
                    else: self.state = 1
            self.value = 0.35 * self.state
        else:
            self.value = 0.0

        self.velmsg.angular.z = self.value
        self.velpub.publish(self.velmsg)

    def set_orientation(self, msg : Odometry):
        self.orientation = Rotation.from_quat([msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,
                                          msg.pose.pose.orientation.z,msg.pose.pose.orientation.w]).as_euler("xyz",degrees=False)[2]

    def decay_vel(self):
        self.value *= 0.9

def main(args=None):
    rclpy.init(args=args)

    rotdance = RotateDance()

    rclpy.spin(rotdance)

    rotdance.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()