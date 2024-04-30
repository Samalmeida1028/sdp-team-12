# Author: Arjun Viswanathan
# Date created: 4/29/24
# Date last modified: 4/29/24
# Description: rotate the chassis every now and then

import rclpy
from rclpy.node import Node
import time
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32

class PlayMusic(Node):
    def __init__(self):
        super().__init__('play_music')

        self.velmsg = Twist()

        self.velpub = self.create_publisher(Twist, "/teleop_vel", 1)
        self.veltimer = self.create_timer(0.5, self.send_twist_messages)
        self.listener = self.create_subscription(Int32, "/can_start_macros", self.update_start, 10)

        self.can_start = 0
        self.notes = [0.026, 0.029, 0.032, 0.035, 0.032, 0.029, 0.026]
        self.note_index = 0

        self.get_logger().info("Play Music macro ready!")

    def update_start(self, msg : Int32):
        self.can_start = msg.data

    def send_twist_messages(self):
        if self.can_start:
            self.velmsg.linear.x = self.notes[self.note_index]
        else:
            self.velmsg.linear.x = 0.0

        self.note_index += 1
        
        if self.note_index == len(self.notes):
            self.note_index = 0

        self.velpub.publish(self.velmsg)

def main(args=None):
    rclpy.init(args=args)
    rot_node = PlayMusic()
    rclpy.spin(rot_node)
    rot_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()