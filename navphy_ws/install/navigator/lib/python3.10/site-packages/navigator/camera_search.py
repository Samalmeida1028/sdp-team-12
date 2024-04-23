# Author: Arjun Viswanathan
# Date created: 4/17/24
# Date last modified: 4/17/24
# Node that waits for search to come in and pans the camera slowly left to right

import rclpy
from rclpy.node import Node
import time
from std_msgs.msg import Int32, Float32MultiArray

class CameraSearch(Node):
    def __init__(self):
        super().__init__('camera_search')

        self.cam_pan_pub = self.create_publisher(Float32MultiArray, "/marker_position", 1)

        self.targetsub = self.create_subscription(Int32, "/target_id", self.set_target, 10)
        self.target_spotted_sub = self.create_subscription(Int32, "/target_spotted", self.update_wait_time, 10)

        self.target_id = 9999

        self.vector = Float32MultiArray()
        self.vector.data = [0.0, 0.0]

        self.wait_start_time = time.time()
        self.move_time = time.time()
        self.wait_time = 5.5
        self.time_passed = 0.0

        self.actions = [0.75*1080, -0.75*1080, -0.75*1080, 0.75*1080] # left, center, right, center
        self.index = 0

        self.timer = self.create_timer(0.1, self.send_pan_commands)

        self.get_logger().info("Camera Search node ready!")

    def set_target(self, tmsg : Int32):
        self.target_id = tmsg.data

    def update_wait_time(self, spotted_msg : Int32):
        time_now = time.time()
        if spotted_msg.data or self.target_id == 9999:
            self.wait_start_time = time_now

        self.time_passed = time_now - self.wait_start_time
        # self.get_logger().info("Waited {} seconds".format(self.time_passed))

    def send_pan_commands(self):
        if self.time_passed >= self.wait_time:
            if time.time() - self.move_time > 5:
                self.vector.data[0] = self.actions[self.index]
                self.index += 1

                if self.index == 4:
                    self.index = 0

                self.move_time = time.time()

            self.vector.data[0] = round(self.vector.data[0]*0.9, 3)
            self.cam_pan_pub.publish(self.vector)
            # self.get_logger().info("Publishing new target {}".format(self.vector.data[0]))

def main(args=None):
    rclpy.init(args=args)

    cam_search = CameraSearch()

    rclpy.spin(cam_search)

    cam_search.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()