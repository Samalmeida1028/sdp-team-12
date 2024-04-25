# Author: Arjun Viswanathan, Samuel Almeida
# Date created: 4/17/24
# Date last modified: 4/24/24
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

        self.camcenterpub = self.create_subscription(Float32MultiArray, "/servoxy_angle", self.set_servo_angles, 10)

        self.target_id = 9999

        self.vector = Float32MultiArray()
        self.servo_angles = [0.0, 0.0]
        self.vector.data = [0.0, 0.0]

        self.wait_start_time = time.time()
        self.wait_time = 5.5
        self.time_passed = 0.0

        self.offset = 100.0
        self.state = -1

        self.timer = self.create_timer(0.03, self.send_pan_commands)

        self.get_logger().info("Camera Search node ready!")

    def set_target(self, tmsg : Int32):
        self.target_id = tmsg.data

    def set_servo_angles(self, anglemsg : Float32MultiArray):
        self.servo_angles[0] = anglemsg.data[0]
        self.servo_angles[1] = anglemsg.data[1]

    def update_wait_time(self, spotted_msg : Int32):
        time_now = time.time()
        if spotted_msg.data or self.target_id == 9999:
            self.wait_start_time = time_now

        self.time_passed = time_now - self.wait_start_time
        # self.get_logger().info("Waited {} seconds".format(self.time_passed))

    def send_pan_commands(self):
        if self.time_passed >= self.wait_time:
            match self.state:
                case -1:
                    if self.servo_angles[0] >= 55: self.state = 1
                    else: self.state = -1
                    pass
                case 1:
                    if self.servo_angles[0] <= -80: self.state = -1
                    else: self.state = 1
                    pass

            # self.get_logger().info("Offset: {}".format(self.state * self.offset))

            self.vector.data[0] = self.state * self.offset
            self.cam_pan_pub.publish(self.vector)

def main(args=None):
    rclpy.init(args=args)

    cam_search = CameraSearch()

    rclpy.spin(cam_search)

    cam_search.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()