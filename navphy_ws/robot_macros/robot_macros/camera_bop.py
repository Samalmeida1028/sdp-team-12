# Author: Arjun Viswanathan, Samuel Almeida
# Date created: 4/26/24
# Date last modified: 4/26/24
# Bop robot macro

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

class CameraBop(Node):
    def __init__(self):
        super().__init__('camera_bop')

        self.cam_pan_pub = self.create_publisher(Float32MultiArray, "/marker_position", 1)
        self.camcenterpub = self.create_subscription(Float32MultiArray, "/servoxy_angle", self.set_servo_angles, 10)

        self.vector = Float32MultiArray()
        self.servo_angles = [0.0, 0.0]
        self.vector.data = [0.0, 0.0]

        self.offset = 640.0
        self.state = -1

        self.timer = self.create_timer(0.03, self.send_pan_commands)

        self.get_logger().info("Camera Bop node ready!")

    def send_pan_commands(self):
        match self.state:
            case -1:
                if self.servo_angles[1] <= -35: self.state = 1
                else: self.state = -1
                pass
            case 1:
                if self.servo_angles[1] >= 25: self.state = -1
                else: self.state = 1
                pass

        # self.get_logger().info("Offset: {}".format(self.state * self.offset))

        self.vector.data[1] = self.state * self.offset
        self.cam_pan_pub.publish(self.vector)

def main(args=None):
    rclpy.init(args=args)

    cam_bop = CameraBop()

    rclpy.spin(cam_bop)

    cam_bop.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()