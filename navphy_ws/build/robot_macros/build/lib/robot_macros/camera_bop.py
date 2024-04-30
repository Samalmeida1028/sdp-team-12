# Author: Arjun Viswanathan
# Date created: 4/29/24
# Date last modified: 4/29/24

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, Float32MultiArray

class CameraBop(Node):
    def __init__(self):
        super().__init__('camera_bop')

        self.cam_pan_pub = self.create_publisher(Float32MultiArray, "/marker_position", 1)
        self.camcenterpub = self.create_subscription(Float32MultiArray, "/servoxy_angle", self.set_servo_angles, 10)
        self.listener = self.create_subscription(Int32, "/can_start_macros", self.update_start, 10)

        self.vector = Float32MultiArray()
        self.servo_angles = [0.0, 0.0]
        self.vector.data = [0.0, 0.0]

        self.offset = 600.0
        self.state = -1
        self.can_start = 0

        self.timer = self.create_timer(0.03, self.send_pan_commands)

        self.get_logger().info("Camera bop node ready!")

    def update_start(self, msg : Int32):
        self.can_start = msg.data

    def set_servo_angles(self, anglemsg : Float32MultiArray):
        self.servo_angles[0] = anglemsg.data[0]
        self.servo_angles[1] = anglemsg.data[1]

    def send_pan_commands(self):
        if self.can_start:
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
        else:
            self.vector.data[1] = 0.0

        self.cam_pan_pub.publish(self.vector)

def main(args=None):
    rclpy.init(args=args)

    cambop = CameraBop()

    rclpy.spin(cambop)

    cambop.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()