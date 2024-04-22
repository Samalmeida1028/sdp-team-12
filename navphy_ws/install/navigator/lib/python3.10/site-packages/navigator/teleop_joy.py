import pygame
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class TeleopJoy(Node):
    def __init__(self):
        super().__init__('teleop_joy')
        pygame.init()
        pygame.joystick.init()

        if pygame.joystick.get_count() == 0:
            print("No joystick detected.")

        self.joystick = pygame.joystick.Joystick(0)
        self.joystick.init()

        self.get_logger().info("Initialized joystick:", self.joystick.get_name())

        self.speedmsg = Twist()
        self.cmdvelpub = self.create_publisher(Twist, "/teleop_vel", 10)
        self.timerfunc = self.create_timer(0.5, self.send_speeds)

        self.get_logger().info("Teleop Joystick node ready!")

    def send_speeds(self):
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                quit()

        yaxis = self.joystick.get_axis(0)
        xaxis = self.joystick.get_axis(1)

        self.get_logger().info("Linear: {}, Angular: {}".format(yaxis, xaxis))

        if (yaxis > 0.2 and yaxis < 0.5) or (yaxis < -0.2 and yaxis > -0.5) or (xaxis > 0.2 and xaxis < 0.5) or (xaxis < -0.2 and xaxis > -0.5):
            self.speedmsg.linear.x = yaxis
            self.speedmsg.angular.z = xaxis
        else:
            self.speedmsg.linear.x = 0
            self.speedmsg.angular.z = 0

        self.cmdvelpub.publish(self.speedmsg)

def main(args=None):
    rclpy.init(args=args)

    joy = TeleopJoy()

    rclpy.spin(joy)

    joy.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()