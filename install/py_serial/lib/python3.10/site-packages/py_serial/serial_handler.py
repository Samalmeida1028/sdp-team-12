#!/usr/bin/env python3
# Author: Arjun Viswanathan, Samuel Almeida
# Date created: 11/28/23
# Date last modified: 11/28/23
# Description: The serial handler node. Reads and writes to serial only here

# Import ROS specific packages
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import serial
import json

class SerHandler(Node):
    def __init__(self):
        super().__init__('ser_handle')
        self.ser = serial.Serial(
                    '/dev/ttyACM1',
                    baudrate=115200,
                    timeout=0.01)

        self.encoder_publisher = self.create_publisher(Float32MultiArray, "encoder_data", 1)
        self.cmdvel_subscriber = self.create_subscription(Float32MultiArray, "cmd_vel_vectors", self.send_motor_commands, 10)

        timer_period = .005
        self.timer = self.create_timer(timer_period, self.get_encoder_info)

        self.get_logger().info('Initialized timer')

        self.encoder_data = Float32MultiArray()
        self.cmdvel_data = Float32MultiArray()

    def send_motor_commands(self,msg):
        self.ser.write(bytes(json.dumps(list(msg.data)) + "\n", "utf-8"))

    def get_encoder_info(self):
        msg = self.ser.readline()
        if msg:
            encoder_info = json.loads(msg.decode())
            print(encoder_info)
            self.encoder_data.data = encoder_info
            self.encoder_publisher.publish(self.encoder_data)

def main(args=None):
    rclpy.init(args=args)

    serhandler = SerHandler()

    rclpy.spin(serhandler)

    serhandler.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()