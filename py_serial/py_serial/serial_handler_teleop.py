#!/usr/bin/env python3
# Author: Arjun Viswanathan, Samuel Almeida
# Date created: 11/28/23
# Date last modified: 11/30/23
# Description: The serial handler node. Reads and writes to serial only here

# Import ROS specific packages
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import serial
import json
import pygame

class SerHandlerTeleop(Node):
    def __init__(self):
        super().__init__('ser_handle_teleop')
        
        self.ser = serial.Serial(
                    '/dev/ttyACM2',
                    baudrate=115200,
                    timeout=0.01)

        pygame.joystick.init()
        pygame.init()

        self.joysticks = []

        for joy in range(pygame.joystick.get_count()):
            self.joysticks.append(pygame.joystick.Joystick(joy))
            self.joysticks[joy].init()

        self.encoder_publisher = self.create_publisher(Float32MultiArray, "encoder_data", 1)
        #self.cmdvel_subscriber = self.create_subscription(Float32MultiArray, "cmd_vel_vectors", self.send_motor_commands, 10)

        timer_period = .005
        self.timer1 = self.create_timer(timer_period, self.get_encoder_info)
        self.timer2 = self.create_timer(timer_period, self.send_motor_commands)

        self.get_logger().info('Initialized timer')

        self.encoder_data = Float32MultiArray()
        #self.cmdvel_data = Float32MultiArray()

    def send_motor_commands(self):
        pygame.event.get()
        for stick in self.joysticks:
            self.ser.write(bytearray(json.dumps([stick.get_axis(0), stick.get_axis(1)]) + "\n", encoding="utf-8"))
        
        #self.ser.write(bytes(json.dumps(list(msg.data)) + "\n", "utf-8"))

    def get_encoder_info(self):
        msg = self.ser.readline()
        if msg:
            encoder_info = json.loads(msg.decode())
            print(encoder_info)
            self.encoder_data.data = encoder_info
            self.encoder_publisher.publish(self.encoder_data)

def main(args=None):
    rclpy.init(args=args)

    serhandlertel = SerHandlerTeleop()

    rclpy.spin(serhandlertel)

    serhandlertel.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()