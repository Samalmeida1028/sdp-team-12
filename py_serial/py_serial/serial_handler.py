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
from std_msgs.msg import String 

class SerHandler(Node):
    def __init__(self):
        super().__init__('ser_handle')
        self.ser = serial.Serial(
                    '/dev/ttyACM1',
                    baudrate=115200,
                    timeout=0.01)

        self.encoder_publisher = self.create_publisher(Float32MultiArray, "/encoder_data", 1)
        self.imu_publisher = self.create_publisher(Float32MultiArray, "/imu_data", 1)
        self.debug_publisher = self.create_publisher(String, "/encoder_debug", 1)
        self.cmdvel_subscriber = self.create_subscription(Float32MultiArray, "cmd_vel_vectors", self.send_motor_commands, 10)

        timer_period = .05
        self.timer = self.create_timer(timer_period, self.get_encoder_info)

        self.get_logger().info('Initialized timer')

        self.encoder_data = Float32MultiArray()
        self.imu_data = Float32MultiArray()
        self.cmdvel_data = Float32MultiArray()

        self.wheel_radius = 0.0508

    def send_motor_commands(self,msg):
        self.ser.write(bytes(json.dumps(list(msg.data)) + "\n", "utf-8"))

    def get_encoder_info(self):
        msg = []
        while(self.ser.in_waiting): 
            msg = self.ser.readline()
        if msg:
            # print(msg.decode())
            info = json.loads(msg.decode())
            self.encoder_data.data = [info['Encoder']['BL']['Pos'], 
                                    info['Encoder']['FL']['Pos'], 
                                    info['Encoder']['FR']['Pos'], 
                                    info['Encoder']['BL']['Vel'], 
                                    info['Encoder']['FL']['Vel'], 
                                    info['Encoder']['FR']['Vel']]
            
            self.imu_data.data = [info['IMU']['Accel']['x'], info['IMU']['Accel']['y'], info['IMU']['Accel']['z'],
                                    info['IMU']['Gyro']['r'], info['IMU']['Gyro']['p'], info['IMU']['Gyro']['y']]

            self.encoder_publisher.publish(self.encoder_data)
            self.imu_publisher.publish(self.imu_data)

def main(args=None):
    rclpy.init(args=args)

    serhandler = SerHandler()

    rclpy.spin(serhandler)

    serhandler.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()