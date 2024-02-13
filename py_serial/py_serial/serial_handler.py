#!/usr/bin/env python3
# Author: Arjun Viswanathan, Samuel Almeida
# Date created: 11/28/23
# Date last modified: 2/13/24
# Description: The serial handler node. Reads and writes to serial only here

# Import ROS specific packages
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import serial
import json
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import threading

class SerHandler(Node):
    def __init__(self):
        super().__init__('ser_handle')
        self.serial1 = serial.Serial(
                    '/dev/ttyACM1',
                    baudrate=115200,
                    timeout=0.01)
        self.serial2 = serial.Serial(
                    '/dev/ttyACM3',
                    baudrate=115200,
                    timeout=0.01)
        
        self.serial1.write(bytearray(json.dumps("Type") + "\n",encoding="utf-8"))
        self.serial2.write(bytearray(json.dumps("Type") + "\n",encoding="utf-8"))
        # # while self.serial1.out_waiting > 0:
        # #     pass
        # # # print(self.serial1)
        response = self.serial1.readline()
        _ = self.serial2.readline()
        # # print(response)
        # for i in range(100):
            # print(response)
        if response:
            if json.loads(response.decode()) == "nav":
                print("NAV")
                self.nav_serial = self.serial1
                self.target_serial = self.serial2
            elif json.loads(response.decode()) == "tracking":
                self.nav_serial = self.serial2
                self.target_serial = self.serial1

        self.encoder_publisher = self.create_publisher(Float32MultiArray, "/encoder_data", 1)
        self.imu_publisher = self.create_publisher(Float32MultiArray, "/imu_data", 1)
        self.servo_xy_publisher = self.create_publisher(Float32MultiArray, "/servoxy_angle", 1)
        # self.cmdvel_subscriber = self.create_subscription(Float32MultiArray, "cmd_vel_vectors", self.send_motor_commands, 10)
        self.marker_subscriber = self.create_subscription(Float32MultiArray, '/marker_position', self.send_marker_position,10)
        self.keyboard_sub = self.create_subscription(Twist, "/cmd_vel", self.send_teleop_commands, 10)

        timer_period = .05
        self.timer = self.create_timer(timer_period, self.get_encoder_info)

        self.get_logger().info('Initialized timer')

        self.encoder_data = Float32MultiArray()
        self.imu_data = Float32MultiArray()
        self.cmdvel_data = Float32MultiArray()
        self.current_angle = Float32MultiArray()
        for i in range(2):
            self.current_angle.data.append(float(0.0))

        self.wheel_radius = 0.0497

        self.debug_publisher = self.create_publisher(String, "/encoder_debug", 1)

    def send_motor_commands(self,msg):
        self.nav_serial.write(bytes(json.dumps(list(msg.data)) + "\n", "utf-8"))

    def send_teleop_commands(self, msg):
        x = msg.linear.x
        z = msg.angular.z
        self.nav_serial.write(bytearray(json.dumps([x,z]) + "\n", encoding="utf-8"))

    def get_encoder_info(self):
        msg = []
        while(self.nav_serial.in_waiting): 
            msg = self.nav_serial.readline()
        if msg:
            # print(msg.decode())
            info = dict(json.loads(msg.decode()))
            # info = json.loads(info_ser)
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

    def send_marker_position(self,msg):
        print(msg)
        self.target_serial.write(bytearray(json.dumps(list(msg.data)) + "\n",encoding="utf-8"))
        self.publish_servo_angles()

    def publish_servo_angles(self):
      serial_in = self.target_serial.readline()
      if serial_in:
        self.angle = list(json.loads(serial_in.decode('utf-8')))
        print(self.angle)
        print(self.angle[0])
        self.current_angle.data[0] = float(self.angle[0])
        self.current_angle.data[1] = float(self.angle[1]) 
        print(self.current_angle)
        self.servo_xy_publisher.publish(self.current_angle)

def main(args=None):
    rclpy.init(args=args)

    serhandler = SerHandler()

    rclpy.spin(serhandler)

    serhandler.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()