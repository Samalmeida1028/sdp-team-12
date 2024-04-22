#!/usr/bin/env python3
# Author: Arjun Viswanathan, Samuel Almeida
# Date created: 11/28/23
# Date last modified: 3/30/24
# Description: The serial handler node. Reads and writes to serial only here

# Import ROS specific packages
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray,Int32,Float32
import serial
import json
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import threading
import time
from pathlib import Path

class SerHandler(Node):
    def __init__(self):
        exists = True
        super().__init__('ser_handle')
        self.serial1 = serial.Serial(
                    '/dev/ttyACM1',
                    baudrate=115200,
                    timeout=0.01)
        
        my_file = Path("/dev/ttyACM3")

        if(not my_file.exists()):
            self.get_logger().info("No second device, not running")
            exists = False
        if(exists):
            self.serial2 = serial.Serial(
                        '/dev/ttyACM3',
                        baudrate=115200,
                        timeout=0.01)
        print("WESTARTING UP BABY AAAHHHHHH")

            
        self.serial1.write(bytearray(json.dumps("Type") + "\n",encoding="utf-8"))
        if exists:
            self.serial2.write(bytearray(json.dumps("Type") + "\n",encoding="utf-8"))
        while self.serial1.out_waiting > 0:
            pass
        # # # print(self.serial1)
        response = self.serial1.readline()
        if exists:
            _ = self.serial2.readline()
        
        print(response)
        # for i in range(100):
            # # print(response)
        if response:
            # print("Hi")
            # print(response.decode())
            if json.loads(response.decode()) == "nav":
                # print("NAV")
                self.nav_serial = self.serial1
                if(exists):
                    self.target_serial = self.serial2
                else:
                    self.target_serial = None
            elif json.loads(response.decode()) == "tracking":
                if(exists):
                    self.nav_serial = self.serial2
                else:
                    self.nav_serial = None
                self.target_serial = self.serial1
        
        # self.tracking_dict = {"centering":0,"offset":[0.0,0.0]}

        self.cmdvel_subscriber = self.create_subscription(Twist, "/cmd_vel_nav", self.set_nav, 10)
        self.teleop_sub = self.create_subscription(Twist, "/teleop_vel", self.set_teleop, 10)
        self.encoder_publisher = self.create_publisher(Float32MultiArray, "/encoder_data", 1)
        self.imu_publisher = self.create_publisher(Float32MultiArray, "/imu_data", 1)
        self.servo_xy_publisher = self.create_publisher(Float32MultiArray, "/servoxy_angle", 1)
        self.marker_subscriber = self.create_subscription(Float32MultiArray, '/marker_position', self.update_tracking,10)
        self.recording_time_subscriber = self.create_subscription(Float32, '/recording_time', self.update_recording_time,1)
        self.recording_max_time_subscriber = self.create_subscription(Int32, '/recording_max_time', self.update_max_recording_time,1)
        self.recording_state_subscriber = self.create_subscription(Int32, '/recording', self.update_recording_state,1)
        self.target_seen = self.create_subscription(Int32, "/target_spotted", self.check_target, 10)
        self.is_centered = False
        self.current_servo_angle = None

        timer_period = .02        
        self.timer = self.create_timer(timer_period, self.run_serial)
        self.timer2 = self.create_timer(timer_period, self.send_motor_commands)
        self.timer3 = self.create_timer(timer_period,self.move_servos)
        self.last_time = time.time()
        self.current_time = time.time()
        self.teleop_time = time.time()
        self.recording_time = 1
        self.recording_max_time = 1
        self.recording_state = 0

        self.get_logger().info('Initialized timer')

        self.encoder_data = Float32MultiArray()
        self.imu_data = Float32MultiArray()
        self.cmdvel_data = Float32MultiArray()
        self.current_angle = Float32MultiArray()
        for i in range(2):
            self.current_angle.data.append(float(0.0))

        self.wheel_radius = 0.0497
        self.x = 0.0
        self.z = 0.0

        self.debug_publisher = self.create_publisher(String, "/encoder_debug", 1)

    def update_recording_time(self,msg):
        self.recording_time = msg.data
    def update_max_recording_time(self,msg):
        self.recording_max_time = msg.data
    def update_recording_state(self,msg):
        self.recording_state = msg.data

    def send_motor_commands(self):
        if(self.nav_serial):
            self.nav_serial.write(bytes(json.dumps([self.x, self.z]) + "\n", "utf-8"))

        if time.time() - self.teleop_time > 3.0:
            self.allow_nav = True

    def run_serial(self):
        self.get_encoder_info()

    def set_teleop(self, teleop_msg : Twist):
        self.teleop_time = time.time()
        self.allow_nav = False
        
        self.x = teleop_msg.linear.x
        self.z = teleop_msg.angular.z

    def set_nav(self, nav_msg : Twist):
        if self.allow_nav:
            self.x = nav_msg.linear.x
            self.z = nav_msg.angular.z

    def get_encoder_info(self):
        msg = []
        if(self.nav_serial):
            while(self.nav_serial.in_waiting): 
                msg = self.nav_serial.readline()
        if msg:
            # # pring(msg.decode())
            info = dict(json.loads(msg.decode()))
            # info = json.loads(info_ser)
            self.encoder_data.data = [info['Encoder']['BL']['Pos'], 
                                    info['Encoder']['FL']['Pos'], 
                                    info['Encoder']['FR']['Pos'],
                                    info['Encoder']['BR']['Pos'], 
                                    info['Encoder']['BL']['Vel'], 
                                    info['Encoder']['FL']['Vel'], 
                                    info['Encoder']['FR']['Vel'],
                                    info['Encoder']['BR']['Vel'],]
            
            self.imu_data.data = [info['IMU']['Accel']['x'], info['IMU']['Accel']['y'], info['IMU']['Accel']['z'],
                                    info['IMU']['Gyro']['r'], info['IMU']['Gyro']['p'], info['IMU']['Gyro']['y']]

            self.encoder_publisher.publish(self.encoder_data)
            self.imu_publisher.publish(self.imu_data)

    def update_tracking(self,msg):
        self.current_servo_angle = list(msg.data)
        # print(msg)
        self.led_state = 0
        # print(self.recording_state)
        if(self.recording_state == 1):
            print(self.recording_time/float(self.recording_max_time))
            if(self.recording_time/float(self.recording_max_time)) > .7:
                self.led_state = 2
            else:
                # print("AAAAAAHHHHHHHHHHH")
                self.led_state = 1

    def move_servos(self):
        if(self.current_servo_angle):
            self.target_serial.write(bytearray(json.dumps([0,0,self.led_state]) + "\n",encoding="utf-8"))
            self.is_centered = False
            self.publish_servo_angles()
            self.current_servo_angle[0] += round((self.angle[0]/180.0)*1920)
            self.current_servo_angle[1] -= round((self.angle[1]/180.0)*1080)
        self.get_logger().warn(str(self.current_servo_angle) + ' ' + str(self.current_angle.data))

    def publish_servo_angles(self):
      serial_in = self.target_serial.readline()
      if serial_in:
        # print(json.loads(serial_in.decode('utf-8')))
        self.angle = list(json.loads(serial_in.decode('utf-8')))
        # # print(self.angle)
        # # print(self.angle[0])
        self.current_angle.data[0] = float(self.angle[0])
        self.current_angle.data[1] = float(self.angle[1]) 
        # # print(self.current_angle)
        self.servo_xy_publisher.publish(self.current_angle)

    def check_target(self,msg):
        self.current_time = time.time()
        result = msg.data
        # print("checking",result)
        # self.get_logger().info("Center value: {}, Msg: {}".format(self.is_centered, result))

        if(result == 0 and self.current_time-self.last_time > 5 and not self.is_centered):
            self.get_logger().info("centering")
            self.target_serial.write(bytearray(json.dumps("center")+ "\n",encoding="utf-8"))
            self.last_time = self.current_time
            self.is_centered = True
        elif(result == 1):
            self.is_centered = False
            self.last_time = self.current_time

def main(args=None):
    rclpy.init(args=args)

    serhandler = SerHandler()

    rclpy.spin(serhandler)

    serhandler.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()