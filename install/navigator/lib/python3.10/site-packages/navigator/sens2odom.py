#!/usr/bin/env python3
# Author: Arjun Viswanathan, Samuel Almeida
# Date created: 11/17/23
# Date last modified: 2/4/23
# Description: Use encoder values and turn into Odometry messages

from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32MultiArray
import rclpy
from rclpy.node import Node
import math
from scipy.spatial.transform import Rotation
from geometry_msgs.msg import Quaternion
from std_msgs.msg import String

class Sensor2Odom(Node):
    def __init__(self):
        self.count = 0
        super().__init__('sens2odom')
        self.current_time = self.get_clock().now().to_msg().sec

        self.wheel_sep = 0.3683 # m
        self.wheel_radius = 0.0476 # m
        self.phi = 0

        self.odompub = self.create_publisher(Odometry, '/wheel/odom', 10)
        self.imupub = self.create_publisher(Imu, "/imu_odom", 10)

        self.encoder_sub = self.create_subscription(Float32MultiArray, "encoder_data", self.encoder_to_odom, 10)
        self.imu_sub = self.create_subscription(Float32MultiArray, "/imu_data", self.imu_to_odom, 10)

        time_period = 0.016
        self.timer = self.create_timer(time_period, self.timer_callback)

        self.encmsg = Odometry()
        self.encmsg.header.frame_id = "odom"
        self.encmsg.child_frame_id = "base_link"

        self.imumsg = Imu()
        self.imumsg.header.frame_id = "odom"

        self.debug_pub = self.create_publisher(String, '/odom_debug', 1)

    def timer_callback(self):
        # print("Trying serial"
        self.encmsg.header.stamp = self.get_clock().now().to_msg()
        self.imumsg.header.stamp = self.get_clock().now().to_msg()

        try:
            self.odompub.publish(self.encmsg)
            self.imupub.publish(self.imumsg)
        except Exception as e:
            debug_msg = String()
            debug_msg.data = str(e)
            self.debug_pub.publish(debug_msg)

    def encoder_to_odom(self,msg):
        encoder_array = msg.data # receiving position and velocity

        lb_pos = encoder_array[0]*2*self.wheel_radius*math.pi
        lf_pos = encoder_array[1]*2*self.wheel_radius*math.pi
        rf_pos = encoder_array[2]*2*self.wheel_radius*math.pi

        lb_vel = encoder_array[3]*self.wheel_radius
        lf_vel = encoder_array[4]*self.wheel_radius
        rf_vel = encoder_array[5]*self.wheel_radius

        l_pos = (lb_pos + lf_pos) / 2
        r_pos = rf_pos # (rb_pos + rf_pos) / 2

        l_vel = (lb_vel + lf_vel) / 2
        r_vel = rf_vel # (rb_vel + rf_vel) / 2

        vel_xy = (l_vel + r_vel) / 2
        pos_xy = (l_pos + r_pos) / 2

        vel_th = (l_vel-r_vel) / self.wheel_sep
        pos_th = (l_pos-r_pos) / self.wheel_sep

        vel_ang = (l_vel-r_vel) / self.wheel_radius # dividing by correct number
        
        self.phi -= vel_th
        # self.phi = -pos_th
        dx = 0
        dy = 0

        px = pos_xy*math.cos(pos_th)
        py = pos_xy*math.sin(pos_th)

        if(vel_xy != 0):
            dx = vel_xy*math.cos(vel_th)
            dy = vel_xy*math.sin(vel_th)

        rot = Rotation.from_euler('xyz', [0,0, self.phi], degrees=True)
        rot_quat = rot.as_quat() # convert angle to quaternion format

        self.encmsg.pose.pose.position.x += dx
        self.encmsg.pose.pose.position.y += dy
        self.encmsg.pose.pose.position.z = 0.0476 # 0.1008
        self.encmsg.pose.pose.orientation = Quaternion(x=rot_quat[0],y=rot_quat[1],z=rot_quat[2],w=rot_quat[3])

        self.encmsg.twist.twist.linear.x = vel_xy
        self.encmsg.twist.twist.angular.z = vel_ang

        self.count+=1
        
        rot_str = String()
        rot_str.data = str(self.phi)
        self.debug_pub.publish(rot_str)

        debug_data = String()
        debug_data.data = "Rot:" + str(self.phi) + "Pos (Actual):" + str(px) + str(py) + \
        "Pos (Integrated):" + str(self.encmsg.pose.pose.position.x) + str(self.encmsg.pose.pose.position.y)
        self.debug_pub.publish(debug_data)
        # print(self.msg.pose)

    def imu_to_odom(self, msg):
        imu_array = msg.data

        self.imumsg.linear_acceleration.x = imu_array[0]
        self.imumsg.linear_acceleration.y = imu_array[1]
        self.imumsg.linear_acceleration.z = imu_array[2]
        self.imumsg.angular_velocity.x = imu_array[3]
        self.imumsg.angular_velocity.y = imu_array[4]
        self.imumsg.angular_velocity.z = imu_array[5]
        self.imumsg.linear_acceleration_covariance[0:9] = 0.1
        self.imumsg.angular_velocity_covariance[0:9] = 0.1

def main(args=None):
    rclpy.init(args=args)

    encsub = Sensor2Odom()

    rclpy.spin(encsub)

    encsub.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()