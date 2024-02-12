#!/usr/bin/env python3
# Author: Arjun Viswanathan
# Date created: 11/5/23
# Date last modified: 2/9/24
# Description: Using Nav2 to navigate to a given pose

from geometry_msgs.msg import PoseStamped,TransformStamped
from tf2_msgs.msg import TFMessage
from std_msgs.msg import Float32MultiArray,String,Float32,Int32
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
import math
from scipy.spatial.transform import Rotation

class Nav2Pose(Node):
    def __init__(self):
        super().__init__('nav2pose')
        # print("Initializing Nav Simple Commander...")

        # self.navigator = BasicNavigator()

        self.current_pose = PoseStamped()
        self.goal = PoseStamped()

        # Wait for navigation to fully activate, since autostarting nav2
        #self.navigator.lifecycleStartup()
        # self.navigator.waitUntilNav2Active(localizer='bt_navigator')

        self.i = 0
        self.startnav = False

        #self.navigator.changeMap('basic_mobile_robot/maps/slam_map.yaml')

        # print("Creating subscribers and callbacks...")
        self.angsub = self.create_subscription(Float32MultiArray, '/servoxy_angle', self.update_angle, 10)
        self.distancesub = self.create_subscription(Float32, '/target_distance', self.update_distance, 10)
        self.odomsub = self.create_subscription(Odometry, '/odometry/filtered', self.set_current_pose, 10)
        self.target_spotted_sub = self.create_subscription(Int32, '/target_spotted', self.set_goal, 10)

        self.goalupdaterpub = self.create_publisher(PoseStamped, "/goal_pose", 10)
        self.currentposepub = self.create_publisher(PoseStamped, "/current_pose", 10)
        self.currentdebugpub = self.create_publisher(String, "/rot_debug", 10)

        self.angles = [0,0]
        self.distance = 0
        self.target_goal = None

        time_period = 0.5
        self.timer = self.create_timer(time_period, self.nav2pose_callback)
        # print("Ready!")

    def set_current_pose(self, odommsg : Odometry):
        self.current_pose.header.frame_id = 'odom'
        self.current_pose.header.stamp = self.get_clock().now().to_msg()
        self.current_pose.pose.position = odommsg.pose.pose.position
        self.current_pose.pose.orientation = odommsg.pose.pose.orientation

        # tf_stamped : TransformStamped = tfmsg.transforms[0]
        # if(tf_stamped.header.frame_id == "odom"):
        #     self.current_pose.pose.position.x = tf_stamped.transform.translation.x
        #     self.current_pose.pose.position.y = tf_stamped.transform.translation.y
        #     self.current_pose.pose.position.z = tf_stamped.transform.translation.z
        #     self.current_pose.pose.orientation = tf_stamped.transform.rotation

        # print("Updated current pose!")

    def set_goal(self, check):
        # goalmsg = [dist, xangle, yangle]
        # print("hi",check.data)
        if(check.data and self.target_goal):
            # print("hello")
            self.goal.header.frame_id = 'odom'
            self.goal.header.stamp = self.get_clock().now().to_msg()

            d = self.target_goal[0]*math.cos(math.radians(self.target_goal[2])) # true dist = seen dist * sin(yangle)
            # print(self.target_goal)
            pos_deg = Rotation.from_quat([self.current_pose.pose.orientation.x,self.current_pose.pose.orientation.y,
                                          self.current_pose.pose.orientation.z,self.current_pose.pose.orientation.w])

            true_rot = (math.radians(self.target_goal[1]) + (pos_deg.as_euler("xyz",degrees=False)[2])) #+math.pi))-math.pi

            self.goal.pose.position.x = self.current_pose.pose.position.x + (d*math.cos(true_rot))/1000 # xf = xi + dsin(phi)
            self.goal.pose.position.y = self.current_pose.pose.position.y + (d*math.sin(true_rot))/1000 # yf = yi + dcos(phi)
            # print(self.goal.pose.position)
            boob = String()
            boob.data = str(true_rot)
            self.currentdebugpub.publish(boob)

            rot = Rotation.from_euler('xyz', [0, 0, true_rot], degrees=False)
            rot_quat = rot.as_quat() # convert angle to quaternion format

            self.goal.pose.orientation.x = self.current_pose.pose.orientation.x + rot_quat[0] # set the orientation to be looking at the marker at the end of navigation
            self.goal.pose.orientation.y = self.current_pose.pose.orientation.y + rot_quat[1]
            self.goal.pose.orientation.z = self.current_pose.pose.orientation.z + rot_quat[2]
            self.goal.pose.orientation.w = self.current_pose.pose.orientation.w + rot_quat[3]

            if self.goal != self.current_pose:
                self.startnav = True
                # print("Updated goal pose!")
            else:
                self.startnav = False

    def update_angle(self,msg):
        self.angles[0] = msg.data[0]
        self.angles[1] = msg.data[1]
    
    def update_distance(self,msg):
        self.distance = msg.data

    def nav2pose_callback(self):
        if(self.angles and self.distance):
            self.target_goal = [self.distance] + self.angles
            
        
        self.currentposepub.publish(self.current_pose)

        if self.startnav:

            if self.current_pose != self.goal:
                self.goalupdaterpub.publish(self.goal)

def main(args=None):
    rclpy.init(args=args)

    nav2pose = Nav2Pose()

    rclpy.spin(nav2pose)

    nav2pose.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()