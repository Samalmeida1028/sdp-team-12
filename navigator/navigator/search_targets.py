#!/usr/bin/env python3
# Author: Arjun Viswanathan
# Date created: 2/13/24
# Date last modified: 2/13/24
# Description: Searches for targets by publishing random goals 

from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32MultiArray,String,Float32,Int32
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import rclpy
from rclpy.node import Node
import math
from scipy.spatial.transform import Rotation
import time
import random

class SearchTargets(Node):
    def __init__(self):
        super().__init__('search_targets')

        self.center = PoseStamped()
        self.current_pose = PoseStamped()
        self.goal = PoseStamped()
        self.prev_goal = PoseStamped()

        # print("Creating subscribers and callbacks...")
        self.odomsub = self.create_subscription(Odometry, '/odometry/filtered', self.set_current_pose, 10)
        self.target_spotted_sub = self.create_subscription(Int32, '/target_spotted', self.set_timing, 10)
        self.scansub = self.create_subscription(LaserScan, "/scan", self.redefine_search, 10)

        self.goalupdaterpub = self.create_publisher(PoseStamped, "/goal_pose", 10)
        self.currentposepub = self.create_publisher(PoseStamped, "/current_pose", 10)

        self.debug = self.create_publisher(String, "/search_debug", 10)

        self.nav_start_time = 0.0
        self.start_time = 0.0
        self.time_now = 0.0
        self.time_passed = 0.0
        self.wait_time = 10.0
        self.search_radius = 3.0
        self.d = 0.5
        self.move_search_area = False

        time_period = 0.5
        self.timer1 = self.create_timer(time_period, self.set_search_goal)
        self.timer2 = self.create_timer(time_period, self.nav_timed_out)
        # print("Ready!")

    def set_current_pose(self, odommsg : Odometry):
        self.current_pose.header.frame_id = 'odom'
        self.current_pose.header.stamp = self.get_clock().now().to_msg()
        self.current_pose.pose.position = odommsg.pose.pose.position
        self.current_pose.pose.orientation = odommsg.pose.pose.orientation

        self.currentposepub.publish(self.current_pose)

    def set_timing(self, spotted : Int32):
        print("Setting wait time")

        if spotted.data:
            self.start_time = time.time()
        else:
            self.time_now = time.time()

        self.time_passed = self.time_now - self.start_time

    def nav_timed_out(self):
        nav_time_now = time.time()
        nav_time = nav_time_now - self.nav_start_time 

        if nav_time >= self.wait_time:
            print("Nav timed out!")
            return True

        return False

    def set_search_goal(self):
        if self.time_passed >= self.wait_time and self.nav_timed_out() and not self.move_search_area:
            print("Setting new goal!")

            self.goal.header.frame_id = 'odom'
            self.goal.header.stamp = self.get_clock().now().to_msg()

            phi = random.random() * 2*math.pi

            if self.d <= self.search_radius:
                self.d += 0.25
                self.move_search_area = False
            else:
                self.move_search_area = True

            test = String()
            test.data = "Dist: " + str(self.d) + ", Phi: " + str(phi)
            self.debug.publish(test)

            pos_rads = Rotation.from_quat([self.current_pose.pose.orientation.x,self.current_pose.pose.orientation.y,
                                          self.current_pose.pose.orientation.z,self.current_pose.pose.orientation.w])

            true_rot = phi + pos_rads.as_euler("xyz", degrees=False)[2]

            self.goal.pose.position.x = self.center.pose.position.x + (self.d*math.cos(true_rot)) # xf = xc + dcos(phi)
            self.goal.pose.position.y = self.center.pose.position.y + (self.d*math.sin(true_rot)) # yf = yc + dsin(phi)
            self.goal.pose.position.z = 0.0497

            rot = Rotation.from_euler('xyz', [0, 0, true_rot], degrees=False)
            rot_quat = rot.as_quat() # convert angle to quaternion format

            self.goal.pose.orientation.x = self.current_pose.pose.orientation.x + rot_quat[0] # set the orientation to be looking at the marker at the end of navigation
            self.goal.pose.orientation.y = self.current_pose.pose.orientation.y + rot_quat[1]
            self.goal.pose.orientation.z = self.current_pose.pose.orientation.z + rot_quat[2]
            self.goal.pose.orientation.w = self.current_pose.pose.orientation.w + rot_quat[3]

            self.nav_start_time = time.time()
            self.goalupdaterpub.publish(self.goal)
        elif self.time_passed < self.wait_time: # only go in when target is seen
            self.set_center()

    def set_center(self):
        print("Setting center of search radius")

        self.center.pose.position.x = self.current_pose.pose.position.x
        self.center.pose.position.y = self.current_pose.pose.position.y
        self.center.pose.position.z = self.current_pose.pose.position.z

        self.center.pose.orientation.x = self.current_pose.pose.orientation.x
        self.center.pose.orientation.y = self.current_pose.pose.orientation.y
        self.center.pose.orientation.z = self.current_pose.pose.orientation.z
        self.center.pose.orientation.w = self.current_pose.pose.orientation.w

    def redefine_search(self, scanmsg : LaserScan):
        if self.move_search_area:
            i = 0 # take LiDAR scan largest value and set goal to half of that
            
        return 0

def main(args=None):
    rclpy.init(args=args)

    search_targets = SearchTargets()

    rclpy.spin(search_targets)

    search_targets.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()