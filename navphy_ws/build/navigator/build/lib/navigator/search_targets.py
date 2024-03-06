#!/usr/bin/env python3
# Author: Arjun Viswanathan
# Date created: 2/13/24
# Date last modified: 3/6/24
# Description: Searches for targets by publishing random goals 

'''
1. When target is spotted, keep setting center of search area to robot position 
2. When target is lost, wait 10 seconds before publishing a goal
    Goal published relative to center of search space
3. Give 10 seconds to navigate to goal within search space, 20 for redefine goal
4. Give new search space goal once navigate to goal is timed out within search space
5. If at search radius limit, then redefine search space using LiDAR 
    Take max scan range, and set goal to half of that
6. Redefine only once, and wait until goal reached or timed out before restarting search
7. Before publishing a random goal, verify using LiDAR data that the goal is not outside the map bounds
    Does not apply for redefine goals because they are already using LiDAR data
'''

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
import numpy as np

class SearchTargets(Node):
    def __init__(self):
        super().__init__('search_targets')
        self.delay_exec() # delay node 30 seconds before starting

        self.center = PoseStamped()
        self.current_pose = PoseStamped()
        self.goal = PoseStamped()
        self.prev_goal = PoseStamped()
        self.scanmsg = LaserScan()

        self.odomsub = self.create_subscription(Odometry, '/odometry/filtered', self.set_current_pose, 10)
        self.target_spotted_sub = self.create_subscription(Int32, '/target_spotted', self.wait_timed_out, 10)
        self.scansub = self.create_subscription(LaserScan, "/scan", self.set_laser_scan, 10)

        self.goalupdaterpub = self.create_publisher(PoseStamped, "/goal_pose", 10)
        self.currentposepub = self.create_publisher(PoseStamped, "/current_pose", 10)

        self.debug = self.create_publisher(String, "/search_debug", 10)

        self.wait_start_time = time.time()
        self.nav_start_time = time.time()
        self.time_passed = time.time()

        self.true_rot = 0.0
        self.wait_time = 60.0
        self.redefine_time = 20.0
        self.search_radius = 1.0
        self.d = 0.5
        self.move_search_area = False

        time_period = 1.0
        self.timer1 = self.create_timer(time_period, self.set_search_goal)

        self.get_logger().info('Search Node Ready!')

    def delay_exec(self):
        t = time.time()
        tf = t + 30

        while t < tf:
            t = time.time()

        return

    # Sets the current pose of the robot from fused odometry messages given by EKF
    def set_current_pose(self, odommsg : Odometry):
        self.current_pose.header.frame_id = 'odom'
        self.current_pose.header.stamp = self.get_clock().now().to_msg()
        self.current_pose.pose.position = odommsg.pose.pose.position
        self.current_pose.pose.orientation = odommsg.pose.pose.orientation

        self.currentposepub.publish(self.current_pose)

    # Sets the center of the search radius
    def set_center(self):
        # self.get_logger().info("Setting center of search radius")

        self.center.pose.position.x = self.current_pose.pose.position.x
        self.center.pose.position.y = self.current_pose.pose.position.y
        self.center.pose.position.z = self.current_pose.pose.position.z

        self.center.pose.orientation.x = self.current_pose.pose.orientation.x
        self.center.pose.orientation.y = self.current_pose.pose.orientation.y
        self.center.pose.orientation.z = self.current_pose.pose.orientation.z
        self.center.pose.orientation.w = self.current_pose.pose.orientation.w

    def set_laser_scan(self, scanmsg : LaserScan):
        self.scanmsg = scanmsg

    # Checks to see if 10 seconds has been waited and returns a boolean
    def wait_timed_out(self, spotted : Int32):
        time_now = time.time()
        if spotted.data:
            self.wait_start_time = time.time()

        self.time_passed = time_now - self.wait_start_time
        # self.get_logger().info("Waited {} seconds".format(self.time_passed))

    # Checks to see if navigation has had enough time before giving a new goal
    # Works for search and redefine
    def nav_timed_out(self):
        nav_time_now = time.time()
        nav_time = nav_time_now - self.nav_start_time 

        # self.get_logger().info("Navigated for {} seconds".format(nav_time))

        if not self.move_search_area:
            if nav_time >= self.wait_time:
                self.get_logger().info("Nav to search timed out!")
                return True
        else:
            if nav_time >= self.redefine_time:
                self.get_logger().info("Nav to redefine timed out!")
                self.move_search_area = False # publish one goal and stop publishing after that
                self.set_center() # reset the center of search space now that it has timed out
                return True

        return False

    # Given we have waited more than 10 seconds and navigation timer has expired and we are not redefining, 
    # publish new goal. Increment distance and pick a random angle to publish a goal in search area
    def set_search_goal(self):
        if self.time_passed >= self.wait_time and self.nav_timed_out() and not self.move_search_area:
            self.goal.header.frame_id = 'odom'
            self.goal.header.stamp = self.get_clock().now().to_msg()

            phi = random.random() * 2*math.pi

            test = String()
            test.data = "Dist: " + str(self.d) + ", Phi: " + str(phi)
            self.debug.publish(test)

            pos_rads = Rotation.from_quat([self.current_pose.pose.orientation.x,self.current_pose.pose.orientation.y,
                                          self.current_pose.pose.orientation.z,self.current_pose.pose.orientation.w])

            self.true_rot = phi + pos_rads.as_euler("xyz", degrees=False)[2]

            self.goal.pose.position.x = self.center.pose.position.x + (self.d*math.cos(self.true_rot)) # xf = xc + dcos(phi)
            self.goal.pose.position.y = self.center.pose.position.y + (self.d*math.sin(self.true_rot)) # yf = yc + dsin(phi)
            self.goal.pose.position.z = 0.0497

            rot = Rotation.from_euler('xyz', [0, 0, self.true_rot], degrees=False)
            rot_quat = rot.as_quat() # convert angle to quaternion format

            self.goal.pose.orientation.x = rot_quat[0] # set the orientation to be looking at the marker at the end of navigation
            self.goal.pose.orientation.y = rot_quat[1]
            self.goal.pose.orientation.z = rot_quat[2]
            self.goal.pose.orientation.w = rot_quat[3]

            if self.d <= self.search_radius:
                self.get_logger().info("Setting new goal to {} at angle {}".format(self.d, phi))
                self.correct_goal()

                self.nav_start_time = time.time()
                self.goalupdaterpub.publish(self.goal)

                self.d += 0.25
                self.move_search_area = False
            else:
                self.d = 0.5
                self.move_search_area = True
                self.redefine_search()
        elif self.time_passed < self.wait_time: # only go in when target is seen
            self.set_center()

    # Publishes ONE goal when redefining search space using LiDAR data
    def redefine_search(self):
        if self.move_search_area:
            pos_rads = Rotation.from_quat([self.current_pose.pose.orientation.x,self.current_pose.pose.orientation.y,
                                          self.current_pose.pose.orientation.z,self.current_pose.pose.orientation.w])
            phi_robot = pos_rads.as_euler("xyz", degrees=False)[2]

            range_max = max(filter(lambda x: not math.isinf(x) and not math.isnan(x), self.scanmsg.ranges))
            max_ind = self.scanmsg.ranges.index(range_max) # index where max scan range occurs
            phi_max = phi_robot + self.scanmsg.angle_min + (max_ind * self.scanmsg.angle_increment) # angle at which max scan range occurs

            self.get_logger().info("Redefining search space to {} at angle {}".format(range_max/2, phi_max))

            self.goal.header.frame_id = 'odom'
            self.goal.header.stamp = self.get_clock().now().to_msg()

            # new goal  x x for redefine is from robot pose
            self.goal.pose.position.x = self.current_pose.pose.position.x + ((range_max/2)*math.cos(phi_max)) # xf = xi + dcos(phi)
            self.goal.pose.position.y = self.current_pose.pose.position.y + ((range_max/2)*math.sin(phi_max)) # yf = yi + dsin(phi)
            self.goal.pose.position.z = 0.0497
            
            rot = Rotation.from_euler('xyz', [0, 0, phi_max], degrees=False)
            rot_quat = rot.as_quat() # convert angle to quaternion format

            self.goal.pose.orientation.x = rot_quat[0] # set the orientation to be looking at the marker at the end of navigation
            self.goal.pose.orientation.y = rot_quat[1]
            self.goal.pose.orientation.z = rot_quat[2]
            self.goal.pose.orientation.w = rot_quat[3]

            self.nav_start_time = time.time()
            self.goalupdaterpub.publish(self.goal)

    def correct_goal(self):
        self.get_logger().info("Checking distance validity with LiDAR scans...")

        initial_dist = math.atan2(self.goal.pose.position.y, self.goal.pose.position.x)
        scanned_dist = self.scanmsg.ranges[int((self.true_rot + self.scanmsg.angle_min) / self.scanmsg.angle_increment)]
        corrected_dist = scanned_dist - 0.5 # add some padding so robot does not go into the wall

        if scanned_dist < initial_dist:
            self.get_logger().info("Invalid distance. Correcting {} to {}".format(initial_dist, corrected_dist))

            self.goal.pose.position.x = self.center.pose.position.x + (corrected_dist*math.cos(self.true_rot)) # xf = xc + dcos(phi)
            self.goal.pose.position.y = self.center.pose.position.y + (corrected_dist*math.sin(self.true_rot)) # yf = yc + dsin(phi)
        else:
            self.get_logger().info("Distance is valid")
    
def main(args=None):
    rclpy.init(args=args)

    search_targets = SearchTargets()

    rclpy.spin(search_targets)

    search_targets.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()