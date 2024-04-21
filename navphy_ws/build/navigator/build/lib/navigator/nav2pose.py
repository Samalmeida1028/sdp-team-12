#!/usr/bin/env python3
# Author: SDP Team 12
# Date created: 11/5/23
# Date last modified: 4/16/24
# Description: Using Nav2 to navigate to a given pose

from geometry_msgs.msg import PoseStamped
from tf2_msgs.msg import TFMessage
from std_msgs.msg import Float32MultiArray,String,Float32,Int32
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
import math
from scipy.spatial.transform import Rotation
import time

class Nav2Pose(Node):
    def __init__(self):
        super().__init__('nav2pose')

        self.current_pose = PoseStamped()
        self.goal = PoseStamped()
        self.prev_goal = PoseStamped()
        self.scanmsg = LaserScan()

        # print("Creating subscribers and callbacks...")
        self.angsub = self.create_subscription(Float32MultiArray, '/servoxy_angle', self.update_angle, 10)
        self.distancesub = self.create_subscription(Float32, '/target_distance', self.update_distance, 10)
        self.odomsub = self.create_subscription(Odometry, '/odometry/filtered', self.set_current_pose, 10)
        self.target_spotted_sub = self.create_subscription(Int32, '/target_spotted', self.set_spotted, 10)
        self.scansub = self.create_subscription(LaserScan, "/scan_filtered", self.set_laser_scan, 10)
        self.idsub = self.create_subscription(Int32, "/target_id", self.set_target_id, 10)

        self.goalupdaterpub = self.create_publisher(PoseStamped, "/goal_pose", 10)
        self.currentposepub = self.create_publisher(PoseStamped, "/current_pose", 10)
        self.nav2posegoalpub = self.create_publisher(PoseStamped, "/nav2pose_goal", 10)

        self.truncate_dist = 2.0
        self.angles = [0,0]
        self.distance = 0
        self.servo_values = None
        self.gpose_orient = 0.0
        self.target_id = 9999
        self.spotted = 0

        self.timer = self.create_timer(0.5, self.nav2pose_callback)
        self.goalpubtimer = self.create_timer(1.0, self.set_goal)

        self.prev_goal_time = time.time()

        self.currentdebugpub = self.create_publisher(String, '/nav2pose_debug', 10)
        self.get_logger().info('Nav2Pose Node Ready!')

    def set_current_pose(self, odommsg : Odometry):
        self.current_pose.header.frame_id = 'odom'
        self.current_pose.header.stamp = self.get_clock().now().to_msg()
        self.current_pose.pose.position = odommsg.pose.pose.position
        self.current_pose.pose.orientation = odommsg.pose.pose.orientation

    def set_laser_scan(self, scanmsg : LaserScan):
        self.scanmsg = scanmsg

    def set_target_id(self, idmsg : Int32):
        self.target_id = idmsg.data

    def set_spotted(self, spottedmsg : Int32):
        self.spotted = spottedmsg.data

    def set_goal(self):
        # self.servo_values = [dist, xangle, yangle]
        if(self.spotted and self.servo_values and self.target_id != 9999):
            self.goal.header.frame_id = 'odom'
            self.goal.header.stamp = self.get_clock().now().to_msg()

            d = self.servo_values[0]*math.cos(math.radians(self.servo_values[2])) # true dist = seen dist * sin(yangle)
            d = (d/1000) - self.truncate_dist
            # print(self.servo_values)
            cpose_orient = Rotation.from_quat([self.current_pose.pose.orientation.x,self.current_pose.pose.orientation.y,
                                          self.current_pose.pose.orientation.z,self.current_pose.pose.orientation.w]).as_euler("xyz",degrees=False)[2]

            self.gpose_orient = math.radians(self.servo_values[1]) + cpose_orient #+math.pi))-math.pi

            if self.gpose_orient > math.pi: # sanity check to keep it within ROS bounds [-pi,pi]
                self.gpose_orient -= (2*math.pi)
            elif self.gpose_orient < -3.12: # min angle of LiDAR is not exactly -pi (-3.14159)
                self.gpose_orient += 6.26

            self.goal.pose.position.x = self.current_pose.pose.position.x + (d*math.cos(self.gpose_orient)) # xf = xi + dsin(phi)
            self.goal.pose.position.y = self.current_pose.pose.position.y + (d*math.sin(self.gpose_orient)) # yf = yi + dcos(phi)
            self.goal.pose.position.z = 0.0497

            test = String()
            test.data = "Goal Orientation: " + str(self.gpose_orient)
            self.currentdebugpub.publish(test)

            rot = Rotation.from_euler('xyz', [0, 0, self.gpose_orient], degrees=False)
            rot_quat = rot.as_quat() # convert angle to quaternion format

            self.goal.pose.orientation.x = rot_quat[0] # set the orientation to be looking at the marker at the end of navigation
            self.goal.pose.orientation.y = rot_quat[1]
            self.goal.pose.orientation.z = rot_quat[2]
            self.goal.pose.orientation.w = rot_quat[3]

            self.correct_goal()
            self.goalupdaterpub.publish(self.goal)
            self.nav2posegoalpub.publish(self.goal)

    def correct_goal(self):
        self.get_logger().info("Checking goal validity with LiDAR scans...")

        initial_dist = math.atan2(self.goal.pose.position.y, self.goal.pose.position.x)

        # self.get_logger().info("{} {} {}".format(self.gpose_orient, self.scanmsg.angle_min, self.scanmsg.angle_increment))

        scanned_dist = self.scanmsg.ranges[int(self.gpose_orient / self.scanmsg.angle_increment)]
        corrected_dist = scanned_dist - 0.5 # add some padding so robot does not go into the wall

        if scanned_dist < initial_dist:
            self.get_logger().info("Invalid distance. Correcting {} to {}".format(initial_dist, corrected_dist))

            self.goal.pose.position.x = self.current_pose.pose.position.x + (corrected_dist*math.cos(self.gpose_orient)) # xf = xc + dcos(phi)
            self.goal.pose.position.y = self.current_pose.pose.position.y + (corrected_dist*math.sin(self.gpose_orient)) # yf = yc + dsin(phi)
        else:
            self.get_logger().info("Distance is valid")

    def update_angle(self,msg : Float32MultiArray):
        self.angles = [0,0]
        self.angles[0] = msg.data[0]
        self.angles[1] = msg.data[1]
    
    def update_distance(self,msg : Float32):
        self.distance = 0
        self.distance = msg.data

    def nav2pose_callback(self):
        if(self.angles and self.distance):
            self.servo_values = [self.distance] + self.angles
            self.distance = None
            self.angles = None
        
        self.currentposepub.publish(self.current_pose)

def main(args=None):
    rclpy.init(args=args)

    nav2pose = Nav2Pose()

    rclpy.spin(nav2pose)

    nav2pose.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()