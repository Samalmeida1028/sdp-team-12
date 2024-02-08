#!/usr/bin/env python3
# Author: Arjun Viswanathan
# Date created: 11/5/23
# Date last modified: 2/8/24
# Description: Using Nav2 to navigate to a given pose

from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32MultiArray
from nav_msgs.msg import Odometry
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
import math
from scipy.spatial.transform import Rotation

class Nav2Pose(Node):
    def __init__(self):
        super().__init__('nav2pose')
        print("Initializing Nav Simple Commander...")

        # self.navigator = BasicNavigator()

        self.current_pose = PoseStamped()
        self.goal = PoseStamped()

        # Wait for navigation to fully activate, since autostarting nav2
        #self.navigator.lifecycleStartup()
        # self.navigator.waitUntilNav2Active(localizer='bt_navigator')

        self.i = 0
        self.startnav = False

        #self.navigator.changeMap('basic_mobile_robot/maps/slam_map.yaml')

        print("Creating subscribers and callbacks...")
        self.coordsub = self.create_subscription(Float32MultiArray, '/target_position', self.setgoal, 10)
        self.odomsub = self.create_subscription(Odometry, '/odometry/filtered', self.set_current_pose, 10)
        self.goalupdaterpub = self.create_publisher(PoseStamped, "/goal_pose", 10)

        time_period = 0.5
        self.timer = self.create_timer(time_period, self.nav2pose_callback)

        print("Ready!")

    def set_current_pose(self, odommsg):
        self.current_pose.header.frame_id = 'map'
        self.current_pose.header.stamp = self.get_clock().now().to_msg()

        self.current_pose.pose.position = odommsg.pose.pose.position
        self.current_pose.pose.orientation = odommsg.pose.pose.orientation

        print("Updated current pose!")

    def setgoal(self, goalmsg):
        # goalmsg = [dist, xangle, yangle]
        self.goal.header.frame_id = 'map'
        self.goal.header.stamp = self.get_clock().now().to_msg()

        d = goalmsg.data[0]*math.cos(math.radians(goalmsg.data[2])) # true dist = seen dist * sin(yangle)
        print(goalmsg)
        
        self.goal.pose.position.x = self.current_pose.pose.position.x + (d*math.cos(math.radians(goalmsg.data[1])))/1000 # xf = xi + dsin(phi)
        self.goal.pose.position.y = self.current_pose.pose.position.y + (d*math.sin(math.radians(goalmsg.data[1])))/1000 # yf = yi + dcos(phi)
        print(self.goal.pose.position)

        rot = Rotation.from_euler('xyz', [0, 0, goalmsg.data[1]], degrees=True)
        rot_quat = rot.as_quat() # convert angle to quaternion format

        self.goal.pose.orientation.x = self.current_pose.pose.orientation.x + rot_quat[0] # set the orientation to be looking at the marker at the end of navigation
        self.goal.pose.orientation.y = self.current_pose.pose.orientation.x + rot_quat[1]
        self.goal.pose.orientation.z = self.current_pose.pose.orientation.x + rot_quat[2]
        self.goal.pose.orientation.w = self.current_pose.pose.orientation.x + rot_quat[3]

        if self.goal != self.current_pose:
            self.startnav = True
            print("Updated goal pose!")
        else:
            self.startnav = False

    def nav2pose_callback(self):
        if self.startnav:
            # other way to do this
            # path = self.navigator.getPath(self.initial_pose, self.goal)
            # smooth_path = self.navigator.smoothPath(path)
            # self.navigator.followPath(smooth_path)

            if self.current_pose != self.goal:
                self.goalupdaterpub.publish(self.goal)
                # self.navigator.goToPose(self.goal, behavior_tree='/home/adam/Desktop/sdp-team-12/basic_mobile_robot/behavior_trees/navigate_to_pose_w_replanning_and_recovery.xml')
                # self.current_pose = self.goal

            # if not self.navigator.isTaskComplete():
            #     # Do something with the feedback
            #     self.i = self.i + 1
            #     feedback = self.navigator.getFeedback()

            #     if feedback and self.i % 5 == 0:
            #         print('Estimated time of arrival: ' + '{0:.0f}'.format(
            #             Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9)
            #             + ' seconds.')

            #         # Some navigation timeout to cancellation
            #         if Duration.from_msg(feedback.navigation_time) > Duration(seconds=600.0):
            #             self.navigator.cancelTask()

            # result = self.navigator.getResult()
            # if result == TaskResult.SUCCEEDED:
            #     print('Goal succeeded!')
            # elif result == TaskResult.CANCELED:
            #     print('Goal was canceled!')
            # elif result == TaskResult.FAILED:
            #     print('Goal failed!')

def main(args=None):
    rclpy.init(args=args)

    nav2pose = Nav2Pose()

    rclpy.spin(nav2pose)

    nav2pose.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()