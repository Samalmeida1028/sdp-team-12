# #!/usr/bin/env python3
# # Author: Arjun Viswanathan
# # Date created: 11/5/23
# # Date last modified: 11/5/23
# # Description: Using Nav2 to navigate to a given pose

# from geometry_msgs.msg import PoseStamped
# from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
# import rclpy
# from rclpy.node import Node
# from rclpy.duration import Duration

# class Nav2Pose(Node):
#     def __init__(self):
#         super().__init__('nav2pose')
#         self.navigator = BasicNavigator()
#         # Wait for navigation to fully activate, since autostarting nav2
#         self.navigator.waitUntilNav2Active()

#         self.initial_pose = PoseStamped()
#         self.goal = PoseStamped()

#         #self.init_navigator(3.45, 2.15, 1.0, 0.0)

#         self.i = 0

#         self.poseSub = self.create_subscription(PoseStamped, '/goal_pose', self.setgoal, 10)
        
#         time_period = 0.5
#         self.timer = self.create_timer(time_period, self.nav2pose_callback)

#     def init_navigator(self, x, y, z, w):
#         # Set our demo's initial pose
#         self.initial_pose.header.frame_id = 'map'
#         self.initial_pose.header.stamp = self.navigator.get_clock().now().to_msg()
#         self.initial_pose.pose.position.x = x
#         self.initial_pose.pose.position.y = y
#         self.initial_pose.pose.orientation.z = z
#         self.initial_pose.pose.orientation.w = w

#         self.navigator.setInitialPose(self.initial_pose)

#         # If desired, you can change or load the map as well
#         # self.navigator.changeMap('/path/to/map.yaml')

#         # You may use the navigator to clear or obtain costmaps
#         # navigator.clearAllCostmaps()  # also have clearLocalCostmap() and clearGlobalCostmap()
#         # global_costmap = navigator.getGlobalCostmap()
#         # local_costmap = navigator.getLocalCostmap()

#     def example_nav2pose(self):
#         goal_pose = PoseStamped()
#         goal_pose.header.frame_id = 'map'
#         goal_pose.header.stamp = self.navigator.get_clock().now().to_msg()
#         goal_pose.pose.position.x = -2.0
#         goal_pose.pose.position.y = -0.5
#         goal_pose.pose.orientation.w = 1.0

#         self.navigator.goToPose(goal_pose)

#     def setgoal(self, goalmsg):
#         self.goal = goalmsg

#     def nav2pose_callback(self):
#         # other way to do this
#         # path = self.navigator.getPath(self.initial_pose, self.goal)
#         # smooth_path = self.navigator.smoothPath(path)
#         # self.navigator.followPath(smooth_path)

#         self.navigator.goToPose(self.goal)

#         if not self.navigator.isTaskComplete():
#             ################################################
#             #
#             # Implement some code here for your application!
#             #
#             ################################################

#             # Do something with the feedback
#             self.i = self.i + 1
#             feedback = self.navigator.getFeedback()
#             if feedback and self.i % 5 == 0:
#                 print('Estimated time of arrival: ' + '{0:.0f}'.format(
#                     Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9)
#                     + ' seconds.')

#                 # Some navigation timeout to cancellation
#                 if Duration.from_msg(feedback.navigation_time) > Duration(seconds=600.0):
#                     self.navigator.cancelTask()

#         result = self.navigator.getResult()
#         if result == TaskResult.SUCCEEDED:
#             print('Goal succeeded!')
#         elif result == TaskResult.CANCELED:
#             print('Goal was canceled!')
#         elif result == TaskResult.FAILED:
#             print('Goal failed!')

# def main(args=None):
#     rclpy.init(args=args)

#     nav2pose = Nav2Pose()

#     rclpy.spin(nav2pose)

#     nav2pose.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()

#! /usr/bin/env python3
# Copyright 2021 Samsung Research America
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
from rclpy.duration import Duration

"""
Basic navigation demo to go to pose.
"""

def main():
    rclpy.init()

    navigator = BasicNavigator()

    # Activate navigation, if not autostarted. This should be called after setInitialPose()
    # or this will initialize at the origin of the map and update the costmap with bogus readings.
    # If autostart, you should `waitUntilNav2Active()` instead.
    # navigator.lifecycleStartup()

    # Wait for navigation to fully activate, since autostarting nav2
    navigator.waitUntilNav2Active()

    # If desired, you can change or load the map as well
    # navigator.changeMap('/path/to/map.yaml')

    # You may use the navigator to clear or obtain costmaps
    # navigator.clearAllCostmaps()  # also have clearLocalCostmap() and clearGlobalCostmap()
    # global_costmap = navigator.getGlobalCostmap()
    # local_costmap = navigator.getLocalCostmap()

    # Go to our demos first goal pose
    goal_pose = PoseStamped()
    goal_pose.header.frame_id = 'map'
    goal_pose.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose.pose.position.x = -2.0
    goal_pose.pose.position.y = -0.5
    goal_pose.pose.orientation.w = 1.0

    # sanity check a valid path exists
    # path = navigator.getPath(initial_pose, goal_pose)

    navigator.goToPose(goal_pose)

    i = 0
    while not navigator.isTaskComplete():
        ################################################
        #
        # Implement some code here for your application!
        #
        ################################################

        # Do something with the feedback
        i = i + 1
        feedback = navigator.getFeedback()
        if feedback and i % 5 == 0:
            print('Estimated time of arrival: ' + '{0:.0f}'.format(
                  Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9)
                  + ' seconds.')

            # Some navigation timeout to demo cancellation
            if Duration.from_msg(feedback.navigation_time) > Duration(seconds=600.0):
                navigator.cancelTask()

            # Some navigation request change to demo preemption
            if Duration.from_msg(feedback.navigation_time) > Duration(seconds=18.0):
                goal_pose.pose.position.x = -3.0
                navigator.goToPose(goal_pose)

    # Do something depending on the return code
    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        print('Goal succeeded!')
    elif result == TaskResult.CANCELED:
        print('Goal was canceled!')
    elif result == TaskResult.FAILED:
        print('Goal failed!')
    else:
        print('Goal has an invalid return status!')

    navigator.lifecycleShutdown()

    exit(0)


if __name__ == '__main__':
    main()