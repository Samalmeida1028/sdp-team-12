# SDP Team 12
# Date created: 2/8/24
# Date last modified: 2/21/24
# Author: Arjun Viswanathan
# Description: launch file to launch all necessary components for target tracking integration with navigation

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    start_target_pub_cmd = Node(
        package='py_img_stream',
        executable='target_pub',
        name='target_pub',
    )

    start_image_pub_cmd = Node(
        package='py_img_stream',
        executable='pub',
        name='image_pub',
    )

    start_nav2pose_cmd = Node(
        package='navigator',
        executable='nav2pose',
        name='nav2pose'
    ) 

    start_search_node = Node(
        package='navigator',
        executable='searchtargets',
        name='search_targets'
    )

    # Launch!
    ld = LaunchDescription()

    ld.add_action(start_target_pub_cmd)
    ld.add_action(start_image_pub_cmd)
    ld.add_action(start_nav2pose_cmd)
    ld.add_action(start_search_node)

    return ld