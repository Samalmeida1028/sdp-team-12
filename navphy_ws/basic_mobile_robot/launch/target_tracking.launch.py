# SDP Team 12
# Date created: 2/8/24
# Date last modified: 4/17/24
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
        executable='img_pub_audio',
        name='pub',
    )

    start_nav2pose_cmd = Node(
        package='navigator',
        executable='nav2pose',
        name='nav2pose'
    ) 

    start_serial_cmd = Node(
        package='py_serial',
        executable='serial_handler',
        name='serial_handler',
    )

    start_camera_search_cmd = Node(
        package='navigator',
        executable='camerasearch',
        name='camerasearch',
    )
    start_obs_updater_cmd = Node(
        package='obs_nodes',
        executable='obs_target_update',
        name='obsupdatetarget',
    )

    # Launch!
    ld = LaunchDescription()

    ld.add_action(start_target_pub_cmd)
    ld.add_action(start_image_pub_cmd)
    # ld.add_action(start_nav2pose_cmd)
    ld.add_action(start_serial_cmd)
    ld.add_action(start_camera_search_cmd)
    # ld.add_action(start_obs_updater_cmd)

    return ld