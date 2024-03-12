# SDP Team 12
# Date created: 2/8/24
# Date last modified: 3/12/24
# Author: Arjun Viswanathan
# Description: launch file to launch all necessary components for target tracking integration with navigation

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, Command, PythonExpression
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition

def generate_launch_description():
    run_serial = LaunchConfiguration('serial')
    run_search = LaunchConfiguration('search')

    declare_run_serial_cmd = DeclareLaunchArgument(
        name='serial',
        default_value='False',
        description='Whether to use serial handler or not'
    )

    declare_run_search_cmd = DeclareLaunchArgument(
        name='search',
        default_value='False',
        description='Whether to use search node or not'
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

    search_node = Node(
        package='navigator',
        executable='searchtargets',
        name='search_targets'
    )
    serial_pub_node = Node(
        package='py_serial',
        executable='serial_handler',
        name='serial_handler',
    )

    start_search_cmd = GroupAction(
        condition=IfCondition(PythonExpression([run_search])),
        actions=[search_node]
    )

    start_serial_cmd = GroupAction(
        condition=IfCondition(PythonExpression([run_serial])),
        actions=[serial_pub_node]
    )

    # Launch!
    ld = LaunchDescription()
    ld.add_action(declare_run_serial_cmd)
    ld.add_action(declare_run_search_cmd)

    ld.add_action(start_image_pub_cmd)
    ld.add_action(start_nav2pose_cmd)
    ld.add_action(start_serial_cmd)
    ld.add_action(start_search_cmd)

    return ld