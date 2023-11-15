import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, Command
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

import xacro

def generate_launch_description():
    pkg_path = os.path.join(get_package_share_directory('basic_mobile_robot'))
    xacro_file = os.path.join(pkg_path, 'models', 'robo_holly.urdf')

    robot_description_config = Command(['xacro ', xacro_file])
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Create a robot_state_publisher node
    params = {'robot_description': robot_description_config, 'use_sim_time': use_sim_time}

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='False',
        description='Use sim time if true'
    )

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    # Launch!
    ld = LaunchDescription()

    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(node_robot_state_publisher)

    return ld