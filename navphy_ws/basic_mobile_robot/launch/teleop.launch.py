# SDP Team 12
# Date created: 2/6/24
# Date last modified: 2/13/24
# Author: Arjun Viswanathan
# Description: launch file to launch all necessary components for teleoperation

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, Command
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg_path = os.path.join(get_package_share_directory('basic_mobile_robot'))

    model_file = os.path.join(pkg_path, 'models', 'robo_holly.urdf')
    robot_localization_file_path = os.path.join(pkg_path, 'config', 'ekf.yaml')
    default_rviz_config_path = os.path.join(pkg_path, 'rviz', 'teleop.rviz')
    slam_params_file = os.path.join(pkg_path, 'config', 'doslam.yaml')

    model = LaunchConfiguration('model')
    rviz_config_file = LaunchConfiguration('rviz_config_file')
    use_sim_time = LaunchConfiguration('use_sim_time')

    remappings = [('/tf', 'tf'),
                ('/tf_static', 'tf_static')]

    declare_autostart_cmd = DeclareLaunchArgument(
        name='autostart', 
        default_value='True',
        description='Automatically startup the nav2 stack'
    )

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='False',
        description='Use sim time if true'
    )

    declare_model_path_cmd = DeclareLaunchArgument(
        name='model', 
        default_value=model_file, 
        description='Absolute path to robot urdf file'
    )

    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        name='rviz_config_file',
        default_value=default_rviz_config_path,
        description='Full path to the RVIZ config file to use'
    )

    start_robot_localization_cmd = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        parameters=[robot_localization_file_path, 
        {'use_sim_time': use_sim_time}]
    )

    start_joint_state_publisher_cmd = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher'
    )

    start_robot_state_publisher_cmd = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'use_sim_time': use_sim_time, 
        'robot_description': Command(['xacro ', model])}],
        remappings=remappings,
        arguments=[model_file]
    )

    start_lidar_cmd = Node(
        name='rplidar_composition',
        package='rplidar_ros',
        executable='rplidar_composition',
        parameters=[{
            'serial_port': '/dev/ttyUSB0',
            'serial_baudrate': 115200,
            'frame_id': 'lidar_link',
            'angle_compensate': True,
            'auto_standby': True,
            'scan_mode': 'Boost'
        }],
    )

    start_slam_cmd = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        parameters=[
            slam_params_file,
            {'use_sim_time': use_sim_time}
        ],
    )

    start_encoder_odom_pub_cmd = Node(
        package='navigator',
        executable='sens2odom',
        name='sens2odom',
        output='screen'
    )

    start_cmdvel_pub_cmd = Node(
        package='navigator',
        executable='cmdvelsub',
        name='cmdvel_pub',
    )

    start_serial_pub_cmd = Node(
        package='py_serial',
        executable='serial_handler',
        name='serial_handler',
    )

    start_rviz_cmd = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file]
    ) 

    start_target_tracking_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_path, 'launch', 'target_tracking.launch.py'))
    )

    # Launch!
    ld = LaunchDescription()

    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_model_path_cmd)
    ld.add_action(declare_rviz_config_file_cmd)
    ld.add_action(start_encoder_odom_pub_cmd)

    ld.add_action(start_cmdvel_pub_cmd)
    ld.add_action(start_serial_pub_cmd)
    ld.add_action(start_robot_localization_cmd)
    ld.add_action(start_robot_state_publisher_cmd)
    ld.add_action(start_joint_state_publisher_cmd)
    ld.add_action(start_lidar_cmd)
    ld.add_action(start_slam_cmd)
    ld.add_action(start_target_tracking_cmd)
    ld.add_action(start_rviz_cmd)

    return ld