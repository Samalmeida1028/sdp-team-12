import os
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, Command

def generate_launch_description():
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

    start_better_lidar_cmd = Node(
        package='sllidar_ros2',
        executable='sllidar_node',
        name='sllidar_node',
        parameters=[{
            'channel_type':'serial',
            'serial_port': '/dev/ttyUSB0', 
            'serial_baudrate': 1000000, 
            'frame_id': 'lidar_link',
            'inverted': False, 
            'angle_compensate': True, 
            'scan_mode': 'DenseBoost'
        }],
    )

    start_lidar_filter_cmd = Node(
        package='navigator',
        executable='filterlidar',
        name='filterlidar'
    )

    ld = LaunchDescription()

    ld.add_action(start_better_lidar_cmd)
    ld.add_action(start_lidar_filter_cmd)

    return ld