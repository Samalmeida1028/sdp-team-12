import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    start_lidar_cmd = Node(
        name='rplidar_composition',
        package='rplidar_ros',
        executable='rplidar_composition',
        output='screen',
        parameters=[{
            'serial_port': '/dev/ttyUSB0',
            'serial_baudrate': 115200,
            'frame_id': 'lidar_link',
            'angle_compensate': True,
            'auto_standby': True,
            'scan_mode': 'Standard'
        }]
    )

    ld = LaunchDescription()
    ld.add_action(start_lidar_cmd)

    return ld