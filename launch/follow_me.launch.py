"""
UWB Follow Me.

실행:
  ros2 launch q1_turtlebot follow_me.launch.py
  ros2 launch q1_turtlebot follow_me.launch.py serial_port:=/dev/ttyUSB1
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('q1_turtlebot')
    config = os.path.join(pkg, 'config', 'follow_me.yaml')

    serial_port = LaunchConfiguration('serial_port', default='/dev/ttyUSB0')

    return LaunchDescription([
        DeclareLaunchArgument(
            'serial_port',
            default_value='/dev/ttyUSB0',
            description='Listener USB serial port'),

        Node(package='q1_turtlebot', executable='follow_me',
             name='follow_me', output='screen',
             parameters=[config, {'serial_port': serial_port}]),
    ])
