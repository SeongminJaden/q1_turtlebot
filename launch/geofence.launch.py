"""
Geofencing.

실행:
  ros2 launch q1_turtlebot geofence.launch.py
  ros2 launch q1_turtlebot geofence.launch.py serial_port:=/dev/ttyUSB1

조종 (별도 터미널):
  ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=cmd_vel_input

구역 편집:
  config/geofence_zones.yaml
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('q1_turtlebot')
    config = os.path.join(pkg, 'config', 'geofence_zones.yaml')

    serial_port = LaunchConfiguration('serial_port', default='/dev/ttyUSB0')

    return LaunchDescription([
        DeclareLaunchArgument(
            'serial_port',
            default_value='/dev/ttyUSB0',
            description='Listener USB serial port'),

        Node(package='q1_turtlebot', executable='geofence',
             name='geofence', output='screen',
             parameters=[config, {'serial_port': serial_port}]),
    ])
