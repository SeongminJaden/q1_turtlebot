"""
예제 3: Geofencing.

Robot SBC에서 실행:
  ros2 launch q1_turtlebot geofence.launch.py

조종 (별도 터미널):
  ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=cmd_vel_input

구역 편집:
  config/geofence_zones.yaml
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('q1_turtlebot')
    config = os.path.join(pkg, 'config', 'geofence_zones.yaml')

    return LaunchDescription([
        # Q1 리스너
        Node(package='q1_gateway', executable='q1_gateway_node',
             name='q1_gateway', output='screen'),

        # Geofence
        Node(package='q1_turtlebot', executable='geofence',
             name='geofence', parameters=[config],
             output='screen'),
    ])
