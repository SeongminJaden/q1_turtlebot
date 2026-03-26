"""
예제 2: UWB Follow Me.

Robot SBC에서 실행:
  ros2 launch q1_turtlebot follow_me.launch.py
  ros2 launch q1_turtlebot follow_me.launch.py serial_port:=/dev/ttyUSB1

사전 조건:
  - Q1 앵커 4개 설치 + 리스너 USB 연결
  - 인적 태그(1E52) 소지
  - 로봇에 개발자 태그(25A0) 부착
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

        # Q1 리스너
        Node(package='q1_gateway', executable='serial_listener',
             name='q1_listener', output='screen',
             parameters=[{'serial_port': serial_port}]),

        # Follow Me
        Node(package='q1_turtlebot', executable='follow_me',
             name='follow_me', parameters=[config],
             output='screen'),
    ])
