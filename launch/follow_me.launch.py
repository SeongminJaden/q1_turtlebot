"""
예제 2: UWB Follow Me.

Robot SBC에서 실행:
  ros2 launch q1_turtlebot follow_me.launch.py

사전 조건:
  - Q1 앵커 4개 설치 + 리스너 USB 연결
  - 인적 태그(HUMAN_01) 소지
  - 로봇에 개발자 태그(DEV_TAG_00) 부착
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('q1_turtlebot')
    config = os.path.join(pkg, 'config', 'follow_me.yaml')

    return LaunchDescription([
        # Q1 리스너
        Node(package='q1_gateway', executable='q1_gateway_node',
             name='q1_gateway', output='screen'),

        # Follow Me
        Node(package='q1_turtlebot', executable='follow_me',
             name='follow_me', parameters=[config],
             output='screen'),
    ])
