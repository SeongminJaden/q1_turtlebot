"""
예제 1: Odom vs UWB 위치 비교 시각화.

Robot SBC에서 실행:
  ros2 launch q1_turtlebot odom_uwb_compare.launch.py

Remote PC에서 RViz2:
  rviz2
  → Add → By topic → /compare/odom_path, /compare/uwb_path, /compare/markers
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # Q1 리스너 (이미 실행 중이면 생략 가능)
        Node(package='q1_gateway', executable='q1_gateway_node',
             name='q1_gateway', output='screen'),

        # Odom-UWB 비교 노드
        Node(package='q1_turtlebot', executable='odom_uwb_compare',
             name='odom_uwb_compare',
             parameters=[{'robot_tag_id': 'DEV_TAG_00'}],
             output='screen'),
    ])
