"""
TurtleBot3 Burger 브링업 (라이다 기본 비활성화).

실행:
  ros2 launch q1_turtlebot bringup.launch.py
  ros2 launch q1_turtlebot bringup.launch.py use_lidar:=true
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    tb3_bringup_dir = get_package_share_directory('turtlebot3_bringup')

    use_lidar = LaunchConfiguration('use_lidar', default='false')
    usb_port = LaunchConfiguration('usb_port', default='/dev/ttyACM0')

    return LaunchDescription([
        # 환경변수 고정
        SetEnvironmentVariable('TURTLEBOT3_MODEL', 'burger'),
        SetEnvironmentVariable('LDS_MODEL', 'LDS-01'),
        SetEnvironmentVariable('ROS_DOMAIN_ID', '30'),

        DeclareLaunchArgument(
            'use_lidar',
            default_value='false',
            description='LiDAR 활성화 여부 (true/false)'),

        DeclareLaunchArgument(
            'usb_port',
            default_value='/dev/ttyACM0',
            description='OpenCR USB 포트'),

        # robot.launch.py (state_publisher + lidar + turtlebot3_node)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(tb3_bringup_dir, 'launch', 'robot.launch.py')),
            launch_arguments={
                'usb_port': usb_port,
                'use_lidar': use_lidar,
            }.items(),
        ),
    ])
