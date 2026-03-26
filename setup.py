import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'q1_turtlebot'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools', 'numpy'],
    zip_safe=True,
    maintainer='SeongminJaden',
    maintainer_email='roboticsmaster@naver.com',
    description='TurtleBot3 + GrowSpace Q1 UWB 실제 로봇 패키지',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'odom_uwb_compare = q1_turtlebot.odom_uwb_compare_node:main',
            'follow_me = q1_turtlebot.follow_me_node:main',
            'geofence = q1_turtlebot.geofence_node:main',
            'uwb_listener = q1_turtlebot.uwb_listener_node:main',
        ],
    },
)
