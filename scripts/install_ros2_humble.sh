#!/bin/bash
# ROS2 Humble 설치 스크립트 (Ubuntu 22.04 / Raspberry Pi)
set -e

echo "=== [1/6] Locale 설정 ==="
sudo apt update && sudo apt install -y locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

echo "=== [2/6] Universe 저장소 활성화 ==="
sudo apt install -y software-properties-common
sudo add-apt-repository -y universe

echo "=== [3/6] ROS 2 apt 저장소 추가 ==="
sudo apt update && sudo apt install -y curl
export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F'"' '{print $4}')
curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${UBUNTU_CODENAME:-${VERSION_CODENAME}})_all.deb"
sudo dpkg -i /tmp/ros2-apt-source.deb

echo "=== [4/6] 시스템 업데이트 ==="
sudo apt update && sudo apt upgrade -y

echo "=== [5/6] ROS 2 Humble Base + 개발도구 설치 ==="
sudo apt install -y ros-humble-ros-base ros-dev-tools

echo "=== [6/6] 환경변수 설정 (.bashrc) ==="
if ! grep -q "source /opt/ros/humble/setup.bash" ~/.bashrc; then
    echo "" >> ~/.bashrc
    echo "# ROS2 Humble" >> ~/.bashrc
    echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
    echo "export ROS_DOMAIN_ID=30" >> ~/.bashrc
    echo "export TURTLEBOT3_MODEL=burger" >> ~/.bashrc
    echo "export LDS_MODEL=LDS-01" >> ~/.bashrc
fi

source /opt/ros/humble/setup.bash

echo ""
echo "=== 설치 완료! ==="
echo "ros2 버전: $(ros2 --version 2>/dev/null || echo 'reboot 후 확인')"
echo "새 터미널을 열거나 'source ~/.bashrc' 실행하세요."
