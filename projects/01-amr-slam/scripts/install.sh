#!/bin/bash
# 安装 SLAM 仿真所需的 ROS2 包（在 WSL 里执行一次）
# 用法：bash scripts/install.sh
set -Eeuo pipefail
DISTRO=${ROS_DISTRO:-lyrical}
echo "目标 ROS2 发行版：$DISTRO"

sudo apt update
sudo apt install -y \
  ros-$DISTRO-slam-toolbox \
  ros-$DISTRO-teleop-twist-keyboard \
  ros-$DISTRO-nav2-map-server \
  ros-$DISTRO-rviz2 \
  ros-$DISTRO-tf2-ros \
  ros-$DISTRO-ros-gz-bridge \
  ros-$DISTRO-rmw-cyclonedds-cpp

echo "基础依赖安装完成。"
echo "注意：Nav2 的 Lyrical 二进制包当前可能不可用；本机使用 /opt/nav2 源码安装。"
