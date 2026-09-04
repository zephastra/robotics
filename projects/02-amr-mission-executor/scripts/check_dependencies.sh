#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "$SCRIPT_DIR/amr_env.sh"

for command_name in colcon gz ros2 python3; do
  command -v "$command_name" >/dev/null 2>&1 || {
    echo "Missing command: $command_name" >&2
    exit 1
  }
done

for package_name in \
  nav2_amcl \
  nav2_bringup \
  nav2_controller \
  nav2_map_server \
  nav2_simple_commander \
  ros_gz_bridge \
  rviz2 \
  tf2_ros; do
  ros2 pkg prefix "$package_name" >/dev/null 2>&1 || {
    echo "Missing ROS package: $package_name" >&2
    exit 1
  }
done

python3 -c "import pytest, yaml" >/dev/null 2>&1 || {
  echo "Missing Python package: pytest or PyYAML" >&2
  exit 1
}

echo "All Project 002 dependencies are available."
echo "ROS_DOMAIN_ID=$ROS_DOMAIN_ID"
echo "GZ_PARTITION=$GZ_PARTITION"
