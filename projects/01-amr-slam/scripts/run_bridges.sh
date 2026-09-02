#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
# shellcheck disable=SC1091
source "$PROJECT_DIR/scripts/amr_env.sh"

echo "Starting the Gazebo-to-ROS bridge ..."
exec ros2 run ros_gz_bridge parameter_bridge \
  --ros-args \
  -p config_file:="$PROJECT_DIR/config/bridge_config.yaml" \
  -r __node:=amr_gz_bridge
