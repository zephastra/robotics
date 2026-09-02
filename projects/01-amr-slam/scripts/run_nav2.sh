#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
cd "$PROJECT_DIR"
# shellcheck disable=SC1091
source "$PROJECT_DIR/scripts/amr_env.sh"
bash "$PROJECT_DIR/scripts/check_runtime.sh"

LOG_DIR="${AMR_LOG_DIR:-/tmp/amr_nav_logs}"
mkdir -p "$LOG_DIR"
MAP_YAML="${1:-$PROJECT_DIR/maps/map.yaml}"

declare -a PROCESS_GROUPS=()

start_background() {
  local log_file="$1"
  shift
  setsid "$@" >"$LOG_DIR/$log_file" 2>&1 &
  PROCESS_GROUPS+=("$!")
}

cleanup() {
  local status=$?
  trap - EXIT INT TERM
  echo
  echo "Stopping AMR navigation processes ..."
  local pid
  for pid in "${PROCESS_GROUPS[@]}"; do
    if kill -0 "$pid" 2>/dev/null; then
      kill -TERM -- "-$pid" 2>/dev/null || true
    fi
  done
  sleep 1
  for pid in "${PROCESS_GROUPS[@]}"; do
    # The group leader may already have exited while ROS launch children remain.
    kill -KILL -- "-$pid" 2>/dev/null || true
  done
  wait 2>/dev/null || true
  exit "$status"
}

trap cleanup EXIT INT TERM

echo "[check] Verifying the saved map ..."
[[ -s "$MAP_YAML" ]] || {
  echo "Map YAML is missing or empty: $MAP_YAML" >&2
  echo "Build a map first, then run: bash scripts/save_map.sh" >&2
  exit 1
}

echo "[check] Verifying Nav2 packages ..."
for package in nav2_bringup nav2_amcl nav2_controller nav2_map_server; do
  ros2 pkg prefix "$package" >/dev/null 2>&1 || {
    echo "Required ROS package is unavailable: $package" >&2
    exit 1
  }
done

echo "[1/5] Starting Gazebo ..."
if [[ "${AMR_HEADLESS:-0}" == "1" ]]; then
  start_background gz.log gz sim -s --headless-rendering -r -v 4 "$PROJECT_DIR/worlds/amr_world.sdf"
else
  start_background gz.log gz sim -r -v 4 "$PROJECT_DIR/worlds/amr_world.sdf"
fi
sleep 8

echo "[2/5] Starting the bridge ..."
start_background bridge.log bash "$PROJECT_DIR/scripts/run_bridges.sh"
sleep 3

echo "[3/5] Starting TF publishers ..."
start_background static_tf.log ros2 run tf2_ros static_transform_publisher \
  --x 0 --y 0 --z 0.22 --roll 0 --pitch 0 --yaw 0 \
  --frame-id base_link --child-frame-id laser_link
start_background odom_tf.log python3 "$PROJECT_DIR/nodes/odom_to_tf.py"
sleep 2

echo "[4/5] Starting localization and navigation ..."
start_background localization.log ros2 launch nav2_bringup localization_launch.py \
  map:="$MAP_YAML" \
  use_sim_time:=true \
  params_file:="$PROJECT_DIR/config/nav2_params.yaml"
start_background navigation.log ros2 launch nav2_bringup navigation_launch.py \
  use_sim_time:=true \
  params_file:="$PROJECT_DIR/config/nav2_params.yaml" \
  use_keepout_zones:=false \
  use_speed_zones:=false
sleep 10

if [[ "${AMR_HEADLESS:-0}" != "1" ]]; then
  echo "[5/5] Starting RViz ..."
  start_background rviz.log ros2 run rviz2 rviz2 \
    -d "$PROJECT_DIR/rviz/rviz_nav.rviz" \
    --ros-args -p use_sim_time:=true
else
  echo "[5/5] Headless mode: RViz disabled."
fi

cat <<EOF

====================================================
1. In RViz, select "2D Pose Estimate" and set the robot pose.
2. Select "Nav2 Goal" and choose a reachable destination.

Logs: $LOG_DIR
Press Ctrl+C to stop all processes cleanly.
====================================================
EOF

wait
