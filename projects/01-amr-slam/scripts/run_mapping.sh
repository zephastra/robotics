#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
cd "$PROJECT_DIR"
# shellcheck disable=SC1091
source "$PROJECT_DIR/scripts/amr_env.sh"
bash "$PROJECT_DIR/scripts/check_runtime.sh"

LOG_DIR="${AMR_LOG_DIR:-/tmp/amr_slam_logs}"
mkdir -p "$LOG_DIR"

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
  echo "Stopping AMR mapping processes ..."
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

echo "[4/5] Starting SLAM Toolbox ..."
start_background slam.log ros2 launch slam_toolbox online_async_launch.py \
  use_sim_time:=true \
  slam_params_file:="$PROJECT_DIR/config/slam_params.yaml"
sleep 5

if [[ "${AMR_HEADLESS:-0}" != "1" ]]; then
  echo "[5/5] Starting RViz ..."
  start_background rviz.log ros2 run rviz2 rviz2 \
    -d "$PROJECT_DIR/rviz/rviz_slam.rviz" \
    --ros-args -p use_sim_time:=true
else
  echo "[5/5] Headless mode: RViz disabled."
fi

cat <<EOF

====================================================
Use the keyboard to drive the robot and build a map:
  Forward i    Reverse ,    Left j    Right l    Stop k
  Initial linear speed: 0.25 m/s
  Initial angular speed: 0.35 rad/s

Save the map from another WSL terminal:
  cd "$PROJECT_DIR" && bash scripts/save_map.sh

Logs: $LOG_DIR
Press Ctrl+C to stop all processes cleanly.
====================================================
EOF

if [[ "${AMR_HEADLESS:-0}" == "1" ]]; then
  wait
else
  # Lower the default angular speed so consecutive scans overlap during turns.
  # The keyboard speed-adjustment keys still work as usual.
  ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args \
    -p speed:=0.25 \
    -p turn:=0.35
fi
