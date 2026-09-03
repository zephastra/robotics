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
INITIAL_POSE_TIMEOUT="${AMR_INITIAL_POSE_TIMEOUT:-60}"

declare -a PROCESS_GROUPS=()

start_background() {
  local log_file="$1"
  shift
  setsid "$@" >"$LOG_DIR/$log_file" 2>&1 &
  PROCESS_GROUPS+=("$!")
}

wait_for_localization() {
  local deadline=$((SECONDS + INITIAL_POSE_TIMEOUT))

  echo "[wait] Waiting for AMCL to publish map -> odom ..."
  while ((SECONDS < deadline)); do
    # grep exits as soon as the first transform arrives. Disable pipefail only
    # inside this probe because tf2_echo can then receive SIGPIPE normally.
    if (
      set +o pipefail
      timeout 3 ros2 run tf2_ros tf2_echo map odom 2>/dev/null |
        grep -m1 -q "Translation:"
    ); then
      echo "[ok] Localization is ready."
      return 0
    fi
    sleep 1
  done

  echo "AMCL did not publish map -> odom within ${INITIAL_POSE_TIMEOUT}s." >&2
  echo "In RViz, use '2D Pose Estimate' to set the robot pose, then run this script again." >&2
  return 1
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

echo "[1/6] Starting Gazebo ..."
if [[ "${AMR_HEADLESS:-0}" == "1" ]]; then
  start_background gz.log gz sim -s --headless-rendering -r -v 4 "$PROJECT_DIR/worlds/amr_world.sdf"
else
  start_background gz.log gz sim -r -v 4 "$PROJECT_DIR/worlds/amr_world.sdf"
fi
sleep 8

echo "[2/6] Starting the bridge ..."
start_background bridge.log bash "$PROJECT_DIR/scripts/run_bridges.sh"
sleep 3

echo "[3/6] Starting TF publishers ..."
start_background static_tf.log ros2 run tf2_ros static_transform_publisher \
  --x 0 --y 0 --z 0.22 --roll 0 --pitch 0 --yaw 0 \
  --frame-id base_link --child-frame-id laser_link
start_background odom_tf.log python3 "$PROJECT_DIR/nodes/odom_to_tf.py"
sleep 2

echo "[4/6] Starting localization ..."
start_background localization.log ros2 launch nav2_bringup localization_launch.py \
  map:="$MAP_YAML" \
  use_sim_time:=true \
  params_file:="$PROJECT_DIR/config/nav2_params.yaml"
sleep 5

if [[ "${AMR_HEADLESS:-0}" != "1" ]]; then
  echo "[5/6] Starting RViz ..."
  start_background rviz.log ros2 run rviz2 rviz2 \
    -d "$PROJECT_DIR/rviz/rviz_nav.rviz" \
    --ros-args -p use_sim_time:=true
else
  echo "[5/6] Headless mode: RViz disabled."
fi

wait_for_localization

echo "[6/6] Starting navigation ..."
start_background navigation.log ros2 launch nav2_bringup navigation_launch.py \
  use_sim_time:=true \
  params_file:="$PROJECT_DIR/config/nav2_params.yaml" \
  use_keepout_zones:=false \
  use_speed_zones:=false
sleep 10

cat <<EOF

====================================================
Localization and navigation are ready.
1. Normally no manual initial pose is needed in this fixed simulation.
2. If the displayed pose is inaccurate, use "2D Pose Estimate" once.
3. Select "Nav2 Goal" and choose a reachable destination.

Logs: $LOG_DIR
Press Ctrl+C to stop all processes cleanly.
====================================================
EOF

wait
