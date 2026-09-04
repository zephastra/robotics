#!/usr/bin/env bash
set -Eeuo pipefail

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DEMO_MODE="${1:-normal}"
LOG_DIR="${AMR_LOG_DIR:-$PROJECT_DIR/logs}"
MAP_FILE="$PROJECT_DIR/maps/warehouse_map.yaml"
WORLD_FILE="$PROJECT_DIR/worlds/warehouse_world.sdf"
NAV2_PARAMS="$PROJECT_DIR/config/nav2_params.yaml"
RVIZ_CONFIG="$PROJECT_DIR/rviz/warehouse_mission.rviz"
MISSION_FILE="${AMR_MISSION_FILE:-$PROJECT_DIR/src/amr_mission_manager/config/warehouse_demo.yaml}"
INITIAL_POSE_TIMEOUT="${AMR_INITIAL_POSE_TIMEOUT:-90}"
NAV2_TIMEOUT="${AMR_NAV2_TIMEOUT:-90}"

# shellcheck disable=SC1091
source "$PROJECT_DIR/scripts/amr_env.sh"

case "$DEMO_MODE" in
  normal | blocked_aisle | low_battery) ;;
  *)
    echo "Usage: bash run_demo.sh [normal|blocked_aisle|low_battery]" >&2
    exit 2
    ;;
esac

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
  echo "Stopping Project 002 processes ..."
  local process_group
  for process_group in "${PROCESS_GROUPS[@]}"; do
    if kill -0 "$process_group" 2>/dev/null; then
      kill -TERM -- "-$process_group" 2>/dev/null || true
    fi
  done
  sleep 1
  for process_group in "${PROCESS_GROUPS[@]}"; do
    kill -KILL -- "-$process_group" 2>/dev/null || true
  done
  wait 2>/dev/null || true
  exit "$status"
}

wait_for_localization() {
  local deadline=$((SECONDS + INITIAL_POSE_TIMEOUT))
  echo "[wait] Waiting for AMCL map -> odom transform ..."
  while ((SECONDS < deadline)); do
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
  return 1
}

wait_for_nav2() {
  local deadline=$((SECONDS + NAV2_TIMEOUT))
  echo "[wait] Waiting for Nav2 action server ..."
  while ((SECONDS < deadline)); do
    if timeout 3 ros2 action info /navigate_to_pose 2>/dev/null |
      grep -Eq '^Action servers: [1-9][0-9]*$'; then
      echo "[ok] Nav2 is ready."
      return 0
    fi
    sleep 1
  done
  echo "Nav2 did not become ready within ${NAV2_TIMEOUT}s." >&2
  return 1
}

trap cleanup EXIT INT TERM
mkdir -p "$LOG_DIR" "$PROJECT_DIR/reports"

for required_file in "$MAP_FILE" "$WORLD_FILE" "$NAV2_PARAMS" "$RVIZ_CONFIG" "$MISSION_FILE"; do
  [[ -s "$required_file" ]] || {
    echo "Required Project 002 file is missing: $required_file" >&2
    exit 1
  }
done

bash "$PROJECT_DIR/scripts/check_dependencies.sh"

if [[ ! -f "$PROJECT_DIR/install/setup.bash" ]]; then
  bash "$PROJECT_DIR/scripts/build.sh"
fi
set +u
# shellcheck disable=SC1091
source "$PROJECT_DIR/install/setup.bash"
set -u

echo "[1/7] Starting Project 002 Gazebo warehouse ..."
if [[ "${AMR_HEADLESS:-0}" == "1" ]]; then
  start_background gazebo.log gz sim -s --headless-rendering -r -v 3 "$WORLD_FILE"
else
  start_background gazebo.log gz sim -r -v 3 "$WORLD_FILE"
fi
sleep 8

echo "[2/7] Starting Project 002 Gazebo bridge ..."
start_background bridge.log ros2 run ros_gz_bridge parameter_bridge \
  --ros-args \
  -p config_file:="$PROJECT_DIR/config/bridge_config.yaml" \
  -r __node:=warehouse_gz_bridge
sleep 3

echo "[3/7] Starting Project 002 TF publishers ..."
start_background static_tf.log ros2 run tf2_ros static_transform_publisher \
  --x 0 --y 0 --z 0.24 --roll 0 --pitch 0 --yaw 0 \
  --frame-id base_link --child-frame-id laser_link
start_background odom_tf.log python3 "$PROJECT_DIR/scripts/odom_to_tf.py"
sleep 2

echo "[4/7] Starting Project 002 map and localization ..."
start_background localization.log ros2 launch nav2_bringup localization_launch.py \
  map:="$MAP_FILE" \
  use_sim_time:=true \
  params_file:="$NAV2_PARAMS"

if [[ "${AMR_HEADLESS:-0}" != "1" ]]; then
  echo "[5/7] Starting Project 002 RViz ..."
  start_background rviz.log ros2 run rviz2 rviz2 \
    -d "$RVIZ_CONFIG" \
    --ros-args -p use_sim_time:=true
else
  echo "[5/7] Headless mode: RViz disabled."
fi

wait_for_localization

echo "[6/7] Starting Project 002 Nav2 ..."
start_background navigation.log ros2 launch nav2_bringup navigation_launch.py \
  use_sim_time:=true \
  params_file:="$NAV2_PARAMS" \
  use_keepout_zones:=false \
  use_speed_zones:=false
wait_for_nav2

if [[ "${AMR_SKIP_MISSION:-0}" == "1" ]]; then
  echo "[7/7] Mission disabled by AMR_SKIP_MISSION=1."
else
  echo "[7/7] Starting Project 002 warehouse mission ..."
  start_background mission.log ros2 launch amr_mission_manager mission_demo.launch.py \
    mission_file:="$MISSION_FILE" \
    report_directory:="$PROJECT_DIR/reports" \
    battery_initial_percentage:="${AMR_BATTERY_INITIAL_PERCENTAGE:-100.0}" \
    battery_drain_per_second:="${AMR_BATTERY_DRAIN_PER_SECOND:-0.05}"

  if [[ "$DEMO_MODE" == "blocked_aisle" ]]; then
    start_background scenario.log bash "$PROJECT_DIR/scripts/blocked_aisle_scenario.sh"
  elif [[ "$DEMO_MODE" == "low_battery" ]]; then
    start_background scenario.log bash "$PROJECT_DIR/scripts/trigger_low_battery_scenario.sh"
  fi
fi

cat <<EOF

============================================================
Project 002 is running independently.
Demo mode: $DEMO_MODE
ROS_DOMAIN_ID: $ROS_DOMAIN_ID
Gazebo partition: $GZ_PARTITION
Logs: $LOG_DIR
Reports: $PROJECT_DIR/reports

The default mission starts automatically unless AMR_SKIP_MISSION=1.
Press Ctrl+C to stop every process started by this script.
============================================================
EOF

wait
