#!/usr/bin/env bash
set -eo pipefail
source "$(dirname "${BASH_SOURCE[0]}")/scripts/env.sh"
cd "$MM_ROOT"
exec 9>"/tmp/mm003-${ROS_DOMAIN_ID}.lock"
flock -n 9 || { echo '003 is already running in this ROS domain.'; exit 1; }
run_id="$(date +%Y%m%d-%H%M%S)-$$"
export MM_REPORT_DIR="$MM_ROOT/reports/$run_id"
export ROS_LOG_DIR="$MM_ROOT/logs/$run_id/ros"
mkdir -p "$MM_REPORT_DIR" "$ROS_LOG_DIR"
log_dir="$MM_ROOT/logs/$run_id"
echo "Project 003 | ROS_DOMAIN_ID=$ROS_DOMAIN_ID | GZ_PARTITION=$GZ_PARTITION"
echo "Logs: $log_dir"
echo "Report: $MM_REPORT_DIR/mission.json"
bash scripts/build.sh >"$log_dir/build.log" 2>&1 || { tail -60 "$log_dir/build.log"; exit 1; }
pids=()
cleanup() {
  trap - EXIT INT TERM
  for pid in "${pids[@]}"; do kill -INT -- "-$pid" 2>/dev/null || true; done
  sleep 2
  for pid in "${pids[@]}"; do kill -TERM -- "-$pid" 2>/dev/null || true; done
  sleep 1
  for pid in "${pids[@]}"; do kill -KILL -- "-$pid" 2>/dev/null || true; done
  wait 2>/dev/null || true
}
trap cleanup EXIT
trap 'exit 130' INT TERM
gz_args=(-r -v 2 worlds/workcell.sdf)
if [[ "${MM_HEADLESS:-0}" == 1 ]]; then gz_args+=(-s --headless-rendering); fi
setsid gz sim "${gz_args[@]}" >"$log_dir/gazebo.log" 2>&1 & pids+=("$!")
setsid ros2 launch "$MM_ROOT/launch/bringup.launch.py" >"$log_dir/bringup.log" 2>&1 & pids+=("$!")
if [[ "${MM_HEADLESS:-0}" != 1 ]]; then
  setsid rviz2 -d "$MM_ROOT/config/view.rviz" --ros-args -p use_sim_time:=true >"$log_dir/rviz.log" 2>&1 & pids+=("$!")
fi
if [[ "${MM_DIAGNOSTIC_ONLY:-0}" == 1 ]]; then
  echo 'Diagnostic mode: simulation only. Ctrl+C to stop.'
  wait
else
  setsid python3 -u -m mobile_manipulator.mission >"$log_dir/mission.log" 2>&1 & mission_pid=$!; pids+=("$mission_pid")
  setsid tail -n +1 -f "$log_dir/mission.log" & pids+=("$!")
  result=0; wait "$mission_pid" || result=$?
  echo "Mission finished (exit $result). Report: $MM_REPORT_DIR/mission.json"
  if [[ "${MM_AUTO_EXIT:-0}" == 1 ]]; then exit "$result"; fi
  echo 'Inspect the scene. Ctrl+C to close this run. Restart the script for a fresh mission.'
  wait
fi
