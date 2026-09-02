#!/usr/bin/env bash
set -Eeuo pipefail

if [[ "${AMR_SKIP_RUNTIME_CHECK:-0}" == "1" ]]; then
  exit 0
fi

declare -a CONFLICTS=()

pgrep -x gz-sim-main >/dev/null 2>&1 && CONFLICTS+=("Gazebo Sim")
pgrep -f '/ros_gz_bridge/parameter_bridge|ros2 run ros_gz_bridge' >/dev/null 2>&1 \
  && CONFLICTS+=("ros_gz_bridge")
pgrep -f '/slam_toolbox/|ros2 launch slam_toolbox' >/dev/null 2>&1 \
  && CONFLICTS+=("slam_toolbox")
pgrep -f '[/]odom_to_tf.py' >/dev/null 2>&1 && CONFLICTS+=("odom_to_tf")

if (( ${#CONFLICTS[@]} > 0 )); then
  printf 'Cannot start: an existing AMR/ROS session was detected (%s).\n' \
    "$(IFS=', '; echo "${CONFLICTS[*]}")" >&2
  echo "Return to the previous launch terminal and press Ctrl+C, then try again." >&2
  exit 1
fi
