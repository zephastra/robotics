#!/usr/bin/env bash

# Shared ROS environment for the AMR simulation. The defaults are intentionally
# local-only because every node in this project runs inside the same WSL distro.
AMR_ROS_DISTRO="${AMR_ROS_DISTRO:-${ROS_DISTRO:-lyrical}}"
ROS_SETUP="/opt/ros/${AMR_ROS_DISTRO}/setup.bash"

if [[ ! -f "$ROS_SETUP" ]]; then
  echo "ROS 2 setup file not found: $ROS_SETUP" >&2
  return 1 2>/dev/null || exit 1
fi

AMR_RESTORE_NOUNSET=0
if [[ $- == *u* ]]; then
  AMR_RESTORE_NOUNSET=1
  set +u
fi

# ROS-generated setup files may reference variables before defining them.
# shellcheck disable=SC1090
source "$ROS_SETUP"

if [[ -f /opt/nav2/setup.bash ]]; then
  # shellcheck disable=SC1091
  source /opt/nav2/setup.bash
elif [[ -f /opt/nav2/install/setup.bash ]]; then
  # shellcheck disable=SC1091
  source /opt/nav2/install/setup.bash
fi

if [[ "$AMR_RESTORE_NOUNSET" == "1" ]]; then
  set -u
fi
unset AMR_RESTORE_NOUNSET

export RMW_IMPLEMENTATION="${AMR_RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
export ROS_AUTOMATIC_DISCOVERY_RANGE="${AMR_DISCOVERY_RANGE:-LOCALHOST}"
unset ROS_LOCALHOST_ONLY

# Gazebo and RViz use GLX through OGRE. On WSLg, Qt's Wayland backend can hand
# them an incompatible native window handle, so use XWayland/XCB by default.
if grep -qi microsoft /proc/sys/kernel/osrelease 2>/dev/null; then
  export QT_QPA_PLATFORM="${AMR_QT_PLATFORM:-xcb}"
fi
