#!/usr/bin/env bash

AMR_ROS_DISTRO="${AMR_ROS_DISTRO:-${ROS_DISTRO:-lyrical}}"
ROS_SETUP="/opt/ros/${AMR_ROS_DISTRO}/setup.bash"

if [[ ! -f "$ROS_SETUP" ]]; then
  echo "ROS 2 setup file not found: $ROS_SETUP" >&2
  return 1 2>/dev/null || exit 1
fi

AMR_MISSION_RESTORE_NOUNSET=0
if [[ $- == *u* ]]; then
  AMR_MISSION_RESTORE_NOUNSET=1
  set +u
fi

# shellcheck disable=SC1090
source "$ROS_SETUP"

if [[ -f /opt/nav2/setup.bash ]]; then
  # shellcheck disable=SC1091
  source /opt/nav2/setup.bash
elif [[ -f /opt/nav2/install/setup.bash ]]; then
  # shellcheck disable=SC1091
  source /opt/nav2/install/setup.bash
fi

if [[ "$AMR_MISSION_RESTORE_NOUNSET" == "1" ]]; then
  set -u
fi
unset AMR_MISSION_RESTORE_NOUNSET

export RMW_IMPLEMENTATION="${AMR_RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
export ROS_AUTOMATIC_DISCOVERY_RANGE="${AMR_DISCOVERY_RANGE:-LOCALHOST}"
export ROS_DOMAIN_ID="${AMR_ROS_DOMAIN_ID:-42}"
export GZ_PARTITION="${AMR_GZ_PARTITION:-amr_warehouse_002}"
unset ROS_LOCALHOST_ONLY

if grep -qi microsoft /proc/sys/kernel/osrelease 2>/dev/null; then
  export QT_QPA_PLATFORM="${AMR_QT_PLATFORM:-xcb}"
fi
