#!/usr/bin/env bash
# Only platform dependencies and this project's paths are loaded.
MM_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
MM_RESTORE_U=0
[[ $- == *u* ]] && MM_RESTORE_U=1 && set +u
source /opt/ros/lyrical/setup.bash
if [[ -f /opt/nav2/setup.bash ]]; then source /opt/nav2/setup.bash; fi
[[ "$MM_RESTORE_U" == 1 ]] && set -u
export ROS_DOMAIN_ID="${MM_DOMAIN_ID:-43}"
export GZ_PARTITION="${MM_GZ_PARTITION:-mobile_manipulator_003}"
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
unset ROS_LOCALHOST_ONLY
export GZ_SIM_SYSTEM_PLUGIN_PATH="$MM_ROOT/build/plugins:${GZ_SIM_SYSTEM_PLUGIN_PATH:-}"
export PYTHONPATH="$MM_ROOT/src:${PYTHONPATH:-}"
export QT_QPA_PLATFORM="${MM_QT_PLATFORM:-xcb}"
