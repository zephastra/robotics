#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
TRIGGER_DELAY_SECONDS="${AMR_LOW_BATTERY_DELAY_SECONDS:-25}"
# shellcheck disable=SC1091
source "$SCRIPT_DIR/amr_env.sh"

sleep "$TRIGGER_DELAY_SECONDS"
for _attempt in $(seq 1 30); do
  if ros2 service list | grep -qx "/mission/battery/force_low"; then
    ros2 service call /mission/battery/force_low std_srvs/srv/Trigger
    exit 0
  fi
  sleep 1
done

echo "Battery force-low service did not become available." >&2
exit 1
