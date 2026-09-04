#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
TARGET_TASK="${AMR_BLOCK_TARGET_TASK:-shelf_a_pickup}"
TASK_WAIT_TIMEOUT_SECONDS="${AMR_BLOCK_TASK_WAIT_TIMEOUT_SECONDS:-300}"
RECOVERY_WAIT_TIMEOUT_SECONDS="${AMR_BLOCK_RECOVERY_TIMEOUT_SECONDS:-180}"

# shellcheck disable=SC1091
source "$SCRIPT_DIR/amr_env.sh"

read_status() {
  timeout 5 ros2 topic echo /mission/status std_msgs/msg/String \
    --once --full-length 2>/dev/null || true
}

wait_for_status() {
  local timeout_seconds="$1"
  local description="$2"
  shift 2
  local deadline=$((SECONDS + timeout_seconds))
  local status

  while ((SECONDS < deadline)); do
    status="$(read_status)"
    local matched=true
    local pattern
    for pattern in "$@"; do
      if ! grep -Fq "$pattern" <<<"$status"; then
        matched=false
        break
      fi
    done
    if [[ "$matched" == true ]]; then
      return 0
    fi
    sleep 1
  done

  echo "Timed out waiting for $description." >&2
  return 1
}

echo "Waiting for mission task '$TARGET_TASK' to start navigation ..."
wait_for_status \
  "$TASK_WAIT_TIMEOUT_SECONDS" \
  "task '$TARGET_TASK' to navigate" \
  "\"state\": \"NAVIGATING\"" \
  "\"current_task\": \"$TARGET_TASK\""

bash "$SCRIPT_DIR/block_aisle.sh"
echo "Aisle will remain blocked until the mission enters clearance recovery."

if ! wait_for_status \
  "$RECOVERY_WAIT_TIMEOUT_SECONDS" \
  "the blocked-path recovery state" \
  "\"blocked_recoveries\": 1"; then
  bash "$SCRIPT_DIR/clear_aisle.sh"
  exit 1
fi

echo "Blocked-path recovery detected; clearing the aisle."
bash "$SCRIPT_DIR/clear_aisle.sh"
