#!/usr/bin/env bash
set -uo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Support both the organized GitHub layout and the flat local development copy.
if [[ -f "$SCRIPT_DIR/amr_env.sh" && -d "$SCRIPT_DIR/../config" && -d "$SCRIPT_DIR/../worlds" ]]; then
  PROJECT_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
  ENV_FILE="$SCRIPT_DIR/amr_env.sh"
elif [[ -f "$SCRIPT_DIR/amr_env.sh" ]]; then
  PROJECT_DIR="$SCRIPT_DIR"
  ENV_FILE="$SCRIPT_DIR/amr_env.sh"
else
  echo "Unable to locate amr_env.sh." >&2
  exit 2
fi

# shellcheck disable=SC1090
source "$ENV_FILE"

CHECK_TIMEOUT="${AMR_CHECK_TIMEOUT:-6}"
PASS_COUNT=0
WARN_COUNT=0
FAIL_COUNT=0

pass() {
  PASS_COUNT=$((PASS_COUNT + 1))
  printf '[PASS] %s\n' "$1"
}

warn() {
  WARN_COUNT=$((WARN_COUNT + 1))
  printf '[WARN] %s\n' "$1"
}

fail() {
  FAIL_COUNT=$((FAIL_COUNT + 1))
  printf '[FAIL] %s\n' "$1"
}

run_ros2() {
  timeout "$CHECK_TIMEOUT" ros2 "$@"
}

check_topic_publisher() {
  local topic="$1"
  local label="$2"
  local output
  local publisher_count

  if ! output="$(run_ros2 topic info "$topic" 2>&1)"; then
    fail "$label: topic $topic is unavailable"
    return
  fi

  publisher_count="$(sed -n 's/^Publisher count: //p' <<<"$output" | head -n 1)"
  if [[ "$publisher_count" =~ ^[0-9]+$ ]] && ((publisher_count > 0)); then
    pass "$label: $topic has $publisher_count publisher(s)"
  else
    fail "$label: $topic has no publisher"
  fi
}

check_lifecycle_active() {
  local node="$1"
  local output

  if output="$(run_ros2 lifecycle get "$node" 2>&1)" && [[ "$output" == *"active [3]"* ]]; then
    pass "$node lifecycle state is active"
  else
    fail "$node is not active (${output:-no response})"
  fi
}

check_transform() {
  local parent="$1"
  local child="$2"

  if (
    set +o pipefail
    timeout "$CHECK_TIMEOUT" ros2 run tf2_ros tf2_echo "$parent" "$child" 2>/dev/null |
      grep -m1 -q "Translation:"
  ); then
    pass "TF $parent -> $child is available"
  else
    fail "TF $parent -> $child is unavailable"
  fi
}

check_navigation_action() {
  local output

  if ! output="$(run_ros2 action info /navigate_to_pose 2>&1)"; then
    fail "/navigate_to_pose action is unavailable"
    return
  fi

  if grep -Eq '^Action servers: [1-9][0-9]*$' <<<"$output"; then
    pass "/navigate_to_pose has an action server"
  else
    fail "/navigate_to_pose has no action server"
  fi

  if grep -Eq '^[[:space:]]+/rviz$' <<<"$output"; then
    pass "RViz Navigation 2 goal client is connected"
  else
    warn "RViz is not listed as a goal client; confirm the Navigation 2 panel is open"
  fi
}

if [[ "${1:-}" == "--help" || "${1:-}" == "-h" ]]; then
  cat <<EOF
Usage: bash scripts/check_nav2.sh

Run this command in a second terminal while scripts/run_nav2.sh is active.
Set AMR_CHECK_TIMEOUT to change the per-check timeout (default: ${CHECK_TIMEOUT}s).
EOF
  exit 0
fi

echo "AMR Nav2 runtime diagnostics"
echo "Project: $PROJECT_DIR"
echo

if command -v ros2 >/dev/null 2>&1; then
  pass "ros2 command is available"
else
  fail "ros2 command is unavailable"
  printf '\nSummary: %d passed, %d warning(s), %d failed.\n' \
    "$PASS_COUNT" "$WARN_COUNT" "$FAIL_COUNT"
  exit 1
fi

for package in nav2_bringup nav2_amcl nav2_controller nav2_map_server nav2_rviz_plugins; do
  if run_ros2 pkg prefix "$package" >/dev/null 2>&1; then
    pass "ROS package $package is available"
  else
    fail "ROS package $package is unavailable"
  fi
done

echo
echo "Topics"
check_topic_publisher /clock "Simulation clock"
check_topic_publisher /map "Saved map"
check_topic_publisher /scan "Laser scan"
check_topic_publisher /odom "Wheel odometry"
check_topic_publisher /cmd_vel "Velocity command chain"

echo
echo "TF chain"
check_transform map odom
check_transform odom base_link
check_transform base_link laser_link

echo
echo "Lifecycle nodes"
check_lifecycle_active /map_server
check_lifecycle_active /amcl
check_lifecycle_active /planner_server
check_lifecycle_active /controller_server
check_lifecycle_active /bt_navigator

echo
echo "Navigation action"
check_navigation_action

echo
printf 'Summary: %d passed, %d warning(s), %d failed.\n' \
  "$PASS_COUNT" "$WARN_COUNT" "$FAIL_COUNT"

if ((FAIL_COUNT > 0)); then
  echo "Nav2 is not fully ready. Inspect /tmp/amr_nav_logs/ for details."
  exit 1
fi

echo "Nav2 is ready."
