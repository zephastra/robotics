#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
PACKAGE_DIR="$PROJECT_DIR/src/amr_mission_manager"
MISSION_FILE="${1:-$PACKAGE_DIR/config/warehouse_demo.yaml}"
REPORT_DIR="${AMR_MISSION_REPORT_DIR:-$PROJECT_DIR/reports}"

# shellcheck disable=SC1091
source "$SCRIPT_DIR/amr_env.sh"

[[ -s "$MISSION_FILE" ]] || {
  echo "Mission file is missing or empty: $MISSION_FILE" >&2
  exit 1
}

if [[ ! -f "$PROJECT_DIR/install/setup.bash" ]]; then
  echo "Mission package is not built yet. Building now ..."
  bash "$SCRIPT_DIR/build.sh"
fi

# shellcheck disable=SC1091
set +u
source "$PROJECT_DIR/install/setup.bash"
set -u
mkdir -p "$REPORT_DIR"

if ! timeout 8 ros2 action info /navigate_to_pose 2>/dev/null |
  grep -Eq '^Action servers: [1-9][0-9]*$'; then
  echo "Nav2 /navigate_to_pose is not ready." >&2
  echo "Start this project first: cd ~/projects/002_amr_mission_executor && AMR_SKIP_MISSION=1 bash run_demo.sh" >&2
  exit 1
fi

echo "Starting warehouse mission: $MISSION_FILE"
echo "Reports: $REPORT_DIR"
echo "Press Ctrl+C to stop the mission nodes."

exec ros2 launch amr_mission_manager mission_demo.launch.py \
  mission_file:="$MISSION_FILE" \
  report_directory:="$REPORT_DIR" \
  battery_initial_percentage:="${AMR_BATTERY_INITIAL_PERCENTAGE:-100.0}" \
  battery_drain_per_second:="${AMR_BATTERY_DRAIN_PER_SECOND:-0.05}"
