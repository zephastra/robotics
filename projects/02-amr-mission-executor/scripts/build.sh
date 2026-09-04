#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"

# shellcheck disable=SC1091
source "$SCRIPT_DIR/amr_env.sh"
cd "$PROJECT_DIR"

colcon build \
  --symlink-install \
  --packages-select amr_mission_manager \
  --event-handlers console_direct+

echo
echo "Build complete. Run: bash scripts/run_mission.sh"
