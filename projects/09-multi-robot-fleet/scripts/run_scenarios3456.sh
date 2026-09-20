#!/usr/bin/env bash
# Run one P5 directed scenario (3-6). Sources ROS here and only here, then hands over
# to the harness with the system interpreter -- the project .venv deliberately has no
# ROS in it, and using .venv/bin/python on a script that imports rclpy fails with a
# misleading `No module named 'em'`.
#
#   bash scripts/run_scenarios3456.sh capability|low_battery|charge_queue|restart
set -o pipefail
SELF="${BASH_SOURCE[0]:-$0}"
HERE="$(cd "$(dirname "$SELF")" && pwd)"
ROOT="$(cd "$HERE/.." && pwd)"
cd "$ROOT" || exit 3

set +u
# shellcheck disable=SC1091
source "$HERE/env.sh"
set -u

exec /usr/bin/python3 "$HERE/run_scenarios3456.py" "$@"
