#!/usr/bin/env bash
# P6 against a LIVE fleet: does the read-only page go grey when the stream stops?
#
#   bash scripts/check_p6_live.sh
#
# Exit codes: 0 = all checks passed, 1 = a check failed, 3 = environment problem.
#
# This sources the ROS environment and then hands over to check_p6_live.py. The split is
# deliberate: the ROS environment is needed to START the fleet, and the check itself must
# never import rclpy, or it becomes part of the system it is judging. Process control and
# HTTP assertions live on the Python side because bash-side JSON parsing is where this
# project keeps losing variables to the cross-environment shell.
#
# The interruptions are real signals (SIGSTOP / SIGCONT / SIGKILL). Nothing here calls a
# fault-injection hook: a hook can only test the paths the hook knows about, and the case
# that matters is the one nobody planned for.

set -uo pipefail

SELF="${BASH_SOURCE[0]:-$0}"
ROOT="$(cd "$(dirname "$SELF")/.." && pwd)"
cd "$ROOT" || exit 3

if [ ! -d install/fleet_bringup ]; then
  echo "check_p6_live.sh: install/ is missing. Run scripts/build.sh first." >&2
  exit 3
fi

if pgrep -f "$ROOT/instal[l]" >/dev/null 2>&1; then
  echo "check_p6_live.sh: a fleet is already running under $ROOT." >&2
  echo "  This check starts its own fleet and kills parts of it; two fleets on one domain" >&2
  echo "  would make every reading meaningless." >&2
  echo "  Stop it first: bash scripts/stop_demo.sh" >&2
  exit 3
fi

# `set +u` around the source is not optional: ROS's setup.bash reads variables such as
# AMENT_TRACE_SETUP_FILES without a default and aborts the script under `set -u`.
set +u
# shellcheck disable=SC1091
source scripts/env.sh
set -u

exec /usr/bin/python3 scripts/check_p6_live.py "$@"
