#!/usr/bin/env bash
# The supported way to run probe_truth_slots.py: sets the ROS environment and picks the
# interpreter that has rclpy. Both matter -- a probe that cannot import rclpy reports
# "the system is silent", which is a lie about the system.
set -uo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "$HERE/.." && pwd)"
cd "$ROOT" || exit 2
# `set -u` plus env.sh is a silent abort in this project; the window is deliberate.
set +u
# shellcheck disable=SC1091
source "$ROOT/scripts/env.sh"
set -u
exec /usr/bin/python3 "$HERE/probe_truth_slots.py" "$@"
