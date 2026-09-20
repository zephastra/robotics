#!/usr/bin/env bash
# The supported way to run probe_gate_state.py: sets the ROS environment and picks the
# interpreter that has rclpy. Both matter -- see the script's own preflight note.
set -uo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "$HERE/.." && pwd)"
cd "$ROOT" || exit 2
# `set -u` plus env.sh is a silent abort in this project; the window is deliberate.
set +u
# shellcheck disable=SC1091
source "$ROOT/scripts/env.sh"
set -u
exec /usr/bin/python3 "$HERE/probe_gate_state.py" "$@"
