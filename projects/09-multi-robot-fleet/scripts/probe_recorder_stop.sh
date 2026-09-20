#!/usr/bin/env bash
# Run the recorder-stop probe against the workspace's ROS environment.
set -o pipefail
SELF="${BASH_SOURCE[0]:-$0}"
HERE="$(cd "$(dirname "$SELF")" && pwd)"
ROOT="$(cd "$HERE/.." && pwd)"
cd "$ROOT" || exit 3

set +u
# shellcheck disable=SC1091
source "$HERE/env.sh"
set -u

exec /usr/bin/python3 "$HERE/probe_recorder_stop.py" "$@"
