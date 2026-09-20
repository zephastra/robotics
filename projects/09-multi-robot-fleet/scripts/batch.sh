#!/usr/bin/env bash
# Run the frozen case list of TEST_AND_ACCEPTANCE section 5.
#
#   bash scripts/batch.sh --dry-run
#   bash scripts/batch.sh --only N02,I01
#   bash scripts/batch.sh
#
# Sources ROS here and only here, then hands over to the system interpreter. The project
# .venv is deliberately ROS-free, and running a script that imports rclpy with it fails with
# a misleading `No module named 'em'` -- which is exactly what happened the first time this
# batch was run, five cases at a time, because it was invoked as
# `.venv/bin/python scripts/batch.py`. `batch.py` now refuses with a dependency error instead
# of recording five NOT_RUN verdicts for an environment mistake.
set -o pipefail
SELF="${BASH_SOURCE[0]:-$0}"
HERE="$(cd "$(dirname "$SELF")" && pwd)"
ROOT="$(cd "$HERE/.." && pwd)"
cd "$ROOT" || exit 3

set +u
# shellcheck disable=SC1091
source "$HERE/env.sh"
set -u

exec /usr/bin/python3 "$HERE/batch.py" "$@"
