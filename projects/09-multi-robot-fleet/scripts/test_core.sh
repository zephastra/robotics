#!/usr/bin/env bash
# 009 core test suite. NO ROS. NO simulator. Fast enough to run on every edit.
#
#   bash scripts/test_core.sh
#
# Exit codes: 0 = all core tests passed, 3 = environment problem, 4 = test failures.

set -uo pipefail

SELF="${BASH_SOURCE[0]:-$0}"
ROOT="$(cd "$(dirname "$SELF")/.." && pwd)"
cd "$ROOT" || exit 3

if [ ! -x .venv/bin/python ]; then
  echo "test_core.sh: .venv missing; create it with /usr/bin/python3 (3.14)." >&2
  exit 3
fi

# Hard isolation check: the P1 core must import with NO ROS environment at all.
if [ -n "${ROS_DISTRO:-}" ]; then
  echo "test_core.sh: refusing to run with ROS sourced (ROS_DISTRO=$ROS_DISTRO)." >&2
  echo "              P1 core tests must pass in a plain interpreter." >&2
  exit 3
fi

echo "interpreter: $(.venv/bin/python --version)  ($(command -v .venv/bin/python))"
.venv/bin/python -m pytest -p no:cacheprovider --color=no "$@"
RC=$?
echo "PYTEST_EXIT=$RC"
if [ "$RC" -ne 0 ]; then
  exit 4
fi
exit 0
