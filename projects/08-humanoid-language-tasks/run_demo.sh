#!/usr/bin/env bash
# 008 demo entry point. Forwards every argument to the package CLI.
#
#   bash run_demo.sh --backend fake --planner rule --instruction "把箱子搬到 B 台"
#
# Requires .venv. Never uses the system Python.
set -euo pipefail

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PYTHON="${PROJECT_ROOT}/.venv/bin/python"

if [ ! -x "${PYTHON}" ]; then
  echo "no virtual environment at ${PROJECT_ROOT}/.venv" >&2
  echo "run: bash scripts/setup.sh --profile core" >&2
  exit 1
fi

# The script must not depend on the caller's working directory.
cd "${PROJECT_ROOT}"

exec "${PYTHON}" -m humanoid008.app "$@"
