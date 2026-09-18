#!/usr/bin/env bash
# 008 test entry point.
#
#   bash run_tests.sh --suite core
#   bash run_tests.sh --suite simulation --seed 7
#
# core        : unit tests only. No MuJoCo, no network.
# integration : wiring tests (none defined yet).
# simulation  : physics tests (none defined yet; arrives with P4).
#
# A suite with no tests exits non-zero instead of pretending to pass.
set -euo pipefail

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PYTHON="${PROJECT_ROOT}/.venv/bin/python"
SUITE="core"

if [ ! -x "${PYTHON}" ]; then
  echo "no virtual environment at ${PROJECT_ROOT}/.venv" >&2
  echo "run: bash scripts/setup.sh --profile core" >&2
  exit 1
fi

# The script must not depend on the caller's working directory.
cd "${PROJECT_ROOT}"

while [ $# -gt 0 ]; do
  case "$1" in
    --suite)   SUITE="${2:-}"; shift 2 ;;
    --suite=*) SUITE="${1#*=}"; shift ;;
    *)         shift ;;
  esac
done

case "${SUITE}" in
  core)        exec "${PYTHON}" -m pytest -q tests/unit ;;
  integration) exec "${PYTHON}" -m pytest -q tests/integration ;;
  simulation)  exec "${PYTHON}" -m pytest -q tests/simulation ;;
  *) echo "unknown suite: '${SUITE}' (expected core, integration or simulation)" >&2; exit 2 ;;
esac
