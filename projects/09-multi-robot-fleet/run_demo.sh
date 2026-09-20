#!/usr/bin/env bash
# 009 demo runner.
#
#   bash run_demo.sh --backend fake --robots 2 --scenario opposing_requests
#
# P1 implements ONLY the `fake` backend. Any other backend exits with code 3
# rather than silently falling back -- a silent fallback would let a report claim
# gazebo_nav2 evidence that was never produced.
#
# Exit codes (TEST_AND_ACCEPTANCE §9):
#   0 = the declared acceptance for this invocation passed
#   2 = input/config error
#   3 = dependency / startup problem
#   4 = task/physical acceptance failed
#   5 = infrastructure / sampling / budget problem
# 130 = user interrupted

set -uo pipefail

BACKEND="fake"
ROBOTS=2
SCENARIO="opposing_requests"
HEADLESS=0
ALLOW_FAULT=0
RUN_ID=""

while [ $# -gt 0 ]; do
  case "$1" in
    --backend)  BACKEND="$2"; shift 2 ;;
    --robots)   ROBOTS="$2"; shift 2 ;;
    --scenario) SCENARIO="$2"; shift 2 ;;
    --headless) HEADLESS=1; shift ;;
    --allow-fault-injection) ALLOW_FAULT=1; shift ;;
    --run-id)   RUN_ID="$2"; shift 2 ;;
    -h|--help)
      sed -n '2,14p' "$0"; exit 0 ;;
    *) echo "run_demo.sh: unknown argument: $1" >&2; exit 2 ;;
  esac
done

SELF="${BASH_SOURCE[0]:-$0}"
ROOT="$(cd "$(dirname "$SELF")" && pwd)"
cd "$ROOT" || exit 3

if [ ! -x .venv/bin/python ]; then
  echo "run_demo.sh: .venv missing; run scripts/bootstrap.sh first" >&2
  exit 3
fi

case "$BACKEND" in
  fake)
    if [ "$HEADLESS" -eq 0 ]; then
      echo "run_demo.sh: note -- the fake backend has no GUI; proceeding."
    fi
    exec .venv/bin/python scripts/fake_demo.py \
      --scenario "$SCENARIO" --robots "$ROBOTS" ${RUN_ID:+--runtime runtime/$RUN_ID}
    ;;
  gazebo_nav2)
    echo "run_demo.sh: backend 'gazebo_nav2' is not implemented yet (P2)." >&2
    echo "             Refusing to fall back to 'fake': that would mislabel evidence." >&2
    exit 3
    ;;
  *)
    echo "run_demo.sh: unknown backend '$BACKEND' (expected: fake | gazebo_nav2)" >&2
    exit 2
    ;;
esac
