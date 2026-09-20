#!/usr/bin/env bash
# Bring up the whole 009 demo: the world, three robots, the traffic coordinator, the task
# dispatcher and the read-only dashboard.
#
#   bash scripts/run_demo.sh                 # r01, r02, r03
#   bash scripts/run_demo.sh r01,r02         # fewer robots
#
# Leaves everything running in the foreground. Ctrl-C once, or run scripts/stop_demo.sh
# from another shell. The teardown is verified either way: a world that is still alive
# after the launch process exits is a world whose numbers will contaminate the next run.
#
# What this does NOT do: it does not submit any task and it does not move anything. Use
# fleet_cli to do that, so that "the demo started" and "a task ran" stay separate claims.
set -uo pipefail

SELF="${BASH_SOURCE[0]:-$0}"
ROOT="$(cd "$(dirname "$SELF")/.." && pwd)"
cd "$ROOT" || exit 3

ROBOTS="${1:-r01,r02,r03}"
DASHBOARD_PORT="${DASHBOARD_PORT:-8080}"
RUN_DIR="reports/demo_$(date -u +%Y%m%dT%H%M%SZ)"
mkdir -p "$RUN_DIR"

if [ ! -d install/fleet_bringup ]; then
  echo "run_demo.sh: install/ is missing. Run scripts/build.sh first." >&2
  exit 3
fi

# shellcheck disable=SC1091
# set +u around the source is not optional: ROS's setup.bash reads variables such as
# AMENT_TRACE_SETUP_FILES without a default and aborts under `set -u`.
set +u
source scripts/env.sh
set -u

echo "=== 009 demo ==="
echo "  robots        : $ROBOTS"
echo "  run dir       : $RUN_DIR"
echo "  dashboard     : http://127.0.0.1:$DASHBOARD_PORT/  (read-only)"
echo "  RViz          : assets/rviz/fleet.rviz"
echo
echo "  the dashboard binds loopback only; it refuses to start on any other address,"
echo "  because it shows fleet state and has no authentication."
echo

# The dashboard serves GET on three paths and returns 405 for everything else. If it is
# already running on the port, say so rather than starting a second one that fails
# obscurely.
if command -v ss >/dev/null 2>&1 && ss -ltn 2>/dev/null | grep -q ":$DASHBOARD_PORT "; then
  echo "run_demo.sh: port $DASHBOARD_PORT is already in use; not starting a dashboard." >&2
  echo "             Stop whatever owns it, or set DASHBOARD_PORT to something else." >&2
  DASHBOARD_PORT=""
fi

if [ -n "$DASHBOARD_PORT" ]; then
  ros2 run fleet_tools fleet_dashboard --port "$DASHBOARD_PORT" \
    > "$RUN_DIR/dashboard.log" 2>&1 &
  DASH_PID=$!
  echo "dashboard pid $DASH_PID"
fi

ros2 launch fleet_bringup fleet.launch.py "robots:=$ROBOTS" \
  > "$RUN_DIR/fleet_launch.log" 2>&1 &
LAUNCH_PID=$!
echo "launch pid    $LAUNCH_PID"
echo

cleanup() {
  echo
  echo "=== stopping ==="
  [ -n "${DASH_PID:-}" ] && kill -INT "$DASH_PID" 2>/dev/null
  kill -INT "$LAUNCH_PID" 2>/dev/null
  for _ in $(seq 1 45); do
    kill -0 "$LAUNCH_PID" 2>/dev/null || break
    sleep 1
  done
  if kill -0 "$LAUNCH_PID" 2>/dev/null; then
    echo "launch still alive after 45 s of SIGINT; escalating on pid $LAUNCH_PID only"
    kill -TERM "$LAUNCH_PID" 2>/dev/null
    for _ in $(seq 1 30); do
      kill -0 "$LAUNCH_PID" 2>/dev/null || break
      sleep 1
    done
  fi
  kill -KILL "$LAUNCH_PID" 2>/dev/null
  sleep 3

  local left
  left="$(pgrep -f "$ROOT/instal[l]" | wc -l)"
  echo "leftover project processes: $left"
  echo "gz processes: $(pgrep -f 'gz-sim-mai[n]' | wc -l)"
  if [ "$left" -ne 0 ]; then
    echo "WARNING: the world is still up. The next run's numbers would be contaminated."
    echo "         Not cleaning up globally on purpose -- that would kill unrelated work."
    echo "         Inspect with: pgrep -a -f '$ROOT'"
  else
    echo "teardown verified clean"
  fi
}
trap cleanup EXIT

echo "fleet is starting; watch $RUN_DIR/fleet_launch.log"
echo "press Ctrl-C to stop"
echo
wait "$LAUNCH_PID"
