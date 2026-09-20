#!/usr/bin/env bash
# P4.5 acceptance: opposing traffic, once per case, each in its own launch.
#
#   bash scripts/acceptance_p4.sh
#
# Why each case gets a fresh world: after a crossing the robots are parked at the
# release nodes on opposite sides. The second case would therefore begin with a robot
# that has to travel back across the corridor it has just used -- and it may not do that
# without a permit, by design. Running both cases in one launch tested an illegal
# starting condition, not the swapped ordering.
#
# Why the teardown is VERIFIED rather than signalled: the first per-case run sent SIGINT,
# waited 60 s, sent SIGTERM -- and started case 2 anyway. Case 2 then reported
# `nav2 action server absent` for r01 and a LATCHED sole-publisher ESTOP for r02 with
# 7597 RESOURCE_UNKNOWN verdicts: two fleets on one domain, two publishers on
# /r02/cmd_vel, two action servers on /r01/navigate_to_pose. Those numbers look like
# traffic results and are not. A case that cannot get a clean world is now NOT RUN.
#
# Everything is scoped to this run: a fresh GZ_PARTITION per case, a timestamped report
# directory, and teardown that signals only the PIDs this script started or can attribute
# to this project by path. There is no killall and no pkill (AGENTS.md rules 19 and 22).
# Project root from this file's own location. A hardcoded path makes a copy of the
# tree not a copy -- see scripts/verify_independence.sh, which is what found this.
_self="${BASH_SOURCE[0]:-$0}"
P="$(cd "$(dirname "$_self")/.." && pwd)"
mkdir -p "$P/reports"

# No `set -u`: sourcing a ROS setup.bash under -u aborts silently.
set -o pipefail
exec > "${FLEET009_P4_ACCEPT_LOG:-$P/reports/p4_acceptance_console.txt}" 2>&1
cd "$P" || { echo "MISSING PROJECT ROOT: $P"; exit 1; }

source scripts/env.sh >/dev/null 2>&1 || true
STAMP="$(date -u +%Y%m%dT%H%M%SZ)"
RUN_DIR="$P/reports/p4_acceptance_$STAMP"
mkdir -p "$RUN_DIR"
echo "run dir : $RUN_DIR"
echo "domain  : ${ROS_DOMAIN_ID:-<unset>}"

# NOTE: /usr/bin/python3, not .venv/bin/python.
#
# The project venv is deliberately ROS-free (that is how the P1 invariant "the core runs
# with no ROS" is provable). Anything that imports rclpy must run on the interpreter ROS
# was built against, and here that is /usr/bin/python3.
#
# .venv/bin/python fails with `ModuleNotFoundError: No module named 'em'` from
# rosidl_pycommon -- which reads like a broken ROS install and is really just the wrong
# interpreter. Same class of trap as the rest of this project's instrument failures: the
# message points somewhere the problem is not.
PY=/usr/bin/python3

world_pids() {
  { pgrep -f "install/fleet_ros"
    pgrep -f "install/fleet_bringup"
    pgrep -f "install/fleet_core"
    pgrep -f "gz sim"
  } 2>/dev/null | sort -u
}

world_is_gone() { [ -z "$(world_pids)" ]; }

teardown_case() {
  local launch_pid="$1" waited=0 left

  echo "--- teardown: signalling only this case's launch pid ($launch_pid) ---"
  kill -INT "$launch_pid" 2>/dev/null
  while [ "$waited" -lt 45 ]; do
    kill -0 "$launch_pid" 2>/dev/null || break
    sleep 1
    waited=$((waited + 1))
  done
  if kill -0 "$launch_pid" 2>/dev/null; then
    echo "launch alive after ${waited}s of SIGINT; SIGTERM to pid $launch_pid only"
    kill -TERM "$launch_pid" 2>/dev/null
  fi

  waited=0
  while [ "$waited" -lt 40 ]; do
    world_is_gone && break
    sleep 1
    waited=$((waited + 1))
  done

  if ! world_is_gone; then
    left="$(world_pids)"
    echo "the world is still up after SIGINT+SIGTERM (${waited}s). Remaining pids:"
    echo "$left" | tr '\n' ' '
    echo
    pgrep -a -f "install/fleet" | head -25
    pgrep -a -f "gz sim" | head -5
    echo "SIGKILL to exactly those pids. This is cleanup of processes THIS script started,"
    echo "addressed by pid. There is still no killall and no pkill."
    kill -KILL $left 2>/dev/null
    sleep 5
  fi

  if ! world_is_gone; then
    echo "FATAL: the world survived SIGKILL. Refusing to run another case against it:"
    echo "       two fleets on one domain put two publishers on /rXX/cmd_vel, which latches"
    echo "       this project's own sole-publisher ESTOP, and two action servers on"
    echo "       /rXX/navigate_to_pose. The result would be attributed to the traffic logic."
    pgrep -a -f "install/fleet" | head -20
    return 1
  fi

  echo "teardown verified clean: no process from this project, no gz sim"
  sleep 5
  return 0
}

run_case() {
  local order="$1" idx="$2"
  local case_dir="$RUN_DIR/$idx"
  local log="$case_dir/fleet_launch.log"
  mkdir -p "$case_dir"

  echo
  echo "=== case $idx: order=$order (bias 2 s) -- fresh launch ==="
  export GZ_PARTITION="fleet009_p4_${STAMP}_${idx}"
  echo "partition: $GZ_PARTITION"

  ros2 launch fleet_bringup fleet.launch.py \
    robots:=r01,r02 \
    use_sim_time:=true \
    occupancy_settle_s:=5.0 \
    nav2_log_level:=info \
    > "$log" 2>&1 &
  local launch_pid=$!
  echo "launch pid $launch_pid"

  # Give the launch a moment to fail loudly here rather than inside the harness.
  sleep 45
  if ! kill -0 "$launch_pid" 2>/dev/null; then
    echo "FATAL: launch exited during start-up. Last 60 lines:"
    tail -60 "$log"
    return 3
  fi

  "$PY" scripts/acceptance_p4.py \
    --order "$order" --bias-s 2.0 --timeout-s 150 \
    --report-dir "$case_dir" 2>&1 | tee "$case_dir.txt"
  local rc=${PIPESTATUS[0]}
  echo "CASE_RC=$rc"

  if ! teardown_case "$launch_pid"; then
    return 3
  fi
  return "$rc"
}

CASE2_RC="not run"
run_case r01_first case1
CASE1_RC=$?

if [ "$CASE1_RC" -eq 3 ]; then
  echo
  echo "case 2 NOT RUN: case 1 did not leave a clean world (or never came up)."
  echo "              Reported rather than attempted -- a contaminated world produces"
  echo "              numbers that look like traffic results and are not."
else
  run_case r02_first case2
  CASE2_RC=$?
fi

echo
echo "=== SUMMARY ==="
echo "run_dir=$RUN_DIR"
echo "case1_rc=$CASE1_RC  case2_rc=$CASE2_RC"
echo "  (0 = pass, 4 = the harness reported a failure, 3 = did not run / never came up,"
echo "   139 = the harness crashed -- see the note in acceptance_p4.py if 139 reappears)"
for f in "$RUN_DIR"/case*/verdict-*.json; do
  [ -f "$f" ] || continue
  echo "--- $f ---"
  "$PY" -c "
import json,sys
d=json.load(open('$f'))
print('  order            :', d['order'])
print('  both_completed   :', d['both_completed'])
if d.get('drive_error'):
    print('  DRIVE ERROR      :', d['drive_error'])
print('  samples          :', d['samples'])
print('  simultaneous     :', d['simultaneous_occupancy_samples'])
print('  unauthorised     :', d['unauthorised_entry_samples'])
print('  queued samples   :', d['busy_or_queued_samples'])
print('  truth usable     :', d['truth']['usable_for_cross_check'])
print('  truth note       :', d['truth']['note'])
for r in d['runs']:
    pp = r.get('permit_path') or {}
    print('  run              :', r['robot'], r['direction'], 'exit=', r['exit_code'])
    print('     permit path   :', {k: pp.get(k) for k in
          ('driver_permit_id','driver_renewals','gate_grants_seen','gate_guard_installed','mismatch')})
g = d.get('gate', {})
print('  gate reasons     :', g.get('reason_counts'))
print('  gate last        :', {k: {kk: vv for kk, vv in v.items() if kk != 'detail'}
                               for k, v in (g.get('last_verdict_per_robot') or {}).items()})
for v in (g.get('first_non_ok_verdicts') or [])[:3]:
    print('   refusal:', v['robot'], v['mode'], v['reason'])
    print('            ', v['detail'][:220])
"
done

echo
echo "=== ALL DONE ==="
