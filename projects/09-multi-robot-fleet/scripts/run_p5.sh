#!/usr/bin/env bash
# P5 acceptance probe: three robots, the full submit path, the refusals, a cancel and
# a fault injection -- and then the outcome, which is the part that has never been read.
#
# Changes since the last attempt, each from a specific observation:
#   * the previous ledger is moved aside, not deleted, so a stale FAILED row from an
#     earlier run cannot be mistaken for this run's result (and is kept as evidence);
#   * the CLI now budgets 25 s for service discovery instead of 5, because at ~55
#     participants a single `wait_for_service` was reporting "no service" on a node
#     that was demonstrably alive;
#   * every CLI call's exit code and full output is kept, so a lost reading is visible
#     as a lost reading rather than silently becoming "the fleet did nothing".
set -uo pipefail

# Project root from this file's own location; the transcript goes into reports/ so a
# copy of the tree stays self-contained.
_self="${BASH_SOURCE[0]:-$0}"
P="$(cd "$(dirname "$_self")/.." && pwd)"
mkdir -p "$P/reports"
OUT="${FLEET009_P5_LOG:-$P/reports/p5_accept_console.txt}"
cd "$P" || exit 3
RUN_DIR="reports/p5_accept_$(date -u +%Y%m%dT%H%M%SZ)"
mkdir -p "$RUN_DIR"
CLI="./install/fleet_tools/lib/fleet_tools/fleet_cli"

set +u
source scripts/env.sh >/dev/null 2>&1
source install/setup.bash >/dev/null 2>&1
set -u

{
echo "run_dir: $RUN_DIR"
echo "=== archive the previous ledger (kept, not deleted) ==="
for f in runtime/fleet.sqlite runtime/fleet.sqlite-wal runtime/fleet.sqlite-shm; do
  [ -e "$f" ] && mv "$f" "$RUN_DIR/prev_$(basename "$f")" && echo "moved aside: $f"
done
[ -e runtime/events.jsonl ] && mv runtime/events.jsonl "$RUN_DIR/prev_events.jsonl" && echo "moved aside: events"
echo

cat > /tmp/ready3.py <<'PY'
import json, subprocess, sys, time
cli, deadline = sys.argv[1], float(sys.argv[2])
end = time.monotonic() + deadline
last = "no snapshot yet"
while time.monotonic() < end:
    try:
        r = subprocess.run([cli, "status", "--json"], capture_output=True, text=True, timeout=90)
        snap = json.loads(r.stdout) if r.returncode == 0 else None
    except Exception as exc:
        snap = None
        last = f"{type(exc).__name__}: {exc}"
    if snap:
        stale = [k for k, v in snap["robots"].items() if not v["fresh"]]
        states = {k: v["operating_state"] for k, v in snap["robots"].items()}
        last = f"stale={stale} states={states}"
        if not stale:
            print("READY", last)
            sys.exit(0)
    time.sleep(2.0)
print("NOT_READY", last)
sys.exit(1)
PY

echo "=== launch ==="
ros2 launch fleet_bringup fleet.launch.py robots:=r01,r02,r03 \
  > "$RUN_DIR/fleet_launch.log" 2>&1 &
LAUNCH_PID=$!
echo "launch pid $LAUNCH_PID"
echo "=== readiness ==="
/usr/bin/python3 /tmp/ready3.py "$CLI" 300
echo "READY_RC=$?"
grep -nE "task_service up|task_service_node.*died" "$RUN_DIR/fleet_launch.log" | head -5
echo

echo "=== baseline ==="
$CLI status; echo "RC=$?"
echo

echo "=== submissions ==="
$CLI submit --request-id p5-A --pick S_left_a --drop S_left_b \
  --payload cargo_A --capability CARRY --service-duration 2.0; echo "A_RC=$?"
$CLI submit --request-id p5-B --pick S_left_a --drop S_left_b \
  --capability INSPECT --service-duration 2.0; echo "B_RC=$?"
$CLI submit --request-id p5-C --pick S_right_a --drop S_right_b \
  --payload cargo_C --capability CARRY --service-duration 2.0; echo "C_RC=$?"
echo
echo "--- idempotent replay of p5-A (same result, nothing new) ---"
$CLI submit --request-id p5-A --pick S_left_a --drop S_left_b \
  --payload cargo_A --capability CARRY --service-duration 2.0; echo "REPLAY_RC=$?"
echo
echo "--- same request_id, different content (same half, so it reaches the ledger) ---"
$CLI submit --request-id p5-A --pick S_left_a --drop S_left_b_typo \
  --payload cargo_A --capability CARRY; echo "CONFLICT_RC=$?"
echo
echo "--- cross-corridor must be refused, not driven around ---"
$CLI submit --request-id p5-X --pick S_left_a --drop S_right_a \
  --payload cargo_X --capability CARRY; echo "CROSS_RC=$?"
echo
echo "--- payload_mode=physical must be refused with the contract's code ---"
$CLI submit --request-id p5-P --pick S_left_a --drop S_left_b \
  --payload cargo_P --payload-mode physical; echo "PHYSICAL_RC=$?"
echo
echo "--- unknown station ---"
$CLI submit --request-id p5-U --pick S_left_a --drop S_nowhere; echo "UNKNOWN_RC=$?"
echo

echo "=== fault injection: only into a robot that is EXECUTING and CARRYING ==="
# Timing is not a precondition. The previous version injected at a fixed wall-clock offset
# under a comment claiming the target "holds no task", and by then r02 was three legs into
# p5-C -- so the fault landed mid-task and the scenario it was meant to prove never ran.
# It produced a real parking event for the wrong reason, which is worse than no event
# because it looks like evidence. This version waits for the precondition, prints it, and
# reports NOT_RUN rather than injecting into an idle robot and calling that a pass.
cat > /tmp/pickfault.py <<'PY'
import json, subprocess, sys, time

cli, budget = sys.argv[1], float(sys.argv[2])
end = time.monotonic() + budget
while time.monotonic() < end:
    try:
        r = subprocess.run([cli, "status", "--json"], capture_output=True, text=True,
                           timeout=90)
        snap = json.loads(r.stdout) if r.returncode == 0 else None
    except Exception:
        snap = None
    if snap:
        held = {p["payload_id"]: p.get("holder", "")
                for p in snap.get("payloads", []) if p.get("state") == "HELD"}
        for t in snap.get("tasks", []):
            pid = t.get("payload_id") or ""
            if (t.get("state") == "EXECUTING" and t.get("robot_id") and pid
                    and held.get(pid) == t["robot_id"]):
                print(f"{t['robot_id']}|{t['task_id']}|{pid}")
                raise SystemExit(0)
    time.sleep(5.0)
print("||")
PY
READ=$(/usr/bin/python3 /tmp/pickfault.py "$CLI" 240)
FAULT_ROBOT=${READ%%|*}
REST=${READ#*|}
FAULT_TASK=${REST%%|*}
FAULT_PAYLOAD=${REST#*|}
if [ -z "$FAULT_ROBOT" ]; then
  echo "  NOT_RUN: no robot was EXECUTING a CARRIED payload within 240 s."
  echo "           Injecting into an idle robot proves nothing about recovery, so this"
  echo "           step is reported as NOT_RUN rather than as a pass."
else
  echo "  precondition: $FAULT_ROBOT is executing $FAULT_TASK carrying $FAULT_PAYLOAD"
  echo "  expected:     MARK_NEEDS_ATTENTION with PAYLOAD_HELD_NEEDS_ATTENTION."
  echo "                It is in the open, not in the corridor, so BLOCK_RESOURCE here"
  echo "                would be the over-blocking this round exists to remove."
  $CLI fault "$FAULT_ROBOT" injected_fault --duration 5.0; echo "FAULT_RC=$?"
  sleep 4
  $CLI fault "$FAULT_ROBOT" ""; echo "FAULT_CLEAR_RC=$?"
fi
echo

echo "=== a task submitted then cancelled: cancel accepted must not mean stopped ==="
$CLI submit --request-id p5-D --pick S_left_b --drop S_left_a \
  --payload cargo_D --capability CARRY --service-duration 5.0; echo "D_RC=$?"
# Wait for a real precondition, not `sleep 6`. If p5-D is still SUBMITTED when the cancel
# arrives then "accepted is not stopped" is never exercised -- a cancel before assignment
# is trivially confirmed -- and the test would pass while proving nothing.
cat > /tmp/waitstate.py <<'PY'
import json, subprocess, sys, time

cli, task_id, budget = sys.argv[1], sys.argv[2], float(sys.argv[3])
end, last = time.monotonic() + budget, "no reading"
while time.monotonic() < end:
    try:
        r = subprocess.run([cli, "status", "--json"], capture_output=True, text=True,
                           timeout=90)
        snap = json.loads(r.stdout) if r.returncode == 0 else None
    except Exception:
        snap = None
    if snap:
        for t in snap.get("tasks", []):
            if t.get("task_id") == task_id:
                last = (f"{t['state']} robot={t.get('robot_id') or '-'} "
                        f"leg={t.get('leg') or '-'}")
                if t.get("state") == "EXECUTING":
                    print(last)
                    raise SystemExit(0)
    time.sleep(3.0)
print(f"NEVER EXECUTING within {budget:.0f}s -- last was {last}")
raise SystemExit(1)
PY
echo "  precondition: $(/usr/bin/python3 /tmp/waitstate.py "$CLI" p5-D 150)"
$CLI cancel p5-D; echo "CANCEL_RC=$?"
echo

echo "=== outcomes: poll all four at once ==="
# Sequential `fleet_cli wait` calls took up to 420 s each and the total blew past the
# time this job is allowed, so the run kept being killed exactly at the point where
# the outcome finally gets read. One poller watching all four tasks makes the wall
# time the LONGEST task rather than the sum of them.
cat > /tmp/polltasks.py <<'PY'
import json, subprocess, sys, time
cli, ids, deadline = sys.argv[1], sys.argv[2].split(","), float(sys.argv[3])
# NEEDS_ATTENTION is not terminal -- it needs a human -- but waiting the full budget
# for it to change is six minutes spent proving nothing. Settled means "polling can
# stop", not "the task is done".
SETTLED = {"SUCCEEDED", "FAILED", "CANCELED", "NEEDS_ATTENTION"}
end = time.monotonic() + deadline
seen, t0 = {}, time.monotonic()
while time.monotonic() < end:
    try:
        r = subprocess.run([cli, "status", "--json"], capture_output=True, text=True,
                           timeout=90)
        snap = json.loads(r.stdout) if r.returncode == 0 else None
    except Exception:
        snap = None
    if snap:
        rows = {t["task_id"]: t for t in snap["tasks"]}
        for i in ids:
            t = rows.get(i)
            if not t:
                continue
            line = (f"{t['state']:15} leg={t['leg'] or '-':16} "
                    f"attempts={t['attempts']} robot={t['robot_id'] or '-'}")
            if seen.get(i) != line:
                print(f"[{time.monotonic()-t0:6.1f}s] {i:7} {line}", flush=True)
                seen[i] = line
        if all(rows.get(i, {}).get("state") in SETTLED for i in ids):
            break
    time.sleep(5.0)
for i in ids:
    print(f"{i}: final {seen.get(i, 'no reading')}")
PY
/usr/bin/python3 /tmp/polltasks.py "$CLI" "p5-A,p5-B,p5-C,p5-D" 420
echo

echo "=== final status ==="
$CLI status; echo "RC=$?"
echo
echo "=== event timeline for this run ==="
$CLI events --path runtime/events.jsonl --limit 300
echo
cp runtime/events.jsonl "$RUN_DIR/events.jsonl" 2>/dev/null || true
cp runtime/fleet.sqlite "$RUN_DIR/fleet.sqlite" 2>/dev/null || true
$CLI status --json > "$RUN_DIR/final_status.json" 2>/dev/null || true

echo
echo "=== what the executors said ==="
grep -nE "leg refused|STALE_RESULT_IGNORED|CANCEL_UNCONFIRMED|FAULT INJECTED|did not accept|adopted permit" \
  "$RUN_DIR/fleet_launch.log" | head -30
echo
echo "=== teardown ==="
kill -INT "$LAUNCH_PID" 2>/dev/null
for _ in $(seq 1 45); do kill -0 "$LAUNCH_PID" 2>/dev/null || break; sleep 1; done
kill -0 "$LAUNCH_PID" 2>/dev/null && { kill -TERM "$LAUNCH_PID" 2>/dev/null; sleep 20; }
kill -0 "$LAUNCH_PID" 2>/dev/null && kill -KILL "$LAUNCH_PID" 2>/dev/null
pkill -f "009_multi_robot_fleet/instal[l]" 2>/dev/null
pkill -f "gz-sim-mai[n]" 2>/dev/null
sleep 6
LEFT=$(pgrep -f "009_multi_robot_fleet/instal[l]" | wc -l)
GZ=$(pgrep -f "gz-sim-mai[n]" | wc -l)
echo "leftover project processes: $LEFT ; gz: $GZ"
[ "$LEFT" -eq 0 ] && [ "$GZ" -eq 0 ] && echo "teardown verified clean" || echo "TEARDOWN NOT CLEAN"
} > "$OUT" 2>&1
echo "WROTE $OUT"; wc -l "$OUT"
