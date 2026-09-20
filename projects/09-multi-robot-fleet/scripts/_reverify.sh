#!/usr/bin/env bash
# =============================================================================
# 009 P2 RE-VERIFICATION on the FIXED drivetrain.
#
# Why every P2 number had to be thrown away: the wheels were flat discs being dragged
# along the floor (see 009_ROOTCAUSE_WHEELS.md). Every measurement taken before that was
# fixed -- arrival error, stop distance, AMCL agreement -- was taken on a robot that could
# not steer. This script re-measures them, and repeats the corridor crossing three times
# so "it works" stops being a single lucky run.
#
# ORDER MATTERS, and the first version got it wrong. The calibrator commands the gate
# directly with Nav2 running but not steering, so whatever the crossings leave behind is
# the runway it measures on. Running the crossings first left the robot near the EAST WALL,
# the calibrator drove into it and kept commanding, and odometry integrated to 14 m. The
# next navigation goal was then rejected with "Start Coordinates of (7.6029, -2.4390) was
# outside bounds" -- which reads like a localisation bug and was actually a test that drove
# into a wall. The calibrator now refuses a short runway, and this script runs it FIRST,
# from the spawn, where the floor ahead is clear.
#
# RESUMABLE. Each phase writes its own file under runtime/reverify/ and is skipped if that
# file already exists. A long serial run that dies must not cost the phases before it --
# that lesson was already paid for once.
#
#   bash _p009_reverify.sh                run whatever is missing
#   PHASES=3,4 bash _p009_reverify.sh     only those
#   FORCE=1 bash _p009_reverify.sh        redo everything
#   TRUTH_TOPIC=/world/warehouse/dynamic_pose/info   give the calibrator ground truth
# =============================================================================
set -o pipefail

REPO="$HOME/projects/009_multi_robot_fleet"
cd "$REPO" || exit 2
source scripts/env.sh >/dev/null 2>&1

OUT="$FLEET009_ROOT/runtime/reverify"
mkdir -p "$OUT/logs"
LOG="$OUT/reverify.txt"
PHASES="${PHASES:-2,6,3,4,5,7}"
FORCE="${FORCE:-0}"
TRUTH_TOPIC="${TRUTH_TOPIC:-/world/warehouse/dynamic_pose/info}"

say()   { printf '%s\n' "$*" | tee -a "$LOG"; }
head2() { say ""; say "=================================================================="; say "  $*"; say "=================================================================="; }
has()   { case ",$PHASES," in *",$1,"*) return 0 ;; *) return 1 ;; esac; }
want()  { [ "$FORCE" = "1" ] || [ ! -s "$OUT/p$1.done" ]; }

# --------------------------------------------------------------------------- #
if has 0; then
  head2 "0. clean slate"
  pkill -f 'ros2 launch' >/dev/null 2>&1; pkill -f gz >/dev/null 2>&1
  pkill -f robot_state_publisher >/dev/null 2>&1; pkill -f 'nav2_' >/dev/null 2>&1
  pkill -f parameter_bridge >/dev/null 2>&1
  sleep 4
fi

if has 1; then
  head2 "1. start the stack"
  setsid nohup ros2 launch fleet_bringup world.launch.py \
      > "$OUT/logs/world.log" 2>&1 < /dev/null &
  setsid nohup ros2 launch fleet_bringup robot.launch.py robot:=r01 start_nav2:=true \
      > "$OUT/logs/robot.log" 2>&1 < /dev/null &
  say "  waiting 85 s"
  sleep 85
fi

# --------------------------------------------------------------------------- #
if has 2; then
  head2 "2. readiness and drivetrain provenance"
  {
    echo "stack:"
    grep -E 'Managed nodes are active' "$OUT/logs/robot.log" | tail -1
    grep -E 'safety gate up' "$OUT/logs/robot.log" | tail -1
    echo
    echo "truth bridge (should be exactly one, in the world launch file):"
    grep -cE 'dynamic_pose/info' "$OUT/logs/world.log" "$OUT/logs/robot.log"
    echo
    echo "drivetrain -- the fix under test:"
    python3 - <<'PY'
import re, pathlib
sdf = pathlib.Path("assets/models/amr_4wd/model.sdf.in").read_text(encoding="utf-8")
fl = sdf.split('<link name="wheel_front_left">')[1].split('</link>')[0]
col = fl.split('<collision')[1].split('</collision>')[0]
vis = fl.split('<visual')[1].split('</visual>')[0]
def pose(b):
    m = re.search(r'<pose>([^<]+)</pose>', b)
    return m.group(1).strip() if m else "NONE"
def g(b):
    m = re.search(r'<cylinder><radius>([\d.]+)</radius>', b)
    return m.group(1) if m else "?"
print(f"  collision: r={g(col)}  pose={pose(col)}")
print(f"  visual   : r={g(vis)}  pose={pose(vis)}")
print(f"  friction : mu={re.search(r'<mu>([\d.]+)</mu>', col).group(1)}"
      f" mu2={re.search(r'<mu2>([\d.]+)</mu2>', col).group(1)}"
      f" fdir1={re.search(r'<fdir1>([^<]+)</fdir1>', col).group(1)}")
j = sdf.split('wheel_front_left_joint')[1].split('</joint>')[0]
print(f"  joint    : axis={re.search(r'<axis><xyz>([^<]+)</xyz>', j).group(1)}"
      f"  pose={'PRESENT (BAD)' if '<pose>' in j else 'none (good)'}")
d = sdf.split('gz-sim-diff-drive-system')[1].split('</plugin>')[0]
print(f"  diffdrive: wheel_radius={re.search(r'<wheel_radius>([\d.]+)</wheel_radius>', d).group(1)}"
      f" separation={re.search(r'<wheel_separation>([\d.]+)</wheel_separation>', d).group(1)}")
PY
  } 2>&1 | sed 's/^/  /' | tee -a "$LOG" > "$OUT/p2.done"

  # HARD GATE. The first session printed "Managed nodes are active: 0" and carried on
  # anyway, so three crossings "ran" and reported a missing action server -- which is a
  # five second failure wearing the clothes of a navigation result. A phase that cannot
  # work must not produce a record that looks like it did.
  if ! grep -q 'Managed nodes are active' "$OUT/logs/robot.log"; then
    say ""
    say "  !! The Nav2 lifecycle did NOT activate. Stopping before the phases that"
    say "     depend on it, so nothing is recorded as a failed attempt."
    say "     The reason is in the log; these lines usually name it:"
    grep -iE 'Original error|Failed to change state|invalid type|not allowed|ERROR|FATAL' \
      "$OUT/logs/robot.log" | tail -10 | sed 's/^/       /' | tee -a "$LOG"
    rm -f "$OUT/p2.done"
    exit 7
  fi
fi

# --------------------------------------------------------------------------- #
# BEFORE the crossings, on purpose: the robot is still at its spawn with clear floor
# ahead, so the calibration measures the plant instead of an east wall.
if has 6 && want 6; then
  head2 "6. stop-distance calibration on the FIXED chassis (run FIRST, from the spawn)"
  say "  ground truth: $TRUTH_TOPIC"
  say "  (the world-level channel drops frame names, so the model is taken positionally;"
  say "   that is valid for ONE robot and the report says so)"
  {
    for pass in 1 2 3; do
      echo "--- pass $pass ---"
      timeout 300 ros2 run fleet_ros stop_distance_calibrator --ros-args \
          -r __ns:=/r01 -p robot_id:=r01 \
          -p speeds:="[0.15,0.25,0.35]" \
          -p ground_truth_topic:="$TRUTH_TOPIC" \
          -p out_json:="$OUT/calib_pass${pass}.json" 2>&1
      echo "pass $pass EXIT=$?"
    done
  } 2>&1 | sed 's/^/  /' | tee -a "$LOG" > "$OUT/p6.done"
fi

# --------------------------------------------------------------------------- #
# crossings: three attempts, alternating direction. One success is an anecdote.
for spec in "3:4.0:-2.0:W->E" "4:-6.0:-2.0:E->W" "5:4.0:-2.0:W->E(again)"; do
  n="${spec%%:*}"; rest="${spec#*:}"
  gx="${rest%%:*}"; rest="${rest#*:}"
  gy="${rest%%:*}"; label="${rest#*:}"
  has "$n" || continue
  want "$n" || { say ""; say "phase $n already done, skipping ($label)"; continue; }
  head2 "$n. corridor crossing $label  (goal $gx, $gy)"
  {
    timeout 420 python3 scripts/probe_crossing.py --robot r01 --world warehouse \
        --x "$gx" --y "$gy" --timeout 280 --hz 5 \
        --out "$OUT/cross_$n.csv"
    echo "EXIT=$?"
  } 2>&1 | sed 's/^/  /' | tee -a "$LOG" > "$OUT/p$n.done"
  cp "$OUT/logs/robot.log" "$OUT/logs/robot_p$n.log" 2>/dev/null
done

# --------------------------------------------------------------------------- #
if has 7 && want 7; then
  head2 "7. arrival accuracy re-measured (independent tf2 buffer, map frame)"
  {
    for g in "-4.0:0.0" "-3.0:-3.0"; do
      x="${g%%:*}"; y="${g#*:}"
      echo "--- goal ($x, $y) ---"
      timeout 300 python3 scripts/nav_goal_measured.py --robot r01 \
          --world warehouse \
          --x "$x" --y "$y" --yaw 0.0 --timeout 200 --watch-scan 2>&1
      echo "EXIT=$?"
    done
  } 2>&1 | sed 's/^/  /' | tee -a "$LOG" > "$OUT/p7.done"
fi

# --------------------------------------------------------------------------- #
head2 "8. failures seen anywhere in the robot log"
grep -iE 'error_code|Aborting handle|Failed to make progress|0 poses|not reachable|outside bounds' \
    "$OUT/logs/robot.log" | grep -viE 'heartbeat|permit' | tail -14 \
    | sed 's/^/  /' | tee -a "$LOG"

head2 "9. cleanup"
pkill -f 'ros2 launch' >/dev/null 2>&1; pkill -f gz >/dev/null 2>&1
pkill -f robot_state_publisher >/dev/null 2>&1; pkill -f 'nav2_' >/dev/null 2>&1
pkill -f parameter_bridge >/dev/null 2>&1
sleep 3
say "  done: $LOG"
