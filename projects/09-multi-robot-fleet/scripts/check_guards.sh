#!/usr/bin/env bash
# Every static guard, in one place, before anything is built.
#
#   bash scripts/check_guards.sh
#
# Exit codes: 0 = all guards passed, 3 = environment problem, 1 = a guard failed.
#
# WHY THIS EXISTS
# ---------------
# All of these guards already existed and were already passing. They were just never
# called from anywhere, so "the build is green" meant "colcon compiled", and the guards
# were only run by hand when someone remembered. Two consequences, both of which cost
# real runs on this project:
#
#   * `self.clients = {}` shadowing a read-only rclpy.node.Node property reached a live
#     three-robot run and crashed the task service at construction.
#   * The guard that was supposed to catch it had a member list that depended on WHICH
#     SHELL invoked it: 109 entries when ROS was sourced, 23 when it was not. In the clean
#     shell the project runs its checks in, half of that check was effectively off, and it
#     still printed success.
#
# So: the guards run here, and they must give the same verdict in any environment. A guard
# whose answer depends on the shell is a bug in the guard, not a reason to pick one shell.
# Each guard reports how it obtained what it needed, so the output says what it actually
# knew instead of implying more.

set -u -o pipefail

SELF="${BASH_SOURCE[0]:-$0}"
ROOT="$(cd "$(dirname "$SELF")/.." && pwd)"
cd "$ROOT" || exit 3

PY=".venv/bin/python"
if [ ! -x "$PY" ]; then
  echo "check_guards.sh: .venv missing." >&2
  echo "  create it with: uv venv --seed --python /usr/bin/python3 .venv" >&2
  exit 3
fi

FAILED=0

guard() {
  local label="$1"
  shift
  printf '\n=== %s\n' "$label"
  # The indenting filter must NOT be able to hide the guard's status. `set -o
  # pipefail` alone is not enough here: this file writes `set -uo pipefail`,
  # which parses as `set -u`, `set -o`, `pipefail` -- i.e. pipefail was never on,
  # the pipeline returned sed's status (always 0), and every guard passed
  # regardless of what it found. (Measured 2026-09-19: a guard that exited 7
  # through this pipeline reported 0.)
  #
  # So read the guard's own status out of PIPESTATUS instead of trusting the
  # pipeline's, and treat "no status" as a failure rather than as success.
  # And do NOT turn `set -e` on here. The previous version ran `set -e 2>/dev/null || true`
  # to "restore" a mode this file never had (the header says `set -u -o pipefail`), which
  # left -e on for the REST OF THE SCRIPT. This function ended with `return 1` on failure,
  # so the first failing guard aborted the suite: everything after it never ran, the FAIL
  # list under-reported, and the closing "all guards passed" line never printed. Measured
  # 2026-09-20: 13 of 17 sections ran, and the four skipped ones included both fleet-level
  # guards added most recently. A suite that stops at the first failure is the D-P17-10
  # shape again -- it reports less than it checked.
  local rc
  "$@" 2>&1 | sed 's/^/  /'
  rc=${PIPESTATUS[0]}
  if [ "${rc:-1}" -eq 0 ]; then
    return 0
  fi
  printf '  [FAIL] %s (exit %s)\n' "$label" "$rc"
  FAILED=1
  return 0
}

guard "undefined names, Node-member shadowing, out-of-scope names" \
  "$PY" scripts/check_undefined_names.py
# Runs BEFORE the build, and before any of the other checks, because the defect it
# finds is invisible to all of them. `self.movement_stages` was read for days and
# assigned nowhere: legal syntax, correct spelling, and unreachable -- hidden behind a
# short-circuited `and`. D-P17-19 is the same shape again, in the one branch of
# CONTRACTS section 5 that had never been selected. Self-tested, because a guard that
# only ever returns 0 has been shown to be quiet, not to work.
guard "the self-attribute guard itself (self-test)" \
  "$PY" scripts/check_self_attrs.py --self-test
guard "every self.<name> read is assigned somewhere in its file" \
  "$PY" scripts/check_self_attrs.py
guard "ROS callbacks can be called with the arguments ROS passes" \
  "$PY" scripts/check_ros_callback_arity.py
guard "XML well-formedness (assets/, config/, src/)" \
  "$PY" scripts/check_xml_wellformed.py
guard "ROS parameter references" \
  "$PY" scripts/check_ros_params.py
guard "AMCL's update step fits inside the gate's escape hatch" \
  "$PY" scripts/check_pose_freshness.py
guard "scenario budgets are declared AND forwarded" \
  "$PY" scripts/check_scenario_budgets.py
guard "the frozen batch case list matches the contract" \
  "$PY" scripts/check_batch_manifest.py
guard "spawn poses and pads against the world" \
  "$PY" scripts/validate_assets.py
guard "traffic geometry against the world" \
  "$PY" scripts/validate_traffic_geometry.py
guard "the truth-liveness criterion itself (self-test)" \
  "$PY" scripts/check_truth_liveness.py --self-test
guard "no recording carries a frozen truth stream" \
  "$PY" scripts/check_truth_liveness.py
guard "the case's runners and the launch's robots:= agree" \
  "$PY" scripts/check_launch_robots.py
# Runs last but is not less important: it asserts a property of the WHOLE fleet that no
# single-robot test can see. Three robots each bridging their own gz transform stream gave
# the odom -> base_footprint edge one publisher per robot on the shared /tf; the arrival
# order raced, tf2 cleared its buffer on the loser, the odom frame ceased to exist, and
# AMCL died on an uncaught tf2::LookupException -- measured as 0/6/4 crashed nodes across
# three identical runs before the fix and 0/0/0 after. Nothing in this file could see it,
# because every existing guard reasons about one robot at a time.
guard "each TF edge has exactly one publisher" \
  "$PY" scripts/check_tf_single_publisher.py
# The same class of defect as the guard above, one layer up: a quantity that only matters
# when more than one robot exists, and that two files can each hold their own copy of. The
# Nav2 lifecycle manager's service timeout defaulted to a 5 s that is fine for one robot and
# aborts a three-robot bringup, which costs the run its evidence rather than reporting a
# slow host. This asserts the value is single-sourced, forwarded, and bounded on both sides.
guard "the localiser age bound is backed by the recordings" \
  "$PY" scripts/check_localiser_age.py

guard "the fleet's Nav2 lifecycle timeout is sized for a fleet" \
  "$PY" scripts/check_nav2_lifecycle_timeout.py

printf '\n'
if [ "$FAILED" -ne 0 ]; then
  echo "check_guards.sh: FAILED -- fix the guard(s) marked [FAIL] above before building."
  exit 1
fi
echo "check_guards.sh: all static guards passed (exit 0)"
