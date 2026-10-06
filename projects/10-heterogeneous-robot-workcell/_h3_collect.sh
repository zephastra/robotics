#!/usr/bin/env bash
# Collect the H3 report's diagnostics into the report directory, so the acceptance record is
# SELF-CONTAINED: every number quoted in acceptance.md can be re-derived from something in here.
#
# The rule this follows (from the H2 round): a report that points at a file outside the repository
# is not a reproducible report. So the instruments AND the raw outputs they produced live next to
# the verdict.
set -u
P=/home/ziling/projects/010_heterogeneous_robot_workcell
cd "$P" || exit 1
RUN=${1:-p4-h3-w5-01}
D="reports/$RUN/diagnostics"
mkdir -p "$D"
PY=./.venv/bin/python
S=/mnt/c/Users/ZiLing/WorkBuddy/2026-09-11-18-34-08

echo "collecting into $D"

# ---- 1. the world builder's own report -----------------------------------------------------------
$PY experiments/w5_h085_plan.py > "$D/out_h3_build.txt" 2>&1
echo "  build            $(wc -l < "$D/out_h3_build.txt") lines"

# ---- 2. the runtime-driven stand test across candidate stations (why 4.13 is retained) -----------
$PY experiments/w5_h085_plan.py --audit > "$D/out_h3_audit.txt" 2>&1
echo "  audit            $(wc -l < "$D/out_h3_audit.txt") lines"

# ---- 3. every world's keyframe vs merged_home -----------------------------------------------------
cp "$S/_diag/out_h3_home.txt" "$D/out_h3_home_derivation.txt" 2>/dev/null \
  || echo "  (home derivation not present; run _h3_home.sh first)"

# ---- 4. `mj_geomDistance`'s distmax is a measuring budget (D109) ---------------------------------
bash _h3_gd3.sh > "$D/out_h3_geomdist.txt" 2>&1
echo "  geomdist         $(wc -l < "$D/out_h3_geomdist.txt") lines"

# ---- 5. the coast: nothing commands the plant, so the vehicle drifts (D111) ----------------------
bash _h3_phase.sh > "$D/out_h3_coast.txt" 2>&1
echo "  coast            $(wc -l < "$D/out_h3_coast.txt") lines"

# ---- 6. the FIRST negative arm, which bulldozed the tray (D112) ----------------------------------
if [ -f "$S/_diag/out_h3_neg1.txt" ]; then
  cp "$S/_diag/out_h3_neg1.txt" "$D/out_h3_neg_open_hand.txt"
  echo "  neg open-hand    copied"
fi

# ---- 7. the other arm's raw console log ----------------------------------------------------------
for f in "$S/_diag/out_h3_P.txt" "$S/_diag/out_h3_N.txt"; do
  [ -f "$f" ] && cp "$f" "$D/$(basename "${f%.txt}")_console.txt"
done

# ---- 8. the scope check that found the `XFER` defect ----------------------------------------------
$PY experiments/check_globals.py experiments/probe_h3_w5.py experiments/w5_h085_plan.py \
    experiments/probe_h2_w5.py experiments/w4_plant.py > "$D/out_h3_globals.txt" 2>&1
echo "  globals          $(tail -1 "$D/out_h3_globals.txt")"

# ---- 9. the instruments themselves, so the numbers can be re-derived ------------------------------
cp experiments/probe_h3_w5.py "$D/probe_h3_w5.py"
cp experiments/w5_h085_plan.py "$D/w5_h085_plan.py"
cp experiments/check_globals.py "$D/check_globals.py"
for s in _h3_gd3.sh _h3_phase.sh _h3_clear2.sh _h3_geom.sh _h3_acts.sh; do
  [ -f "$s" ] && cp "$s" "$D/$s"
done
echo "  instruments      copied"

ls -la "$D" | tail -20
