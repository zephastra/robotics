set -u
P=/home/ziling/projects/010_heterogeneous_robot_workcell
W=/mnt/c/Users/ZiLing/WorkBuddy/2026-09-11-18-34-08
cd "$P" || exit 1

cp reports/p4-h3-w5-01/acceptance.md            "$W/010_H3_ACCEPTANCE.md"
cp reports/p4-h3-w5-neg-01/acceptance.md        "$W/010_H3_NEGATIVE_ACCEPTANCE.md"
cp reports/p4-h3-w5-01/diagnostics/README.md    "$W/010_H3_DIAGNOSTICS_README.md"
cp reports/p4-h3-w5-01/report.json              "$W/010_H3_report.json"
cp reports/p4-h3-w5-neg-01/report.json          "$W/010_H3_negative_report.json"
cp experiments/probe_h3_w5.py                   "$W/010_H3_probe.py"
cp experiments/w5_h085_plan.py                  "$W/010_H3_build_world.py"
cp tests/test_h3_integration.py                 "$W/010_H3_tests.py"
cp experiments/check_globals.py                 "$W/010_H3_check_globals.py"

echo "--- copied ---"
ls -la "$W"/010_H3* | sed 's/^/  /'
