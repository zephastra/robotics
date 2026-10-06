#!/bin/bash
set -u
P=/home/ziling/projects/010_heterogeneous_robot_workcell
cd "$P" || exit 1
LOG=/mnt/c/Users/ZiLing/WorkBuddy/2026-09-11-18-34-08/_diag/out_h3_run1.txt
{
  echo "=== H3 positive arm: $(date -Is) ==="
  timeout 3000 ./.venv/bin/python experiments/probe_h3_w5.py --run-id p4-h3-w5-01
  rc=$?
  echo "=== rc=$rc $(date -Is) ==="
} > "$LOG" 2>&1
