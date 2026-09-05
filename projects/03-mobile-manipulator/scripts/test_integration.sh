#!/usr/bin/env bash
set -eo pipefail
source "$(dirname "${BASH_SOURCE[0]}")/env.sh"
case "${1:-full}" in
  full)
    MM_HEADLESS="${MM_HEADLESS:-1}" MM_AUTO_EXIT=1 bash "$MM_ROOT/run_demo.sh"
    ;;
  cancel)
    result_dir="$MM_ROOT/reports/cancel-$(date +%Y%m%d-%H%M%S)-$$"
    mkdir -p "$result_dir"
    # Do not include a first-time C++ configure/build in the service deadline.
    bash "$MM_ROOT/scripts/build.sh" >"$result_dir/build.log" 2>&1 || { tail -40 "$result_dir/build.log"; exit 1; }
    setsid env MM_HEADLESS="${MM_HEADLESS:-1}" MM_AUTO_EXIT=0 bash "$MM_ROOT/run_demo.sh" >"$result_dir/launcher.log" 2>&1 & demo_pid=$!
    stop_demo() { kill -TERM "$demo_pid" 2>/dev/null || true; wait "$demo_pid" 2>/dev/null || true; }
    trap stop_demo EXIT
    trap 'exit 130' INT TERM
    python3 "$MM_ROOT/tools/test_cancel.py" | tee "$result_dir/result.json"
    echo "Cancellation evidence: $result_dir"
    ;;
  *) echo 'Usage: bash scripts/test_integration.sh [full|cancel]'; exit 2 ;;
esac
