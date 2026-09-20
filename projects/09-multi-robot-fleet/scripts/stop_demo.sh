#!/usr/bin/env bash
# Stop a fleet started by scripts/run_demo.sh, and PROVE it is gone.
#
#   bash scripts/stop_demo.sh
#
# Exit codes: 0 = verified clean, 1 = something is still running.
#
# Why this verifies instead of just signalling: the P4 acceptance lost a whole case to a
# teardown that sent SIGINT, printed "still alive after 60 s", and then started the next
# case anyway. Two fleets ended up on one DDS domain, the second case's robots latched an
# emergency stop and reported `RESOURCE_UNKNOWN`, and the numbers looked like a traffic
# result. They were contamination. A teardown that does not verify is a teardown that lies.
#
# This never uses a blanket pkill on a bare pattern: `pkill -f "<project path>"` matches the
# shell that is running it and kills its own parent. The bracket form (`instal[l]`) breaks
# the self-match, and the search is anchored to this project's install directory.
set -uo pipefail

SELF="${BASH_SOURCE[0]:-$0}"
ROOT="$(cd "$(dirname "$SELF")/.." && pwd)"
cd "$ROOT" || exit 3

PATTERN="$ROOT/instal[l]"

count() {
  pgrep -f "$PATTERN" 2>/dev/null | wc -l
}

before="$(count)"
echo "project processes before: $before"
if [ "$before" -eq 0 ]; then
  echo "nothing to stop"
else
  echo "sending SIGINT to $before process(es) under $ROOT"
  # shellcheck disable=SC2046
  kill -INT $(pgrep -f "$PATTERN" 2>/dev/null) 2>/dev/null
  for _ in $(seq 1 45); do
    [ "$(count)" -eq 0 ] && break
    sleep 1
  done

  if [ "$(count)" -ne 0 ]; then
    echo "still $(( $(count) )) alive after 45 s; sending SIGTERM"
    # shellcheck disable=SC2046
    kill -TERM $(pgrep -f "$PATTERN" 2>/dev/null) 2>/dev/null
    for _ in $(seq 1 30); do
      [ "$(count)" -eq 0 ] && break
      sleep 1
    done
  fi

  if [ "$(count)" -ne 0 ]; then
    echo "still $(( $(count) )) alive after SIGTERM; sending SIGKILL"
    # shellcheck disable=SC2046
    kill -KILL $(pgrep -f "$PATTERN" 2>/dev/null) 2>/dev/null
    sleep 5
  fi
fi

echo "project processes after : $(count)"
echo "gz processes after      : $(pgrep -f 'gz-sim-mai[n]' 2>/dev/null | wc -l)"

if [ "$(count)" -ne 0 ]; then
  echo
  echo "FAILED: the fleet did not stop. Anything measured next would be contaminated."
  echo "Remaining:"
  pgrep -a -f "$PATTERN" 2>/dev/null | head -20
  exit 1
fi

echo
echo "teardown verified clean (exit 0)"
