#!/usr/bin/env bash
# TEST_AND_ACCEPTANCE section 10, end to end: the reproduction and release gates.
#
#   bash scripts/run_p7_matrix.sh [--no-relocation]
#
# Three passes on purpose:
#
#   1. the static gates, immediately. They are cheap and they catch a stale document, a
#      forbidden absolute path and an uncovered ignore rule without starting anything.
#   2. the relocation pass: scripts/verify_independence.sh copies the tree to a temp path,
#      builds it from scratch there, runs the core suite there, compares it file by file
#      against the frozen input manifest, and then launches a REAL fleet from the copy.
#      That run is item 10.1's evidence, and nothing static can substitute for it.
#   3. the gates again, with that report fed in.
#
# All three write under reports/p7/ and none overwrites another: TEST_AND_ACCEPTANCE
# section 5 says a re-run is recorded, not that it replaces the first result.
#
# Exit code is pass 3's: 0 every gate satisfied, 4 a gate FAILED, 5 a gate is NOT_RUN
# (which includes item 10.8, the user gate -- a release gate that closes itself is not one).

set -uo pipefail

SELF="${BASH_SOURCE[0]:-$0}"
ROOT="$(cd "$(dirname "$SELF")/.." && pwd)"
cd "$ROOT" || exit 3

STAMP="$(date -u +%Y%m%dT%H%M%SZ)"
OUT="$ROOT/reports/p7"
mkdir -p "$OUT"

PY=".venv/bin/python"
if [ ! -x "$PY" ]; then
  echo "run_p7_matrix.sh: .venv missing" >&2
  exit 3
fi

echo "=============================================================================="
echo "  P7 matrix -- section 10 release gates        $(date -u +%Y-%m-%dT%H:%M:%SZ)"
echo "=============================================================================="

echo
echo "--- pass 1/3: static gates"
"$PY" scripts/check_p7_gates.py | tee "$OUT/gates_static_$STAMP.txt"
static_rc=${PIPESTATUS[0]}
echo "  (exit $static_rc)"

if [ "${1:-}" = "--no-relocation" ]; then
  echo
  echo "relocation pass skipped on request: item 10.1 stays NOT_RUN and the gate stays open"
  exit "$static_rc"
fi

echo
echo "--- pass 2/3: relocation"
echo "    copying the tree to a temp path, building, testing, then a real fleet run"
echo "    (this takes several minutes; the log is $OUT/relocation_$STAMP.txt)"
FLEET009_INDEP_FLEET=1 bash scripts/verify_independence.sh > "$OUT/relocation_$STAMP.txt" 2>&1
reloc_rc=$?
echo "    verify_independence.sh exit=$reloc_rc"
tail -30 "$OUT/relocation_$STAMP.txt"

echo
echo "--- pass 3/3: gates, with item 10.1's evidence"
"$PY" scripts/check_p7_gates.py --independence-report "$OUT/relocation_$STAMP.txt" \
  | tee "$OUT/gates_final_$STAMP.txt"
final_rc=${PIPESTATUS[0]}

echo
echo "  reports:"
echo "    $OUT/gates_static_$STAMP.txt     (static only)"
echo "    $OUT/relocation_$STAMP.txt       (the relocation run)"
echo "    $OUT/gates_final_$STAMP.txt      <- the section 10 verdict"
exit "$final_rc"
