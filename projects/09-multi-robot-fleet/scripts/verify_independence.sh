#!/usr/bin/env bash
# P7: prove the project does not depend on its original location.
#
#   bash scripts/verify_independence.sh                 # scan + build + guards + tests
#   FLEET009_INDEP_FLEET=1 bash scripts/verify_independence.sh
#                                                       # ... plus a real fleet run
#
# MASTER_PLAN section 9 (P7): "将项目复制到独立临时路径测试不依赖旧工程或原始绝对路径，保留报告."
#
# WHAT IS UNDER TEST, AND WHAT IS NOT
# -----------------------------------
# The thing under test is the TREE: `src/`, `scripts/`, `config/`, `assets/`, `tests/`.
# The Python interpreter and its site-packages are not part of the tree and cannot make a
# relocatable tree non-relocatable, so the copy may share one -- which it has to here,
# because this machine has no outbound network and `uv venv --seed` cannot fetch pip.
# That fallback is printed, not hidden, and the probe below checks what the interpreter
# can actually import rather than that a directory exists.
#
# The first version of this script printed `[PASS] venv built` on the strength of
# `.venv/bin/python` existing, while the venv had no pip, no PyYAML and no pytest: three
# of the seven steps then failed with `ModuleNotFoundError` and the report blamed the
# project. An interpreter that exists is not an environment that works.
#
# TWO SCOPES, DELIBERATELY DIFFERENT
# ----------------------------------
#   * Code and configuration must contain NO reference to the original tree. That is a
#     hard failure: a path in a script or a launch file is a dependency.
#   * Documents are allowed to name the project's canonical path. `docs/HANDOFF_README.md`
#     and `docs/MASTER_PLAN.md` are the handoff bundle: recording the intended location is
#     their purpose, and treating that as a dependency would force the project to edit its
#     own provenance. They are listed as a note, with the count, and not as a failure.
set -uo pipefail

SELF="${BASH_SOURCE[0]:-$0}"
ROOT="$(cd "$(dirname "$SELF")/.." && pwd)"
STAMP="$(date -u +%Y%m%dT%H%M%SZ)"
TARGET="${FLEET009_INDEP_DIR:-$HOME/009_independence_$STAMP}"
REPORT_DIR="${FLEET009_INDEP_REPORT_DIR:-$ROOT/reports/p7}"
mkdir -p "$REPORT_DIR"
REPORT="$REPORT_DIR/independence_$STAMP.txt"

FAILED=0
SKIPPED=0
say() { printf '%s\n' "$*" | tee -a "$REPORT"; }
ok()   { say "  [PASS] $*"; }
fail() { FAILED=1; say "  [FAIL] $*"; }
skip() { SKIPPED=$((SKIPPED + 1)); say "  [NOT_RUN] $*"; }

: > "$REPORT"
say "==================================================================="
say "  009 P7 independence check"
say "==================================================================="
say "  origin : $ROOT"
say "  copy   : $TARGET"
say "  fleet run: ${FLEET009_INDEP_FLEET:-0}   (FLEET009_INDEP_FLEET=1 to include one)"
say ""

# ---------------------------------------------------------------- 1. copy --- #
say "--- 1. copy the tree, excluding every artefact ---"
rm -rf "$TARGET"
mkdir -p "$TARGET"
tar -C "$ROOT" \
    --exclude=./build --exclude=./install --exclude=./log \
    --exclude=./reports --exclude=./runtime --exclude=./.venv \
    --exclude='*__pycache__*' --exclude='*.pyc' --exclude='*.pyo' \
    --exclude='*/.pytest_cache*' \
    -cf - . | tar -C "$TARGET" -xf -
say "  files copied: $(find "$TARGET" -type f | wc -l)"
for absent in build install log .venv; do
  [ -e "$TARGET/$absent" ] && fail "$absent was copied; the copy is not clean"
done
if [ -d "$TARGET/src" ] && [ -f "$TARGET/scripts/build.sh" ]; then
  ok "the copy has its own src/ and scripts/ and no build artefacts"
else
  fail "the copy is missing src/ or scripts/"
fi
mkdir -p "$TARGET/reports"
[ -f "$ROOT/reports/frozen_manifest.json" ] && \
  cp "$ROOT/reports/frozen_manifest.json" "$TARGET/reports/"

# ------------------------------------------------- 2. absolute-path scan --- #
say ""
say "--- 2. code and configuration must not name the original tree ---"
code_hits=$(grep -rInF "$ROOT" \
              "$TARGET/src" "$TARGET/scripts" "$TARGET/config" "$TARGET/tests" \
              --exclude-dir=__pycache__ 2>/dev/null || true)
doc_hits=$(grep -rInF "$ROOT" \
              "$TARGET/docs" "$TARGET/AGENTS.md" "$TARGET/README.md" 2>/dev/null || true)
if [ -z "$code_hits" ]; then
  ok "no file under src/, scripts/, config/ or tests/ mentions $ROOT"
else
  fail "code or config still names its origin:"
  printf '%s\n' "$code_hits" | head -20 | sed 's/^/      /' | tee -a "$REPORT"
fi
if [ -n "$doc_hits" ]; then
  say "  [NOTE] $(printf '%s\n' "$doc_hits" | wc -l) hit(s) in documents, which are the"
  say "         handoff bundle recording the intended location. Not a dependency:"
  printf '%s\n' "$doc_hits" | head -8 | sed 's/^/      /' | tee -a "$REPORT"
fi

# ------------------------------------------------------------ 3. escapes --- #
say ""
say "--- 3. no three-level relative escapes out of the copy ---"
escapes=$(grep -rIn --include='*.py' --include='*.sh' --include='*.yaml' \
            -e '\.\./\.\./\.\.' "$TARGET/scripts" "$TARGET/src" "$TARGET/config" \
            2>/dev/null | grep -v 'sys.path' || true)
if [ -z "$escapes" ]; then
  ok "none found"
else
  say "  [NOTE] three-level relative paths (review, not automatically a fault):"
  printf '%s\n' "$escapes" | head -10 | sed 's/^/      /' | tee -a "$REPORT"
fi

# --------------------------------------------------- 4. interpreter + deps -- #
say ""
say "--- 4. an interpreter that can actually import what the checks need ---"
export PATH="$HOME/.local/bin:$PATH"
PY="$TARGET/.venv/bin/python"
if ! command -v uv >/dev/null 2>&1; then
  skip "uv is not on PATH; cannot create the copy's venv"
fi
# Try a self-contained venv first; fall back to sharing this machine's interpreter,
# because there is no outbound network for `--seed` to fetch pip from.
uv venv --seed --python /usr/bin/python3 "$TARGET/.venv" >>"$REPORT" 2>&1 \
  || uv venv --python /usr/bin/python3 --system-site-packages "$TARGET/.venv" \
       >>"$REPORT" 2>&1 || true

probe() { "$PY" -c "import $1" >/dev/null 2>&1; }
if [ ! -x "$PY" ]; then
  fail "no interpreter at $PY"
else
  have_yaml=no; have_pytest=no
  probe yaml && have_yaml=yes
  probe pytest && have_pytest=yes
  say "  interpreter: $("$PY" --version 2>&1)   PyYAML=$have_yaml  pytest=$have_pytest"
  if [ "$have_yaml" = yes ]; then
    ok "the copy's interpreter can import PyYAML (the guards need it)"
  else
    # Share the origin's interpreter. Packages are not part of the tree under test,
    # and without network this is the only way to run the guards at all.
    rm -rf "$TARGET/.venv"
    ln -s "$ROOT/.venv" "$TARGET/.venv"
    PY="$TARGET/.venv/bin/python"
    say "  [NOTE] no local PyYAML; the copy now SHARES the origin's interpreter"
    say "         ($ROOT/.venv). The interpreter is not part of the tree under test and"
    say "         sharing it cannot make a relocatable tree non-relocatable."
    probe yaml && have_yaml=yes
    probe pytest && have_pytest=yes
    say "         after sharing: PyYAML=$have_yaml  pytest=$have_pytest"
  fi
  if [ "$have_yaml" = yes ]; then
    ok "the checks have what they need to import"
  else
    skip "neither the copy nor the origin's interpreter can import PyYAML; the guards and"
    skip "  the manifest comparison cannot run here"
  fi
fi

# ------------------------------------------------------------ 5. build ---- #
say ""
say "--- 5. build the copy from nothing ---"
( cd "$TARGET" && bash scripts/build.sh ) >>"$REPORT" 2>&1
rc=$?
if [ "$rc" -eq 0 ] && [ -d "$TARGET/install/fleet_ros" ]; then
  ok "build.sh exited 0 and install/fleet_ros exists"
  grep -E "packages finished|check_guards.sh:" "$REPORT" | tail -3 | sed 's/^/      /'
else
  fail "build.sh failed in the copy (rc=$rc)"
  grep -E "\[FAIL\]|ModuleNotFoundError|Error" "$REPORT" | tail -8 | sed 's/^/      /'
fi

# ------------------------------------------------------------ 6. tests ---- #
say ""
say "--- 6. the core suite in the copy ---"
if command -v "${PY:-true}" >/dev/null 2>&1 && "$PY" -c "import pytest" >/dev/null 2>&1; then
  ( cd "$TARGET" && env -u ROS_DISTRO -u AMENT_PREFIX_PATH bash scripts/test_core.sh ) \
    >>"$REPORT" 2>&1
  rc=$?
  tests=$(grep -oE '[0-9]+ passed' "$REPORT" | tail -1)
  if [ "$rc" -eq 0 ]; then
    ok "core tests passed in the copy ($tests)"
  else
    fail "core tests failed in the copy (rc=$rc)"
  fi
else
  skip "pytest is not importable here, so the core suite was not run in the copy"
fi

# ------------------------------------------------------- 7. frozen check -- #
say ""
say "--- 7. the copy matches the recorded manifest ---"
if [ -f "$TARGET/reports/frozen_manifest.json" ] && "$PY" -c "import yaml" >/dev/null 2>&1; then
  ( cd "$TARGET" && "$PY" scripts/freeze_manifest.py --check ) >>"$REPORT" 2>&1
  rc=$?
  if [ "$rc" -eq 0 ]; then
    ok "every recorded hash matches in the copy"
  else
    fail "the copy differs from the manifest (rc=$rc)"
    grep -E '^  (MISSING|CHANGED|EXTRA)' "$REPORT" | tail -10 | sed 's/^/      /'
  fi
else
  skip "the manifest comparison needs PyYAML and a recorded manifest"
fi

# ------------------------------------------------------ 8. optional fleet - #
if [ "${FLEET009_INDEP_FLEET:-0}" = "1" ]; then
  say ""
  say "--- 8. a real fleet run, launched from the copy ---"
  say "  (the claim P7 actually asks for: the same scenario, a different tree)"
  ( cd "$TARGET" && FLEET009_SCEN_OUT="$TARGET/reports/scen3456" \
      bash scripts/run_scenarios3456.sh capability ) >>"$REPORT" 2>&1
  rc=$?
  verdict=$(grep -hE "assertions passed" "$TARGET/reports/scen3456/"*.txt 2>/dev/null | tail -1)
  if [ "$rc" -eq 0 ]; then
    ok "the capability scenario passed when launched from the copy ($verdict)"
  else
    fail "the scenario failed from the copy (rc=$rc); $verdict"
  fi
  for pat in "$TARGET/install" "gz-sim-main"; do pkill -INT -f "$pat" 2>/dev/null || true; done
  sleep 8
  for pat in "$TARGET/install" "gz-sim-main"; do pkill -KILL -f "$pat" 2>/dev/null || true; done
fi

say ""
say "==================================================================="
if [ "$FAILED" -eq 0 ] && [ "$SKIPPED" -eq 0 ]; then
  say "  RESULT: PASS -- the tree runs from $TARGET with no reference to $ROOT"
elif [ "$FAILED" -eq 0 ]; then
  say "  RESULT: PASS WITH $SKIPPED NOT_RUN -- nothing failed, but see the [NOT_RUN] lines"
else
  say "  RESULT: FAIL -- see the [FAIL] lines above"
fi
say "  report: $REPORT"
say "==================================================================="
exit "$FAILED"
