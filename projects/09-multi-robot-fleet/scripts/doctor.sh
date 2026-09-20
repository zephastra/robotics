#!/usr/bin/env bash
# 009 doctor: READ-ONLY dependency and environment check.
#
#   bash scripts/doctor.sh          # human readable
#   bash scripts/doctor.sh --quiet  # exit code only
#
# Exit codes (TEST_AND_ACCEPTANCE §9):
#   0 = all checked items present
#   2 = input/config error
#   3 = dependency / startup problem
#
# This script never installs anything, never starts a simulator, and never
# modifies the environment. It answers one question: "is what this project
# needs actually here?" It deliberately reports UNKNOWN rather than guessing.

set -uo pipefail

QUIET=0
[ "${1:-}" = "--quiet" ] && QUIET=1

# ROS's setup.bash references variables that are legitimately unset on a fresh
# shell (e.g. COLCON_PREFIX_PATH). With `set -u` active that aborts the script,
# which is why sourcing has to be bracketed by set +u / set -u.
_009_source_ros() {
  set +u
  # shellcheck disable=SC1090,SC1091
  . "$1" >/dev/null 2>&1
  local rc=$?
  set -u
  return $rc
}

FAIL=0
WARN=0

# These helpers MUST return 0.
#
# `[ "$QUIET" -eq 0 ] && printf ...` returns the *test's* status when quiet is
# on, which makes `cmd && ok || bad` fire the `bad` branch even on success --
# a quiet run that reports everything as missing. Always return 0 explicitly.
say()  { if [ "$QUIET" -eq 0 ]; then printf '%s\n' "$*"; fi; return 0; }
ok()   { if [ "$QUIET" -eq 0 ]; then printf '  [ OK ]      %s\n' "$*"; fi; return 0; }
bad()  { FAIL=1; printf '  [ MISSING ] %s\n' "$*"; return 0; }
warn() { WARN=1; if [ "$QUIET" -eq 0 ]; then printf '  [ WARN ]    %s\n' "$*"; fi; return 0; }
unk()  { if [ "$QUIET" -eq 0 ]; then printf '  [ UNKNOWN ] %s\n' "$*"; fi; return 0; }

_009_self="${BASH_SOURCE[0]:-$0}"
ROOT="$(cd "$(dirname "$_009_self")/.." && pwd)"

say "009 doctor -- read-only"
say "project root: $ROOT"
say ""

# ---------------------------------------------------------------- project files
say "project files"
for f in AGENTS.md README.md docs/MASTER_PLAN.md docs/CONTRACTS.md \
         docs/TEST_AND_ACCEPTANCE.md docs/AI_EXECUTION_PROMPTS.md \
         docs/IMPLEMENTATION_STATUS.md docs/ENVIRONMENT.md; do
  # if/else rather than &&/||: the &&/|| form is what produced the false
  # "MISSING" report described above.
  if [ -f "$ROOT/$f" ]; then ok "$f"; else bad "$f"; fi
done
say ""

# ------------------------------------------------------------------------ OS
say "operating system"
if [ -r /etc/os-release ]; then
  # shellcheck disable=SC1091
  . /etc/os-release
  ok "${PRETTY_NAME:-unknown}"
  case "${VERSION_ID:-}" in
    26.04) ;;
    *) warn "expected Ubuntu 26.04 (frozen stack), got ${VERSION_ID:-unknown}" ;;
  esac
else
  bad "/etc/os-release unreadable"
fi
ok "nproc=$(nproc)"
say ""

# ---------------------------------------------------------------------- python
say "python"
PY="$(command -v python3 || true)"
if [ -n "$PY" ]; then
  ok "python3 -> $PY ($(python3 --version 2>&1))"
  case "$(python3 --version 2>&1)" in
    *3.14*) ;;
    *) warn "expected Python 3.14 (ROS Lyrical ABI); other versions will split the interpreter" ;;
  esac
else
  bad "python3 not on PATH"
fi
say ""

# ------------------------------------------------------------------------- ROS
say "ros 2"
DISTRO="${FLEET009_ROS_DISTRO:-lyrical}"
ROS_SETUP="/opt/ros/${DISTRO}/setup.bash"
if [ -f "$ROS_SETUP" ]; then
  ok "base: $ROS_SETUP"
  say "  available distros: $(ls -1 /opt/ros 2>/dev/null | tr '\n' ' ')"
  _009_source_ros "$ROS_SETUP"
  if command -v ros2 >/dev/null 2>&1; then
    ok "ros2 -> $(command -v ros2)"
  else
    bad "ros2 not on PATH after sourcing $ROS_SETUP"
  fi
  if python3 -c "import rclpy" >/dev/null 2>&1; then
    ok "import rclpy"
  else
    bad "import rclpy failed under $PY"
  fi
else
  bad "ROS base not found: $ROS_SETUP"
fi
say ""

# ------------------------------------------------------------------- ROS pkgs
say "ros packages"
if command -v ros2 >/dev/null 2>&1; then
  for p in nav2_msgs ros_gz_bridge ros_gz_sim rviz2 xacro robot_state_publisher; do
    if ros2 pkg prefix "$p" >/dev/null 2>&1; then ok "$p"; else bad "$p"; fi
  done
  # These come from the /opt/nav2 source overlay, not the base distro.
  for p in nav2_bringup nav2_amcl; do
    if ros2 pkg prefix "$p" >/dev/null 2>&1; then
      ok "$p ($(ros2 pkg prefix "$p" 2>/dev/null))"
    else
      warn "$p NOT found -- expected from /opt/nav2 overlay; source scripts/env.sh first (P2 blocker)"
    fi
  done
else
  bad "skipped: ros2 unavailable"
fi
say ""

# ---------------------------------------------------------------------- gazebo
say "gazebo"
if command -v gz >/dev/null 2>&1; then
  ok "gz -> $(command -v gz)"
  GZV="$(gz sim --versions 2>/dev/null | head -1)"
  if [ -n "$GZV" ]; then
    ok "gz sim --versions = $GZV"
    case "$GZV" in
      10.*) ;;
      *) warn "frozen stack is gz-sim 10.x; got $GZV -- re-verify docs and docs/ENVIRONMENT.md" ;;
    esac
  else
    unk "gz sim --versions returned nothing"
  fi
else
  bad "gz not on PATH"
fi
say ""

# ------------------------------------------------------------------- graphics
say "graphics (headless is the supported path on this machine)"
if [ -d /usr/share/vulkan/icd.d ]; then
  if [ -f /usr/share/vulkan/icd.d/nvidia_icd.json ]; then
    ok "nvidia vulkan ICD present"
  else
    warn "no nvidia_icd.json -- Gazebo GUI likely falls back to dzn/lvp; use --headless"
  fi
  say "  icds: $(ls -1 /usr/share/vulkan/icd.d 2>/dev/null | tr '\n' ' ')"
fi
command -v nvidia-smi >/dev/null 2>&1 && ok "nvidia-smi present" || warn "nvidia-smi absent"
say ""

# ------------------------------------------------------------------------ disk
say "resources"
AVAIL_KB="$(df -Pk "$ROOT" 2>/dev/null | awk 'NR==2{print $4}')"
if [ -n "${AVAIL_KB:-}" ]; then
  ok "free on project volume: $((AVAIL_KB / 1024 / 1024)) GiB"
else
  unk "could not read free space for $ROOT"
fi
say ""

# ----------------------------------------------------------------------- isolate
say "isolation"
printf '  ROS_DOMAIN_ID=%s  GZ_PARTITION=%s  RMW_IMPLEMENTATION=%s\n' \
  "${ROS_DOMAIN_ID:-<unset>}" "${GZ_PARTITION:-<unset>}" "${RMW_IMPLEMENTATION:-<unset>}"
[ -n "${ROS_DOMAIN_ID:-}" ] || warn "ROS_DOMAIN_ID unset -- source scripts/env.sh"
[ -n "${GZ_PARTITION:-}" ] || warn "GZ_PARTITION unset -- source scripts/env.sh"
say ""

# ----------------------------------------------------------------------- result
if [ "$FAIL" -ne 0 ]; then
  say "RESULT: dependency/startup problem (exit 3)"
  exit 3
elif [ "$WARN" -ne 0 ]; then
  say "RESULT: usable with warnings; see WARN lines above (exit 0)"
  exit 0
else
  say "RESULT: all checked items present (exit 0)"
  exit 0
fi
