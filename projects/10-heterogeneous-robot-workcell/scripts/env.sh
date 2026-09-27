#!/usr/bin/env bash
# 010 environment setup. Source this, do not execute it.
#
#   source scripts/env.sh
#
# Contract (same shape as 009's, for the same reasons):
#   * sources ONLY the ROS distro and overlays this project has verified
#   * resolves every path from the project root or ament share -- never from cwd
#   * prints exactly what is missing instead of failing silently
#   * does NOT write anything to global ~/.bashrc
#
# Domain isolation is not cosmetic. 009 pins ROS_DOMAIN_ID=42 (009/scripts/env.sh:28, measured
# 2026-09-20), and two projects on one host sharing a domain share one graph: 010's clock
# publisher would show up in 009's `ros2 topic list`, and 009's fleet would appear in 010's.
# So this file refuses to run on 009's domain rather than trusting the caller to remember.

_010_self="${BASH_SOURCE[0]:-$0}"
_010_dir="$(cd "$(dirname "$_010_self")" && pwd)"
export WORKCELL010_ROOT="$(cd "$_010_dir/.." && pwd)"

export WORKCELL010_ROS_DISTRO="${WORKCELL010_ROS_DISTRO:-lyrical}"
export WORKCELL010_NAV2_OVERLAY="${WORKCELL010_NAV2_OVERLAY:-/opt/nav2/setup.bash}"

# ---- domain --------------------------------------------------------------
if [ "${ROS_DOMAIN_ID:-}" = "42" ]; then
  printf 'env.sh: FATAL: ROS_DOMAIN_ID=42 is 009'"'"'s domain. Unset it or set\n' >&2
  printf 'env.sh:        WORKCELL010_DOMAIN to a free id (default 43) and source again.\n' >&2
  return 1 2>/dev/null || exit 1
fi
export ROS_DOMAIN_ID="${WORKCELL010_DOMAIN:-43}"
if [ "$ROS_DOMAIN_ID" = "42" ]; then
  printf 'env.sh: FATAL: WORKCELL010_DOMAIN=42 is 009'"'"'s domain.\n' >&2
  return 1 2>/dev/null || exit 1
fi

# ---- DDS pinned to loopback ---------------------------------------------
# WSL2 exposes several UP interfaces, and with default discovery the graph is only PARTIALLY
# visible and which nodes are missing changes between runs -- which reads exactly like a
# broken bridge. 009 measured this and pinned loopback; 010 inherits the conclusion, not the
# file. Loopback is right for a single-host simulation and WRONG across hosts.
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
if [ -f "$WORKCELL010_ROOT/config/cyclonedds.xml" ]; then
  export CYCLONEDDS_URI="file://$WORKCELL010_ROOT/config/cyclonedds.xml"
fi

# ---- ROS base ------------------------------------------------------------
_010_ros_setup="/opt/ros/${WORKCELL010_ROS_DISTRO}/setup.bash"
if [ -f "$_010_ros_setup" ]; then
  # shellcheck disable=SC1090
  source "$_010_ros_setup"
else
  printf 'env.sh: MISSING ROS base: %s\n' "$_010_ros_setup" >&2
  printf 'env.sh: available: %s\n' "$(ls -1 /opt/ros 2>/dev/null | tr '\n' ' ')" >&2
fi

# nav2 is a source build on this machine, not part of /opt/ros/lyrical.
if [ -f "$WORKCELL010_NAV2_OVERLAY" ]; then
  # shellcheck disable=SC1090
  source "$WORKCELL010_NAV2_OVERLAY"
else
  printf 'env.sh: nav2 overlay not found (skipped): %s\n' "$WORKCELL010_NAV2_OVERLAY" >&2
fi

# ---- project paths -------------------------------------------------------
export WORKCELL010_CONFIG="$WORKCELL010_ROOT/config"
export WORKCELL010_ASSETS="$WORKCELL010_ROOT/assets"
export WORKCELL010_REPORTS="$WORKCELL010_ROOT/reports"
# The ROS side runs under the system interpreter and must import workcell_ipc, which lives
# with the sim side in src/. The .venv is deliberately not on this path: it holds MuJoCo and
# would put a second mujoco on the ROS interpreter.
export PYTHONPATH="$WORKCELL010_ROOT/src${PYTHONPATH:+:$PYTHONPATH}"
mkdir -p "$WORKCELL010_REPORTS" 2>/dev/null

if [ -n "${WORKCELL010_VERBOSE:-}" ]; then
  printf 'env.sh: WORKCELL010_ROOT=%s\n' "$WORKCELL010_ROOT"
  printf 'env.sh: ROS_DISTRO=%s  ROS_DOMAIN_ID=%s\n' "${ROS_DISTRO:-<unset>}" "$ROS_DOMAIN_ID"
  printf 'env.sh: RMW=%s\n' "${RMW_IMPLEMENTATION:-<unset>}"
  printf 'env.sh: CYCLONEDDS_URI=%s\n' "${CYCLONEDDS_URI:-<unset>}"
  printf 'env.sh: python3=%s\n' "$(command -v python3)"
fi
unset _010_ros_setup _010_dir _010_self
