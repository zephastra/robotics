#!/usr/bin/env bash
# 009 environment setup. Source this, do not execute it.
#
#   source scripts/env.sh
#
# Contract (MASTER_PLAN §6):
#   * sources ONLY the ROS distro and overlays this project has verified
#   * resolves every path from the project root or ament share -- never from cwd
#   * prints exactly what is missing instead of failing silently
#   * does NOT write anything to global ~/.bashrc
#
# Every overlay below is opt-in by existence; sourcing it is only attempted when
# the file is present. Anything not sourced is reported so the caller knows the
# difference between "found nothing" and "did not look".

# ---- project root (independent of the caller's cwd) ----------------------
# BASH_SOURCE works under `source`; fall back to $0 for `bash scripts/env.sh`.
_009_self="${BASH_SOURCE[0]:-$0}"
_009_dir="$(cd "$(dirname "$_009_self")" && pwd)"
export FLEET009_ROOT="$(cd "$_009_dir/.." && pwd)"

# ---- pinned values -------------------------------------------------------
# Ubuntu 26.04 / ROS 2 Lyrical / gz-sim 10.4.0 -- see docs/ENVIRONMENT.md
export FLEET009_ROS_DISTRO="${FLEET009_ROS_DISTRO:-lyrical}"

# Dedicated domain so 009 never cross-talks with other projects on this machine.
# Not yet verified free -- doctor.sh reports the current value. Override here.
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"

# ---- DDS: pin discovery to loopback (see config/cyclonedds.xml) ----------
# Without this the graph is only PARTIALLY visible, and which nodes are missing changes
# between runs. That is a discovery race across WSL2's four UP interfaces, and it reads
# exactly like a broken bridge: `ros2 topic echo` says "does not appear to be published
# yet" while the publisher is demonstrably alive in the launch log.
#
# Loopback is correct for a single-host simulation. It is WRONG for real multi-machine
# operation: revisit config/cyclonedds.xml before 009 spans more than one host.
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
if [ -f "$FLEET009_ROOT/config/cyclonedds.xml" ]; then
  export CYCLONEDDS_URI="file://$FLEET009_ROOT/config/cyclonedds.xml"
fi

# Fresh Gazebo partition per run; run_demo.sh overrides this per invocation so
# two concurrent 009 runs cannot see each other's topics.
export GZ_PARTITION="${GZ_PARTITION:-fleet009_$$}"

# ---- ROS base ------------------------------------------------------------
_009_ros_setup="/opt/ros/${FLEET009_ROS_DISTRO}/setup.bash"
if [ -f "$_009_ros_setup" ]; then
  # shellcheck disable=SC1090
  source "$_009_ros_setup"
else
  printf 'env.sh: MISSING ROS base: %s\n' "$_009_ros_setup" >&2
  printf 'env.sh: available distros: %s\n' "$(ls -1 /opt/ros 2>/dev/null | tr '\n' ' ')" >&2
fi

# ---- source-built overlays (opt-in) --------------------------------------
# nav2_bringup / nav2_amcl are NOT in /opt/ros/lyrical on this machine; they
# come from the /opt/nav2 source build. Whether that overlay actually provides
# them is still NOT_RUN -- see docs/IMPLEMENTATION_STATUS.md.
_009_overlays=(
  "${FLEET009_NAV2_OVERLAY:-/opt/nav2/setup.bash}"
  "${FLEET009_MOVEIT_OVERLAY:-/opt/moveit2/setup.bash}"
)
for _009_ov in "${_009_overlays[@]}"; do
  if [ -f "$_009_ov" ]; then
    # shellcheck disable=SC1090
    source "$_009_ov"
  else
    printf 'env.sh: overlay not found (skipped): %s\n' "$_009_ov" >&2
  fi
done
unset _009_ov

# ---- project-local paths -------------------------------------------------
export FLEET009_CONFIG="$FLEET009_ROOT/config"
export FLEET009_ASSETS="$FLEET009_ROOT/assets"
export FLEET009_RUNTIME="$FLEET009_ROOT/runtime"
export FLEET009_REPORTS="$FLEET009_ROOT/reports"
export FLEET009_DOCS="$FLEET009_ROOT/docs"

mkdir -p "$FLEET009_RUNTIME" "$FLEET009_REPORTS" 2>/dev/null

# ---- aliases used by the launch files ------------------------------------
# The launch files cannot assume they were invoked from this shell, and they must
# not guess the project root from cwd. These three names are the contract between
# env.sh and fleet_bringup:
#   FLEET_REPO_ROOT    source tree, for assets and as a fallback for rendered models
#   FLEET_RUNTIME_DIR  generated artifacts (rendered models, generated params)
#   FLEET_MODELS_DIR   runtime/models, added to GZ_SIM_RESOURCE_PATH
export FLEET_REPO_ROOT="$FLEET009_ROOT"
export FLEET_RUNTIME_DIR="$FLEET009_RUNTIME"
export FLEET_MODELS_DIR="$FLEET009_RUNTIME/models"
mkdir -p "$FLEET_MODELS_DIR" 2>/dev/null

# ---- this workspace's own install space (LAST, so it wins) ---------------
# Sourced after every system overlay: if our fleet_core/fleet_ros/... are already
# built, they must take precedence over anything with a similar name elsewhere.
_009_local_install="$FLEET009_ROOT/install/setup.bash"
if [ -f "$_009_local_install" ]; then
  # shellcheck disable=SC1090
  source "$_009_local_install"
fi

if [ -n "${FLEET009_VERBOSE:-}" ]; then
  printf 'env.sh: FLEET009_ROOT=%s\n' "$FLEET009_ROOT"
  printf 'env.sh: ROS_DISTRO=%s\n' "${ROS_DISTRO:-<unset>}"
  printf 'env.sh: ROS_DOMAIN_ID=%s\n' "${ROS_DOMAIN_ID:-<unset>}"
  printf 'env.sh: GZ_PARTITION=%s\n' "${GZ_PARTITION:-<unset>}"
  printf 'env.sh: FLEET_MODELS_DIR=%s\n' "${FLEET_MODELS_DIR:-<unset>}"
  printf 'env.sh: own install sourced=%s\n' \
    "$([ -f "$_009_local_install" ] && echo yes || echo 'no (not built yet)')"
  printf 'env.sh: AMENT_PREFIX_PATH=%s\n' "${AMENT_PREFIX_PATH:-<unset>}"
fi
unset _009_local_install
