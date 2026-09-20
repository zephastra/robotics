#!/usr/bin/env bash
# Build the four 009 packages into ./install.
#
#   bash scripts/build.sh            # build everything
#   bash scripts/build.sh --clean    # remove build/ install/ log/ first
#
# Why a thin wrapper instead of calling colcon directly: the two things that go wrong
# here are forgetting to source the environment and building with the wrong Python,
# and both produce confusing errors much later. This checks both up front.
set -euo pipefail

_self="${BASH_SOURCE[0]:-$0}"
ROOT="$(cd "$(dirname "$_self")/.." && pwd)"
cd "$ROOT"

if [ "${1:-}" = "--clean" ]; then
  echo "removing build/ install/ log/"
  rm -rf build install log
fi

# shellcheck disable=SC1091
#
# set +u around the source is not optional. ROS's setup.bash reads variables such as
# AMENT_TRACE_SETUP_FILES without a default, so under `set -u` it aborts with
# "unbound variable" partway through sourcing. The symptom is a half configured
# environment and a confusing failure much later, so -u is suspended for exactly as
# long as the sourcing takes and restored immediately afterwards.
set +u
source scripts/env.sh
set -u

if ! command -v colcon >/dev/null 2>&1; then
  echo "build.sh: colcon not found. Install it (pip install colcon-common-extensions)" >&2
  exit 2
fi

if [ -z "${ROS_DISTRO:-}" ]; then
  echo "build.sh: ROS_DISTRO is unset after sourcing scripts/env.sh" >&2
  exit 2
fi

if ! python3 -c 'import rclpy' 2>/dev/null; then
  echo "build.sh: python3 cannot import rclpy." >&2
  echo "          Check that ROS_DISTRO=$ROS_DISTRO matches the python3 on PATH:" >&2
  echo "          $(command -v python3) -> $(python3 -V 2>&1)" >&2
  exit 2
fi

# Static guards BEFORE compiling. Each of these catches a fault whose runtime symptom
# points somewhere else entirely, and a build that reports success while a guard was
# skipped is how this project lost two three-robot runs to a shadowed `self.clients`.
bash scripts/check_guards.sh
echo

echo "colcon build in $ROOT"
echo "  ROS_DISTRO = $ROS_DISTRO"
echo "  python3    = $(command -v python3) ($(python3 -V 2>&1))"
echo

# --symlink-install so editing a launch file or a Node does not require a rebuild.
colcon build \
  --symlink-install \
  --base-paths src \
  --build-base build \
  --install-base install \
  --packages-select fleet_interfaces fleet_core fleet_adapter fleet_ros fleet_bringup fleet_tools fleet_evaluation

# ---------------------------------------------------------------------------
# Render the per-robot models.
#
# A launch refuses to start without runtime/models/amr_4wd_<robot>/model.sdf, and until
# this was added NOTHING produced them: build.sh built packages, run_demo.sh launched, and
# the render step existed only in whoever's memory ran it first. A copy of the tree on a
# clean machine therefore built successfully and then died at launch with
# "rendered model not found". That is a procedure that was not relocatable even though the
# tree was, and it is exactly what scripts/verify_independence.sh exists to find.
# ---------------------------------------------------------------------------
echo
echo "rendering per-robot models into runtime/models/"
robots="$(python3 - <<'RENDER_LIST'
import pathlib
import yaml

spawns = (yaml.safe_load(pathlib.Path("config/spawns.yaml").read_text(encoding="utf-8"))
          or {}).get("spawns") or {}
if not spawns:
    raise SystemExit("build.sh: config/spawns.yaml lists no robots")
print(" ".join(sorted(spawns)))
RENDER_LIST
)" || exit 4
for _robot in $robots; do
  python3 scripts/render_model.py --robot "$_robot" --out runtime/models || exit 4
done
unset _robot

echo
echo "build.sh: done. Source the workspace before launching:"
echo "    source scripts/env.sh"
