#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"

# shellcheck disable=SC1091
source "$SCRIPT_DIR/amr_env.sh"
cd "$PROJECT_DIR"

if [[ ! -f "$PROJECT_DIR/install/setup.bash" ]]; then
  bash "$SCRIPT_DIR/build.sh"
fi

# shellcheck disable=SC1091
set +u
source "$PROJECT_DIR/install/setup.bash"
set -u
PYTHONDONTWRITEBYTECODE=1 python3 -m pytest \
  "$PROJECT_DIR/src/amr_mission_manager/test" \
  "$PROJECT_DIR/tests" \
  -p no:cacheprovider \
  -q
