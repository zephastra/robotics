#!/usr/bin/env bash
set -Eeuo pipefail

if [[ $# -ne 4 ]]; then
  echo "Usage: $0 X Y Z LABEL" >&2
  exit 2
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "$SCRIPT_DIR/amr_env.sh"

POSITION_X="$1"
POSITION_Y="$2"
POSITION_Z="$3"
POSITION_LABEL="$4"
SERVICE_NAME="/world/warehouse_mission_world/set_pose"
REQUEST="name: 'movable_test_cart', position: {x: ${POSITION_X}, y: ${POSITION_Y}, z: ${POSITION_Z}}, orientation: {w: 1.0}"

for _attempt in $(seq 1 20); do
  if response="$(
    gz service \
      -s "$SERVICE_NAME" \
      --reqtype gz.msgs.Pose \
      --reptype gz.msgs.Boolean \
      --timeout 3000 \
      --req "$REQUEST" 2>&1
  )" && grep -q "data: true" <<<"$response"; then
    echo "Movable test cart moved to $POSITION_LABEL at ($POSITION_X, $POSITION_Y)."
    exit 0
  fi
  sleep 1
done

echo "Unable to move movable_test_cart through $SERVICE_NAME." >&2
exit 1
