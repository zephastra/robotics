#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
# shellcheck disable=SC1091
source "$PROJECT_DIR/scripts/amr_env.sh"

OUTPUT_PREFIX="${1:-$PROJECT_DIR/maps/map}"
mkdir -p "$(dirname "$OUTPUT_PREFIX")"

echo "Saving map to ${OUTPUT_PREFIX}.pgm and ${OUTPUT_PREFIX}.yaml ..."
ros2 run nav2_map_server map_saver_cli -f "$OUTPUT_PREFIX"

test -s "${OUTPUT_PREFIX}.pgm"
test -s "${OUTPUT_PREFIX}.yaml"
echo "Map saved successfully."
