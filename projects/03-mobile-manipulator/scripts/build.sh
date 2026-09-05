#!/usr/bin/env bash
set -Eeuo pipefail
source "$(dirname "${BASH_SOURCE[0]}")/env.sh"
python3 "$MM_ROOT/tools/generate_assets.py"
if [[ ! -f "$MM_ROOT/build/Makefile" || "$MM_ROOT/CMakeLists.txt" -nt "$MM_ROOT/build/Makefile" ]]; then
  cmake -S "$MM_ROOT" -B "$MM_ROOT/build" -DCMAKE_BUILD_TYPE=Release
fi
cmake --build "$MM_ROOT/build" -j2
gz sdf -k "$MM_ROOT/worlds/workcell.sdf"
