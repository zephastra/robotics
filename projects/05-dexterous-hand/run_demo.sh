#!/usr/bin/env bash
set -Eeuo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$ROOT"
[[ -x .venv/bin/python ]] || { echo 'Run bash scripts/setup.sh first'; exit 1; }
export PYTHONPATH="$ROOT/src"
export PYTHONDONTWRITEBYTECODE=1
# Per-process WSLg renderer selection; no driver or system configuration changes.
if [[ "${H005_RENDERER:-auto}" == software ]]; then
  export GALLIUM_DRIVER=llvmpipe
elif [[ -f /usr/lib/wsl/lib/libd3d12.so ]]; then
  export GALLIUM_DRIVER=d3d12
fi
exec .venv/bin/python -m hand005.app "$@"
