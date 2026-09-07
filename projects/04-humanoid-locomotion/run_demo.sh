#!/usr/bin/env bash
set -Eeuo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$ROOT"
[[ -x .venv/bin/python && -f assets/manifest.json ]] || { echo 'Run bash scripts/setup.sh first.'; exit 1; }
export PYTHONPATH="$ROOT/src"
unset PYTHONHOME
export PYTHONDONTWRITEBYTECODE=1
# Per-process WSL graphics selection only; never change Mesa or shell profiles.
if [[ "${H004_RENDERER:-auto}" == software ]]; then
  export GALLIUM_DRIVER=llvmpipe
elif [[ "${H004_RENDERER:-auto}" == d3d12 ]] || { [[ "${H004_RENDERER:-auto}" == auto ]] && [[ -f /usr/lib/wsl/lib/libd3d12.so && -f /usr/lib/x86_64-linux-gnu/dri/d3d12_dri.so ]]; }; then
  export GALLIUM_DRIVER="${GALLIUM_DRIVER:-d3d12}"
  export MESA_D3D12_DEFAULT_ADAPTER_NAME="${H004_GPU_ADAPTER:-NVIDIA}"
fi
exec .venv/bin/python -m humanoid004.app "$@"
