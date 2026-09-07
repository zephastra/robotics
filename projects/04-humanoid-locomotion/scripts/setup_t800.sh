#!/usr/bin/env bash
set -Eeuo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"
[[ -x .venv/bin/python ]] || { echo 'Run bash scripts/setup.sh first.'; exit 1; }
command -v uv >/dev/null || { echo 'uv is required.'; exit 1; }
uv pip install --python .venv/bin/python 'MNN==3.6.1' 'patchelf==0.19.1.0'
.venv/bin/python scripts/fix_mnn_stack.py
REV=335c60e88772c26c7852d0abd6b3c7439037dd8f
if [[ ! -d .cache/engineai_native_sdk/.git ]]; then
  mkdir -p .cache
  git clone --filter=blob:none --no-checkout https://github.com/engineai-robotics/engineai_robotics_native_sdk.git .cache/engineai_native_sdk
  git -C .cache/engineai_native_sdk sparse-checkout init --cone
  git -C .cache/engineai_native_sdk sparse-checkout set assets/resource/robot/t800 assets/resource/environment assets/config/t800
fi
git -C .cache/engineai_native_sdk checkout --detach "$REV"
.venv/bin/python scripts/prepare_t800.py
echo 'T800 ready. Run bash run_demo.sh --robot t800'
