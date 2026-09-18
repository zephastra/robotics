#!/usr/bin/env bash
set -Eeuo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"
command -v uv >/dev/null || { echo 'Install uv: https://docs.astral.sh/uv/getting-started/installation/'; exit 1; }
command -v git >/dev/null
[[ -x .venv/bin/python ]] || uv venv --python 3.12.13 .venv
uv pip install --python .venv/bin/python -r requirements.txt
.venv/bin/python scripts/fix_mnn_stack.py
if [[ ! -d .cache/engineai_native_sdk/.git ]]; then
  git clone --filter=blob:none --no-checkout https://github.com/engineai-robotics/engineai_robotics_native_sdk.git .cache/engineai_native_sdk
fi
git -C .cache/engineai_native_sdk sparse-checkout init --cone
git -C .cache/engineai_native_sdk sparse-checkout set assets/resource/robot/t800 assets/resource/environment assets/config/t800
git -C .cache/engineai_native_sdk checkout --detach 335c60e88772c26c7852d0abd6b3c7439037dd8f
.venv/bin/python scripts/prepare_t800.py
if [[ ! -d .cache/menagerie/.git ]]; then
  git clone --filter=blob:none --no-checkout https://github.com/google-deepmind/mujoco_menagerie.git .cache/menagerie
fi
git -C .cache/menagerie sparse-checkout init --cone
git -C .cache/menagerie sparse-checkout set wonik_allegro
git -C .cache/menagerie checkout --detach 8161bba264d7fa7c99ca301e91e7fb44737676ad
.venv/bin/python scripts/prepare_model.py
