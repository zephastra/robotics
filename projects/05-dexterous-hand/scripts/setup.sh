#!/usr/bin/env bash
set -Eeuo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"
command -v uv >/dev/null || { echo 'Install uv: https://docs.astral.sh/uv/getting-started/installation/'; exit 1; }
command -v git >/dev/null
if [[ ! -x .venv/bin/python ]]; then uv venv --python 3.12.13 .venv; fi
uv pip install --python .venv/bin/python -r requirements.txt
REV=8161bba264d7fa7c99ca301e91e7fb44737676ad
if [[ ! -d .cache/menagerie/.git ]]; then
  mkdir -p .cache
  git clone --filter=blob:none --no-checkout https://github.com/google-deepmind/mujoco_menagerie.git .cache/menagerie
fi
git -C .cache/menagerie sparse-checkout init --cone
git -C .cache/menagerie sparse-checkout set wonik_allegro
git -C .cache/menagerie checkout --detach "$REV"
.venv/bin/python scripts/prepare_assets.py
echo '005 ready: bash run_demo.sh'
