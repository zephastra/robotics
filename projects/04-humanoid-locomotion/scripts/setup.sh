#!/usr/bin/env bash
set -Eeuo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
command -v uv >/dev/null || { echo 'Install uv first: https://docs.astral.sh/uv/getting-started/installation/'; exit 1; }
command -v git >/dev/null || { echo 'git is required'; exit 1; }
cd "$ROOT"
if [[ ! -x .venv/bin/python ]]; then uv venv --python 3.12.13 .venv; fi
uv pip install --python .venv/bin/python -r requirements.txt
uv pip install --python .venv/bin/python 'torch==2.7.1+cpu' --index-url https://download.pytorch.org/whl/cpu
UPSTREAM_COMMIT=276801e46c5d433564f24658bac64f254b7d2d4b
if [[ ! -d .cache/unitree_rl_gym/.git ]]; then
  mkdir -p .cache
  git clone --filter=blob:none --no-checkout https://github.com/unitreerobotics/unitree_rl_gym.git .cache/unitree_rl_gym
fi
git -C .cache/unitree_rl_gym sparse-checkout init --cone
git -C .cache/unitree_rl_gym sparse-checkout set resources/robots/g1_description deploy/pre_train/g1 deploy/deploy_mujoco
git -C .cache/unitree_rl_gym checkout --detach "$UPSTREAM_COMMIT"
.venv/bin/python scripts/prepare_assets.py
echo '004 setup complete. This environment does not load ROS or historical projects.'
