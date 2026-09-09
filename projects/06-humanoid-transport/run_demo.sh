#!/usr/bin/env bash
set -Eeuo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$ROOT"
if [[ ! -x .venv/bin/python || ! -f assets/combined.xml ]]; then
  echo 'Run bash scripts/setup.sh first.' >&2
  exit 1
fi
export PYTHONPATH="$ROOT/src${PYTHONPATH:+:$PYTHONPATH}"
exec .venv/bin/python -m humanoid006.app "$@"
