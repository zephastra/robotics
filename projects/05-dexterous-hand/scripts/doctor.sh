#!/usr/bin/env bash
set -Eeuo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
[[ -x "$ROOT/.venv/bin/python" ]] || { echo 'Missing .venv: bash scripts/setup.sh'; exit 1; }
export PYTHONPATH="$ROOT/src"
exec "$ROOT/.venv/bin/python" "$ROOT/scripts/doctor.py"
