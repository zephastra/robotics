#!/usr/bin/env bash
set -Eeuo pipefail
source "$(dirname "${BASH_SOURCE[0]}")/env.sh"
python3 -m pytest "$MM_ROOT/tests" -q
