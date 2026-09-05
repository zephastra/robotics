#!/usr/bin/env bash
set -eo pipefail
source "$(dirname "${BASH_SOURCE[0]}")/env.sh"
for command in python3 cmake c++ gz ros2 rsync flock setsid; do
  command -v "$command" >/dev/null || { echo "Missing system command: $command"; exit 1; }
done
python3 "$MM_ROOT/tools/check_environment.py"
