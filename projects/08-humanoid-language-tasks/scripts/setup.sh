#!/usr/bin/env bash
# 008 setup. Creates this project's own isolated virtual environment.
#
#   bash scripts/setup.sh --profile core
#   bash scripts/setup.sh --profile sim
#
# core : PyYAML + pytest only. No MuJoCo, no MNN, no downloaded assets.
# sim  : adds the simulation stack. Needs network access.
#
# The interpreter is pinned to Python 3.12 (the validated compatibility
# baseline from docs/MASTER_PLAN.md section 7). The system python3 on this
# machine is 3.14, so the ambient interpreter is deliberately NOT used.
set -euo pipefail

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
VENV_DIR="${PROJECT_ROOT}/.venv"
PROFILE="core"
PYTHON_REQUEST="3.12"

while [ $# -gt 0 ]; do
  case "$1" in
    --profile)   PROFILE="${2:-}"; shift 2 ;;
    --profile=*) PROFILE="${1#*=}"; shift ;;
    -h|--help)   sed -n '2,13p' "$0"; exit 0 ;;
    *) echo "unknown argument: $1" >&2; exit 2 ;;
  esac
done

case "${PROFILE}" in
  core|sim) ;;
  *) echo "unknown profile: '${PROFILE}' (expected core or sim)" >&2; exit 2 ;;
esac

REQ="${PROJECT_ROOT}/requirements-${PROFILE}.lock"
if [ ! -f "${REQ}" ]; then
  echo "missing requirements file: ${REQ}" >&2
  exit 1
fi

echo "== profile=${PROFILE}  interpreter=${PYTHON_REQUEST}  venv=${VENV_DIR}"

if command -v uv >/dev/null 2>&1; then
  echo "== using uv"
  if [ ! -x "${VENV_DIR}/bin/python" ]; then
    uv venv --python "${PYTHON_REQUEST}" "${VENV_DIR}"
  fi
  uv pip install --python "${VENV_DIR}/bin/python" -r "${REQ}"
  uv pip install --python "${VENV_DIR}/bin/python" -e "${PROJECT_ROOT}" --no-deps
else
  echo "== uv not found; falling back to python${PYTHON_REQUEST} -m venv"
  if ! command -v "python${PYTHON_REQUEST}" >/dev/null 2>&1; then
    echo "python${PYTHON_REQUEST} not found on PATH." >&2
    echo "This project is pinned to Python ${PYTHON_REQUEST}." >&2
    echo "Install it, or install uv so it can provide the interpreter." >&2
    exit 1
  fi
  if [ ! -x "${VENV_DIR}/bin/python" ]; then
    "python${PYTHON_REQUEST}" -m venv "${VENV_DIR}"
  fi
  "${VENV_DIR}/bin/python" -m pip install --upgrade pip
  "${VENV_DIR}/bin/python" -m pip install -r "${REQ}"
  "${VENV_DIR}/bin/python" -m pip install -e "${PROJECT_ROOT}" --no-deps
fi

ACTUAL="$("${VENV_DIR}/bin/python" -c 'import sys; print("%d.%d" % sys.version_info[:2])')"
if [ "${ACTUAL}" != "${PYTHON_REQUEST}" ]; then
  echo "expected Python ${PYTHON_REQUEST}, got ${ACTUAL}" >&2
  exit 1
fi

# The MNN 3.6.1 wheel ships a shared object that requests an executable stack.
# Clear that request locally (reversible, backup recorded) so the CPU walking
# policy can load without a stack-permission refusal.
if [ "${PROFILE}" = "sim" ]; then
  echo "== fixing MNN executable-stack request"
  "${VENV_DIR}/bin/python" "${PROJECT_ROOT}/scripts/fix_mnn_stack.py"
fi

echo
echo "done: profile=${PROFILE} python=${ACTUAL}"
echo "next: .venv/bin/python scripts/doctor.py --profile ${PROFILE}"
