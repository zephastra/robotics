"""008 environment doctor.

Read-only: it inspects and reports. It never installs, repairs, downloads or
modifies anything, and it never prints secrets.

    .venv/bin/python scripts/doctor.py --profile core
    .venv/bin/python scripts/doctor.py --profile sim
"""

from __future__ import annotations

import argparse
import importlib.util
import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]

EXIT_OK = 0
EXIT_FAILED = 1

PINNED_PYTHON = (3, 12)

SIM_MODULES = ("mujoco", "numpy", "MNN", "PIL")


def check(label: str, ok: bool, detail: str = "") -> bool:
    mark = "ok  " if ok else "FAIL"
    suffix = f" - {detail}" if detail else ""
    print(f"[{mark}] {label}{suffix}")
    return ok


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        prog="doctor.py",
        description="008 environment doctor (read-only).",
    )
    parser.add_argument("--profile", choices=("core", "sim"), default="core")
    args = parser.parse_args(argv)

    failures = 0

    version = sys.version.split()[0]

    if not check("python >= 3.12", sys.version_info[:2] >= (3, 12), version):
        failures += 1

    pin_ok = sys.version_info[:2] == PINNED_PYTHON
    if not check(
        "python pinned to 3.12",
        pin_ok,
        version if pin_ok else f"running {version}; use .venv/bin/python scripts/doctor.py",
    ):
        failures += 1

    if not check(
        "project layout",
        (PROJECT_ROOT / "src" / "humanoid008").is_dir(),
        str(PROJECT_ROOT),
    ):
        failures += 1

    venv_dir = PROJECT_ROOT / ".venv"
    venv_ok = venv_dir.is_dir()
    if not check(
        "virtual environment exists",
        venv_ok,
        str(venv_dir) if venv_ok else "missing; run: bash scripts/setup.sh --profile core",
    ):
        failures += 1

    in_venv = sys.prefix != sys.base_prefix
    if not check(
        "running inside a virtual environment",
        in_venv,
        sys.prefix if in_venv else "use .venv/bin/python scripts/doctor.py",
    ):
        failures += 1

    if not check("import yaml", importlib.util.find_spec("yaml") is not None, "core dependency"):
        failures += 1

    if args.profile == "sim":
        for module in SIM_MODULES:
            if not check(f"import {module}", importlib.util.find_spec(module) is not None, "sim dependency"):
                failures += 1

    reports = PROJECT_ROOT / "reports"
    check("reports/ present", reports.is_dir(), str(reports))

    print()
    if failures:
        print(f"{failures} check(s) failed")
        return EXIT_FAILED
    print("all checks passed")
    return EXIT_OK


if __name__ == "__main__":
    raise SystemExit(main())
