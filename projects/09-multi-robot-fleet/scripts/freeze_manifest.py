#!/usr/bin/env python3
"""P7: freeze the project and record what was frozen.

Writes two artefacts:

  reports/frozen_manifest.json   machine-readable: SHA-256 of every behaviour-defining
                                 file, plus the environment, the guard list and the
                                 scenario budgets that were in force
  docs/FROZEN.md                 the same thing for a reader, with the hashes so a
                                 later run can prove it used the same inputs

Why a hash list rather than a git tag: there is no git repository here (the project was
scaffolded from a handoff bundle), so the only way to say "these are the inputs" is to
write down what they were. `verify_independence.sh` re-runs this in the copied tree and
compares, which is what turns the manifest into evidence rather than a note.

Usage:
    scripts/freeze_manifest.py            # write both artefacts
    scripts/freeze_manifest.py --check    # compare against the recorded manifest
Exit 0 clean, 1 on a --check mismatch, 3 on an environment problem.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import platform
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
MANIFEST = ROOT / "reports" / "frozen_manifest.json"
FROZEN_DOC = ROOT / "docs" / "FROZEN.md"

#: Directories that define behaviour. `reports/` and `runtime/` are outputs; the hash
#: list must describe inputs, or it changes every time it is run.
INCLUDE_DIRS = ("assets", "config", "src", "scripts", "tests", "docs")
INCLUDE_FILES = (
    "pyproject.toml", "run_demo.sh", "stop_demo.sh", "README.md",
    "LICENSE", "THIRD_PARTY_NOTICES.md", "AGENTS.md",
)
EXCLUDE_PARTS = {"__pycache__", ".pytest_cache", "build", "install", "log",
                 "reports", "runtime", ".venv", ".git"}


def iter_files() -> "list[Path]":
    seen: set[Path] = set()
    out: list[Path] = []
    for d in INCLUDE_DIRS:
        base = ROOT / d
        if not base.is_dir():
            continue
        for path in sorted(base.rglob("*")):
            if not path.is_file():
                continue
            if EXCLUDE_PARTS & set(path.parts):
                continue
            # The manifest describes inputs, so it must not hash its own outputs.
            if path == MANIFEST or path == FROZEN_DOC:
                continue
            if path not in seen:
                seen.add(path)
                out.append(path)
    for name in INCLUDE_FILES:
        path = ROOT / name
        if path.is_file() and path not in seen:
            seen.add(path)
            out.append(path)
    return out


def sha256(path: Path) -> str:
    h = hashlib.sha256()
    with path.open("rb") as fh:
        for chunk in iter(lambda: fh.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


def guards() -> "list[str]":
    """The guards check_guards.sh actually runs, read from the script."""
    import re

    text = (ROOT / "scripts" / "check_guards.sh").read_text(encoding="utf-8")
    return sorted(set(re.findall(r'\$PY"\s+scripts/([\w.]+)', text)))


def environment() -> dict:
    def run(cmd: "list[str]") -> str:
        try:
            return subprocess.run(cmd, capture_output=True, text=True,
                                  timeout=20).stdout.strip().splitlines()[0]
        except Exception:
            return ""

    return {
        "host": platform.node(),
        "kernel": platform.release(),
        "python": platform.python_version(),
        "ros_distro": os.environ.get("FLEET009_ROS_DISTRO", "lyrical"),
        "ros_domain_id": os.environ.get("ROS_DOMAIN_ID", ""),
        "rmw": os.environ.get("RMW_IMPLEMENTATION", ""),
        "gz_version": run(["gz", "sim", "--versions"]) or "not probed",
        "project_root": str(ROOT),
    }


def scenario_budgets() -> dict:
    import yaml

    out: dict = {}
    scen = ROOT / "config" / "scenarios"
    if scen.is_dir():
        for path in sorted(scen.glob("*.yaml")):
            body = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
            out[path.name] = {
                "budgets": body.get("budgets") or {},
                "robots": body.get("robots") or {},
            }
    return out


def collect() -> dict:
    files = iter_files()
    return {
        "schema_version": "1",
        "frozen_at_utc": subprocess.run(["date", "-u", "+%Y-%m-%dT%H:%M:%SZ"],
                                        capture_output=True, text=True).stdout.strip(),
        "environment": environment(),
        "guards": guards(),
        "packages": ["fleet_interfaces", "fleet_core", "fleet_adapter", "fleet_ros",
                     "fleet_bringup", "fleet_tools", "fleet_evaluation"],
        "scenario_budgets": scenario_budgets(),
        "file_count": len(files),
        "files": {str(p.relative_to(ROOT)): sha256(p) for p in files},
    }


def write_doc(body: dict) -> None:
    env = body["environment"]
    lines = [
        "# 009 — frozen inputs",
        "",
        f"Frozen at **{body['frozen_at_utc']}** on `{env['host']}`.",
        "",
        "There is no git repository in this project (it was scaffolded from a handoff",
        "bundle), so \"these are the inputs\" is written down as a hash list instead of a",
        "commit id. `scripts/verify_independence.sh` re-derives this list inside a",
        "temporary copy of the tree; if the two disagree, the copy was not the same project.",
        "",
        "## Environment",
        "",
        "| | |",
        "|---|---|",
        f"| host | `{env['host']}` |",
        f"| kernel | `{env['kernel']}` |",
        f"| python | `{env['python']}` |",
        f"| ROS | `{env['ros_distro']}` |",
        f"| ROS_DOMAIN_ID | `{env['ros_domain_id']}` |",
        f"| RMW | `{env['rmw']}` |",
        f"| gz sim | `{env['gz_version']}` |",
        "",
        "## Static guards run by `build.sh`",
        "",
    ]
    lines += [f"- `scripts/{g}`" for g in body["guards"]]
    lines += [
        "",
        "## Scenario budgets declared at freeze time",
        "",
        "These are the numbers a scenario states up front. `fleet.launch.py` applies them",
        "and prints them, so a run's log says which of them were in force.",
        "",
    ]
    for name, spec in sorted(body["scenario_budgets"].items()):
        lines.append(f"### `{name}`")
        lines.append("")
        for section, values in sorted(spec.get("budgets", {}).items()):
            rendered = ", ".join(f"`{k}={v}`" for k, v in sorted(values.items()))
            lines.append(f"- **{section}**: {rendered}")
        for rid, values in sorted(spec.get("robots", {}).items()):
            rendered = ", ".join(f"`{k}={v}`" for k, v in sorted(values.items()))
            lines.append(f"- **{rid}** (per robot): {rendered}")
        lines.append("")
    lines += [
        f"## Files ({body['file_count']})",
        "",
        "`sha256` of every input that defines behaviour. Outputs (`reports/`, `runtime/`,",
        "`build/`, `install/`, `log/`) are deliberately excluded: a manifest that changes",
        "whenever a run happens measures nothing.",
        "",
        "| file | sha256 |",
        "|---|---|",
    ]
    for rel, digest in sorted(body["files"].items()):
        lines.append(f"| `{rel}` | `{digest[:16]}…` |")
    lines.append("")
    FROZEN_DOC.write_text("\n".join(lines), encoding="utf-8")


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--check", action="store_true",
                    help="compare the tree against the recorded manifest")
    args = ap.parse_args(argv)

    body = collect()
    if args.check:
        if not MANIFEST.is_file():
            print(f"freeze_manifest: no manifest at {MANIFEST}", file=sys.stderr)
            return 3
        want = json.loads(MANIFEST.read_text(encoding="utf-8"))
        diffs: list[str] = []
        for rel, digest in sorted(want["files"].items()):
            got = body["files"].get(rel)
            if got is None:
                diffs.append(f"MISSING  {rel}")
            elif got != digest:
                diffs.append(f"CHANGED  {rel}")
        for rel in sorted(set(body["files"]) - set(want["files"])):
            diffs.append(f"EXTRA    {rel}")
        print(f"freeze_manifest --check: {len(want['files'])} recorded, "
              f"{len(body['files'])} present, {len(diffs)} difference(s)")
        for d in diffs[:40]:
            print(f"  {d}")
        return 1 if diffs else 0

    MANIFEST.parent.mkdir(parents=True, exist_ok=True)
    MANIFEST.write_text(json.dumps(body, indent=2, sort_keys=True), encoding="utf-8")
    write_doc(body)
    print(f"wrote {MANIFEST.relative_to(ROOT)} ({body['file_count']} files hashed)")
    print(f"wrote {FROZEN_DOC.relative_to(ROOT)}")
    print(f"guards: {len(body['guards'])}  scenarios: {len(body['scenario_budgets'])}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
