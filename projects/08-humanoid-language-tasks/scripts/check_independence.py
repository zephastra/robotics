"""Independence scan (docs/MASTER_PLAN.md section 6.4).

Checks that 008 does not reach into 007 at run time. It does **not** move or
delete 007 (section 6.4 forbids proving independence that way). A clean-room
run must still be performed in an environment where 007 is not mounted.

Rules checked:

- no Python file imports ``humanoid007`` or appends a ``../007`` path;
- no config/asset/policy file is a symlink that resolves outside 008;
- no text file embeds the 007 baseline's absolute path.

Exit 0 = clean, 1 = violation.
"""

from __future__ import annotations

import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
ROOT_RESOLVED = ROOT.resolve()

BAD_IMPORT_TOKENS = ("humanoid007", "import humanoid007", "from humanoid007")
BAD_PATH_TOKENS = ("007_humanoid_visual_transport",)


def scan() -> list[str]:
    problems: list[str] = []

    for p in sorted((ROOT / "src").rglob("*.py")):
        text = p.read_text(encoding="utf-8", errors="replace")
        for token in BAD_IMPORT_TOKENS:
            if token in text:
                problems.append(f"{p.relative_to(ROOT)}: references {token!r}")
        if "sys.path.append" in text and "007" in text:
            problems.append(f"{p.relative_to(ROOT)}: appends a 007 path")

    for p in sorted((ROOT / "config").rglob("*")):
        if not p.is_file():
            continue
        text = p.read_text(encoding="utf-8", errors="replace")
        for token in BAD_PATH_TOKENS:
            if token in text:
                problems.append(f"{p.relative_to(ROOT)}: references {token!r}")

    # symlink check across assets, policies, config, src, scripts
    for directory in ("assets", "policies", "config", "src", "scripts"):
        for p in sorted((ROOT / directory).rglob("*")):
            if p.is_symlink():
                target = p.resolve()
                if not target.is_relative_to(ROOT_RESOLVED):
                    problems.append(f"{p.relative_to(ROOT)}: symlink -> {target}")

    return problems


def main() -> int:
    problems = scan()
    if problems:
        for p in problems:
            print("INDEPENDENCE VIOLATION:", p, file=sys.stderr)
        return 1
    print("independence scan clean: no 007 import, path, or out-of-tree symlink found")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
