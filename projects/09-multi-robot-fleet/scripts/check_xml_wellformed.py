#!/usr/bin/env python3
"""Parse every XML file in the tree and fail on any that is not well formed.

Why this is worth a build step of its own. Two failures in this project had the same
shape: a file that LOOKS like a comment problem and behaves like a configuration problem.

  * An SDF whose comment contained a double hyphen. libsdformat is lenient, so Gazebo
    loaded a model that a strict parser rejects, and an experiment was run against a stale
    model, producing a confident and wrong conclusion about wheel friction.
  * config/cyclonedds.xml with a double hyphen in a comment. CycloneDDS could not parse
    it, fell back to a broken configuration, and the measured participant ceiling went
    from 32 to ZERO. The symptom was every ROS process failing to create a node.

Neither was caught, because nothing in the build parsed these files. The SDFs were
partially covered by the asset validator, which only looks at the two files it knows
about, and config/*.xml was covered by nothing at all.

So: walk the tree, parse everything XML, and report file, line and column. An XML comment
may not contain a double hyphen, so a well-formedness check covers that mistake too, which
is the whole point. Any new XML asset is covered automatically.
"""

from __future__ import annotations

import sys
import xml.etree.ElementTree as ET
from pathlib import Path

PATTERNS = ("*.xml", "*.sdf", "*.sdf.in", "*.urdf", "*.xacro", "*.config")
SKIP_DIRS = {".git", "__pycache__", "build", "install", "log", ".venv", "node_modules"}


def candidates(roots: list[Path]) -> list[Path]:
    found: list[Path] = []
    for root in roots:
        if not root.exists():
            continue
        for pat in PATTERNS:
            for p in root.rglob(pat):
                if any(part in SKIP_DIRS for part in p.parts):
                    continue
                # .sdf.in templates carry __ROBOT__ placeholders, which are plain text and
                # do not affect well-formedness, so they are checked as-is.
                found.append(p)
    return sorted(set(found))


def main(argv: list[str]) -> int:
    roots = [Path(a) for a in (argv or ["assets", "config", "src"])]
    files = candidates(roots)
    bad = 0
    print(f"  XML files found: {len(files)}")
    for f in files:
        try:
            ET.parse(f)
        except ET.ParseError as exc:
            print(f"  [FAIL] {f}: not well formed XML: {exc}")
            bad += 1
    if bad:
        print(f"  RESULT: {bad} file(s) are not well formed XML")
        print("  Most common cause here: a double hyphen inside a comment."
              " XML comments may not contain one.")
        return 1
    print("  RESULT: every XML file parses (exit 0)")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
