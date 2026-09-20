"""Check (and optionally repair) XML comment defects across the 009 tree.

Defect: `--` inside an XML comment. XML forbids it. libsdformat tolerates it, so
gz sim loads the world happily -- but a strict parser (Python's ElementTree, most
linters, many CI tools) rejects the file outright. Relying on the lenient parser means
the asset is only valid for one consumer, and the failure surfaces much later in
whatever tool happens not to be libsdformat.

This scans every XML-ish asset (worlds, models, urdf, package.xml, model.config) rather
than a hand maintained list, because the hand maintained list is how the original
offender was missed: it was in a world file comment, and the first version of this
script only looked at two files.

Repair note, which is the subtle part: the fix must collapse the whole hyphen RUN.

    str.replace("--", "-")   on "----"  ->  "--"   (still illegal, fix silently fails)
    re.sub(r"-{2,}", "-", _) on "----"  ->  "-"    (correct)

An even length run therefore survives a naive replace. The script re-checks after
repairing so a fix that did not take cannot be reported as success.

Usage:
    scripts/fix_sdf_comments.py            # check only; exit 1 if any defect
    scripts/fix_sdf_comments.py --fix      # repair in place, then re-check
"""

from __future__ import annotations

import argparse
import re
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]

SUFFIXES = {".sdf", ".urdf", ".xml", ".config", ".xacro", ".in"}

COMMENT_RE = re.compile(r"<!--(.*?)-->", re.DOTALL)
HYPHEN_RUN = re.compile(r"-{2,}")


def xml_files() -> list[Path]:
    out: list[Path] = []
    for base in ("assets", "src", "config"):
        d = ROOT / base
        if not d.is_dir():
            continue
        for p in sorted(d.rglob("*")):
            if p.is_file() and p.suffix in SUFFIXES:
                # model.sdf.in and similar templates count; .gitkeep does not.
                out.append(p)
    return out


def defects(path: Path) -> list[str]:
    text = path.read_text(encoding="utf-8")
    found: list[str] = []
    for m in COMMENT_RE.finditer(text):
        body = m.group(1)
        if "--" in body:
            line = text[: m.start()].count("\n") + 1
            snippet = " ".join(body.split())[:64]
            found.append(f"{path.relative_to(ROOT)}:{line}: '--' in comment near {snippet!r}")
    return found


def repair(path: Path) -> int:
    text = path.read_text(encoding="utf-8")
    out: list[str] = []
    last = 0
    changed = 0
    for m in COMMENT_RE.finditer(text):
        out.append(text[last:m.start()])
        body = m.group(1)
        if "--" in body:
            body, n = HYPHEN_RUN.subn("-", body)
            changed += n
        out.append("<!--" + body + "-->")
        last = m.end()
    out.append(text[last:])
    if changed:
        path.write_text("".join(out), encoding="utf-8")
    return changed


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--fix", action="store_true", help="repair in place, then re-check")
    args = ap.parse_args(argv)

    files = xml_files()
    if not files:
        print("fix_sdf_comments: no XML assets found; nothing checked", file=sys.stderr)
        return 2

    if args.fix:
        total = 0
        for p in files:
            n = repair(p)
            if n:
                print(f"  repaired {n} hyphen run(s) in {p.relative_to(ROOT)}")
                total += n
        print(f"  total repairs: {total}")
        remaining = [d for p in files for d in defects(p)]
        if remaining:
            print("REPAIR INCOMPLETE (a fix that did not take is not a fix):",
                  file=sys.stderr)
            for line in remaining:
                print("    " + line, file=sys.stderr)
            return 1
        print("  re-check clean")
    else:
        bad = [d for p in files for d in defects(p)]
        print(f"fix_sdf_comments: checked {len(files)} XML file(s)")
        if bad:
            print(f"  {len(bad)} defect(s):", file=sys.stderr)
            for line in bad:
                print("    " + line, file=sys.stderr)
            print("  run with --fix to repair", file=sys.stderr)
            return 1
        print("  no '--' inside any XML comment")

    # Strict parse of the SDF/URDF assets: the comment rule is a means to this end.
    broken = []
    for p in files:
        if p.suffix not in {".sdf", ".urdf", ".in"}:
            continue
        try:
            ET.fromstring(p.read_text(encoding="utf-8"))
        except ET.ParseError as exc:
            broken.append(f"{p.relative_to(ROOT)}: {exc}")
    if broken:
        print("STRICT PARSE FAILED:", file=sys.stderr)
        for line in broken:
            print("    " + line, file=sys.stderr)
        return 4
    print("  strict parse: all SDF/URDF/XML assets are well formed")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
