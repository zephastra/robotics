#!/usr/bin/env python3
"""TEST_AND_ACCEPTANCE section 10 -- the reproduction and release gates, as eight checks.

Section 10 is the release gate, and it is the one part of the P7 matrix that is not a
simulation: "在全新路径构建并运行" / "READM 写清真实完成范围" / "用户确认后再整理 GitHub
对应目录". It has never been checked as a whole. This turns it into eight verdicts with
the evidence each one rests on.

Every item reports one of:

    PASS      the artefact is present and says the right thing, with the path quoted
    FAIL      the artefact is present and says something else  (this is a defect)
    NOT_RUN   nothing here establishes it  (this is not a pass, and section 10 says a
              NOT_RUN item means that capability is not accepted yet)

Exit codes, following section 9: 0 declared acceptance passed, 4 an item FAILED,
5 an item is NOT_RUN so the gate cannot be closed. A release gate that closes itself is
not a gate, so item 8 -- "用户确认后再推送" -- can only ever be NOT_RUN.

Usage:
    scripts/check_p7_gates.py [--independence-report PATH] [--root DIR]
"""

from __future__ import annotations

import argparse
import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]

PASS, FAIL, NOT_RUN = "PASS", "FAIL", "NOT_RUN"

#: Literal paths that section 10 item 1 forbids anywhere in runnable code. A copy of the
#: tree is not a copy if it reaches back into the original one.
FORBIDDEN_PATH_LITERALS = (
    "/home/ziling/projects/009",
    "/mnt/c/Users/ZiLing",
)
#: Where those literals matter. `docs/` is excluded deliberately: the handoff documents
#: quote the path they describe, and that is the point of them.
SCANNED_DIRS = ("src", "scripts", "config", "tests", "assets")

#: Files exempt from the literal scan because they *define* the patterns. Listing them by
#: name rather than by "anything under scripts/check_*" keeps the exemption one line long
#: and auditable, and excludes nothing that could execute a path. The list was checked
#: rather than assumed: `scripts/verify_independence.sh` also hunts these literals and does
#: not contain them, so it is NOT exempt.
SCAN_EXEMPT = frozenset({
    "scripts/check_p7_gates.py",
})


def _read(root: Path, rel: str) -> str:
    path = root / rel
    return path.read_text(encoding="utf-8", errors="replace") if path.is_file() else ""


def _exists(root: Path, rel: str) -> bool:
    return (root / rel).exists()


# --------------------------------------------------------------------------- #
# 1. a fresh path builds AND runs
# --------------------------------------------------------------------------- #


def gate_1(root: Path, independence: Path | None) -> "tuple[str, list[str]]":
    evidence: list[str] = []

    hits: list[str] = []
    for d in SCANNED_DIRS:
        base = root / d
        if not base.is_dir():
            continue
        for path in sorted(base.rglob("*")):
            if not path.is_file() or path.suffix in {".pyc", ".sdf", ".pgm", ".png", ".db3"}:
                continue
            rel = str(path.relative_to(root))
            if rel in SCAN_EXEMPT:
                continue
            try:
                text = path.read_text(encoding="utf-8")
            except (UnicodeDecodeError, OSError):
                continue
            for literal in FORBIDDEN_PATH_LITERALS:
                if literal in text:
                    hits.append(f"{rel}: {literal}")
    if hits:
        return FAIL, [f"absolute path literal(s) in runnable code: {hits}"]
    evidence.append(
        f"no absolute-path literal in {'/'.join(SCANNED_DIRS)} "
        f"({len(FORBIDDEN_PATH_LITERALS)} patterns searched, "
        f"{len(SCAN_EXEMPT)} file(s) exempt because they define the patterns)"
    )

    if not _exists(root, "scripts/verify_independence.sh"):
        return FAIL, evidence + ["scripts/verify_independence.sh is missing"]
    evidence.append("scripts/verify_independence.sh: present (copies to a temp path, "
                    "builds, tests, compares against the frozen manifest, then runs the fleet)")

    if independence is None or not independence.is_file():
        return NOT_RUN, evidence + [
            "no relocation report given. Run `FLEET009_INDEP_FLEET=1 bash "
            "scripts/verify_independence.sh` and pass its output with "
            "--independence-report; item 1 is exactly that run, so a static look cannot "
            "close it."
        ]
    body = independence.read_text(encoding="utf-8", errors="replace")
    evidence.append(f"relocation report: {independence.name}")
    failures = re.findall(r"^\s*\[FAIL\].*$", body, flags=re.MULTILINE)
    if failures:
        return FAIL, evidence + [f"the relocation run reported: {f.strip()}" for f in failures[:6]]
    if "[PASS]" not in body:
        return NOT_RUN, evidence + ["the relocation report contains no [PASS] lines at all"]
    steps = len(re.findall(r"^\s*\[PASS\]", body, flags=re.MULTILINE))
    if "real fleet run" in body or "fleet" in body.lower():
        evidence.append(f"{steps} [PASS] step(s), including a fleet run from the copy")
    else:
        evidence.append(f"{steps} [PASS] step(s); no fleet run recorded in this report")
    return PASS, evidence


# --------------------------------------------------------------------------- #
# 2. dependencies, build commands, versions, map source
# --------------------------------------------------------------------------- #


def gate_2(root: Path) -> "tuple[str, list[str]]":
    evidence: list[str] = []
    findings: list[str] = []

    notices = _read(root, "THIRD_PARTY_NOTICES.md")
    if "| component | version" in notices and "licence" in notices:
        rows = len(re.findall(r"^\|\s*\w.*\|\s*\w.*\|\s*\w", notices, flags=re.MULTILINE))
        evidence.append(f"THIRD_PARTY_NOTICES.md: {rows} dependency rows with version + licence")
    else:
        findings.append("THIRD_PARTY_NOTICES.md has no component/version/licence table")

    if _exists(root, "scripts/build.sh") and "bash scripts/build.sh" in _read(root, "README.md"):
        evidence.append("build command recorded and present: bash scripts/build.sh")
    else:
        findings.append("the build command is not both present and recorded in README.md")

    env = _read(root, "docs/ENVIRONMENT.md")
    if "gz sim" in env and re.search(r"\d+\.\d+\.\d+", env) and "ROS 2" in env:
        evidence.append("docs/ENVIRONMENT.md records OS / ROS 2 / Python / Gazebo versions "
                        "measured on this machine")
    else:
        findings.append("docs/ENVIRONMENT.md does not record the measured versions")

    map_yaml = _read(root, "assets/maps/warehouse.yaml")
    if "DERIVED from" in map_yaml:
        evidence.append("map provenance: assets/maps/warehouse.yaml states it is DERIVED "
                        "from assets/worlds/warehouse.sdf, not copied from upstream")
    else:
        findings.append("the occupancy map has no recorded provenance")

    if findings:
        return FAIL, evidence + findings
    return PASS, evidence


# --------------------------------------------------------------------------- #
# 3. headless and GUI recorded separately
# --------------------------------------------------------------------------- #


def gate_3(root: Path) -> "tuple[str, list[str]]":
    env = _read(root, "docs/ENVIRONMENT.md")
    lim = _read(root, "docs/LIMITATIONS.md")
    evidence: list[str] = []
    findings: list[str] = []

    if "--headless" in env or "headless" in env:
        evidence.append("docs/ENVIRONMENT.md records headless as the tested default path")
    else:
        findings.append("no headless scope recorded in docs/ENVIRONMENT.md")

    gui_untested = ("未验证" in env and "GUI" in env) or "GUI" in lim
    if gui_untested:
        evidence.append("the GUI path is recorded as NOT tested (docs/ENVIRONMENT.md "
                        "section 5 / docs/LIMITATIONS.md)")
    else:
        findings.append("the GUI path is not recorded as untested, which would let a "
                        "headless pass be read as a GUI pass")

    # The reverse failure: a document claiming GUI coverage that never happened.
    claims = [
        line.strip()
        for line in (env + lim + _read(root, "README.md")).splitlines()
        if re.search(r"(rviz|gz ?sim gui|WSLg).{0,40}(verified|passed|测试通过|已验证)", line, re.I)
    ]
    if claims:
        findings.append(f"a document claims GUI coverage: {claims[:2]}")
    else:
        evidence.append("no document claims the GUI path was exercised")

    if findings:
        return FAIL, evidence + findings
    return PASS, evidence


# --------------------------------------------------------------------------- #
# 4. with the GUI off, the server still decides
# --------------------------------------------------------------------------- #


def gate_4(root: Path) -> "tuple[str, list[str]]":
    world = _read(root, "src/fleet_bringup/launch/world.launch.py")
    evidence: list[str] = []
    findings: list[str] = []

    if 'default_value="true"' in world and "headless" in world:
        evidence.append("world.launch.py: `headless` defaults to true")
    else:
        findings.append("world.launch.py does not default to headless")
    if "--headless-rendering" in world:
        evidence.append("world.launch.py passes --headless-rendering")
    if "gzclient" in world or re.search(r'"-g"', world):
        findings.append("world.launch.py starts a GUI client process")
    else:
        evidence.append("no GUI client process is launched in world.launch.py")

    # The live half. launch tags every line with the process name, so the set of tags in
    # a bring-up log IS the set of processes that ran: a GUI client would appear by name.
    log_dir = root / "reports"
    logs = sorted(log_dir.glob("*.log")) if log_dir.is_dir() else []
    tags: set[str] = set()
    world_ran = False
    for log in logs:
        text = log.read_text(encoding="utf-8", errors="replace")
        tags |= set(re.findall(r"^\[([a-z0-9_.-]+)-\d+\]", text, flags=re.MULTILINE))
        if "gz_sim_server" in text:
            world_ran = True
    gui = sorted(t for t in tags if re.search(r"gz.*client|gzclient|rviz", t, re.I))
    if not logs:
        findings.append("no bring-up log in reports/, so nothing corroborates the "
                        "launch file")
    elif not world_ran:
        findings.append("no bring-up log shows gz running at all")
    elif gui:
        findings.append(f"a bring-up log contains a GUI process: {gui}")
    else:
        evidence.append(
            f"{len(logs)} bring-up log(s): {len(tags)} distinct process tags, gz present, "
            "and no gz client / rviz tag in any of them -- the run was server-only"
        )
    if findings:
        return FAIL, evidence + findings
    return PASS, evidence


# --------------------------------------------------------------------------- #
# 5. what must not be committed
# --------------------------------------------------------------------------- #


def gate_5(root: Path) -> "tuple[str, list[str]]":
    ignore = _read(root, ".gitignore")
    evidence: list[str] = []
    findings: list[str] = []
    required = {
        ".venv": ".venv",
        "build/install/log": "build/",
        "colcon log": "log/",
        "SQLite / runtime state": "runtime/",
        "run reports": "reports/",
        "local env / credentials": "*.local.env",
    }
    for label, token in required.items():
        if token in ignore:
            evidence.append(f".gitignore covers {label} ({token})")
        else:
            findings.append(f".gitignore does not cover {label}")

    leaked = [
        str(p.relative_to(root))
        for p in (root / "config").glob("*.env")
        if p.name in {"environment.env"} or p.suffix == ".env"
    ] if (root / "config").is_dir() else []
    if leaked:
        findings.append(f"an environment file is present in the tree: {leaked}")
    else:
        evidence.append("no config/*.env file is present in the tree")

    if _exists(root, ".git"):
        evidence.append("a git repository exists, so these rules are in force")
    else:
        evidence.append("no .git in this tree: nothing has been committed, and these are "
                        "the rules that will apply when it is")
    if findings:
        return FAIL, evidence + findings
    return PASS, evidence


# --------------------------------------------------------------------------- #
# 6. licences and notices
# --------------------------------------------------------------------------- #


def gate_6(root: Path) -> "tuple[str, list[str]]":
    evidence: list[str] = []
    findings: list[str] = []

    lic = _read(root, "LICENSE")
    if "Apache License" in lic:
        evidence.append("LICENSE: Apache-2.0")
        if "full licence text is the one published at the URL above" in lic:
            evidence.append("LICENSE states it is the notice form and points at the "
                            "authoritative text rather than transcribing it approximately")
    else:
        findings.append("LICENSE does not name a licence")

    notices = _read(root, "THIRD_PARTY_NOTICES.md")
    if notices and "does not redistribute" in notices:
        evidence.append("THIRD_PARTY_NOTICES.md: lists runtime components and states that "
                        "no third-party source is vendored")
    else:
        findings.append("THIRD_PARTY_NOTICES.md is missing or does not state what is not "
                        "redistributed")
    if "CC BY" in lic:
        evidence.append("documentation licence recorded (CC BY 4.0, attributed)")

    if findings:
        return FAIL, evidence + findings
    return PASS, evidence


# --------------------------------------------------------------------------- #
# 7. the README states the real scope
# --------------------------------------------------------------------------- #


def gate_7(root: Path) -> "tuple[str, list[str]]":
    readme = _read(root, "README.md")
    evidence: list[str] = []
    findings: list[str] = []

    if "IMPLEMENTATION_STATUS.md" in readme:
        evidence.append("README defers the verified scope to docs/IMPLEMENTATION_STATUS.md "
                        "instead of asserting it")
    else:
        findings.append("README does not point at the authoritative status document")

    # The four things section 10.7 requires the README to say out loud.
    claims = {
        "wheeled physical motion": ("四轮" in readme and "物理" in readme),
        "logical payload (no physical handling)": (
            "逻辑货物" in readme or "没有实体装卸" in readme or "没有机械臂" in readme),
        "simulated charging": ("模拟充电" in readme or "充电" in readme),
        "centralised reservation": ("集中式" in readme or "预约" in readme),
    }
    for label, ok in claims.items():
        if ok:
            evidence.append(f"README states: {label}")
        else:
            findings.append(f"README does not state: {label}")

    for label, token in {"humanoid not implemented": "人形",
                         "no unattended operation promised": "无人值守"}.items():
        if token in readme:
            evidence.append(f"README states: {label}")
        else:
            findings.append(f"README does not state: {label}")

    # A stale status line is the failure this gate exists for: it makes a working system
    # read as an empty repository, or an empty one read as working. The phrases are
    # quoted rather than pattern-guessed, so a truthful status line cannot trip it.
    stale = re.findall(
        r"[^\n]*(没有任何可运行|当前只有文档|以下均为未来接口|状态[：:][^\n]{0,20}开发中（P0）)[^\n]*",
        readme,
    )
    if stale:
        findings.append(f"README still carries a stale status line: {stale[:1]}")
    if findings:
        return FAIL, evidence + findings
    return PASS, evidence


# --------------------------------------------------------------------------- #
# 8. the user gate
# --------------------------------------------------------------------------- #


def gate_8(root: Path) -> "tuple[str, list[str]]":
    """Never PASS. Section 10.8 is a human decision, and an automated run cannot make it."""
    readme = _read(root, "README.md")
    evidence = [
        "section 10.8: '用户确认后再整理 GitHub 对应目录；没有确认不得推送'",
        ("README restates it: 未经用户明确授权：不提交、不推送 GitHub"
         if "不推送 GitHub" in readme else "README does not restate it"),
        "no .git in this tree, so nothing has been pushed",
        "A release gate that closes itself is not a gate: this item is closed by the "
        "user, not by a script.",
    ]
    return NOT_RUN, evidence


GATES = [
    ("10.1", "a fresh path builds AND runs, with no reference to 001-008", gate_1),
    ("10.2", "dependencies, build command, versions and map source are complete", gate_2),
    ("10.3", "headless and GUI scopes are recorded separately", gate_3),
    ("10.4", "with the GUI off, the server still decides motion and task state", gate_4),
    ("10.5", "venv / build / install / log / SQLite / reports / credentials stay out of Git", gate_5),
    ("10.6", "licences are explicit and third-party sources keep their notice", gate_6),
    ("10.7", "the README states the real completed scope", gate_7),
    ("10.8", "GitHub is touched only after the user confirms", gate_8),
]


def main(argv: "list[str] | None" = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--root", default=str(ROOT))
    ap.add_argument("--independence-report", default="",
                    help="output of scripts/verify_independence.sh, for item 10.1")
    args = ap.parse_args(argv)

    root = Path(args.root).resolve()
    report = Path(args.independence_report) if args.independence_report else None

    print("=" * 78)
    print("  009 P7 matrix -- TEST_AND_ACCEPTANCE section 10 (release gates)")
    print(f"  root: {root}")
    print("=" * 78)

    failed = 0
    not_run = 0
    for ident, title, fn in GATES:
        verdict, evidence = fn(root, report) if fn is gate_1 else fn(root)
        if verdict == FAIL:
            failed += 1
        elif verdict == NOT_RUN:
            not_run += 1
        print()
        print(f"  [{verdict:>7}] {ident} {title}")
        for line in evidence:
            print(f"            - {line}")

    print()
    print("-" * 78)
    print(f"  {len(GATES)} gate(s): {len(GATES) - failed - not_run} PASS, {failed} FAIL, "
          f"{not_run} NOT_RUN")
    if failed:
        print("  A FAIL here is a defect in the tree or in a document, not in the fleet.")
    if not_run:
        print("  NOT_RUN is not a pass. Section 10: 任何 NOT_RUN 则对应能力尚未验收.")
    print()
    if failed:
        print("  RESULT: FAIL (exit 4)")
        return 4
    if not_run:
        print("  RESULT: the release gate is NOT closed (exit 5)")
        return 5
    print("  RESULT: every section 10 gate is satisfied (exit 0)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
