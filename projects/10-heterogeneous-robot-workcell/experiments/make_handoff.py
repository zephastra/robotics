"""Generate `docs/HANDOFF.md` from the task board and the runs' own acceptance artefacts.

WHY A GENERATOR AND NOT A PARAGRAPH
-----------------------------------
`docs/HANDOFF.md` has rotted twice. On 2026-09-25 it still said `ACTIVE_TASK: P3-WORLD-01` and
cited `P1-GATE-07` as the gate, two days after `D052` superseded that gate with `p1-gate-08` and
after all four P3 tasks closed. Nothing failed, because **a hand-written "current state" has no
criterion, no test, and no one who is hurt when it is false** (`D043`).

So the state is derived, and the file says so at the top:
  * the task rows come from `docs/TASK_BOARD.md`;
  * each evidence row says WHICH file carries the verdict and what state that file is in -- a
    directory that is absent, a declared file that is absent, an unparseable file and a diagnostic
    file with no verdict are four different facts, and rendering them all as MISSING is what put
    "MISSING" beside `DONE` on the same page;
  * the freeze lines come from `config/p3_freeze.json`;
  * `--check` regenerates in memory and compares, so drift is a red light rather than a surprise.

AND WHAT IS NOT DERIVED: `ACTIVE_TASK`, which is read from the hand-written `docs/CLAIMS.md`. A
claim is a statement about who has started work, and it cannot be computed from a board -- while a
derived "first eligible task" is a recommendation, and rendering it as ACTIVE_TASK asserts something
nobody recorded. The claim lives in a hand-written file precisely because this one is regenerated.

WHAT IS DELIBERATELY *NOT* DERIVED is the "cannot say" list at the bottom. That is a judgement
about what the evidence does not support, not a measurement, so it is declared with a date and a
pointer to `DECISIONS.md` where each boundary is argued. A generated file should not pretend its
opinions were computed.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import pathlib
import re
import sys

ROOT = pathlib.Path(__file__).resolve().parents[1]
OUT = ROOT / 'docs' / 'HANDOFF.md'
BOARD = ROOT / 'docs' / 'TASK_BOARD.md'
FREEZE = ROOT / 'config' / 'p3_freeze.json'

#: The runs whose verdict files are the authority for a stage, and WHICH FILE CARRIES THE VERDICT.
#: Listed rather than discovered so that a run added for an experiment does not silently become
#: "the" evidence for a task; and the file name is declared rather than searched for, because
#: "whatever JSON is in this directory" is what produced six false MISSING rows. The fourth element
#: is the file this repository actually writes for that kind of run:
#:   * `gate.json`      -- a frozen-world or shared-world gate
#:   * `p3_nav_gate.json` -- the P3 navigation judge
#:   * `acceptance.json` -- a judged run
#:   * `report.json`    -- a probe's own report, which carries `verdict` for some probes and only
#:                        `status` for others, so the reader says which field it read.
EVIDENCE = (
    ('P1', 'the shared world gate', 'reports/p1-gate-08', 'gate.json'),
    ('P1', 'H, the humanoid tray sequence', 'reports/p1-h-seq-08', 'report.json'),
    ('P1', 'A, the fixed arm pick and place', 'reports/p1-a-p05', 'acceptance.json'),
    ('P1', 'C, the vehicle transfer', 'reports/p1-c-crown-02', 'acceptance.json'),
    ('P1', 'N, single-vehicle navigation (historical)', 'reports/p1-n-nav-07', 'acceptance.json'),
    ('P1', 'N, arrival re-judged (index over 8 cases)', 'reports/p1-n-arrival-01', None),
    ('P2', 'C in the assembled world', 'reports/p2-c-in-cell-02', 'report.json'),
    ('P2', 'C yaw attribution', 'reports/p2-c-yaw-attrib-01', 'report.json'),
    ('P3', 'the formal cell', 'reports/p3-world-01', 'gate.json'),
    ('P3', 'single-vehicle navigation, bare (historical)', 'reports/p3-nav-05',
     'p3_nav_gate.json'),
    ('P3', 'single-vehicle navigation, loaded (historical)', 'reports/p3-nav-06',
     'p3_nav_gate.json'),
    ('P3', 'navigation re-judged: variant derived from content',
     'reports/p3-nav-requal-01', None),
    ('P3', 'vision and coordinate validation', 'reports/p3-vision-02', 'report.json'),
)

#: Where a claim lives. HAND-WRITTEN on purpose: the generated file cannot hold one, because the
#: next generation would erase it.
CLAIMS = ROOT / 'docs' / 'CLAIMS.md'
VERDICT_FIELDS = ('verdict', 'status')

NOT_DERIVED = """\
This section is a judgement, not a measurement: it is declared rather than derived, and each line
is argued somewhere in `docs/DECISIONS.md`. Read it together with the numbers above, never instead
of them.

- **Not "the four tasks ran in one world."** The shared world gate loads one `MjModel` holding four
  roles; they hold separate stations and **never interact**.
- **Not "C's transfer is commissioned in the assembled world."** In the navigation arena C's fixture
  is an **obstacle, not a docking target**: no interlock, no ALIGN window, no receiving roller row.
- **Not "the load is real."** The loaded variant uses a **declared stand-in mount** (derived z
  0.240 m from the chassis top and the lidar height). The AMR is still a stand-in (`P1-ENV-01`).
- **Not "it drove in."** The start pose is a **parked pose**, not a drive to the station.
- **Not "two vehicles ran together."** The navigation arena is **single-vehicle**; the other roles
  exist in it only as footprints.
- **Not "arrival yaw has an independent truth."** `sim_report.truth` records position only, so the
  gate records that row **NOT_RUN**.
- **Not "P3's vision is P4's vision."** That camera and second part are added by the probe through
  `MjSpec`; the calibration used the simulator's geom labels (an oracle a real camera does not
  have), and there is no sensor noise, no ROS, no point cloud and no PCL claim.
- **Not "A is production grasping" / "A's envelope extrapolates" / "C's numbers transfer" to any
  other pitch, roller diameter or tray.**
- ⚠️ **`reports/p1-n-nav-07` is history, not current.** It was produced by `probe_n_sim.py`
  `c4d3aa19…` and `probe_n_ros.py` `4efd133e…`, which are no longer the files on disk. Re-run
  before citing it as evidence about current code.
- ⚠️ **`probe_n_ros`'s `STATE_STREAM_STOPPED_EARLY` is not a health indicator** -- the successful
  run reports it too. Read the bridge's `counters`, not its `status`.
"""


def parse_board():
    """Task rows from the board, plus any id that appears twice with different statuses."""
    rows, seen = [], {}
    for line in BOARD.read_text(encoding='utf-8').splitlines():
        if not line.startswith('|'):
            continue
        cells = [c.strip() for c in line.strip().strip('|').split('|')]
        if len(cells) < 3 or cells[0] in ('ID', '---') or set(cells[0]) <= {'-'}:
            continue
        if not re.match(r'^P\d', cells[0]):
            continue
        status = cells[1].replace('*', '').strip()
        rows.append({'id': cells[0], 'status': status, 'dep': cells[2]})
        seen.setdefault(cells[0], []).append(status)
    conflicts = {k: v for k, v in seen.items() if len(set(v)) > 1}
    return rows, conflicts


def stage_done(rows, stage):
    """Are ALL tasks of a stage DONE? Dependencies are written as `P3`, not as `P3-NAV-02`."""
    mine = [r for r in rows if r['id'].startswith(f'{stage}-')]
    return bool(mine) and all(r['status'] == 'DONE' for r in mine)


def dependency_satisfied(rows, dep):
    """A dependency cell is free text. Three shapes are understood, and anything else is reported
    as unparsed rather than silently treated as satisfied.

    The third shape matters: the board writes most dependencies WITHOUT the stage prefix -- `N-06`,
    `GATE-07`, `CORE-01` -- while ids carry it (`P1-N-06`). Without the suffix rule those rows read
    as unparsable, and the derived `ready` column says `no` for a task whose dependency is in fact
    DONE, which is the same rot in the other direction.
    """
    tokens = re.findall(r'\bP[0-7]\b|\bP[0-7]-[A-Z]+-\d+\b|\b[A-Z]+-\d+\b', dep)
    if not tokens:
        if '无' in dep:
            return True, []
        return None, dep

    def done(token):
        if re.fullmatch(r'P[0-7]', token):
            return stage_done(rows, token)
        if re.fullmatch(r'P[0-7]-[A-Z]+-\d+', token):
            return any(r['id'] == token and r['status'] == 'DONE' for r in rows)
        matches = [r for r in rows if r['id'].endswith(f'-{token}')]
        if len(matches) != 1:
            return None
        return matches[0]['status'] == 'DONE'

    resolved = [(tok, done(tok)) for tok in tokens]
    if any(state is None for _tok, state in resolved):
        return None, [tok for tok, state in resolved if state is None]
    pending = [tok for tok, state in resolved if not state]
    return not pending, pending


def run_evidence(path, declared):
    """Read one evidence directory, keeping the states that used to be conflated APART.

    The states are: the directory is not there; it is there and the declared file is not; the file
    will not parse; the file parses but carries no verdict (a diagnostic report); or it carries one.
    Each is a different fact about the world, and the previous version rendered all but the last as
    MISSING.
    """
    directory = ROOT / path
    if not directory.is_dir():
        return {'state': 'DIR_MISSING', 'file': None, 'verdict': None,
                'detail': 'the directory does not exist'}
    if declared is None:
        # An INDEX: a directory that holds an index OVER cases rather than one verdict. `--` in the
        # verdict column is the honest reading, and it is a different fact from NO_VERDICT.
        index_file = directory / 'summary.json'
        if not index_file.is_file():
            return {'state': 'FILE_MISSING', 'file': 'summary.json', 'verdict': None,
                    'detail': 'declared as an index, but summary.json is not there'}
        digest = hashlib.sha256(index_file.read_bytes()).hexdigest()[:16]
        extra = ''
        try:
            doc = json.loads(index_file.read_text(encoding='utf-8'))
            if 'changed' in doc:
                extra = (f"; it records that {len(doc['changed'])} of "
                         f"{len(doc.get('cases') or {})} cases changed identity or verdict: "
                         f"{doc['changed'] or 'none'}")
        except (OSError, ValueError):
            pass
        return {'state': 'INDEX', 'file': 'summary.json', 'verdict': None, 'sha256': digest,
                'detail': 'an index over cases rather than a single run; the per-case verdicts are '
                          'in `cases/<run>/`' + extra}
    candidate = directory / declared
    siblings = sorted(item.name for item in directory.glob('*.json'))
    if not candidate.is_file():
        return {'state': 'FILE_MISSING', 'file': declared, 'verdict': None,
                'detail': f'{declared} is not in this directory, which holds '
                          f'{siblings or "no JSON files at all"}'}
    try:
        data = json.loads(candidate.read_text(encoding='utf-8'))
        digest = hashlib.sha256(candidate.read_bytes()).hexdigest()[:16]
    except (OSError, ValueError) as exc:
        return {'state': 'UNPARSEABLE', 'file': declared, 'verdict': None,
                'detail': f'{type(exc).__name__}: {exc}'}
    if not isinstance(data, dict):
        return {'state': 'UNPARSEABLE', 'file': declared, 'verdict': None,
                'detail': f'the top level is {type(data).__name__}, not an object with a verdict'}
    field = next((name for name in VERDICT_FIELDS if name in data), None)
    if field is None:
        return {'state': 'NO_VERDICT', 'file': declared, 'verdict': None, 'sha256': digest,
                'detail': f'no {" or ".join(VERDICT_FIELDS)} field: this file is diagnostic data, '
                          f'and a verdict must not be invented from it'}
    counts = data.get('counts') or {}
    failed = data.get('failed')
    return {'state': 'OK', 'file': declared, 'verdict': data[field], 'field': field,
            'checks': counts.get('checks', len(data.get('checks', [])) or '?'),
            'failed': failed if isinstance(failed, list) else [],
            'sha256': digest, 'bytes': candidate.stat().st_size,
            'detail': f'verdict read from the {field!r} field'}


def parse_claims(path=None):
    """The ACTIVE_TASK, from the hand-written record and from nowhere else.

    A derived "first eligible task" is a recommendation. Calling it ACTIVE_TASK asserts that
    somebody has started, and there was no record of that anywhere: `AGENTS.md` said the active task
    lives in HANDOFF while HANDOFF is regenerated from the board, so a claim written into it would
    be wiped by the next generation. Returns (active, all_rows, note).
    """
    path = path or CLAIMS
    if not path.is_file():
        return [], [], f'CLAIM_RECORD_MISSING: {path} does not exist, so no claim can be read'
    rows = []
    for line in path.read_text(encoding='utf-8').splitlines():
        if not line.startswith('|'):
            continue
        cells = [cell.strip() for cell in line.strip().strip('|').split('|')]
        if len(cells) < 5 or cells[0] in ('task', '---') or set(cells[0]) <= {'-'}:
            continue
        rows.append({'task': cells[0], 'by': cells[1], 'at': cells[2], 'released': cells[3],
                     'note': cells[4]})
    active = [row for row in rows if not row['released'] or row['released'] == '-']
    return active, rows, None


def render():
    rows, conflicts = parse_board()
    lines = [
        '<!-- GENERATED FILE -- do not hand-edit. See the note at the top. -->',
        '# 010 handoff',
        '',
        '> **This file is GENERATED by `experiments/make_handoff.py`.** Do not hand-edit it: the',
        '> hand-written "current state" it replaces said `ACTIVE_TASK: P3-WORLD-01` and cited',
        '> `P1-GATE-07` for two days after both were superseded (`D043`: a state sentence has no',
        '> criterion, no test, and nobody is hurt when it is false). Regenerate with',
        '> `./.venv/bin/python experiments/make_handoff.py`; verify with `--check`.',
        '',
        f"> Derived from `docs/TASK_BOARD.md` (sha256 "
        f"{hashlib.sha256(BOARD.read_bytes()).hexdigest()[:16]}), the runs' own "
        f"`acceptance.json`, and `config/p3_freeze.json`.",
        '',
    ]

    done = [r for r in rows if r['status'] == 'DONE']
    open_rows = [r for r in rows if r['status'] != 'DONE']
    ready = []
    for r in open_rows:
        ok, _pending = dependency_satisfied(rows, r['dep'])
        if ok:
            ready.append(r)
    claims, _all_claims, claim_note = parse_claims()
    lines += [
        '## ACTIVE_TASK -- read from `docs/CLAIMS.md`, not derived',
        '',
    ]
    if claim_note:
        lines += [f'**unknown** -- {claim_note}', '']
    elif not claims:
        lines += ['**none** -- no unreleased claim is recorded. A claim is a fact about who has '
                  'started, and it cannot be derived from the board.', '']
    else:
        for claim in claims:
            lines += [f"- **`{claim['task']}`** -- claimed by {claim['by']} at {claim['at']}"
                      + (f"; {claim['note']}" if claim['note'] and claim['note'] != '-' else '')]
        lines += ['']
    lines += [
        '## NEXT_ELIGIBLE_TASKS -- recommendations, and NOT claims',
        '',
        f"Derived: {len(done)} of {len(rows)} board rows are DONE; {len(open_rows)} are open, of "
        f"which {len(ready)} have their declared dependency satisfied. The list below says which "
        f"tasks COULD be claimed. It does not say anybody has.",
        '',
    ]
    for row in ready[:6]:
        lines.append(f"- `{row['id']}` (board status `{row['status']}`, dependency "
                     f"`{row['dep']}` satisfied)")
    if not ready:
        lines.append('- none: every open row still has an unsatisfied dependency')
    lines.append('')
    if conflicts:
        lines += ['### ⚠️ Board rows that disagree with themselves', '']
        for tid, statuses in sorted(conflicts.items()):
            lines.append(f'- `{tid}` appears {len(statuses)} times with statuses {statuses}. '
                         f'The LAST row was used. A duplicate id with two statuses is how a board '
                         f'quietly keeps a stale row alive.')
        lines.append('')

    lines += ['## Task board', '', '| ID | status | dependency | ready? |', '|---|---|---|---|']
    for r in rows:
        ok, _p = dependency_satisfied(rows, r['dep'])
        mark = 'DONE' if r['status'] == 'DONE' else ('yes' if ok else 'no')
        lines.append(f"| `{r['id']}` | {r['status']} | {r['dep']} | {mark} |")
    lines.append('')

    lines += ['## Evidence index: which file carries the verdict, and what state it is in', '',
              '| stage | what | run | state | verdict | checks | file | sha256 |',
              '|---|---|---|---|---|---|---|---|']
    problems = []
    for stage, what, path, declared in EVIDENCE:
        ev = run_evidence(path, declared)
        if ev['state'] != 'OK':
            problems.append((stage, path, ev))
        verdict = f"**{ev['verdict']}**" if ev.get('verdict') else '--'
        lines.append(f"| {stage} | {what} | `{path}` | {ev['state']} | {verdict} | "
                     f"{ev.get('checks', '--')} | `{ev['file'] or '--'}` | "
                     f"`{ev.get('sha256', '--')}` |")
    lines.append('')
    if problems:
        lines += ['### Evidence that is not a clean verdict', '']
        for stage, path, ev in problems:
            lines.append(f"- **{ev['state']}** `{path}` -- {ev['detail']}")
        lines.append('')

    if FREEZE.is_file():
        frozen = json.loads(FREEZE.read_text(encoding='utf-8'))
        cov = frozen.get('code_coverage', {})
        lines += ['## Freeze', '',
                  f"- `config/p3_freeze.json`: **{len(frozen['artefacts'])} artefacts**, "
                  f"**{len(frozen['code'])} scripts**, **{len(frozen['thresholds'])} thresholds**.",
                  f"- Script coverage: {len(cov.get('named_by_board', []))} scripts named by a "
                  f"`P3-` board row; missing from the freeze: "
                  f"{cov.get('missing_from_code') or 'none'}.",
                  '- **A manifest cannot notice what is missing from it.** Coverage is derived from '
                  'the board rather than typed (`D058`). Verify with '
                  '`./.venv/bin/python experiments/make_p3_freeze.py --check`.',
                  '']

    lines += ['## Before you cite anything: what the evidence does NOT support', '',
              NOT_DERIVED]
    return '\n'.join(lines)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--check', action='store_true',
                    help='regenerate in memory and compare with the file on disk')
    args = ap.parse_args()
    text = render()
    if args.check:
        if not OUT.is_file():
            print(f'[FAIL] {OUT.relative_to(ROOT)} does not exist; run without --check to write it')
            return 1
        if OUT.read_text(encoding='utf-8') != text:
            print(f'[FAIL] {OUT.relative_to(ROOT)} is out of date; regenerate and review the diff')
            return 1
        print(f'[OK] {OUT.relative_to(ROOT)} matches what the board and the runs say')
        return 0
    OUT.write_text(text, encoding='utf-8')
    print(f'wrote {OUT.relative_to(ROOT)} ({len(text.splitlines())} lines)')
    return 0


if __name__ == '__main__':
    sys.exit(main())
