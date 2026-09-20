"""Read the corridor's usage out of the event stream.

WHY THIS EXISTS
===============

`config/resources.yaml` gives `mid` a capacity of 1, and three of section 5's cases are about
the order two robots use it in:

    N04  r02 waits; r02 crosses only after r01 has FULLY released
    N05  the order follows who asked first, not a fixed robot priority
    N06  the corridor is used in order, with no rear-ending

None of those can be read from the end-of-run snapshot. The snapshot's maps are last-writer
state: by the time the case ends, `crossings` is empty and `crossed_legs` is a sorted set of
leg ids with no times in it. "Who went first" is gone.

The task service already writes it down -- `crossing_started`, `crossing_complete`,
`crossing_failed`, `crossing_reaped` -- so this reads those four kinds and reconstructs who
occupied the corridor when. The service says as much about its own event file: "P6's timeline
reads this file, not a status field."

WHY THIS IS NOT A TIME THRESHOLD
================================

Two properties are checked, and neither needs a tolerance:

  * EXCLUSIVITY -- no two robots inside the passage at once. An overlap is
    `start_b < end_a and start_a < end_b` whatever the clock says; no margin to tune.
  * ORDER -- which robot's crossing opened first. That is a comparison of two timestamps from
    the same file and the same clock.

So there is no number here that could be widened to make a failing case pass. That is the
point: the alternative is asserting "the corridor was free for `n` seconds", and `n` is then a
threshold someone will eventually lower.

A RECORDING THAT IS NOT THERE IS NOT A PASS
===========================================

`exclusive([])` returns False and says so. An empty interval list means the run did not use
the corridor at all, which for a case whose whole subject is corridor use is a failure to
observe, not a clean sheet. Project rule: something that was never checked is not a pass.
"""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any, Iterable, Mapping

#: The four kinds the task service emits about a crossing. Anything else is ignored.
OPENS = "crossing_started"
CLOSES = ("crossing_complete", "crossing_failed", "crossing_reaped")


def read_events(path: Path) -> list[dict]:
    """Every event in the file, in order. A missing or unreadable file reads as empty.

    Empty rather than an exception: a case that recorded nothing and a case whose recording
    was deleted are both "there is no evidence", and the caller's next step is the same
    (report it). Raising here would make the runner report an exception instead of a finding.
    """
    try:
        text = path.read_text(encoding="utf-8", errors="replace")
    except OSError:
        return []
    out: list[dict] = []
    for line in text.splitlines():
        line = line.strip()
        if not line:
            continue
        try:
            row = json.loads(line)
        except ValueError:
            continue
        if isinstance(row, dict):
            out.append(row)
    return out


def crossing_intervals(events: Iterable[Mapping[str, Any]]) -> list[dict]:
    """One record per occupancy of the passage.

    Pairs an opening event with the next closing event for the SAME subject (the task id, which
    is what `crossing_started` carries as its `subject`). A retry opens a second interval after
    its predecessor closed; an opening that arrives while one is still open supersedes it,
    because the driver for that task was replaced and the first interval's end was never
    written.

    An interval left open at the end of the file is treated as NEVER RELEASED -- it runs to
    infinity for the overlap test and is flagged `unclosed`. That is deliberately the
    conservative reading: the property being checked is that the passage held one robot at a
    time, and an occupancy with no recorded release is one where we cannot prove the robot
    left. Closing it at the last timestamp instead would make two robots that both never
    released look like they touched exactly once and not overlapped.
    """
    rows = list(events)
    last_t = None
    for row in rows:
        t = row.get("t")
        if isinstance(t, (int, float)):
            last_t = float(t)

    open_by_subject: dict[Any, dict] = {}
    out: list[dict] = []

    def _close(record: dict, at: Any, why: str) -> None:
        record["end"] = float(at) if isinstance(at, (int, float)) else record["start"]
        record["closed_by"] = why
        out.append(record)

    for row in rows:
        kind = row.get("kind")
        if kind != OPENS and kind not in CLOSES:
            continue
        subject = row.get("subject")
        data = row.get("data") or {}
        t = row.get("t")
        if kind == OPENS:
            previous = open_by_subject.pop(subject, None)
            if previous is not None:
                _close(previous, t, "superseded")
            if isinstance(t, (int, float)):
                open_by_subject[subject] = {
                    "task_id": subject,
                    "robot": data.get("robot"),
                    "direction": data.get("direction"),
                    "leg": data.get("leg"),
                    "start": float(t),
                    "end": None,
                    "closed_by": None,
                    "unclosed": False,
                }
        else:
            record = open_by_subject.pop(subject, None)
            if record is not None:
                _close(record, t, kind)

    for record in open_by_subject.values():
        record["unclosed"] = True
        record["end"] = float("inf")
        record["closed_by"] = "end of recording"
        record["end_of_recording"] = last_t
        out.append(record)

    out.sort(key=lambda r: (r["start"], str(r.get("robot"))))
    return out


def _fmt(value: float) -> str:
    """A time for a sentence. Infinity is the 'never released' case, so it says so."""
    return "end of recording" if value == float("inf") else f"{value:.1f}"


def overlaps(intervals: Iterable[Mapping[str, Any]]) -> list[tuple[dict, dict]]:
    """Every pair of occupancies by DIFFERENT robots that share a moment.

    Two intervals by the same robot are not an overlap: a robot re-entering the passage after
    its own failed attempt is sequential by construction, and reporting it would drown the
    finding that matters.
    """
    rows = [dict(r) for r in intervals]
    out: list[tuple[dict, dict]] = []
    for i, a in enumerate(rows):
        for b in rows[i + 1:]:
            if a.get("robot") == b.get("robot"):
                continue
            if a["start"] < b["end"] and b["start"] < a["end"]:
                out.append((a, b))
    return out


def exclusive(intervals: Iterable[Mapping[str, Any]]) -> tuple[bool, str]:
    """The corridor held one robot at a time. Returns (held, a sentence either way)."""
    rows = [dict(r) for r in intervals]
    if not rows:
        return False, ("no crossing was recorded, so corridor exclusivity was never "
                       "exercised -- a case about the corridor that used it zero times "
                       "checked nothing")
    bad = overlaps(rows)
    if bad:
        a, b = bad[0]
        more = "" if len(bad) == 1 else f" (and {len(bad) - 1} more)"
        return False, (
            f"{a.get('robot')} and {b.get('robot')} occupied the corridor in overlapping "
            f"EPISODES{more}: {a.get('robot')} "
            f"[{_fmt(a['start'])}, {_fmt(a['end'])}] vs {b.get('robot')} "
            f"[{_fmt(b['start'])}, {_fmt(b['end'])}]. An episode starts when the driver "
            f"process is launched, so it INCLUDES waiting for the permit: two episodes "
            f"overlapping is what correct queueing looks like, and this check does not "
            f"currently separate queueing from driving. See docs/DECISIONS.md D-P16-04 -- "
            f"do not assert this until the driver reports when it holds the permit.")
    unclosed = [r for r in rows if r.get("unclosed")]
    note = ""
    if unclosed:
        note = (f"; {len(unclosed)} occupancy(ies) had no recorded release and were treated "
                f"as never released")
    robots = sorted({str(r.get("robot")) for r in rows})
    return True, (f"{len(rows)} occupancy(ies) by {robots} did not overlap{note}")


def for_recording(intervals: Iterable[Mapping[str, Any]]) -> list[dict]:
    """The same rows, fit to be written into a JSON report.

    `end` is infinity for an occupancy with no recorded release. That is the right value for
    the overlap test and the wrong thing to put in a summary a person reads -- and `Infinity`
    is not strict JSON either. Times are rounded so two readings of one file compare equal.
    """
    out: list[dict] = []
    for row in intervals:
        copy = dict(row)
        copy["start"] = round(float(copy["start"]), 3)
        if copy.get("end") == float("inf"):
            copy["end"] = None
            copy["end_note"] = "no release was recorded; treated as never released"
        else:
            copy["end"] = round(float(copy["end"]), 3)
        out.append(copy)
    return out


def first_crossing_order(intervals: Iterable[Mapping[str, Any]]) -> list[str]:
    """The robots in the order their FIRST occupancy opened.

    First, not all: `crossing_order: [r01, r02]` is a claim about who asked first, and a robot
    that goes on to cross again later does not change who went first.
    """
    seen: list[str] = []
    for record in sorted((dict(r) for r in intervals), key=lambda r: r["start"]):
        robot = record.get("robot")
        if robot is None:
            continue
        name = str(robot)
        if name not in seen:
            seen.append(name)
    return seen
