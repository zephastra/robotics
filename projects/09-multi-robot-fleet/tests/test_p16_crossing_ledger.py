"""Corridor usage, reconstructed from the event stream the task service writes.

Three of section 5's cases are about the order two robots use the shared passage:

    N04  r02 waits; r02 crosses only after r01 has FULLY released
    N05  the order follows who asked, not a fixed robot priority
    N06  the corridor is used in order, with no rear-ending

The snapshot cannot answer any of them -- it is last-writer state, and "who went first" is
gone by the time the case ends. So these tests work from the event shape the service actually
emits, and the central one is a REPRODUCTION: a stream in which two robots overlap must be
reported as a failure, with both robots and both intervals named.
"""

from __future__ import annotations

import sys
from pathlib import Path

SRC = Path(__file__).resolve().parents[1] / "src"
for _pkg in ("fleet_core",):
    _p = SRC / _pkg
    if str(_p) not in sys.path:
        sys.path.insert(0, str(_p))

from fleet_core import crossing_ledger as cl  # noqa: E402


def _start(t, task, robot, direction="west_to_east"):
    return {"t": t, "kind": "crossing_started", "subject": task,
            "data": {"robot": robot, "direction": direction, "leg": "L1"}}


def _close(kind, t, task, robot):
    return {"t": t, "kind": kind, "subject": task, "data": {"robot": robot}}


def _complete(t, task, robot):
    return _close("crossing_complete", t, task, robot)


def _failed(t, task, robot):
    return _close("crossing_failed", t, task, robot)


def _reaped(t, task, robot):
    return _close("crossing_reaped", t, task, robot)


# --------------------------------------------------------------------------- #
# the property N04/N05/N06 are about
# --------------------------------------------------------------------------- #


def test_sequential_use_by_two_robots_is_exclusive():
    events = [
        _start(100.0, "t-r01", "r01"), _complete(160.0, "t-r01", "r01"),
        _start(161.0, "t-r02", "r02"), _complete(220.0, "t-r02", "r02"),
    ]
    intervals = cl.crossing_intervals(events)
    ok, detail = cl.exclusive(intervals)
    assert ok, detail
    assert "did not overlap" in detail
    assert cl.first_crossing_order(intervals) == ["r01", "r02"]


def test_two_robots_inside_the_passage_at_once_is_the_failure_this_exists_for():
    """A reproduction, not a smoke test: r02's crossing opens while r01's is still open.

    This is the rear-ending and the face-to-face collision in one shape, and the check has to
    name both robots and both intervals -- a bare False would not say which case broke.
    """
    events = [
        _start(100.0, "t-r01", "r01"), _complete(160.0, "t-r01", "r01"),
        _start(150.0, "t-r02", "r02"), _complete(210.0, "t-r02", "r02"),
    ]
    ok, detail = cl.exclusive(cl.crossing_intervals(events))
    assert not ok
    # Asserted on the SUBSTANCE -- which robots, which intervals -- rather than on a phrase, so
    # this does not have to be edited every time the sentence is improved. The first version
    # matched "same time" and broke the moment the message named its interval correctly.
    assert "r01" in detail and "r02" in detail
    assert "100.0" in detail and "160.0" in detail and "150.0" in detail
    assert "EPISODES" in detail, "the message must say what interval it is reporting on"


def test_a_long_containing_overlap_is_still_an_overlap():
    """r01 is inside for the whole of r02's occupancy -- the easiest overlap to miss if the
    test only compares starts."""
    events = [
        _start(100.0, "t-r01", "r01"),
        _start(110.0, "t-r02", "r02"), _complete(120.0, "t-r02", "r02"),
        _complete(200.0, "t-r01", "r01"),
    ]
    assert not cl.exclusive(cl.crossing_intervals(events))[0]


def test_touching_intervals_are_not_an_overlap():
    """One releases at 160.0 and the next opens at 160.0. Zero overlap is not an overlap, and
    a check that demanded a gap would be a threshold someone would have to pick."""
    events = [
        _start(100.0, "t-r01", "r01"), _complete(160.0, "t-r01", "r01"),
        _start(160.0, "t-r02", "r02"), _complete(220.0, "t-r02", "r02"),
    ]
    assert cl.exclusive(cl.crossing_intervals(events))[0]


def test_the_same_robot_entering_twice_is_not_an_overlap():
    """A retried attempt by one robot supersedes the previous one; reporting that as two
    robots colliding would drown the finding that matters."""
    events = [
        _start(100.0, "t-r01", "r01"), _failed(130.0, "t-r01", "r01"),
        _start(131.0, "t-r01", "r01"), _complete(190.0, "t-r01", "r01"),
    ]
    intervals = cl.crossing_intervals(events)
    assert len(intervals) == 2
    assert cl.exclusive(intervals)[0]


def test_nothing_recorded_is_reported_as_nothing_checked():
    """The project's rule: something never checked is not a pass."""
    ok, detail = cl.exclusive([])
    assert not ok
    assert "never exercised" in detail


# --------------------------------------------------------------------------- #
# pairing opens with closes
# --------------------------------------------------------------------------- #


def test_a_failed_attempt_is_closed_by_its_failure():
    events = [_start(1.0, "t", "r01"), _failed(9.0, "t", "r01")]
    row = cl.crossing_intervals(events)[0]
    assert (row["start"], row["end"], row["closed_by"]) == (1.0, 9.0, "crossing_failed")


def test_a_reaped_crossing_is_closed_too():
    """The service emits `crossing_reaped` when the task was already being stopped, and the
    driver's own report never arrives. Without this the occupancy would look endless."""
    events = [_start(1.0, "t", "r01"), _reaped(4.0, "t", "r01")]
    assert cl.crossing_intervals(events)[0]["end"] == 4.0


def test_an_opening_that_arrives_while_one_is_open_supersedes_it():
    events = [
        _start(1.0, "t", "r01"),
        _start(5.0, "t", "r01"),   # the driver was replaced; no close was ever written
        _complete(20.0, "t", "r01"),
    ]
    intervals = cl.crossing_intervals(events)
    assert [r["closed_by"] for r in intervals] == ["superseded", "crossing_complete"]
    assert intervals[0]["end"] == 5.0


def test_an_occupancy_left_open_is_treated_as_never_released():
    """Conservative on purpose. No recorded release means we cannot prove the robot left, and
    the property is that the passage held one robot at a time."""
    events = [_start(1.0, "a", "r01"), _start(10.0, "b", "r02")]
    intervals = cl.crossing_intervals(events)
    assert all(r["unclosed"] for r in intervals)
    assert all(r["end"] == float("inf") for r in intervals)
    ok, detail = cl.exclusive(intervals)
    assert not ok, "two robots that both never released are inside together"
    assert "r01" in detail and "r02" in detail


def test_the_never_released_note_appears_when_there_is_no_overlap_to_report():
    """The other half of the same fact, and a separate test because `exclusive` reports the
    overlap when there is one and the note when there is not. Asserting both on one stream was
    this test's first version, and it failed on a correct message."""
    events = [_start(1.0, "a", "r01"), _complete(5.0, "a", "r01"), _start(6.0, "b", "r02")]
    intervals = cl.crossing_intervals(events)
    assert len([r for r in intervals if r["unclosed"]]) == 1
    ok, detail = cl.exclusive(intervals)
    assert ok, detail
    assert "never released" in detail


def test_an_open_occupancy_takes_part_in_the_overlap_test():
    """The case that would be missed if an open interval were closed at the last timestamp:
    r01 opened first and never released, r02 came and went inside it."""
    events = [_start(1.0, "a", "r01"), _start(10.0, "b", "r02"), _complete(20.0, "b", "r02")]
    ok, detail = cl.exclusive(cl.crossing_intervals(events))
    assert not ok
    assert "r01" in detail and "r02" in detail
    assert "end of recording" in detail, "the unclosed interval must be legible in the sentence"


def test_unrelated_event_kinds_are_ignored():
    events = [
        {"t": 1.0, "kind": "submitted", "subject": "t", "data": {}},
        _start(2.0, "t", "r01"), _complete(3.0, "t", "r01"),
        {"t": 4.0, "kind": "succeeded", "subject": "t", "data": {}},
    ]
    assert len(cl.crossing_intervals(events)) == 1


def test_a_crossing_with_no_robot_is_kept_but_not_attributed():
    """The event should always carry the robot. If it does not, the occupancy must still be
    counted -- dropping it would hide a robot in the corridor."""
    events = [{"t": 1.0, "kind": "crossing_started", "subject": "t", "data": {}},
              {"t": 2.0, "kind": "crossing_complete", "subject": "t", "data": {}}]
    intervals = cl.crossing_intervals(events)
    assert len(intervals) == 1 and intervals[0]["robot"] is None
    assert cl.first_crossing_order(intervals) == []


# --------------------------------------------------------------------------- #
# order
# --------------------------------------------------------------------------- #


def test_the_order_is_by_first_entry_not_by_every_entry():
    """`crossing_order: [r01, r02]` is a claim about who asked FIRST. A robot that crosses
    again later does not change that."""
    events = [
        _start(10.0, "a", "r01"), _complete(20.0, "a", "r01"),
        _start(30.0, "b", "r02"), _complete(40.0, "b", "r02"),
        _start(50.0, "c", "r01"), _complete(60.0, "c", "r01"),
    ]
    assert cl.first_crossing_order(cl.crossing_intervals(events)) == ["r01", "r02"]


def test_a_three_robot_case_reports_all_three():
    events = [
        _start(10.0, "a", "r03"), _complete(20.0, "a", "r03"),
        _start(21.0, "b", "r01"), _complete(30.0, "b", "r01"),
        _start(31.0, "c", "r02"), _complete(40.0, "c", "r02"),
    ]
    assert cl.first_crossing_order(cl.crossing_intervals(events)) == ["r03", "r01", "r02"]


# --------------------------------------------------------------------------- #
# reading the file
# --------------------------------------------------------------------------- #


def test_a_missing_event_file_reads_as_no_events(tmp_path):
    assert cl.read_events(tmp_path / "nope.jsonl") == []


def test_the_recording_copy_is_strict_json():
    """`Infinity` is valid for Python's json and not for strict JSON, and a report a person
    reads should not carry it."""
    events = [_start(1.0, "a", "r01")]
    rows = cl.for_recording(cl.crossing_intervals(events))
    assert rows[0]["end"] is None
    assert "never released" in rows[0]["end_note"]
    import json
    assert json.loads(json.dumps(rows, allow_nan=False)) == rows


def test_the_recording_copy_keeps_the_times_it_can():
    events = [_start(1.0, "a", "r01"), _complete(9.5, "a", "r01")]
    rows = cl.for_recording(cl.crossing_intervals(events))
    assert (rows[0]["start"], rows[0]["end"]) == (1.0, 9.5)
    assert "end_note" not in rows[0]


def test_a_truncated_line_does_not_lose_the_rest_of_the_recording(tmp_path):
    """A run that was killed mid-write leaves one bad line. Losing the whole file to it would
    turn a readable recording into an exception."""
    path = tmp_path / "events.jsonl"
    path.write_text(
        '{"t": 1.0, "kind": "crossing_started", "subject": "t", "data": {"robot": "r01"}}\n'
        '{"t": 2.0, "kind": "crossing_compl\n'
        '{"t": 3.0, "kind": "crossing_complete", "subject": "t", "data": {"robot": "r01"}}\n',
        encoding="utf-8")
    rows = cl.read_events(path)
    assert len(rows) == 2
    assert cl.crossing_intervals(rows)[0]["end"] == 3.0
