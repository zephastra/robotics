"""The truth-liveness guard must discriminate frozen from live from partial from short.

Round 17 wrote a wrong conclusion because nothing checked the slot count against
the case's robot declaration. This guard is the check. Test it by handing it
recordings it must reject -- a guard that only ever says "live" is what let a
missing body go unnoticed.

The four verdicts, and what each one means for a safety verdict read off the
recording:

    live     every model's ROOT slot advanced; the judge's verdicts mean something
    partial  at least one body never moved while another did; not a clean run
    frozen   no body ever moved; NO safety verdict may be read as a pass
    unknown  too few truth-bearing samples to judge; a refusal, not a pass

A model's root slot is `m * LINKS_PER_MODEL`; slots 1..7 are LINK offsets inside
the robot fed by /joint_states, so a pinned body still has spinning wheels. That
is why the criterion is per-model and not "did any slot move" -- see D-P17-14.
"""
from __future__ import annotations

import json
import pathlib
import subprocess
import sys

import pytest

ROOT = pathlib.Path(__file__).resolve().parent.parent
GUARD = ROOT / "scripts/check_truth_liveness.py"
sys.path.insert(0, str(ROOT / "scripts"))

import check_truth_liveness as ctl  # noqa: E402


def _recording(tmp_path: pathlib.Path, name: str, *, slots: int, move: bool,
               robots: list[str], n: int = 60, qos: bool = True,
               move_all: bool = False) -> pathlib.Path:
    """Write a synthetic recording: a samples file plus a recorder audit."""
    d = tmp_path / name
    d.mkdir(parents=True, exist_ok=True)

    lines = []
    for i in range(n):
        truth = {}
        for s in range(slots):
            x = float(s) * 0.001
            # Slot 0 is model 0's ROOT (world pose). Slots 1..7 are LINK offsets
            # inside the robot, fed by /joint_states -- a different stream. The
            # liveness criterion reads ROOT slots only, i.e. m * LINKS_PER_MODEL.
            root = (s % 8 == 0)
            if move and s == 0:
                x += i * 0.01          # model 0's body walks away from its spawn
            if move_all and root:
                x += i * 0.01          # every model's body walks
            truth[str(s)] = [x, 0.0, 0.0]
        lines.append(json.dumps({"t": i * 0.05, "truth": truth, "odom": {}}))
    (d / f"samples-{name}.jsonl").write_text("\n".join(lines) + "\n", encoding="utf-8")

    audit = {"robots": robots, "truth_messages": n, "truth_slots_recorded": slots}
    if qos:
        audit["truth_qos_depth"] = 2000
    (d / f"recorder-{name}.json").write_text(json.dumps(audit), encoding="utf-8")
    return d


def _run(root: pathlib.Path, *flags: str) -> subprocess.CompletedProcess:
    return subprocess.run(
        [sys.executable, str(GUARD), *flags, str(root)],
        capture_output=True, text=True, cwd=ROOT,
    )


# --------------------------------------------------------------------------- #
# unit level
# --------------------------------------------------------------------------- #
def test_frozen_verdict(tmp_path):
    d = _recording(tmp_path, "frozen", slots=24, move=False, robots=["r01", "r02", "r03"])
    verdict, ev = ctl.liveness(next(d.glob("samples-*.jsonl")))
    assert verdict == "frozen"
    assert ev["samples_with_movement"] == 0
    assert ev["models_in_world"] == 3


def test_partial_when_only_one_models_body_moves(tmp_path):
    """The D-P17-14 correction, stated as a test.

    This fixture walks slot 0 and leaves slots 8 and 16 pinned. Slot 0 is model
    0's ROOT; slots 8 and 16 are the roots of models 1 and 2. So one body moves
    and two do not.

    The superseded criterion called this `live`, because it counted a sample as
    moved if ANY slot advanced and slots 1..7 (wheel link offsets, from
    /joint_states) keep changing on a pinned body. That is exactly the false
    negative D-P17-14 was written for, so `partial` is the verdict that must
    hold here.
    """
    d = _recording(tmp_path, "partial", slots=24, move=True, robots=["r01", "r02", "r03"])
    verdict, ev = ctl.liveness(next(d.glob("samples-*.jsonl")))
    assert verdict == "partial"
    assert ev["alive_models"] == [0]                # only model 0's root advanced
    assert ev["dead_models"] == [1, 2]              # models 1 and 2 are pinned
    assert ev["samples_with_movement"] > 0          # the any-slot signal still fires


def test_live_verdict(tmp_path):
    """Every model's body must be moving for the recording to be `live`.

    Without this test the correction could be satisfied by a guard that never
    says `live` at all -- and such a guard would fail every real run, which is
    its own kind of useless.
    """
    d = _recording(tmp_path, "live", slots=24, move=True, move_all=True,
                   robots=["r01", "r02", "r03"])
    verdict, ev = ctl.liveness(next(d.glob("samples-*.jsonl")))
    assert verdict == "live"
    assert ev["samples_with_movement"] > 0
    assert ev["dead_models"] == []                  # no body standing still
    assert ev["alive_models"] == [0, 1, 2]


def test_short_when_world_has_fewer_models_than_robots(tmp_path):
    """2 models for 3 robots: exactly the round-17 mistake.

    This test is about the model-count comparison, not about liveness. Both
    bodies that ARE present move, so the verdict is `live` -- and that is the
    point: a recording can be perfectly live and still describe the wrong world.
    The guard reports it as SHORT and fails (see
    test_short_recording_fails_the_guard), because no safety verdict from it may
    be quoted either way.
    """
    d = _recording(tmp_path, "short", slots=16, move=True, move_all=True,
                   robots=["r01", "r02", "r03"])
    verdict, ev = ctl.liveness(next(d.glob("samples-*.jsonl")))
    assert verdict == "live"                       # both present bodies move
    assert ev["models_in_world"] == 2              # but only two bodies
    assert ev["dead_models"] == []
    assert ev["models_in_world"] < len(ctl.robots_expected(
        next(d.glob("samples-*.jsonl"))))


def test_unknown_below_min_samples(tmp_path):
    d = _recording(tmp_path, "tiny", slots=24, move=True, robots=["r01"], n=10)
    verdict, _ = ctl.liveness(next(d.glob("samples-*.jsonl")))
    assert verdict == "unknown"


# --------------------------------------------------------------------------- #
# guard level: the exit code is the deliverable
# --------------------------------------------------------------------------- #
def test_frozen_recording_fails_the_guard(tmp_path):
    _recording(tmp_path, "frozen", slots=24, move=False, robots=["r01", "r02", "r03"])
    res = _run(tmp_path)
    assert res.returncode == 1
    assert "FROZEN" in res.stdout


def test_short_recording_fails_the_guard(tmp_path):
    _recording(tmp_path, "short", slots=16, move=True, move_all=True,
               robots=["r01", "r02", "r03"])
    res = _run(tmp_path)
    assert res.returncode == 1
    assert "SHORT" in res.stdout
    assert "robots:=" in res.stdout


def test_matching_recording_passes_the_guard(tmp_path):
    """2 models, 2 robots, both bodies moving: nothing to report, exit 0.

    This is the only synthetic recording that is allowed to pass, and it must
    pass for the RIGHT reason -- so the fixture moves every model's root. When
    only slot 0 moved this recording scored `partial`, which is why the
    superseded version of this test was asserting exit 0 on a verdict it had not
    read.
    """
    _recording(tmp_path, "ok", slots=16, move=True, move_all=True,
               robots=["r01", "r02"])
    res = _run(tmp_path)
    assert res.returncode == 0, res.stdout
    # The line-level markers, not the bare words: the summary line always reads
    # "FROZEN: 0   PARTIAL: 0", so a substring test on "PARTIAL" fails on a clean
    # recording. Match the markers the guard prints per recording instead.
    assert "<== SHORT" not in res.stdout
    assert "[PARTIAL]" not in res.stdout
    assert "[FROZEN]" not in res.stdout
    assert "[  live ]" in res.stdout
    assert "PARTIAL: 0" in res.stdout and "FROZEN: 0" in res.stdout


def test_pre_fix_recordings_are_skipped_by_default(tmp_path):
    """A recording written before the fix cannot be a regression."""
    _recording(tmp_path, "old", slots=24, move=False, robots=["r01", "r02", "r03"],
               qos=False)
    res = _run(tmp_path)
    assert res.returncode == 0, res.stdout
    assert "skipped" in res.stdout

    res_all = _run(tmp_path, "--all")
    assert "FROZEN" in res_all.stdout


def test_audit_mode_does_not_fail_a_build(tmp_path):
    """--all is a survey; a historical recording must not gate the build."""
    _recording(tmp_path, "frozen", slots=24, move=False, robots=["r01", "r02", "r03"])
    res = _run(tmp_path, "--all")
    assert res.returncode == 0, res.stdout


def test_underscore_directory_is_not_a_result(tmp_path):
    """reports/_invalid/ holds mistaken runs as evidence, not as results."""
    d = _recording(tmp_path, "bad", slots=16, move=True, robots=["r01", "r02", "r03"])
    archived = tmp_path / "_invalid"
    archived.mkdir()
    d.rename(archived / d.name)
    res = _run(tmp_path)
    assert res.returncode == 0, res.stdout
