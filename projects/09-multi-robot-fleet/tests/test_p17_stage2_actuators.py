"""The stage-2 actuators: what they build, what they refuse, and what they admit.

`D-P17-26` was ten cases whose fault was never delivered because the action they named had
no implementation. These tests exist so the replacement cannot regress into the same shape:
a name that is dispatched but does nothing, or one that does something other than what it
reports.

Nothing here touches a live simulator. What is asserted is the part that can be wrong
without a simulator and still produce a confident verdict: the command that gets built, the
refusals, and the honesty of the note that goes into the record.
"""
from __future__ import annotations

import importlib.util
import pathlib
import sys

import pytest

ROOT = pathlib.Path(__file__).resolve().parent.parent
BATCH = ROOT / "scripts/batch.py"


def _load_batch():
    spec = importlib.util.spec_from_file_location("_batch_actuators", BATCH)
    mod = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(mod)
    return mod


@pytest.fixture()
def batch():
    return _load_batch()


class _FakeRun:
    """Stands in for subprocess.run and records every command."""

    def __init__(self, stdout: str = "true", returncode: int = 0):
        self.stdout = stdout
        self.returncode = returncode
        self.calls: list[list[str]] = []

    def __call__(self, cmd, **_kwargs):
        self.calls.append(list(cmd))

        class P:
            pass

        p = P()
        p.stdout = self.stdout
        p.stderr = ""
        p.returncode = self.returncode
        return p


# --------------------------------------------------------------------------- #
# the world name comes from the file, not from a constant
# --------------------------------------------------------------------------- #
def test_world_name_is_read_from_the_sdf(batch):
    """A hardcoded name that stops matching the file reads as a broken gz install."""
    assert batch._world_name() == "warehouse"


def test_world_name_refuses_a_missing_world(batch, tmp_path):
    (tmp_path / "assets/worlds").mkdir(parents=True)
    (tmp_path / "assets/worlds/warehouse.sdf").write_text(
        "<sdf version='1.10'><world/></sdf>", encoding="utf-8")
    old = batch.os.environ.get("FLEET_REPO_ROOT")
    batch.os.environ["FLEET_REPO_ROOT"] = str(tmp_path)
    try:
        with pytest.raises(batch.CaseFailed):
            batch._world_name()
    finally:
        if old is None:
            batch.os.environ.pop("FLEET_REPO_ROOT", None)
        else:
            batch.os.environ["FLEET_REPO_ROOT"] = old


# --------------------------------------------------------------------------- #
# spawn / despawn
# --------------------------------------------------------------------------- #
def test_spawn_obstacle_builds_a_cylinder_of_the_requested_radius(batch, monkeypatch):
    fake = _FakeRun()
    monkeypatch.setattr(batch.subprocess, "run", fake)
    monkeypatch.setattr(batch.time, "sleep", lambda _s: None)

    ok, note = batch.spawn_obstacle("f02-block", 0.6, 0.0, 0.65)
    assert ok, note
    cmd = fake.calls[0]
    assert cmd[:4] == ["ros2", "run", "ros_gz_sim", "create"]
    assert "-world" in cmd and cmd[cmd.index("-world") + 1] == "warehouse"
    assert "-name" in cmd and cmd[cmd.index("-name") + 1] == "f02-block"
    sdf = cmd[cmd.index("-string") + 1]
    # 0.65 m radius is the number that makes F02's "blocked for good" claim checkable:
    # its diameter 1.30 m exceeds the 1.2 m gap in the barrier.
    assert "<radius>0.6500</radius>" in sdf
    assert "<cylinder>" in sdf
    # The obstacle has to be taller than the lidar plane or nothing sees it.
    assert batch.OBSTACLE_HEIGHT_M > 0.30
    # The note must let a reader check the claim without doing arithmetic: F02 says the gap is
    # blocked for good, and that is a statement about the DIAMETER against the 1.2 m gap.
    assert "r=0.65" in note and "d=1.30" in note and "f02-block" in note


def test_spawn_obstacle_refuses_renaming(batch, monkeypatch):
    """A silently renamed duplicate would be one `despawn_obstacle` could not remove."""
    fake = _FakeRun()
    monkeypatch.setattr(batch.subprocess, "run", fake)
    monkeypatch.setattr(batch.time, "sleep", lambda _s: None)
    batch.spawn_obstacle("b", 0.0, 0.0, 0.25)
    cmd = fake.calls[0]
    assert cmd[cmd.index("-allow_renaming") + 1] == "false"


def test_spawn_obstacle_reports_failure_when_gz_says_nothing_good(batch, monkeypatch):
    fake = _FakeRun(stdout="Error: entity already exists")
    monkeypatch.setattr(batch.subprocess, "run", fake)
    monkeypatch.setattr(batch.time, "sleep", lambda _s: None)
    ok, note = batch.spawn_obstacle("dup", 0.0, 0.0, 0.25)
    assert ok is False
    assert "was not created" in note


def test_despawn_obstacle_asks_the_world_to_remove_it(batch, monkeypatch):
    """The world's own remove service is the PRIMARY path.

    Measured 2026-09-19 on F01's second step: `ros2 run ros_gz_sim delete_entity` hung and
    was killed by its own 30 s timeout, while `gz service` answered at once on the same
    world -- the same path `pause_world` uses and that works. The note has to say which
    path answered, or a reader cannot tell a service removal from a CLI removal.
    """
    fake = _FakeRun()
    monkeypatch.setattr(batch.subprocess, "run", fake)
    monkeypatch.setattr(batch.time, "sleep", lambda _s: None)
    ok, note = batch.despawn_obstacle("f01-block")
    assert ok, note
    cmd = fake.calls[0]
    assert cmd[0] == "gz" and cmd[1] == "service", cmd
    assert cmd[cmd.index("-s") + 1].endswith("/remove"), cmd
    assert cmd[cmd.index("--reqtype") + 1] == "gz.msgs.Entity", cmd
    req = cmd[cmd.index("--req") + 1]
    assert 'name: "f01-block"' in req, req
    assert "type: 2" in req, f"2 is MODEL on the gz side: {req}"
    assert "f01-block" in note
    assert "remove service" in note, f"the note must name the path: {note!r}"


def test_despawn_obstacle_falls_back_to_the_cli_and_says_so(batch, monkeypatch):
    """A fallback that is never exercised is not a fallback."""
    class ServiceDown(_FakeRun):
        def __call__(self, cmd, **_kwargs):
            self.calls.append(list(cmd))

            class P:
                pass
            p = P()
            p.returncode = 1 if cmd[0] == "gz" else 0
            p.stdout = ""
            p.stderr = "no such service" if cmd[0] == "gz" else ""
            return p

    fake = ServiceDown()
    monkeypatch.setattr(batch.subprocess, "run", fake)
    monkeypatch.setattr(batch.time, "sleep", lambda _s: None)
    ok, note = batch.despawn_obstacle("f01-block")
    assert ok, note
    assert len(fake.calls) >= 2, fake.calls
    cli = fake.calls[1]
    assert "delete_entity" in cli, cli
    assert cli[cli.index("--name") + 1] == "f01-block", cli
    assert cli[cli.index("--type") + 1] == "6", "6 is MODEL on the ros_gz_sim side"
    assert "CLI" in note, f"the note must name the path: {note!r}"


def test_despawn_obstacle_refuses_an_empty_name(batch):
    """`-name ""` builds a model called "" that nothing can ever address."""
    ok, note = batch.despawn_obstacle("")
    assert not ok
    assert "name" in note


# --------------------------------------------------------------------------- #
# pause / resume
# --------------------------------------------------------------------------- #
def _gz_calls(fake):
    """The `gz service` calls among everything the actuator shelled out to.

    A test that indexes `fake.calls[0]` is asserting an ORDER that is not part of the
    contract: the pause step reads the clock first, so its service call is no longer first.
    """
    return [c for c in fake.calls if c and c[0] == "gz"]


def test_pause_and_resume_send_the_opposite_flag(batch, monkeypatch):
    fake = _FakeRun()
    monkeypatch.setattr(batch.subprocess, "run", fake)

    ok, note = batch.set_world_paused(True)
    assert ok, note
    gzs = _gz_calls(fake)
    assert "pause: true" in gzs[0][gzs[0].index("--req") + 1]
    assert "paused" in note

    ok, note = batch.set_world_paused(False)
    assert ok, note
    gzs = _gz_calls(fake)
    assert "unpause: true" in gzs[1][gzs[1].index("--req") + 1]
    assert "resumed" in note


def test_pause_targets_the_world_control_service(batch, monkeypatch):
    fake = _FakeRun()
    monkeypatch.setattr(batch.subprocess, "run", fake)
    batch.set_world_paused(True)
    cmd = _gz_calls(fake)[0]
    assert cmd[cmd.index("-s") + 1] == "/world/warehouse/control"
    assert cmd[cmd.index("--reqtype") + 1] == "gz.msgs.WorldControl"


def test_a_false_reply_is_not_success(batch, monkeypatch):
    """gz answers a malformed request with false, not an error."""
    fake = _FakeRun(stdout="false")
    monkeypatch.setattr(batch.subprocess, "run", fake)
    ok, note = batch.set_world_paused(True)
    assert ok is False
    assert "expected true" in note


def test_a_paused_world_still_yields_a_magnitude_for_a_jump(batch, monkeypatch):
    """The freeze is part of I05's mechanism, so the pause must not blind the jump.

    `/clock` stops publishing while the world is paused and the clock cannot move during a
    freeze, so the value sampled just before the pause IS the value at jump time. The note
    must say which of the two it is: a remembered number is not a read number.
    """
    fake = _FakeRun()
    monkeypatch.setattr(batch.subprocess, "run", fake)
    monkeypatch.setattr(batch.time, "sleep", lambda _s: None)
    monkeypatch.setattr(batch, "_sim_now", lambda: 47.5)

    ok, note = batch.set_world_paused(True)
    assert ok, note
    assert "47.500" in note, f"the pause should state what it froze at: {note!r}"

    # After the freeze /clock is silent, which is what None means here.
    monkeypatch.setattr(batch, "_sim_now", lambda: None)
    ok, note = batch.jump_clock(-30.0)
    assert ok, note
    assert "47.500" in note, note
    assert "before the world was frozen" in note, (
        f"the note must say the value came from the pause, not from a read: {note!r}")


# --------------------------------------------------------------------------- #
# jump_clock -- including the limit it must not hide
# --------------------------------------------------------------------------- #
def test_a_zero_jump_is_refused_as_not_a_fault(batch, monkeypatch):
    ok, note = batch.jump_clock(0.0)
    assert ok is False
    assert "does not move the clock" in note


def test_a_backwards_jump_uses_a_reset_and_states_the_real_magnitude(batch, monkeypatch):
    """I05 declares -30.0 s. gz cannot set an arbitrary earlier time, so the honest thing is
    to do the one backwards move that exists and say what it actually was."""
    fake = _FakeRun()
    monkeypatch.setattr(batch.subprocess, "run", fake)
    times = iter([47.5, 0.0])
    monkeypatch.setattr(batch, "_sim_now", lambda: next(times, 0.0))

    ok, note = batch.jump_clock(-30.0)
    assert ok, note
    req = fake.calls[0][fake.calls[0].index("--req") + 1]
    assert "reset" in req and "time_only: true" in req, "the only backwards move gz has"
    assert "47.5" in note and "-47.5" in note, "the ACTUAL magnitude, not the requested one"
    assert "-30.0" in note, "and the requested one, so a reader can see they differ"


def test_a_forward_jump_runs_to_an_absolute_sim_time(batch, monkeypatch):
    fake = _FakeRun()
    monkeypatch.setattr(batch.subprocess, "run", fake)
    monkeypatch.setattr(batch, "_sim_now", lambda: 10.0)
    ok, note = batch.jump_clock(5.0)
    assert ok, note
    req = fake.calls[0][fake.calls[0].index("--req") + 1]
    assert "run_to_sim_time" in req and "sec: 15" in req
    assert "+5.000" in note


def test_a_jump_with_no_readable_clock_is_refused(batch, monkeypatch):
    monkeypatch.setattr(batch, "_sim_now", lambda: None)
    ok, note = batch.jump_clock(-30.0)
    assert ok is False
    assert "could not be read" in note


# --------------------------------------------------------------------------- #
# drive_to_pose
# --------------------------------------------------------------------------- #
def test_drive_to_pose_refuses_without_a_runner(batch):
    ok, note = batch.drive_to_pose("", -2.5, -1.4)
    assert ok is False
    assert "needs a runner" in note


def test_drive_to_pose_maps_each_exit_code_to_its_own_meaning(batch, monkeypatch):
    """A missing action server must not be recorded as a robot that failed to navigate."""
    for code, expected in ((0, "reached"), (1, "rejected or aborted"),
                           (2, "no action server"), (3, "timed out")):
        fake = _FakeRun(returncode=code)
        monkeypatch.setattr(batch.subprocess, "run", fake)
        ok, note = batch.drive_to_pose("r02", -2.5, -1.4)
        assert ok is (code == 0), (code, note)
        assert expected in note, (code, note)
        assert fake.calls[0][0] == "/usr/bin/python3"
        assert "nav_goal.py" in fake.calls[0][1]


# --------------------------------------------------------------------------- #
# the dispatch itself
# --------------------------------------------------------------------------- #
def test_fire_step_reaches_every_new_action(batch, monkeypatch):
    seen: list[str] = []

    def fake_spawn(name, x, y, r):
        seen.append(f"spawn:{name}")
        return True, "spawned"

    def fake_despawn(name):
        seen.append(f"despawn:{name}")
        return True, "removed"

    def fake_pause(paused):
        seen.append(f"paused:{paused}")
        return True, "ok"

    def fake_jump(s):
        seen.append(f"jump:{s}")
        return True, "ok"

    def fake_drive(runner, x, y):
        seen.append(f"drive:{runner}")
        return True, "reached"

    monkeypatch.setattr(batch, "spawn_obstacle", fake_spawn)
    monkeypatch.setattr(batch, "despawn_obstacle", fake_despawn)
    monkeypatch.setattr(batch, "set_world_paused", fake_pause)
    monkeypatch.setattr(batch, "jump_clock", fake_jump)
    monkeypatch.setattr(batch, "drive_to_pose", fake_drive)

    class CD:
        name = "F01"

    steps = [
        {"do": {"action": "spawn_obstacle", "x": 0.6, "y": 0.1, "radius_m": 0.25}},
        {"do": {"action": "despawn_obstacle", "name": "gone"}},
        {"do": {"action": "pause_world"}},
        {"do": {"action": "resume_world"}},
        {"do": {"action": "jump_clock", "seconds": -30.0}},
        {"do": {"action": "drive_to_pose", "runner": "r02", "x": -2.5, "y": -1.4}},
    ]
    for step in steps:
        ok, _note = batch.fire_step(None, step, CD())
        assert ok, step

    # The obstacle name is optional in the declared vocabulary, so an unnamed spawn must get
    # a deterministic name from the case label rather than an empty string.
    assert seen[0] == "spawn:obstacle-F01"
    assert seen[1:] == ["despawn:gone", "paused:True", "paused:False", "jump:-30.0",
                        "drive:r02"]


def test_an_unknown_action_is_still_refused(batch):
    class CD:
        name = "X1"

    ok, note = batch.fire_step(None, {"do": {"action": "no_such_action"}}, CD())
    assert ok is False
    assert "not implemented" in note
