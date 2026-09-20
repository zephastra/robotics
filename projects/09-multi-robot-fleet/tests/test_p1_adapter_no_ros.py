"""L1 §14 + the P1 isolation guarantee.

Two things are checked here:

  * the fake adapter behaves as a *motion stand-in*, and its status keeps
    "accepted a cancel" separate from "confirmed stopped"
  * **nothing in the P1 code imports ROS.** This is enforced by reading the
    source, not by trusting intent. If someone later adds `import rclpy` to the
    core, this test fails immediately rather than at P2 in a confusing way.
"""

from __future__ import annotations

import sys
from pathlib import Path

import pytest

SRC = Path(__file__).resolve().parents[1] / "src"

from fleet_adapter.adapter_base import AdapterPhase, MotionCommand
from fleet_adapter.fake_adapter import FakeAdapter


# --------------------------------------------------------------------------- #
# ROS isolation -- the P1 contract
# --------------------------------------------------------------------------- #

ROS_MODULES = ("rclpy", "rospy", "rcl_interfaces", "rosidl_runtime_py")


def test_no_ros_module_is_imported_by_the_p1_packages():
    import fleet_core  # noqa: F401
    import fleet_adapter  # noqa: F401

    leaked = [m for m in ROS_MODULES if m in sys.modules]
    assert not leaked, f"P1 must be ROS-free, but these were imported: {leaked}"


@pytest.mark.parametrize("package", ["fleet_core", "fleet_adapter"])
def test_p1_source_contains_no_ros_import(package):
    """Parse with `ast` rather than scanning text.

    A substring scan gives false positives on the docstring in
    fleet_core/__init__.py, which *mentions* rclpy in prose. Only real import
    statements count.
    """
    import ast

    roots = sorted((SRC / package).rglob("*.py"))
    assert roots, f"no sources found for {package}"

    offenders: list[str] = []
    for path in roots:
        tree = ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
        for node in ast.walk(tree):
            if isinstance(node, ast.Import):
                for alias in node.names:
                    root = alias.name.split(".")[0]
                    if root in ROS_MODULES:
                        offenders.append(f"{path}:{node.lineno}: import {alias.name}")
            elif isinstance(node, ast.ImportFrom):
                root = (node.module or "").split(".")[0]
                if root in ROS_MODULES:
                    offenders.append(f"{path}:{node.lineno}: from {node.module} import ...")
    assert not offenders, "ROS imports found in P1 core:\n" + "\n".join(offenders)


def test_p1_core_imports_without_a_ros_environment():
    """The core must work in a plain venv with no ROS sourced at all."""
    import importlib

    for name in ("fleet_core", "fleet_core.domain", "fleet_core.ledger",
                 "fleet_core.resources", "fleet_core.task_machine",
                 "fleet_core.allocator", "fleet_core.clock", "fleet_core.events",
                 "fleet_core.validation"):
        importlib.import_module(name)


# --------------------------------------------------------------------------- #
# fake adapter motion
# --------------------------------------------------------------------------- #


def cmd(generation: int = 1, x: float = 10.0, y: float = 0.0) -> MotionCommand:
    return MotionCommand(task_id="task-A", robot_id="r01", generation=generation,
                         target_x=x, target_y=y)


def test_fake_adapter_labels_itself_fake():
    """A report must be able to say which backend produced it."""
    a = FakeAdapter(robot_id="r01")
    assert a.status().backend == "fake"


def test_fake_adapter_moves_towards_the_target():
    a = FakeAdapter(robot_id="r01", speed_mps=1.0)
    a.start(cmd(x=10.0, y=0.0))
    a.tick(1.0)
    assert a.x == pytest.approx(1.0)
    assert a.status().phase is AdapterPhase.MOVING


def test_fake_adapter_arrives_and_stops():
    a = FakeAdapter(robot_id="r01", speed_mps=1.0)
    a.start(cmd(x=2.0, y=0.0))
    a.tick(5.0)
    st = a.status()
    assert st.phase is AdapterPhase.ARRIVED
    assert a.x == pytest.approx(2.0)
    assert st.speed_mps == 0.0


def test_fake_adapter_zero_distance_target_arrives_immediately():
    a = FakeAdapter(robot_id="r01", x=1.0, y=1.0)
    a.start(cmd(x=1.0, y=1.0))
    a.tick(0.1)
    assert a.status().arrived is True


def test_negative_dt_is_rejected():
    a = FakeAdapter(robot_id="r01")
    with pytest.raises(ValueError):
        a.tick(-0.1)


# --------------------------------------------------------------------------- #
# cancel semantics -- the distinction the whole design rests on
# --------------------------------------------------------------------------- #


def test_cancel_acknowledges_without_claiming_stopped():
    a = FakeAdapter(robot_id="r01", speed_mps=1.0)
    a.start(cmd(generation=3, x=10.0))
    a.tick(1.0)

    assert a.cancel(generation=3) is True
    st = a.status()
    assert st.phase is AdapterPhase.STOPPING
    assert st.stopped_confirmed is False, "acknowledging a cancel is not proof of stopping"


def test_stopped_confirmed_only_after_the_stop_completes():
    a = FakeAdapter(robot_id="r01")
    a.start(cmd(generation=3))
    a.cancel(generation=3)
    st = a.tick(0.1)
    assert st.phase is AdapterPhase.STOPPED
    assert st.stopped_confirmed is True


def test_cancel_with_a_stale_generation_is_refused():
    a = FakeAdapter(robot_id="r01")
    a.start(cmd(generation=5))
    assert a.cancel(generation=4) is False
    assert a.status().phase is AdapterPhase.MOVING


def test_new_command_supersedes_the_old_generation():
    a = FakeAdapter(robot_id="r01")
    a.start(cmd(generation=1, x=10.0))
    a.start(cmd(generation=2, x=-10.0))
    assert a.status().generation == 2
    # The old cancel must no longer take effect.
    assert a.cancel(generation=1) is False


def test_shutdown_does_not_claim_the_robot_stopped():
    """A dead adapter is UNKNOWN, not safe: the last velocity was never retracted."""
    a = FakeAdapter(robot_id="r01")
    a.start(cmd())
    a.shutdown()
    st = a.status()
    assert st.phase is AdapterPhase.FAULTED
    assert "NOT retracted" in st.message


def test_faulted_adapter_refuses_new_commands():
    a = FakeAdapter(robot_id="r01")
    a.inject_fault("nav server gone")
    a.start(cmd(generation=9))
    assert a.status().generation == 0, "a faulted adapter must not adopt a new command"


def test_distance_travelled_accumulates():
    a = FakeAdapter(robot_id="r01", speed_mps=1.0)
    a.start(cmd(x=3.0))
    a.tick(1.0)
    a.tick(1.0)
    assert a.distance_travelled_m == pytest.approx(2.0)
