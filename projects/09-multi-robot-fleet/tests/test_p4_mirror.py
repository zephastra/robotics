"""P4.2 mirror support: adopting a permit granted elsewhere.

A robot's gate must judge from ITS OWN pose, so it keeps its own copy of the traffic
geometry. That copy needs to know what the coordinator granted -- without the robot
pretending it did the granting. These tests cover the two ways that goes wrong: a
stale mirror outliving a renewal, and a mirror inheriting a life it was not given.
"""

from __future__ import annotations

import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
SRC = ROOT / "src"
for _pkg in ("fleet_core", "fleet_adapter"):
    _p = SRC / _pkg
    if str(_p) not in sys.path:
        sys.path.insert(0, str(_p))

from fleet_core import (  # noqa: E402
    CrossingManager,
    ReasonCode,
    ResourceState,
    load_traffic_config,
)
from fleet_core.domain import Permit  # noqa: E402

CONFIG = ROOT / "config" / "resources.yaml"
L, W = 0.60, 0.45


@pytest.fixture(scope="module")
def tcfg():
    return load_traffic_config(CONFIG)


def fresh(tcfg) -> CrossingManager:
    """A mirror manager: nothing is verified free, exactly like a robot at boot."""
    return CrossingManager(tcfg)


def node_pose(cfg, name):
    n = cfg.nodes[name]
    return (n.x, n.y, n.yaw)


def adopt(mgr, *, direction="west_to_east", task="task-A", generation=1, epoch=1,
          expires_at_wall=1000.0, boot_id="boot-1"):
    mgr.adopt_permit(
        permit_id=f"permit-{task}-{generation}", task_id=task, robot_id="r01",
        direction=direction, boot_id=boot_id, revision=0, generation=generation,
        epoch=epoch, expires_at_wall=expires_at_wall,
    )


def move(mgr, *, task="task-A", generation=1, epoch=1, pose=(0.0, 0.0, 0.0),
         twist=(0.0, 0.0, 0.0), wall_now=100.0):
    return mgr.check_movement(
        task_id=task, boot_id="boot-1", revision=0, generation=generation, epoch=epoch,
        wall_now=wall_now, pose=pose, twist=twist, length_m=L, width_m=W,
        localization_valid=True,
    )


# --------------------------------------------------------------------------- #
# install
# --------------------------------------------------------------------------- #


def test_install_refuses_a_permit_with_no_resources(tcfg):
    mgr = fresh(tcfg)
    with pytest.raises(ValueError):
        mgr.book.install(Permit(
            permit_id="p", task_id="t", robot_id="r01", resources=(),
            boot_id="b", revision=0, generation=1, epoch=1,
            granted_at=0.0, expires_at=1e9))


def test_install_clears_unknown_so_the_permit_is_usable(tcfg):
    """A mirror never ran a start-up sweep, so its resources are UNKNOWN.

    Without this the freshly installed permit would be blocked by the very
    `RESOURCE_UNKNOWN` rule it was meant to satisfy.
    """
    mgr = fresh(tcfg)
    assert mgr.state_of("mid") is ResourceState.UNKNOWN
    adopt(mgr)
    assert mgr.state_of("mid") is ResourceState.RESERVED
    assert move(mgr).allowed is True


def test_install_replaces_a_previous_hold_for_the_same_task(tcfg):
    """A renewal must not leave the superseded permit behind."""
    mgr = fresh(tcfg)
    adopt(mgr, generation=1, expires_at_wall=200.0)
    first = mgr.book.holders(mgr.rid("mid"))
    assert len(first) == 1

    adopt(mgr, generation=2, expires_at_wall=900.0)
    holds = mgr.book.holders(mgr.rid("mid"))
    assert len(holds) == 1, "the old mirror survived the renewal"
    assert holds[0].permit.generation == 2
    assert holds[0].permit.expires_at == 900.0


# --------------------------------------------------------------------------- #
# the mirror drives the same decision
# --------------------------------------------------------------------------- #


def test_a_mirrored_permit_authorises_the_same_crossing_as_a_real_one(tcfg):
    mgr = fresh(tcfg)
    adopt(mgr)
    for pose in (node_pose(tcfg, "align_west"), (0.0, 0.0, 0.0), node_pose(tcfg, "exit_east")):
        assert move(mgr, pose=pose).allowed is True, f"mirror refused a permitted pose {pose}"


def test_a_mirror_does_not_authorise_the_other_side(tcfg):
    """Per-resource provenance survives the mirror path too."""
    mgr = fresh(tcfg)
    adopt(mgr, direction="west_to_east")
    check = move(mgr, pose=node_pose(tcfg, "exit_west"))
    assert check.allowed is False
    assert check.stop_boundary_hit == ("mid_left",)


def test_an_expired_mirror_stops_the_robot(tcfg):
    """The deadline is receiver-side, so a late-arriving message cannot extend it."""
    mgr = fresh(tcfg)
    adopt(mgr, expires_at_wall=150.0)
    assert move(mgr, wall_now=140.0).allowed is True
    check = move(mgr, wall_now=151.0)
    assert check.allowed is False
    assert check.reason is ReasonCode.PERMIT_EXPIRED


def test_a_mirror_from_a_stale_generation_is_refused(tcfg):
    mgr = fresh(tcfg)
    adopt(mgr, generation=1)
    check = move(mgr, generation=2)
    assert check.allowed is False
    assert check.reason is ReasonCode.PERMIT_STALE


def test_a_mirror_from_another_epoch_is_refused(tcfg):
    """A coordinator restart bumps the epoch; old authority must not survive it."""
    mgr = fresh(tcfg)
    adopt(mgr, epoch=1)
    check = move(mgr, epoch=2)
    assert check.allowed is False
    assert check.reason is ReasonCode.PERMIT_STALE


def test_adopting_an_unknown_direction_is_a_programming_error(tcfg):
    mgr = fresh(tcfg)
    with pytest.raises(ValueError, match="unknown direction"):
        adopt(mgr, direction="north_to_south")


def test_without_a_mirror_the_robot_is_stopped_at_the_corridor(tcfg):
    """The negative control: the same pose with no mirror must be refused."""
    mgr = fresh(tcfg)
    check = move(mgr)
    assert check.allowed is False
