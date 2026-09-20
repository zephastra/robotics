"""P4.6: the epoch contract, and lifting a resource out of UNKNOWN.

Every test here exists because of one specific live two-robot run
(``p4_acceptance_20260915T155904Z``), and each one is a regression that is silent if
it comes back:

* ``test_the_coordinator_stamps_its_own_epoch_on_a_grant`` -- the coordinator passed
  the *caller's* epoch into the lease book while comparing against its own. The
  driver sent 0, the coordinator's epoch is 1, so every grant it made was invisible
  to its own permit publisher. The wire carried ``granted=false`` for a whole run,
  the gate's mirror stayed empty, the robot's own gate stopped it, and Nav2 timed out
  crossing an empty corridor. The run logged 37 successful renewals the entire time.
* ``test_a_lapsed_bundle_can_be_reverified_from_fresh_poses`` -- nothing could lift a
  resource out of UNKNOWN after a permit lapsed, so one aborted crossing made the
  corridor unusable for the rest of the session.

The epoch test reads the ROS source rather than importing it, because the defect was
never in the arithmetic -- it was in *which* epoch the coordinator chose to write
down, and the only way to pin that is to look.

The last test in this file documents behaviour that is deliberately NOT fixed. It is
written to fail if someone "improves" the gate without doing the design work first.
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

CONFIG = ROOT / "config" / "resources.yaml"
COORDINATOR = SRC / "fleet_ros" / "fleet_ros" / "coordinator_node.py"
ACQUIRE_SRV = SRC / "fleet_interfaces" / "srv" / "AcquirePassage.srv"
DRIVER = SRC / "fleet_ros" / "fleet_ros" / "staged_crossing.py"
INSTALLED_COORDINATOR = (ROOT / "install" / "fleet_ros" / "lib" / "python3.14"
                         / "site-packages" / "fleet_ros" / "coordinator_node.py")

# r01 / r02 footprint, from config/fleet.yaml.
L, W = 0.60, 0.45

WEST_TO_EAST = "west_to_east"
EAST_TO_WEST = "east_to_west"

# What the coordinator's `_epoch` is set to. A grant is scoped to it, and the whole
# point of the fix is that the caller never gets to choose it.
COORD_EPOCH = 1

# A pose well inside the corridor rectangle (x in [-1.75, 1.75], y in [-0.65, 0.65]),
# so that a scan has something to find. No configured node sits inside -- that is the
# entry precondition -- so an interior pose has to be written out explicitly.
INSIDE_CORRIDOR = (0.0, 0.0, 0.0)


# --------------------------------------------------------------------------- #
# helpers
# --------------------------------------------------------------------------- #


@pytest.fixture(scope="module")
def tcfg():
    return load_traffic_config(CONFIG)


def healthy(tcfg) -> CrossingManager:
    """A manager whose crossing has been checked, the way the node checks it.

    Goes through ``mark_verified_free`` rather than poking the state dict, so a change
    to the FREE precondition cannot pass this suite.
    """
    mgr = CrossingManager(tcfg)
    for name in tcfg.resources:
        mgr.mark_verified_free(name, source="test: pre-run geometry check")
    return mgr


def node_pose(tcfg, name: str) -> tuple[float, float, float]:
    n = tcfg.nodes[name]
    return (n.x, n.y, n.yaw)


def ask(mgr: CrossingManager, direction: str, *, task: str = "task-A",
        robot: str = "r01", gen: int = 1, epoch: int = COORD_EPOCH, wall_now: float = 0.0):
    spec = mgr.direction_spec(direction)
    return mgr.acquire(
        direction, task_id=task, robot_id=robot, boot_id=f"boot-{robot}", revision=0,
        generation=gen, epoch=epoch, now=wall_now, wall_now=wall_now,
        pose=node_pose(mgr.cfg, spec.wait_node), length_m=L, width_m=W,
        localization_valid=True,
    )


def outside_scan(mgr: CrossingManager, poses_over: dict | None = None):
    """Both robots parked at their own waiting points, which are outside everything."""
    poses = {
        "r01": node_pose(mgr.cfg, "wait_west"),
        "r02": node_pose(mgr.cfg, "wait_east"),
    }
    poses.update(poses_over or {})
    return mgr.verified_clear(poses, {"r01": (L, W), "r02": (L, W)})


def _function_body(source: str, name: str) -> str:
    """Slice one method body out, from ``def name(`` to the next top-level ``def``."""
    marker = f"    def {name}("
    start = source.index(marker)
    rest = source[start + len(marker):]
    nxt = rest.find("\n    def ")
    return rest[:nxt] if nxt >= 0 else rest


# --------------------------------------------------------------------------- #
# the epoch contract
# --------------------------------------------------------------------------- #


def test_a_grant_is_scoped_to_the_epoch_it_was_stamped_with(tcfg):
    """The rule the coordinator broke. Stated where it can be run."""
    mgr = healthy(tcfg)
    res = ask(mgr, WEST_TO_EAST)
    assert res.granted is True
    assert res.permit.epoch == COORD_EPOCH

    held = mgr.held_resources(task_id="task-A", boot_id="boot-r01", revision=0,
                              generation=1, epoch=COORD_EPOCH, wall_now=0.0)
    assert set(held) == set(mgr.bundle(WEST_TO_EAST))

    # And the other direction of the same rule: a permit stamped under one epoch must
    # not be visible under another. Otherwise "invisible grant" and "stale grant" are
    # indistinguishable, and the check above is decoration.
    assert mgr.held_resources(task_id="task-A", boot_id="boot-r01", revision=0,
                              generation=1, epoch=COORD_EPOCH + 1, wall_now=0.0) == ()


def test_the_coordinator_stamps_its_own_epoch_on_a_grant():
    """Source-level, because the bug was a choice of value, not broken arithmetic.

    ``_srv_acquire`` used to forward ``int(req.epoch)`` into the lease book. That made
    the epoch a value the supervised party chose, which cannot detect a restarted
    supervisor -- it can only switch the check off. It did: the driver sent 0, the
    book recorded 0, ``_live_permit`` compared against 1, and every grant the
    coordinator made was reported to every gate as ``granted=false``.
    """
    body = _function_body(COORDINATOR.read_text(encoding="utf-8"), "_srv_acquire")
    assert "epoch=self._epoch" in body, (
        "acquire must stamp the coordinator's own epoch; a grant scoped to an epoch "
        "the caller chose is invisible to this node's own permit publisher")
    assert "req.epoch" not in body, (
        "acquire must not read an epoch from the request: the supervised party cannot "
        "be the authority on which coordinator generation it is talking to")
    assert "resp.epoch = self._epoch" in body, (
        "acquire must RETURN the epoch it stamped, or the client has nothing to scope "
        "its renewals to and every renewal is refused as stale.\n"
        "This assertion is inside the sliced body on purpose. `_srv_renew` already "
        "contains `resp.epoch = self._epoch`, so a file-wide search for that string "
        "passes while this line is missing -- which is exactly how it shipped once, "
        "with the patcher's idempotency marker and its self-check both fooled by the "
        "same non-unique string.")


def test_the_acquire_request_has_no_epoch_field_left_to_misuse():
    """The structural half of the fix: remove the input, not just the bad read."""
    text = ACQUIRE_SRV.read_text(encoding="utf-8")
    request, sep, response = text.partition("\n---\n")
    assert sep, "AcquirePassage.srv has no request/response separator"
    assert "int32 epoch" not in request, (
        "AcquirePassage.Request must not carry an epoch: a caller-chosen epoch can "
        "only ever disable the check it is supposed to enforce")
    assert "int32 epoch" in response, (
        "AcquirePassage.Response must return the epoch, or a client cannot scope its "
        "own renewals")


def test_the_module_the_nodes_actually_import_carries_the_epoch_contract():
    """The check that would have caught this a whole simulation run earlier.

    ``build.sh`` uses ``--symlink-install``, so the installed module is a symlink to the
    source and this normally re-tests the same file. That is the point: it fails loudly
    if that assumption ever stops holding, and it is the copy the running nodes import
    -- not the one a developer has open in an editor.
    """
    if not INSTALLED_COORDINATOR.exists():
        pytest.skip("workspace not built; nothing is installed yet")

    body = _function_body(INSTALLED_COORDINATOR.read_text(encoding="utf-8"), "_srv_acquire")
    assert "epoch=self._epoch" in body, "installed acquire does not stamp its own epoch"
    assert "resp.epoch = self._epoch" in body, (
        "the installed coordinator does not return the epoch from acquire, so every "
        "client renewal is refused as stale")


def test_reverification_cannot_outrun_the_start_up_sweep():
    """Re-verification may lift UNKNOWN, but never *first* declare a corridor free.

    The sweep waits ``occupancy_settle_s`` for the robots to settle, because a first
    sample taken while a model is still dropping is not a position. The re-verification
    timer starts immediately, so without the guard below it uses exactly the evidence the
    sweep declines, and reaches the conclusion first. A live run did that: the coordinator
    logged ``re-verify (automatic): promoted ['mid','mid_left','mid_right']`` at t=7.3 s
    and ``start-up sweep: ...`` at t=7.8 s -- and the earlier docstring claimed this could
    not happen.
    """
    src = COORDINATOR.read_text(encoding="utf-8")

    auto = _function_body(src, "_auto_reverify")
    assert "if not self._sweep_done:" in auto, (
        "the automatic pass must return while the start-up sweep is outstanding")
    assert "self._reverify_unknown(" in auto, (
        "and must still run the pass once the sweep is done, or recovery is lost")

    requested = _function_body(src, "_srv_reverify")
    assert '"refused": "start-up occupancy sweep has not completed"' in requested, (
        "an on-demand pass before the sweep must say so rather than promote, or a caller "
        "cannot tell 'nothing to promote' from 'not ready yet'")


# --------------------------------------------------------------------------- #
# verified clearance
# --------------------------------------------------------------------------- #


def test_a_complete_observation_of_empty_rectangles_is_freeable(tcfg):
    mgr = healthy(tcfg)
    scan = outside_scan(mgr)
    assert scan.complete is True
    assert scan.unlocatable == ()
    assert set(scan.freeable) == set(tcfg.resources)
    assert scan.occupied == ()


def test_a_robot_inside_a_rectangle_makes_it_occupied(tcfg):
    """The reason re-verification is not "just clear it and carry on"."""
    mgr = healthy(tcfg)
    scan = outside_scan(mgr, {"r01": INSIDE_CORRIDOR})
    assert scan.complete is True
    assert "mid" not in scan.freeable
    assert ("mid", ("r01",)) in scan.occupied


def test_one_unlocatable_robot_promotes_nothing(tcfg):
    """Fail closed, and say which robot is the reason.

    A missing pose is "we do not know where r02 is", which is not evidence that r02 is
    somewhere else. A scan that returned an empty ``freeable`` list instead would be
    indistinguishable from "the scan ran and everything was held".
    """
    mgr = healthy(tcfg)
    scan = outside_scan(mgr, {"r02": None})
    assert scan.complete is False
    assert scan.unlocatable == ("r02",)
    assert scan.freeable == ()


def test_a_robot_missing_from_the_observation_counts_as_unlocatable(tcfg):
    """Omitting a robot must not look the same as that robot being elsewhere."""
    mgr = healthy(tcfg)
    scan = mgr.verified_clear({"r01": node_pose(tcfg, "wait_west")},
                              {"r01": (L, W), "r02": (L, W)})
    assert scan.complete is False
    assert scan.unlocatable == ("r02",)
    assert scan.freeable == ()


def test_a_held_resource_is_never_freeable(tcfg):
    """A live permit authorises somebody to be inside, so poses cannot clear it."""
    mgr = healthy(tcfg)
    assert ask(mgr, WEST_TO_EAST).granted is True

    scan = outside_scan(mgr)
    assert scan.complete is True
    assert set(scan.held) == set(mgr.bundle(WEST_TO_EAST))
    assert "mid" in scan.held and "mid" not in scan.freeable


def test_the_candidate_filter_does_not_widen_clearance(tcfg):
    """``only`` narrows the question; it must never answer a different one."""
    mgr = healthy(tcfg)
    only_mid = mgr.verified_clear(
        {"r01": node_pose(tcfg, "wait_west"), "r02": node_pose(tcfg, "wait_east")},
        {"r01": (L, W), "r02": (L, W)},
        only=("mid",))
    assert only_mid.freeable == ("mid",)
    assert "mid_right" not in only_mid.freeable
    assert "mid_left" not in only_mid.freeable


def test_marking_a_resource_verified_free_also_clears_the_lease_book(tcfg):
    """Two answers to one question is the failure this whole module exists to prevent.

    Found by this suite rather than by reasoning. ``mark_verified_free`` set this
    manager's state to FREE and left the lease book's own UNKNOWN mark set, and
    ``ResourceBook.acquire`` refuses anything still marked unknown. So a re-verified
    resource read as FREE and stayed unacquirable: the new re-verification pass would
    have reported success and changed nothing at all.

    Nothing noticed before, because the only previous caller was the start-up sweep,
    which runs before anything has been marked unknown.
    """
    mgr = healthy(tcfg)
    assert ask(mgr, WEST_TO_EAST).granted is True
    mgr.expire_due(wall_now=1e6)

    # Lapsed: the book refuses it, not just this manager's state.
    lapsed = ask(mgr, EAST_TO_WEST, task="task-B", robot="r02", wall_now=1e6)
    assert lapsed.granted is False
    assert lapsed.reason is ReasonCode.RESOURCE_UNKNOWN

    for name in ("mid", "mid_left", "mid_right"):
        mgr.mark_verified_free(name, source="test: complete observation, nobody inside")

    # Same request, same clock, one verified clearance later: it must succeed.
    after = ask(mgr, EAST_TO_WEST, task="task-B", robot="r02", wall_now=1e6)
    assert after.granted is True, (
        f"a resource this manager reports as FREE was still refused by the lease book: "
        f"{after.reason} blocked_by={after.blocked_by}")


# --------------------------------------------------------------------------- #
# the gap this closes
# --------------------------------------------------------------------------- #


def test_a_lapsed_bundle_can_be_reverified_from_fresh_poses(tcfg):
    """One aborted crossing used to brick the corridor for the rest of the session.

    The sequence below is the one the live run produced: grant, lapse, UNKNOWN. In
    that run nothing could lift it -- ``ConfirmClear`` refuses on an UNKNOWN resource
    by design and the start-up sweep runs once -- so the second case could never
    acquire and the fleet was stuck for good. Re-verification is the way out, and it
    has to be as strict as the sweep, not looser.
    """
    mgr = healthy(tcfg)
    assert ask(mgr, WEST_TO_EAST).granted is True
    assert mgr.state_of("mid") is ResourceState.RESERVED

    mgr.expire_due(wall_now=1e6)
    assert mgr.state_of("mid") is ResourceState.UNKNOWN
    assert mgr.state_of("mid_right") is ResourceState.UNKNOWN

    # While the robot is still inside the corridor, the corridor is not promoted. This
    # is the case a clock-trusting re-verification would reopen, and the one that must
    # stay shut.
    #
    # `mid_right` IS promoted in the same scan, and that is correct rather than an
    # oversight: the east exit buffer is a different rectangle, nobody is in it, and no
    # permit is held on it. State tracks occupancy per rectangle; only GRANTING is
    # atomic per bundle, and `acquire` still refuses a bundle with any member not FREE.
    # So a lone FREE member is inert -- it cannot be used without its partner.
    inside = mgr.verified_clear(
        {"r01": INSIDE_CORRIDOR, "r02": node_pose(tcfg, "wait_east")},
        {"r01": (L, W), "r02": (L, W)},
        only=("mid", "mid_right"))
    assert inside.complete is True
    assert "mid" not in inside.freeable
    assert ("mid", ("r01",)) in inside.occupied
    assert inside.freeable == ("mid_right",)

    # Once it is genuinely elsewhere, the same observation clears the corridor.
    scan = mgr.verified_clear(
        {"r01": node_pose(tcfg, "wait_west"), "r02": node_pose(tcfg, "wait_east")},
        {"r01": (L, W), "r02": (L, W)},
        only=("mid", "mid_right"))
    assert scan.complete is True
    assert set(scan.freeable) == {"mid", "mid_right"}

    for name in scan.freeable:
        mgr.mark_verified_free(
            name, source="test: complete observation, nobody inside, no permit held")
    assert mgr.state_of("mid") is ResourceState.FREE

    # And the whole point: somebody else can now get in.
    other = ask(mgr, EAST_TO_WEST, task="task-B", robot="r02")
    assert other.granted is True, other.detail


# --------------------------------------------------------------------------- #
# a limitation, pinned on purpose
# --------------------------------------------------------------------------- #


def test_a_robot_that_lapses_inside_the_bundle_has_no_legal_way_out(tcfg):
    """NOT FIXED. Written down so it cannot be mistaken for an oversight.

    The gate refuses motion into a rectangle this robot holds no permit for, and it
    does not distinguish *entering* from *leaving*. So a robot whose permit lapses
    while it is inside the corridor is stopped where it stands, and cannot re-acquire
    either: ``acquire`` requires being at a legal asking point with the whole region
    clear, which is precisely what being inside is not.

    Re-verification cannot help here and must not pretend to: the robot really is in
    the rectangle, so the corridor stays UNKNOWN. That is correct behaviour with an
    unfinished recovery story behind it -- the fleet stays blocked until something
    physically moves that robot.

    Why it did not block the observed run: the driver renews every few seconds against a
    12 s TTL, so a lapse-inside needs the driver to die or the coordinator to become
    unreachable. In the recorded failure the robot was stopped *outside* the mouth, and
    re-verification does cover that.

    Proposed, deliberately NOT implemented here: a **retreat permission** -- an
    allowance for a robot already inside a rectangle to move, valid only while the
    commanded motion strictly reduces its overlap with the protected region. That is a
    change to the safety layer, and it needs its own design, tests and live run.
    ``docs/DECISIONS.md`` D-P4-12 records it. Until then this scenario is NOT_RUN.
    """
    mgr = healthy(tcfg)
    assert ask(mgr, WEST_TO_EAST).granted is True
    mgr.expire_due(wall_now=1e6)

    wall = 1e6 + 1.0
    assert mgr.held_resources(task_id="task-A", boot_id="boot-r01", revision=0,
                              generation=1, epoch=COORD_EPOCH, wall_now=wall) == ()

    check = mgr.check_movement(
        task_id="task-A", boot_id="boot-r01", revision=0, generation=1,
        epoch=COORD_EPOCH, wall_now=wall, pose=INSIDE_CORRIDOR,
        twist=(0.35, 0.0, 0.0), length_m=L, width_m=W, localization_valid=True)
    assert check.allowed is False, (
        "if this ever passes, a retreat rule has been added -- and it needs its own "
        "tests for direction, magnitude and the case where retreat and entry are the "
        "same command")
    # Both are ordinary outcomes of the same event: it held the bundle and no longer
    # does. Asserting the family rather than one member keeps the test about the gap.
    assert check.reason in (ReasonCode.PERMIT_EXPIRED, ReasonCode.PERMIT_STALE)
    assert check.stop_boundary_hit == ("mid",)

    # And the corridor stays UNKNOWN rather than being promoted out from under it.
    scan = outside_scan(mgr, {"r01": INSIDE_CORRIDOR})
    assert scan.complete is True
    assert "mid" not in scan.freeable
    assert mgr.state_of("mid") is ResourceState.UNKNOWN
