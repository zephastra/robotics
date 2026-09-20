"""P10: ONE boundary. The refusal and the clearance proof must not disagree about it.

THE DEFECT
----------
Two components that must agree about one boundary were computing it from two expressions::

    gate refusal      `_unpermitted_hits`   rect + reserve(speed) + footprint_margin
    clearance proof   `confirm_clear`       rect +                  footprint_margin

``reserve`` is speed-dependent -- 0.323 m at the 0.35 m/s this stack's stop model was fitted
to, 0.100 m at rest -- so the passage driver's "cleared" was strictly weaker than the gate's
"clear", and ``ConfirmClear`` could return ``cleared=True`` for a pose the gate refuses in the
same instant. Worse, the release node's placement was derived from the weaker one::

    documented derivation   buffer_outer 2.85 + margin 0.15 + half_extent 0.30 + tol 0.80 = 4.10
    the enforced boundary   buffer_outer 2.85 + reserve 0.323 + margin 0.15 + half_extent 0.30
                            + tol 0.80 = 4.423

so ``release_east``/``release_west`` at 4.2 satisfied a boundary that was not being enforced.
Live evidence: the gate refused with ``unpermitted ['mid_right'], holdings none
(reserve 0.356 m)`` while the driver's own ``confirm_clear`` stage had just reported success.

WHAT IS ASSERTED, AND WHY EACH ONE IS FALSIFIABLE
-------------------------------------------------
1. **the two call sites use one function** -- by AST, so a re-introduction of a second
   expression fails the suite instead of a simulation run. (A substring assertion would not
   survive this round's own history: an explanatory comment naming the old call defeated three
   such assertions.)
2. **the boundary is monotone in speed** -- so a clearance proved at the worst measured speed
   cannot be falsified by accelerating.
3. **the measured speed is derived from the measurement table**, so the number in the clearance
   budget cannot drift from the number the stop model was fitted to.
4. **a pose that is clear of the bundle is never refused for it** -- the property itself,
   swept over positions, at the speed the gate may authorise.
5. **the release node satisfies the boundary that is actually enforced**, including the arrival
   tolerance, so a crossing that worked cannot still be refused at the last step.
"""

from __future__ import annotations

import ast
import re
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
SRC = ROOT / "src"
for _pkg in ("fleet_core", "fleet_adapter"):
    _p = SRC / _pkg
    if str(_p) not in sys.path:
        sys.path.insert(0, str(_p))

from fleet_core import CrossingManager, ResourceState, load_traffic_config  # noqa: E402
from fleet_core.geometry import footprint_corners  # noqa: E402

CONFIG = ROOT / "config" / "resources.yaml"
TRAFFIC_SRC = SRC / "fleet_core" / "fleet_core" / "traffic.py"

# r01 / r02 footprint, from config/fleet.yaml.
L, W = 0.60, 0.45

DIRECTIONS = ("west_to_east", "east_to_west")


@pytest.fixture(scope="module")
def tcfg():
    return load_traffic_config(CONFIG)


# --------------------------------------------------------------------------- #
# reading code as code
# --------------------------------------------------------------------------- #


def _method(cls: str, name: str) -> ast.FunctionDef:
    tree = ast.parse(TRAFFIC_SRC.read_text(encoding="utf-8"))
    for node in ast.walk(tree):
        if isinstance(node, ast.ClassDef) and node.name == cls:
            for item in node.body:
                if isinstance(item, ast.FunctionDef) and item.name == name:
                    return item
    raise AssertionError(f"{cls}.{name} is not in {TRAFFIC_SRC}")


def _called_names(node: ast.AST) -> set[str]:
    names: set[str] = set()
    for n in ast.walk(node):
        if isinstance(n, ast.Call):
            if isinstance(n.func, ast.Attribute):
                names.add(n.func.attr)
            elif isinstance(n.func, ast.Name):
                names.add(n.func.id)
    return names


def test_the_refusal_and_the_clearance_proof_use_one_function():
    """The specific defect, as a structural property of the code that runs.

    A substring check is not enough and this file says why: three assertions elsewhere in this
    session were defeated by an explanatory comment containing the very name they forbade.
    """
    for name in ("stop_boundary", "_unpermitted_hits"):
        called = _called_names(_method("CrossingManager", name))
        assert "protected_region" in called, (
            f"CrossingManager.{name} builds its own boundary again instead of calling "
            "protected_region; that is how the refusal and the clearance proof came to "
            "disagree by a speed-dependent reserve")

    called = _called_names(_method("CrossingManager", "confirm_clear"))
    assert "clearance_region" in called, (
        "confirm_clear does not use the boundary being enforced")
    assert "region_of" not in called, (
        "confirm_clear is computing its own boundary again: `cleared` must not be provable "
        "against a boundary the gate does not use")


# --------------------------------------------------------------------------- #
# the arithmetic
# --------------------------------------------------------------------------- #


def test_the_clearance_boundary_contains_the_refusal_boundary_up_to_the_measured_speed(tcfg):
    """Monotone in speed, which is what makes `cleared` imply `not refused`.

    `envelope_m` grows with speed, so a clearance proved at the speed a robot happens to have
    is falsified by accelerating. Proving it at the fastest MEASURED speed removes the gap -- up
    to that speed and no further, which is what `_over_the_measured_range` is.

    The sweep deliberately runs PAST the measured speed first, so the bound is demonstrated
    rather than assumed: at 0.60 m/s the containment genuinely fails, and a future edit that
    quietly claims a wider range has to deal with this test.
    """
    mgr = CrossingManager(tcfg)
    limit = tcfg.stop.max_measured_speed_mps

    def contained(speed: float) -> tuple[bool, str]:
        for direction in DIRECTIONS:
            names = mgr.bundle(direction)
            big = mgr.clearance_region(names)
            small = mgr.protected_region(names, speed_mps=speed)
            assert len(small.rects) == len(big.rects)
            for s, b in zip(small.rects, big.rects):
                if not (b.x_min <= s.x_min + 1e-9 and b.x_max >= s.x_max - 1e-9
                        and b.y_min <= s.y_min + 1e-9 and b.y_max >= s.y_max - 1e-9):
                    return False, direction
        return True, ""

    for speed in (0.0, 0.05, 0.15, limit * 0.5, limit):
        ok, direction = contained(speed)
        assert ok, (
            f"{direction} at {speed} m/s: the refusal boundary is not inside the clearance "
            "boundary, so a cleared pose can still be refused")

    beyond = limit * 1.7
    ok, _ = contained(beyond)
    assert not ok, (
        f"the containment held at {beyond:.2f} m/s, above the fastest speed the stop model was "
        "measured at. Either the model was re-measured and this bound is now wrong, or the "
        "clearance speed is no longer derived from the measurement table")


def test_the_gate_cannot_command_faster_than_the_speed_the_stop_model_was_measured_at(tcfg):
    """The bound above is only respected if the gate's own clamp is inside it.

    The clearance boundary is monotone up to `max_measured_speed_mps`; past it the refusal
    boundary is larger again and `cleared` stops implying `not refused`. So the guarantee is
    conditional on the gate's clamp, and the clamp is a literal in a different file from the
    measurement. Nothing but a test can tie the two together, so here it is.
    """
    launch = (SRC / "fleet_bringup" / "launch" / "robot.launch.py").read_text(encoding="utf-8")
    match = re.search(r'"max_linear_mps":\s*([0-9.]+)', launch)
    assert match, (
        "the gate's speed clamp is no longer a literal `\"max_linear_mps\": <number>` in "
        "robot.launch.py, so this tie cannot be checked; keep it a literal or move the check")
    clamp = float(match.group(1))
    assert clamp <= tcfg.stop.max_measured_speed_mps + 1e-9, (
        f"the gate may command {clamp} m/s but the stop model was measured only up to "
        f"{tcfg.stop.max_measured_speed_mps} m/s. Above the measured speed the refusal "
        "boundary is larger than the clearance boundary, so a pose ConfirmClear calls clear "
        "can be refused. Either measure the stop distance at the faster speed or lower the "
        "clamp.")


def test_the_clearance_speed_is_the_fastest_speed_the_stop_model_was_measured_at(tcfg):
    """Derived from the table, not typed twice.

    The largest key of `reported_stop_distance_m` is a term in the release node's clearance
    budget. If it moves, the budget and the node positions must be re-derived -- so this fails
    rather than letting the two drift.
    """
    raw = tcfg.raw["traffic"]["stop_model"]["reported_stop_distance_m"]
    expected = max(float(k) for k in raw)
    assert tcfg.stop.max_measured_speed_mps == pytest.approx(expected)
    assert expected == pytest.approx(0.35), (
        "the stop measurement table's largest speed changed; the clearance budget in "
        "config/resources.yaml and the release node positions are derived from it")
    assert not raw == {}, "an empty measurement table would silently disable the reserve"


def test_the_shipped_config_actually_carries_a_measured_speed(tcfg):
    """The fallback exists for fixtures, and must not be reachable from the shipped file."""
    assert tcfg.stop.max_measured_speed_mps > 0.0, (
        "config/resources.yaml declares no measured speed, so the clearance proof would fall "
        "back to the footprint margin alone -- the exact boundary mismatch this file is about")


# --------------------------------------------------------------------------- #
# the property
# --------------------------------------------------------------------------- #


def test_a_pose_that_is_clear_of_the_bundle_is_never_refused_for_it(tcfg):
    """`cleared` implies `not refused`, swept over positions, at the gate's own speed.

    Every pose on the release row that the clearance proof calls clear must also not be refused
    for that bundle. The refusal runs at 0.35 m/s, which is the speed the clearance boundary is
    built at, so the two must agree everywhere on the row -- not only at the node.
    """
    mgr = CrossingManager(tcfg)
    mgr._state.update({n: ResourceState.FREE for n in mgr.cfg.resources})
    spec = mgr.direction_spec("west_to_east")
    release = mgr.node(spec.release_node)
    names = mgr.bundle("west_to_east")
    region = mgr.clearance_region(names)

    checked = 0
    for i in range(0, 121):
        x = 2.0 + i * 0.05
        corners = footprint_corners(x, release.y, release.yaw, L, W)
        if region.box_overlap(corners):
            continue
        check = mgr.check_movement(
            task_id="task-none", boot_id="b1", revision=0, generation=1, epoch=0,
            wall_now=0.0, pose=(x, release.y, release.yaw), twist=(0.35, 0.0, 0.0),
            length_m=L, width_m=W, localization_valid=True,
        )
        refused_for = sorted(set(check.stop_boundary_hit) & set(names))
        assert not refused_for, (
            f"x={x:.2f} is clear of the bundle against the boundary ConfirmClear uses, but "
            f"the gate refuses it for {refused_for}. `cleared` does not imply `not refused`")
        checked += 1

    assert checked >= 20, f"only {checked} poses exercised the property; the sweep is broken"


def test_the_release_node_satisfies_the_boundary_that_is_actually_enforced(tcfg):
    """Including the arrival tolerance, because clearing for a perfect arrival is not clearing."""
    mgr = CrossingManager(tcfg)
    for direction in DIRECTIONS:
        names = mgr.bundle(direction)
        region = mgr.clearance_region(names)
        release = mgr.node(mgr.direction_spec(direction).release_node)

        assert not region.box_overlap(
            footprint_corners(release.x, release.y, release.yaw, L, W)), (
            f"{direction}: the release node is inside the enforced boundary")

        sign = 1.0 if release.x >= 0 else -1.0
        short = release.x - sign * tcfg.node_reach_tolerance_m
        assert not region.box_overlap(
            footprint_corners(short, release.y, release.yaw, L, W)), (
            f"{direction}: a robot that stops {tcfg.node_reach_tolerance_m:.2f} m short of the "
            "release node -- which is inside the arrival tolerance, so a legitimate arrival -- "
            "is inside the enforced boundary. It would clear the old, weaker check and then be "
            "refused by the gate for the resource it had just been allowed to leave.")
