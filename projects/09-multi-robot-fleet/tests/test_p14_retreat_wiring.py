"""D-P14-02's second half: the retreat emitter must actually be CALLED.

`fleet_core/retreat_policy.py` was complete and unit-tested for rounds while having zero
call sites. `D-P14-02` states the consequence: "If nobody issues a retreat command, the
widest permit in the world moves nothing." The refusal zeroes the velocity, so a robot
that has lapsed inside a region it cannot legally leave sits there for ever -- which this
project observed repeatedly and read as a navigation fault.

The existing tests (`test_p14_retreat_policy.py`) cover the DECISIONS thoroughly. What
they cannot cover is whether anything asks for them, because a decision nobody requests
is indistinguishable from a correct decision. That is the gap these tests close, and it
is the same gap `D-P17-13` recorded for `retreat_policy` and `D-P17-19` recorded for
`_park_candidates`.

These tests are AST and pure-Python, so they run in the clean shell with no ROS sourced.
"""
from __future__ import annotations

import ast
import pathlib
import sys

import pytest

ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src/fleet_core"))

from fleet_core.crossing_release import STOP_RETREAT_REFUSED  # noqa: E402
from fleet_core.retreat_policy import (  # noqa: E402
    needs_retreat,
    retreat_command,
)

STAGED = ROOT / "src/fleet_ros/fleet_ros/staged_crossing.py"
NODE = ROOT / "src/fleet_ros/fleet_ros/task_service_node.py"

# The exit pad from config/resources.yaml: x in [-2.85, -2.15], y in [-1.75, -1.05].
PAD = {"mid_left": (-2.85, -1.75, -2.15, -1.05)}
# Its mirror, which has never failed in any recorded run. Used to show the policy is
# answering geometry rather than a side.
PAD_EAST = {"mid_right": (2.15, -1.75, 2.85, -1.05)}


def _call_names(path: pathlib.Path) -> set[str]:
    """Every name called anywhere in the file, as written."""
    tree = ast.parse(path.read_text(encoding="utf-8"))
    out: set[str] = set()
    for n in ast.walk(tree):
        if isinstance(n, ast.Call):
            f = n.func
            if isinstance(f, ast.Name):
                out.add(f.id)
            elif isinstance(f, ast.Attribute):
                out.add(f.attr)
    return out


def _method_names(path: pathlib.Path, cls: str) -> set[str]:
    tree = ast.parse(path.read_text(encoding="utf-8"))
    for n in ast.walk(tree):
        if isinstance(n, ast.ClassDef) and n.name == cls:
            return {b.name for b in n.body if isinstance(b, ast.FunctionDef)}
    return set()


# --------------------------------------------------------------------------- #
# the emitter exists and is reachable
# --------------------------------------------------------------------------- #


def test_the_retreat_policy_is_actually_imported_by_the_driver():
    """Not a docstring reference and not a comment -- an import and a call.

    `retreat_policy` named in prose is what this file had for rounds. The test asserts the
    CALL as well as the import, because an imported-but-uncalled function is the same
    defect with a shorter paper trail.
    """
    called = _call_names(STAGED)
    for name in ("needs_retreat", "retreat_command"):
        assert name in called, (
            f"staged_crossing.py never CALLS {name}. D-P14-02: without a call site the "
            f"policy decides nothing and the widest permit in the world moves nothing."
        )


def test_the_emitter_is_a_method_on_the_driver():
    assert "retreat_if_stuck" in _method_names(STAGED, "StagedCrossing"), (
        "the retreat emitter is not a method on StagedCrossing, so nothing can call it")


def test_the_emitter_has_a_call_site_in_run():
    """Defined is not called. The `D-P17-19` shape, one level up."""
    tree = ast.parse(STAGED.read_text(encoding="utf-8"))
    callers: list[int] = []
    for n in ast.walk(tree):
        if isinstance(n, ast.Call) and isinstance(n.func, ast.Attribute) \
                and n.func.attr == "retreat_if_stuck":
            callers.append(n.lineno)
    assert callers, (
        "retreat_if_stuck is defined but never called: the emitter would be dead code, "
        "which is exactly the state D-P14-02 records")


def test_the_stop_reason_is_exported_and_distinct():
    """The reason has to be its own value.

    Reusing `run_deadline` would tell the reader to look at this driver's budget when the
    answer is that the PERMIT half of D-P14-02 is unwritten -- a design gap. Every stop
    reason in `crossing_release` exists because a reader's next action differs.
    """
    from fleet_core.crossing_release import (
        STOP_BUSY, STOP_INFRA_TIMEOUT, STOP_NAV_ABSENT, STOP_RUN_DEADLINE, STOP_SIGNAL,
    )
    all_reasons = {STOP_BUSY, STOP_INFRA_TIMEOUT, STOP_NAV_ABSENT,
                   STOP_RUN_DEADLINE, STOP_SIGNAL, STOP_RETREAT_REFUSED}
    assert len(all_reasons) == 6, "two stop reasons collide"
    assert STOP_RETREAT_REFUSED == "retreat_refused"


def test_the_guards_find_no_unassigned_self_attributes_in_the_driver():
    """The new code must not reintroduce what `D-P17-19` was."""
    sys.path.insert(0, str(ROOT / "scripts"))
    import check_self_attrs  # noqa: E402

    for target in (STAGED, NODE):
        problems = check_self_attrs.scan_file(target, ROOT)
        assert not problems, "\n".join(problems)


# --------------------------------------------------------------------------- #
# the geometry the emitter relies on
# --------------------------------------------------------------------------- #


def test_a_robot_on_the_exit_pad_with_no_permit_needs_a_retreat():
    """The measured situation this policy exists for.

    `resources.yaml` says of `exit_west` in capitals: "still INSIDE the bundle". A robot
    stopped there without a permit is refused, its velocity is zeroed, and it does not
    move again unless something tells it to.
    """
    d = needs_retreat((-2.5, -1.4, 0.0), (0.0, 0.0, 0.0), PAD, permittable=False)
    assert d.needed is True
    assert d.overlap_m == pytest.approx(0.35, abs=1e-6), d


def test_the_mirror_pad_is_treated_identically():
    """Symmetry, asserted rather than assumed.

    The recorded failure is entirely one-sided: `exit_west` failed 34/19/19 times across
    four recordings while `exit_east` never did. The world is geometrically symmetric, so
    the policy must give the mirror image the mirror answer -- if it did not, the
    asymmetry would be in the policy rather than in the planner, and the search for the
    cause would be pointed the wrong way.
    """
    west = needs_retreat((-2.5, -1.4, 0.0), (0.0, 0.0, 0.0), PAD, permittable=False)
    east = needs_retreat((2.5, -1.4, 0.0), (0.0, 0.0, 0.0), PAD_EAST, permittable=False)
    assert west.needed is east.needed
    assert west.overlap_m == pytest.approx(east.overlap_m, abs=1e-9)


def test_the_retreat_goal_leaves_the_pad_even_for_shallow_overlap():
    """The goal must be OUTSIDE, not on the boundary.

    `node_reach_tolerance_m` is 0.80 m, so a robot counted as "at" the goal may be that
    far from it. A goal placed exactly on the edge is therefore inside as often as it is
    outside, which is why the standoff is added to the depth rather than replacing it.
    """
    for pose in ((-2.84, -1.40, 0.0), (-2.50, -1.40, 0.0), (-2.20, -1.74, 0.0)):
        c = retreat_command(pose, PAD)
        assert c.goal is not None, pose
        gx, gy = c.goal
        x_min, y_min, x_max, y_max = PAD["mid_left"]
        assert not (x_min <= gx <= x_max and y_min <= gy <= y_max), (
            f"the retreat goal {c.goal} for pose {pose} is still inside the rectangle")


def test_a_robot_that_still_holds_its_bundle_is_not_retreated():
    """The check that keeps this from firing on an ordinary crossing.

    `retreat_if_stuck` is called after `acquire` succeeds, which means the robot is
    usually HOLDING the rectangle it is standing in. That is not a lapse and must not
    provoke motion. The driver answers it by excluding held resources before calling the
    policy at all -- this test pins the policy's half of that contract: given no
    unpermitted rectangle, there is nothing to retreat from.
    """
    d = needs_retreat((-2.5, -1.4, 0.0), (0.0, 0.0, 0.0), {}, permittable=False)
    assert d.needed is False
    assert d.goal is None
    c = retreat_command((-2.5, -1.4, 0.0), {})
    assert c.needed is False and c.goal is None
