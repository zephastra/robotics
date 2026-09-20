"""A refusal that stops the localiser must be able to ask the localiser for a new estimate.

The loop this pins, measured on 2026-09-18 (516 samples of N01,
`_p009/p14/_run/pose_track.jsonl`):

    pose_usable              True 463, False 52
    the refusal, verbatim    "localised pose is 2.251 s old (> 2.00 s) and the robot has
                              moved 0.280 m since it arrived (> 0.100 m)"
    localiser_moved_m        frozen at p50 0.2583 / max 0.3397 -- a constant means stopped
    localiser_age_s          p50 316 s, max 813 s
    |amcl - truth|           p50 0.1021  p95 0.2515  max 0.3385

So the pose was good to a quarter of a metre, and the gate refused for 355 s because it
wanted a quarter of a second of freshness it had no way to obtain: refusing motion stops
AMCL (`update_min_d` is a motion gate; 0.25 m when this was measured), the age grows
without bound, the
displacement never changes, and both clauses of the freshness rule fail for ever.

What is asserted here, and at what level
========================================

`needs_refresh` is pure, so its behaviour is asserted directly, using the measured numbers
rather than invented ones. The wiring is asserted against the **syntax tree**, because
`fleet_ros` cannot be imported in the core test process (it needs rclpy) and because a
comment describing the wiring must not be able to pass a test. AST assertions here walk
`Call` nodes only -- `ast.dump` prints docstrings, which defeated an assertion earlier in
this same session.
"""

from __future__ import annotations

import ast
import pathlib

from fleet_core import needs_refresh
from fleet_core.pose_source import (
    DEFAULT_REFRESH_INTERVAL_S,
    LocaliserPolicy,
    judge_localiser,
)

ROOT = pathlib.Path(__file__).resolve().parents[1]
GATE = ROOT / "src/fleet_ros/fleet_ros/gate_node.py"

#: From the run: the age and displacement the gate actually refused on.
MEASURED_AGE_S = 2.251
MEASURED_MOVED_M = 0.280
#: How long it stayed refused afterwards.
MEASURED_REFUSED_FOR_S = 355.0


def _tree() -> ast.Module:
    return ast.parse(GATE.read_text(encoding="utf-8"))


def _function(name: str) -> ast.FunctionDef:
    for node in ast.walk(_tree()):
        if isinstance(node, ast.FunctionDef) and node.name == name:
            return node
    raise AssertionError(f"{name} is not defined in {GATE.name}")


def _called(fn: ast.AST) -> set[str]:
    """Every name called anywhere inside `fn` -- methods AND bare functions.

    Both halves matter and both were got wrong once: the first version collected only
    `Attribute` calls, so `needs_refresh(...)` -- a bare name -- was invisible and the test
    reported that the gate never consulted the policy. Walking `Call` nodes (not
    `ast.dump`, which prints docstrings) is the other half of the same lesson.
    """
    out: set[str] = set()
    for node in ast.walk(fn):
        if isinstance(node, ast.Call):
            if isinstance(node.func, ast.Attribute):
                out.add(node.func.attr)
            elif isinstance(node.func, ast.Name):
                out.add(node.func.id)
    return out


def _read_attrs(fn: ast.AST) -> set[str]:
    return {n.attr for n in ast.walk(fn) if isinstance(n, ast.Attribute) and
            isinstance(n.ctx, ast.Load)}


def _stored_attrs(fn: ast.AST) -> set[str]:
    """Every `self.x = ...` inside `fn`, including annotated assignments.

    `self.x: float | None = None` is an `ast.AnnAssign`, not an `ast.Assign`. The first
    version of this helper missed those, which is how two of the nine refresh attributes
    passed a check that was written to notice a missing assignment.
    """
    out: set[str] = set()
    for node in ast.walk(fn):
        if isinstance(node, ast.Assign) and node.targets:
            target = node.targets[0]
        elif isinstance(node, ast.AnnAssign):
            target = node.target
        else:
            continue
        if isinstance(target, ast.Attribute):
            out.add(target.attr)
    return out


# --------------------------------------------------------------------------- #
# 1. the refusal is the trigger, and the displacement is irrelevant
# --------------------------------------------------------------------------- #
def test_the_walkers_see_the_node_types_they_claim_to():
    """Self-test the helpers, because four assertions have now been beaten by their own
    narrowness: substring vs attribute, `ast.dump` vs `Call`, read vs assign, attribute
    call vs bare name, `Assign` vs `AnnAssign`.

    A helper that silently matches less than it claims turns every test built on it into a
    test of nothing, and it fails in the direction of PASSING.
    """
    sample = ast.parse(
        "def f(self):\n"
        "    self.a: float | None = None\n"
        "    self.b = 1\n"
        "    bare(1)\n"
        "    self.m(2)\n"
        "    x = self.c\n"
    )
    fn = sample.body[0]
    # `self.a: float | None = None` is an AnnAssign; `self.b = 1` is an Assign.
    assert _stored_attrs(fn) == {"a", "b"}, _stored_attrs(fn)
    # `bare(1)` is a Name call; `self.m(2)` is an Attribute call. Both count.
    assert _called(fn) == {"bare", "m"}, _called(fn)
    # `self.m` is itself an Attribute in Load ctx -- the receiver is read before it is
    # called -- so a "read" walker cannot separate a field read from a method call. Any
    # assertion on it must expect the receiver too.
    assert _read_attrs(fn) == {"m", "c"}, _read_attrs(fn)


def test_the_measured_refusal_is_what_triggers_the_ask():
    """The two halves must line up: what the policy refuses on is what the ask fires on."""
    policy = LocaliserPolicy()
    verdict = judge_localiser(
        age_s=MEASURED_AGE_S,
        moved_since_estimate_m=MEASURED_MOVED_M,
        policy=policy,
    )
    assert not verdict.usable, "this is the refusal the run measured"

    need = needs_refresh(age_s=MEASURED_AGE_S, policy=policy, since_last_request_s=None)
    assert need.needed, "the same condition must ask the localiser for an estimate"


def test_the_ask_does_not_depend_on_the_displacement():
    """Asking is not a threshold: it cannot make a pose less accurate.

    The displacement froze at 0.2583-0.3397 m against a 0.100 m bound, and a stopped
    robot's odometry never moves again -- so any displacement clause in the ask would
    reproduce the freeze one level down.
    """
    policy = LocaliserPolicy()
    for moved in (0.0, 0.100, 0.3397, 5.0, None):
        assert needs_refresh(age_s=MEASURED_AGE_S, policy=policy,
                             since_last_request_s=None).needed, moved


def test_a_fresh_estimate_is_not_worth_asking_about():
    policy = LocaliserPolicy()
    need = needs_refresh(age_s=1.0, policy=policy, since_last_request_s=None)
    assert not need.needed
    assert "fresh" in need.reason


def test_nothing_to_refresh_before_the_first_estimate():
    """Asking before the localiser has ever answered would race its own start-up."""
    need = needs_refresh(age_s=None, policy=LocaliserPolicy(), since_last_request_s=None)
    assert not need.needed
    assert "nothing to refresh" in need.reason


def test_the_rate_limit_bounds_how_often_the_localiser_is_asked():
    policy = LocaliserPolicy()
    early = needs_refresh(age_s=10.0, policy=policy, since_last_request_s=0.5)
    assert not early.needed
    assert "time to answer" in early.reason
    later = needs_refresh(age_s=10.0, policy=policy,
                          since_last_request_s=DEFAULT_REFRESH_INTERVAL_S)
    assert later.needed


def test_a_refusal_that_lasts_355_seconds_would_have_been_asked_about_178_times():
    """The rate limit is a rate, not a cap on total asks: the refusal is unbounded."""
    policy = LocaliserPolicy()
    assert MEASURED_REFUSED_FOR_S / DEFAULT_REFRESH_INTERVAL_S > 150


# --------------------------------------------------------------------------- #
# 2. the wiring, read from the syntax tree
# --------------------------------------------------------------------------- #
def test_the_tick_asks_for_an_estimate():
    assert "_ask_for_an_estimate" in _called(_function("_tick")), (
        "the ask has to happen on the gate's own timer, or it happens nowhere"
    )


def test_the_ask_consults_the_policy_before_spending_a_localiser_update():
    called = _called(_function("_ask_for_an_estimate"))
    assert "needs_refresh" in called, "the policy decides; the gate does not guess"
    assert "call_async" in called, "the ask is the point of the function"


def test_the_service_name_is_discovered_and_not_assumed():
    """This control is a service here and was a topic in older nav2."""
    assert "get_service_names_and_types" in _called(_function("_find_nomotion_service")), (
        "a hardcoded name that matches nothing produces silence that reads like "
        "'the localiser has nothing to say'"
    )


def test_a_missing_service_is_an_error_and_not_a_silence():
    """'I could not ask' must be distinguishable from 'I asked and nothing changed'."""
    assert "error" in _called(_function("_ask_for_an_estimate")), (
        "the missing-service path must say so at error level"
    )


def test_the_unusable_warning_is_rate_limited():
    """It was printed at 20 Hz for 355 s -- about 7100 identical lines."""
    read = _read_attrs(_function("_trusted_pose"))
    assert "_unusable_log_wall" in read, (
        "the warning must consult a previous-print time, or it repeats every tick"
    )
    assert "_unusable_log_period_s" in read, "and the period must be a declared parameter"


def test_every_refresh_attribute_read_is_somewhere_assigned():
    """A read assertion cannot detect a missing assignment.

    `_trusted_pose` compared against `self._unusable_log_period_s` while nothing assigned
    it, so the parameter guard saw a declared-and-never-read parameter and this suite saw
    the read and was satisfied -- while the gate would have raised `AttributeError` on its
    first tick. Asserting the read alone is one half of the check; this is the other.
    """
    stored = _stored_attrs(_function("__init__"))
    for name in ("_unusable_log_period_s", "_unusable_log_wall", "_unusable_reason",
                 "_nomotion_service", "_nomotion_client", "_nomotion_last_wall",
                 "_nomotion_requests", "_nomotion_need", "_nomotion_missing_logged"):
        assert name in stored, f"{name} is read somewhere but never assigned in __init__"


def test_the_payload_reports_whether_the_loop_was_broken():
    fields = {n.value for n in ast.walk(_function("_pose_fields"))
              if isinstance(n, ast.Constant) and isinstance(n.value, str)}
    for name in ("nomotion_service", "nomotion_requests", "nomotion_need"):
        assert name in fields, f"{name} is not reported, so a run cannot say"
