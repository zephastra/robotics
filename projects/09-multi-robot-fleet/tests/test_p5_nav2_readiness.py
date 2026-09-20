"""D-P5-23: "the executor is not up yet" must be a wait, not a failure.

Two healthy runs recorded `auto-charge-<robot>-<epoch>-1` as `FAILED` with `attempts=2`,
because the task service creates a charge run on its first tick -- 13-16 s before Nav2
reaches ACTIVE -- and the adapter's retry loop spent its whole budget on goals that an
inactive `bt_navigator` refused.

The tests divide in two, and both halves matter:

  * `Nav2Readiness` is pure and gets real behavioural tests with a fake clock. The rule
    that keeps this change safe is `should_wait_for_recovery(None) is False`: an answer we
    do not have must behave exactly as the code did before the check existed.
  * the wiring inside the ROS nodes gets structural tests, because `fleet_ros` imports
    rclpy and cannot be imported here. Each one pins a property that would silently
    reintroduce the defect -- readiness asked before the goal is sent, an attempt that can
    be given back, a bounded wait, and a `nav_state` that says which case it was.
"""

from __future__ import annotations

import ast
from pathlib import Path

from fleet_adapter import NAV2_LC_ACTIVE, Nav2Readiness, should_wait_for_recovery

ROOT = Path(__file__).resolve().parents[1]
ADAPTER = ROOT / "src" / "fleet_ros" / "fleet_ros" / "nav2_adapter_node.py"
SERVICE = ROOT / "src" / "fleet_ros" / "fleet_ros" / "task_service_node.py"
READINESS = ROOT / "src" / "fleet_adapter" / "fleet_adapter" / "nav2_readiness.py"


class FakeClock:
    def __init__(self, start: float = 1000.0) -> None:
        self.t = start

    def __call__(self) -> float:
        return self.t

    def advance(self, dt: float) -> None:
        self.t += dt


# --------------------------------------------------------------------------- #
# the rule that keeps it safe
# --------------------------------------------------------------------------- #


def test_cannot_tell_is_not_inactive():
    """The whole safety argument in one assertion.

    `None` means "no answer yet", not "not ready". If it held a leg back, an unreachable
    lifecycle service would stop the fleet -- a new failure mode introduced by a fix.
    """
    assert should_wait_for_recovery(None) is False


def test_only_an_explicit_false_holds_a_leg_back():
    assert should_wait_for_recovery(False) is True
    assert should_wait_for_recovery(True) is False


# --------------------------------------------------------------------------- #
# the object, with a fake clock
# --------------------------------------------------------------------------- #


def test_nothing_is_known_until_something_is_asked():
    r = Nav2Readiness(clock=FakeClock())
    assert r.active() is None
    assert r.due() is True, "the first poll must ask"
    assert r.asks == 0, "asking is the caller's job; this object only decides when"


def test_an_answer_is_reused_until_the_ttl_expires():
    clock = FakeClock()
    r = Nav2Readiness(clock=clock, ttl_s=0.5)
    r.record(NAV2_LC_ACTIVE)
    assert r.active() is True
    assert r.due() is False, "a 10 Hz publisher must not become a 10 Hz service call"
    clock.advance(0.49)
    assert r.due() is False
    clock.advance(0.02)
    assert r.due() is True


def test_active_is_decided_by_state_id_not_by_truthiness():
    r = Nav2Readiness(clock=FakeClock())
    r.record(NAV2_LC_ACTIVE)
    assert r.active() is True
    r.record(NAV2_LC_ACTIVE - 1)          # INACTIVE
    assert r.active() is False
    r.record(0)                            # UNKNOWN, id 0
    assert r.active() is False, "an id that is not ACTIVE is not active, whatever its value"


def test_a_failed_ask_becomes_unknown_rather_than_inactive():
    """A question that failed is not evidence about the executor."""
    r = Nav2Readiness(clock=FakeClock())
    r.record(NAV2_LC_ACTIVE)
    assert r.active() is True
    r.record(None)
    assert r.active() is None, "the ask failed; we no longer know, and must not claim a state"
    assert should_wait_for_recovery(r.active()) is False


def test_an_ask_in_flight_keeps_what_was_already_observed():
    """A pending request says nothing about the state, so it must erase nothing."""
    clock = FakeClock()
    r = Nav2Readiness(clock=clock, ttl_s=0.5)
    r.record(2)                            # observed INACTIVE
    assert r.active() is False
    r.asked_without_an_answer()
    assert r.active() is False, "the last observation must survive an unanswered ask"
    assert r.due() is False, "and it must not re-ask immediately"
    clock.advance(0.6)
    assert r.due() is True


def test_describe_says_what_is_known_and_how_old_it_is():
    clock = FakeClock()
    r = Nav2Readiness(clock=clock)
    assert "unknown" in r.describe()
    r.record(2)
    clock.advance(1.25)
    text = r.describe()
    assert "active=False" in text and "1.25s" in text


def test_the_readiness_module_imports_no_ros():
    """It lives in fleet_adapter, which must not import ROS (that is what makes it testable).

    Checked on the IMPORT statements rather than on the text. The module's comments name
    `lifecycle_msgs/msg/State` to say which constant `ACTIVE` is, and a substring check
    forbids the documentation along with the dependency -- which is what my first version of
    this test did, and it failed.
    """
    tree = ast.parse(READINESS.read_text(encoding="utf-8"), filename=str(READINESS))
    imported: "list[str]" = []
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            imported += [a.name for a in node.names]
        elif isinstance(node, ast.ImportFrom) and node.module:
            imported.append(node.module)
    offending = sorted(m for m in imported if m.split(".")[0] in {"rclpy", "lifecycle_msgs"})
    assert offending == [], f"ROS imports inside a ROS-free package: {offending}"
    assert "time" in imported, "it does need a clock"


# --------------------------------------------------------------------------- #
# the wiring inside the nodes
# --------------------------------------------------------------------------- #


def _source(path: Path) -> str:
    return path.read_text(encoding="utf-8")


def _function(path: Path, name: str) -> "tuple[str, ast.FunctionDef]":
    source = _source(path)
    tree = ast.parse(source, filename=str(path))
    for node in ast.walk(tree):
        if isinstance(node, ast.FunctionDef) and node.name == name:
            return source, node
    raise AssertionError(f"{name} not found in {path}")


def _segment(path: Path, name: str) -> str:
    source, node = _function(path, name)
    return ast.get_source_segment(source, node) or ""


def test_readiness_is_asked_before_a_goal_is_sent():
    body = _segment(ADAPTER, "_drive")
    wait = body.index("_wait_for_nav2_ready")
    send = body.index("send_goal_async")
    assert wait < send, (
        "the leg must wait for its executor before it spends an attempt on a goal an "
        "inactive bt_navigator will refuse"
    )


def test_a_readiness_refusal_gives_the_attempt_back():
    body = _segment(ADAPTER, "_drive")
    assert "attempt -= 1" in body, (
        "a refusal explained by readiness must not consume the retry budget"
    )
    assert "should_wait_for_recovery" in body, (
        "and it must be decided by an observed lifecycle answer, not assumed"
    )
    assert body.index("should_wait_for_recovery") < body.index("attempt -= 1")


def test_the_readiness_probe_never_waits_on_the_future():
    """The node's own docstring forbids nested waits; this one would deadlock the leg."""
    body = _segment(ADAPTER, "_nav2_ready")
    for token in ("spin(", "sleep(", "wait_for_server", "spin_until_future_complete"):
        assert token not in body, f"{token!r} in _nav2_ready would nest a wait"
    assert "done()" in body, "it must harvest a finished future, not block on one"
    assert "_lc_lock" in body, (
        "three threads call this; two concurrent call_async would leave neither answer "
        "harvestable"
    )


def test_the_wait_is_bounded_and_interruptible():
    body = _segment(ADAPTER, "_wait_for_nav2_ready")
    assert "deadline" in body, "a wait with no deadline is a hang"
    assert "is_cancel_requested" in body, (
        "a cancel must be able to end a leg that is waiting rather than driving"
    )


def test_the_readiness_timeout_is_declared_and_read():
    source = _source(ADAPTER)
    assert 'declare_parameter("nav2_ready_timeout_s"' in source
    assert 'get_parameter("nav2_ready_timeout_s")' in source
    assert "self.nav2_ready_timeout_s" in source


def test_nav_state_reports_readiness_rather_than_only_the_client():
    source = _source(ADAPTER)
    assert '"" if self.nav_client else "NO_NAV2_ACTION_SERVER"' not in source, (
        "the old label was true while the server's node was still INACTIVE, which is the "
        "one case the field exists for"
    )
    assert source.count("self._nav_state_label()") >= 2, (
        "both the state message and the leg result must carry it"
    )


def test_a_not_active_leg_is_not_reported_as_a_navigation_failure():
    source = _source(ADAPTER)
    assert "ReasonCode.NAV2_NOT_ACTIVE" in source, (
        "an inactive executor and a navigation failure need opposite responses"
    )
    assert 'nav_state = "EXCEPTION"' in source  # the other path is untouched


def test_the_dispatcher_does_not_spend_an_attempt_on_a_wait():
    body = _segment(SERVICE, "_handle_leg_result")
    assert "WAIT_NOT_A_FAILURE" in body, "the wait class must be consulted"
    assert body.index("WAIT_NOT_A_FAILURE") < body.index("run.attempts += 1"), (
        "the deferral must be decided BEFORE the attempt is counted"
    )
    assert "run.waits < self.wait_budget" in body, (
        "an unbounded 'not yet' is a task that never resolves"
    )
    assert '"leg_deferred"' in body, "a wait that is counted is diagnosable"


def test_a_wait_does_not_survive_a_cancel_or_a_successful_leg():
    source = _source(SERVICE)
    assert "WAIT_NOT_A_FAILURE = frozenset({" in source, "declared in one readable place"
    assert "ReasonCode.NAV2_NOT_ACTIVE," in source
    assert "run.waits = 0" in source, "a leg that succeeded resets the wait count"


def test_both_new_parameters_are_declared():
    """`check_ros_params.py` covers the reads; this pins the declarations next to them."""
    adapter = _source(ADAPTER)
    service = _source(SERVICE)
    assert 'declare_parameter("nav2_ready_timeout_s"' in adapter
    assert 'declare_parameter("wait_budget_per_task"' in service
