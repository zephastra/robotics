"""A stopped crossing driver must release its Nav2 goal, and must say why it stopped.

The defect this pins
====================

Measured on 2026-09-17 in `reports/batch_20260917T122048Z`, from the dispatcher's event
log, the driver's log and the Nav2 log -- three independent writers, same story:

    t=673.6   crossing driver #1 started (pid 4975)
    t=678.8   bt_navigator: Begin navigating -> wait_west     (accepted, running)
    t=854.5   the dispatcher hit its 180 s budget and killed driver #1
    t=854.5   driver #2 started, same command_id
    t=862.7   driver #2's goal REJECTED: "another navigator is processing"
    t=871.8   driver #1's orphaned goal finally aborted (error 105)
    t=872.6   driver #2 had already spent its 3 attempts and gave up

So the failure was not navigation and not contention between two live clients: **the kill
did not cancel the goal.** The retry then waited 9.026 s, measured from its own log
(attempts at +0.000, +3.019, +6.027 and a give-up at +9.026), while the orphan released
at +9.123. It lost by **0.097 s** -- twice, with the same timings, which is what makes it
a race rather than a flake.

A second defect rode along: SIGTERM's default action ends the process, so driver #1 never
reached its `write_text` and **left no report at all**. The first attempt is invisible.

What is asserted here, and at what level
========================================

The arithmetic is asserted directly, because it is pure. The wiring (a signal handler that
asks the node to stop, a report written on both the signal path and the crash path) is
asserted against the **syntax tree**, not against the text: a comment that describes the
old behaviour must not be able to fail -- or pass -- a test, and this session has already
lost five assertions to exactly that. `fleet_ros` cannot be imported in the core test
process (it needs rclpy), so the AST is the only honest level available here.
"""

from __future__ import annotations

import ast
import pathlib

from fleet_core import (
    STOP_BUSY,
    STOP_RUN_DEADLINE,
    STOP_SIGNAL,
    ReleasePolicy,
    stop_is_early_enough,
)

ROOT = pathlib.Path(__file__).resolve().parents[1]
DRIVER = ROOT / "src/fleet_ros/fleet_ros/staged_crossing.py"

#: The retry's own log, second by second, relative to its first rejected goal.
MEASURED_ATTEMPTS_S = (0.000, 3.019, 6.027)
#: When the abandoned goal finally released the server, relative to the same instant.
MEASURED_ORPHAN_LIFETIME_S = 9.123
#: When the retry gave up.
MEASURED_GIVE_UP_S = 9.026


def _tree() -> ast.Module:
    return ast.parse(DRIVER.read_text(encoding="utf-8"))


def _function(name: str) -> ast.FunctionDef:
    for node in ast.walk(_tree()):
        if isinstance(node, ast.FunctionDef) and node.name == name:
            return node
    raise AssertionError(f"{name} is not defined in {DRIVER.name}")


# --------------------------------------------------------------------------- #
# 1. the race, in the numbers that produced it
# --------------------------------------------------------------------------- #
def test_the_old_patience_loses_the_race_the_new_one_does_not():
    """Three attempts 3 s apart is 9 s of patience against an 8 s orphan.

    The old shape gave up at 9.026 s; the server freed at 9.123 s. This is the whole
    defect in one comparison, so it is asserted rather than described.
    """
    # The shape that shipped before this change: 3 x 3 s.
    old = ReleasePolicy(release_margin_s=15.0, busy_budget_s=3 * 3.0, retry_delay_s=3.0)
    assert old.busy_exhausted(0.0, MEASURED_GIVE_UP_S) is True, (
        "the old policy must be reproducible as giving up here, or this test is not "
        "describing the defect"
    )
    assert MEASURED_GIVE_UP_S < MEASURED_ORPHAN_LIFETIME_S, (
        "the log says it gave up before the orphan released; if that stops being true "
        "this test is asserting the wrong defect"
    )

    shipped = ReleasePolicy()
    assert shipped.busy_exhausted(0.0, MEASURED_ORPHAN_LIFETIME_S) is False, (
        "the shipped budget must still be waiting when the server frees"
    )


def test_every_attempt_in_the_log_fits_inside_a_count_based_budget():
    """The old budget ran out right after the last attempt -- not before it.

    This is why the bug looked like a refusal: the driver did get three real answers.
    """
    old = ReleasePolicy(busy_budget_s=3 * 3.0, retry_delay_s=3.0)
    assert [old.busy_exhausted(0.0, t) for t in MEASURED_ATTEMPTS_S] == [False] * 3


def test_the_busy_budget_is_a_duration_so_a_slower_retry_still_tolerates_it():
    """A count-based policy changes meaning when the delay changes; this one does not."""
    patient = ReleasePolicy(busy_budget_s=60.0, retry_delay_s=10.0)
    assert patient.busy_exhausted(0.0, 59.9) is False
    assert patient.busy_exhausted(0.0, 60.0) is True


# --------------------------------------------------------------------------- #
# 2. the driver stops itself before the dispatcher would kill it
# --------------------------------------------------------------------------- #
def test_the_driver_stops_strictly_before_the_dispatcher_deadline():
    """`dispatcher_crossing_timeout_s` is 180 s in the run that failed."""
    policy = ReleasePolicy()
    started = 1000.0
    run_deadline = policy.run_deadline(started, timeout_s=180.0)
    assert stop_is_early_enough(run_deadline, started + 180.0)


def test_the_margin_is_larger_than_the_measured_orphan_lifetime():
    """15 s clears 9.123 s, which is what an uncancelled kill costs.

    If the margin ever drops below the orphan lifetime, the next driver is back inside
    the same race even though the release code is present.
    """
    policy = ReleasePolicy()
    started = 0.0
    margin_used = (started + 180.0) - policy.run_deadline(started, 180.0)
    assert margin_used > MEASURED_ORPHAN_LIFETIME_S


def test_a_tiny_budget_cannot_produce_a_deadline_in_the_past():
    """A configured timeout of 4 s must still leave the driver time to release."""
    policy = ReleasePolicy()
    started = 500.0
    assert policy.run_deadline(started, timeout_s=4.0) >= started + 2.0


def test_no_stage_may_outlive_the_run_budget():
    """The parameter means the whole crossing, per its own docstring."""
    policy = ReleasePolicy()
    run_deadline = policy.run_deadline(0.0, timeout_s=180.0)
    for stage_start in (0.0, 100.0, 160.0, 170.0):
        assert policy.stage_deadline(stage_start, 180.0, run_deadline) <= run_deadline


# --------------------------------------------------------------------------- #
# 3. the report distinguishes the ways a run can stop
# --------------------------------------------------------------------------- #
def test_a_signal_stop_names_the_stage_and_the_duration():
    text = ReleasePolicy().stopped_detail(STOP_SIGNAL, "stage_wait", 3.25)
    assert "stage_wait" in text and "3.2" in text
    assert "cancelled" in text, "a reader must know the goal was released, not abandoned"


def test_a_budget_stop_is_a_different_sentence_from_a_signal_stop():
    policy = ReleasePolicy()
    signal_text = policy.stopped_detail(STOP_SIGNAL, "stage_wait", 1.0)
    budget_text = policy.stopped_detail(STOP_RUN_DEADLINE, "stage_wait", 1.0)
    assert signal_text != budget_text
    assert "budget" in budget_text
    assert "signal" not in budget_text


def test_a_busy_stop_says_the_holder_may_not_be_this_driver():
    """The 2026-09-17 holder was a *previous* driver's uncancelled goal.

    A message that implies the driver collided with itself would send a reader looking in
    the wrong place -- which is what rounds 6-9 did with `nav2 status=6`.
    """
    text = ReleasePolicy().busy_detail("wait_west", 60.0, 61)
    assert "60.0" in text
    assert "need not be this driver" in text
    assert STOP_BUSY not in ("",)


# --------------------------------------------------------------------------- #
# 4. the wiring, read from the syntax tree
# --------------------------------------------------------------------------- #
def test_main_installs_a_handler_for_sigterm_and_sigint():
    """SIGTERM's default action is what lost the report; it has to be replaced."""
    main = _function("main")
    installed: set[str] = set()
    for node in ast.walk(main):
        if (isinstance(node, ast.Call)
                and isinstance(node.func, ast.Attribute)
                and node.func.attr == "signal"
                and isinstance(node.func.value, ast.Name)
                and node.func.value.id == "signal"
                and node.args):
            arg = node.args[0]
            if (isinstance(arg, ast.Attribute)
                    and isinstance(arg.value, ast.Name)
                    and arg.value.id == "signal"):
                installed.add(arg.attr)
    assert {"SIGTERM", "SIGINT"} <= installed, (
        f"expected signal.signal for SIGTERM and SIGINT, found {sorted(installed)}"
    )


def test_the_signal_handler_only_asks_the_node_to_stop():
    """No rclpy work from a handler: the cancel must happen on the spinning thread."""
    main = _function("main")
    handler = None
    for node in ast.walk(main):
        if isinstance(node, ast.FunctionDef) and node.name == "_on_signal":
            handler = node
    assert handler is not None, "the handler is defined inside main"
    calls = sorted(_called_attrs(handler))
    assert calls == ["request_stop"], f"the handler must only request_stop, saw {calls}"


def _called_attrs(fn: ast.AST) -> set[str]:
    """Every attribute called anywhere inside `fn`.

    Walking `Call` nodes rather than dumping the tree on purpose: a docstring is an AST
    node whose value is a string, so `ast.dump` prints it, and this function's own
    docstring names the two calls it must not make. An assertion written against the dump
    is an assertion a comment can satisfy -- which is the same trap as a substring check,
    one level deeper.
    """
    out: set[str] = set()
    for node in ast.walk(fn):
        if isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute):
            out.add(node.func.attr)
    return out


def test_request_stop_exists_and_does_not_touch_rclpy():
    fn = _function("request_stop")
    assigned = {
        node.targets[0].attr
        for node in ast.walk(fn)
        if isinstance(node, ast.Assign)
        and node.targets
        and isinstance(node.targets[0], ast.Attribute)
    }
    assert "_stop" in assigned, "request_stop must set the flag the loops read"
    assert _called_attrs(fn) == set(), (
        "request_stop must make no calls at all: cancelling from a signal handler would "
        f"race the spinning thread (saw {sorted(_called_attrs(fn))})"
    )


def test_the_count_based_retry_loop_is_gone():
    """`for attempt in range(1, 4)` was the defect; a substring check could not tell it
    apart from a comment quoting it, so the loop is matched as syntax."""
    iterators: list[str] = []
    for node in ast.walk(_tree()):
        if isinstance(node, ast.For):
            iterators.append(ast.unparse(node.iter))
    assert "range(1, 4)" not in iterators, (
        f"a fixed-attempt retry loop survived: {iterators}"
    )


def test_the_driver_cancels_its_goal_on_the_stop_path():
    """Finding the release: the stop branch must cancel before returning."""
    drive = _function("drive_to")
    cancels = [
        node for node in ast.walk(drive)
        if isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute)
        and node.func.attr == "cancel_goal_async"
    ]
    assert cancels, "drive_to must cancel its goal somewhere on its exit paths"


def test_the_crash_path_still_writes_the_report():
    """A crash is a run too. 'The driver died' and 'the driver never started' differ."""
    main = _function("main")
    handlers = [n for n in ast.walk(main) if isinstance(n, ast.ExceptHandler)]
    assert handlers, "main must handle exceptions"
    for handler in handlers:
        names = [
            n.func.attr for n in ast.walk(handler)
            if isinstance(n, ast.Call) and isinstance(n.func, ast.Attribute)
        ]
        assert "write_text" in names, (
            "the exception path must write the report, or a crashed driver is invisible"
        )
