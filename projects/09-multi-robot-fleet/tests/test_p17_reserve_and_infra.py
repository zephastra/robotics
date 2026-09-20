"""②-c and ②-d: the reserve check, and telling a slow box apart from a stuck controller.

Each test is written against the defect it prevents, and each names the measured run.

The reserve check (D-P17-08)
----------------------------
N05 seed2 (2026-09-18): r02 entered its last stage with the permit in hand and 2.2 s of a
195.0 s budget left. It could not finish, and nothing had asked whether it could. The cost
is not r02's -- it is the queue's: the passage is capacity-1, so a robot that fails inside it
is a robot everyone else is stuck behind. Refusing to ENTER is cheap; failing after entry is
not.

INFRA_TIMEOUT (CONTRACTS section 9)
-----------------------------------
Before this, a stage that ran out of interval printed "timeout after Ns", the same words as a
controller that could not drive. The two want opposite actions -- re-run the box, or debug the
controller -- so one word for both is one word too few.
"""
from __future__ import annotations

import importlib
import pathlib
import sys

import pytest

ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src/fleet_core"))

POLICY_MODULE = "fleet_core.crossing_release"


@pytest.fixture(scope="module")
def cr():
    return importlib.import_module(POLICY_MODULE)


# --------------------------------------------------------------------------- #
# ②-c the reserve check
# --------------------------------------------------------------------------- #


def test_a_driver_with_less_budget_than_the_passage_may_not_enter(cr):
    """The measured N05 seed2 case: 2.2 s left against a passage that takes far longer."""
    p = cr.ReleasePolicy()
    now = 1000.0
    assert p.reserve_ok(expected_remaining_s=30.0, now=now, run_deadline=now + 2.2) is False


def test_a_driver_with_enough_budget_may_enter(cr):
    """The check must not refuse the runs that would have succeeded, or it buys nothing."""
    p = cr.ReleasePolicy()
    now = 1000.0
    assert p.reserve_ok(expected_remaining_s=30.0, now=now, run_deadline=now + 45.0) is True


def test_the_boundary_is_inclusive(cr):
    """Exactly enough is enough. An off-by-one here refuses a run that would have passed."""
    p = cr.ReleasePolicy()
    now = 1000.0
    assert p.reserve_ok(expected_remaining_s=30.0, now=now, run_deadline=now + 30.0) is True


def test_an_undeclared_expectation_does_not_refuse(cr):
    """0.0 means 'the scenario declared nothing', and a driver may not invent the number.

    This is what keeps the patch a no-op for every case that has not opted in: no estimate,
    no refusal. A check that guessed would be measuring its own opinion.
    """
    p = cr.ReleasePolicy()
    now = 1000.0
    assert p.reserve_ok(expected_remaining_s=0.0, now=now, run_deadline=now + 0.1) is True


def test_the_refusal_names_both_numbers(cr):
    """A refusal a reader cannot act on is a refusal that gets ignored."""
    p = cr.ReleasePolicy()
    detail = p.reserve_detail(expected_remaining_s=30.0, now=1000.0,
                              run_deadline=1002.2, stage="stage_cross")
    assert "2.2s" in detail and "30.0s" in detail, detail
    assert "stage_cross" in detail, detail
    assert "capacity-1" in detail, (
        "the detail does not say WHY refusing is better than entering: the queue inherits "
        "the failure"
    )


# --------------------------------------------------------------------------- #
# ②-d INFRA vs control
# --------------------------------------------------------------------------- #


def test_running_out_of_run_budget_after_a_queue_is_a_budget_result(cr):
    """The stage had NOT overrun its own share; the run budget simply ended. Not infra."""
    p = cr.ReleasePolicy()
    verdict = p.classify_stop(stage="stage_release", stage_elapsed_s=5.0,
                              stage_expected_s=20.0, run_deadline=1000.0, now=1000.5)
    assert verdict == cr.STOP_RUN_DEADLINE


def test_a_stage_that_overran_its_own_share_is_infrastructure(cr):
    """Measured shape: the stage was still the bottleneck when the money ran out."""
    p = cr.ReleasePolicy()
    verdict = p.classify_stop(stage="stage_release", stage_elapsed_s=95.0,
                              stage_expected_s=20.0, run_deadline=1000.0, now=1000.5)
    assert verdict == cr.STOP_INFRA_TIMEOUT


def test_no_declared_estimate_never_claims_infrastructure(cr):
    """Without a declared share there is no evidence of slowness, so the old word stands."""
    p = cr.ReleasePolicy()
    verdict = p.classify_stop(stage="stage_release", stage_elapsed_s=95.0,
                              stage_expected_s=0.0, run_deadline=1000.0, now=1000.5)
    assert verdict == cr.STOP_RUN_DEADLINE


def test_the_infra_word_is_distinct_from_every_other_stop_reason(cr):
    """`stopped_by` is read by a person deciding what to do next, so the values must differ."""
    reasons = {cr.STOP_SIGNAL, cr.STOP_RUN_DEADLINE, cr.STOP_BUSY, cr.STOP_NAV_ABSENT,
               cr.STOP_INFRA_TIMEOUT}
    assert len(reasons) == 5, "two stop reasons share a value, so the report cannot tell them apart"


def test_the_infra_detail_says_what_to_do(cr):
    """'Re-run on a faster machine' is the action; a verdict without one is a shrug."""
    p = cr.ReleasePolicy()
    detail = p.stopped_detail_infra("stage_release", 95.0, 20.0)
    assert "INFRA_TIMEOUT" in detail, detail
    assert "95.0" in detail and "20.0" in detail, detail
    assert "faster" in detail, detail


def test_stopped_detail_still_routes_the_old_reasons(cr):
    """Adding a branch must not detach the existing ones."""
    p = cr.ReleasePolicy()
    assert "signal" in p.stopped_detail(cr.STOP_SIGNAL, "stage_cross", 1.0).lower()
    assert "budget" in p.stopped_detail(cr.STOP_RUN_DEADLINE, "stage_cross", 1.0).lower()
    assert "INFRA_TIMEOUT" in p.stopped_detail(cr.STOP_INFRA_TIMEOUT, "stage_cross", 1.0)


# --------------------------------------------------------------------------- #
# ②-d the sentence that was wrong
# --------------------------------------------------------------------------- #


def test_the_run_deadline_does_withhold_the_allowance_from_an_unqueued_driver(cr):
    """The docstring used to say an unqueued driver 'is unaffected'. The arithmetic disagrees.

    This test exists to hold the CORRECTED sentence in place: if someone later makes the
    behaviour match the old words, or the words drift back, one of these two assertions fails.
    """
    p = cr.ReleasePolicy(queue_allowance_s=90.0)
    started, timeout = 1000.0, 285.0
    hard = p.hard_deadline(started, timeout)
    run = p.run_deadline(started, timeout)
    assert run < hard, (
        "run_deadline is no longer below hard_deadline, so the unqueued case is unaffected "
        "and the corrected docstring is now the wrong one"
    )
    assert (hard - run) == pytest.approx(90.0), (
        "the withheld amount is not the allowance; re-derive the docstring"
    )


def test_a_queued_driver_gets_its_allowance_back(cr):
    """The credit is real, which is what makes withholding defensible rather than a bug."""
    p = cr.ReleasePolicy(queue_allowance_s=90.0)
    started, timeout = 1000.0, 285.0
    run = p.run_deadline(started, timeout)
    assert p.after_queue(run, queued_s=60.0, started_at=started, timeout_s=timeout) > run


def test_the_credit_is_capped_by_the_hard_deadline(cr):
    """A queue may not buy a driver a longer life than the dispatcher will allow."""
    p = cr.ReleasePolicy(queue_allowance_s=90.0)
    started, timeout = 1000.0, 285.0
    hard = p.hard_deadline(started, timeout)
    assert p.after_queue(hard - 1.0, queued_s=1000.0,
                         started_at=started, timeout_s=timeout) == pytest.approx(hard)


# --------------------------------------------------------------------------- #
# the wiring, which is where a correct function goes unused
# --------------------------------------------------------------------------- #


def test_the_driver_calls_the_reserve_check_before_it_crosses():
    """A policy method with no caller is a policy that does not exist.

    The check must be placed after the grant (when the queue credit is known) and before
    `stage_cross` (when the robot commits to a capacity-1 resource). Asserted on the source
    because the alternative is a live three-robot run.
    """
    src = (ROOT / "src/fleet_ros/fleet_ros/staged_crossing.py").read_text(encoding="utf-8")
    assert "reserve_ok(" in src, "the reserve check has no caller"
    body = src.split("def run(self)")[1]
    i_reserve = body.index("reserve_ok(")
    i_cross = body.index('("stage_cross"')
    i_after = body.index("after_queue(")
    assert i_after < i_reserve < i_cross, (
        "the reserve check is not placed between the queue credit and the crossing"
    )


def test_the_report_carries_the_reserve_decision():
    """Whether the driver refused, and against what estimate, has to be in the report.

    `complete: false` does not distinguish "never entered" from "failed inside", and those
    are the two outcomes the queue cares about.
    """
    src = (ROOT / "src/fleet_ros/fleet_ros/staged_crossing.py").read_text(encoding="utf-8")
    for field in ('"insufficient_reserve"', '"expected_passage_s"'):
        assert field in src, f"the report does not carry {field}"


def test_the_task_service_passes_the_scenario_estimate_through():
    """The declaration has to reach the driver, or the check reads a default forever."""
    src = (ROOT / "src/fleet_ros/fleet_ros/task_service_node.py").read_text(encoding="utf-8")
    assert '"--expected-passage-s"' in src, "the driver is never given the estimate"
    assert 'declare_parameter("crossing_expected_s"' in src, (
        "the scenario cannot declare an estimate, so the parameter is always 0.0"
    )
