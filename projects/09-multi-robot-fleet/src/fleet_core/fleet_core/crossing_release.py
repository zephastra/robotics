"""When a crossing driver may stop, and what it owes the robot when it does.

Two facts from one live run (2026-09-17, ``reports/batch_20260917T122048Z``), read from
the dispatcher's own event log and the Nav2 log rather than inferred:

1. At t=854.5 the dispatcher hit its 180 s budget and killed the passage driver. **The
   kill did not cancel the driver's Nav2 goal.** Nav2 kept navigating the abandoned
   target and only gave up at t=871.8 (``error 105 Failed to make progress``). The retry
   launched at t=854.5 sent its goal at t=862.7 and was answered *"Requested navigation
   from navigate_to_pose while another navigator is processing, rejecting request"*.
   That retry tolerated three attempts 3 s apart, gave up at t=871.7, and lost the race
   by **0.1 s** -- twice, with identical timings.

2. The killed driver wrote **no report at all**. SIGTERM's default action ends the
   process, so ``main`` never reached its ``write_text``. The first attempt left nothing.

So the releasing party has to be the driver itself, and it has to do it while the
dispatcher is still willing to wait. This module is that arithmetic, plus the words for
saying why. ``fleet_ros.staged_crossing`` is the caller.

Three decisions, each carrying the number it came from:

* ``release_margin_s`` -- the driver stops this long *before* the dispatcher's deadline.
  The measured orphan lived 8 s past its kill, so any margin below that puts the next
  driver back inside the same race. The default is 15 s: the margin has to exceed the
  orphan lifetime, and the orphan lifetime is what an uncancelled goal costs.

* ``busy_budget_s`` -- how long a rejected goal may be waited out. A **duration**, not an
  attempt count. The old policy said "3 attempts", and its real patience was 3 x 3 s --
  below the observed orphan lifetime. A count is the wrong unit here because the delay is
  chosen by the caller, so "three attempts" describes nothing about the server.

* the stop itself -- refusing to conflate three different situations. ``signal`` (someone
  asked us to stop), ``run_deadline`` (our own budget ran out), ``busy`` (the server never
  freed). Each implies a different next action for whoever reads the report, so each gets
  its own word rather than one shared "failed".
"""

from __future__ import annotations

from dataclasses import dataclass

#: How the driver came to stop. Distinct values because they imply different next
#: actions for a reader: a signal means look at the dispatcher's budget, a deadline means
#: look at this driver's budget, busy means look at whatever else holds that robot's
#: navigator, and absent means Nav2 was not running.
STOP_SIGNAL = "signal"
STOP_RUN_DEADLINE = "run_deadline"
STOP_BUSY = "busy"
STOP_NAV_ABSENT = "nav2_absent"
#: The box could not run the scenario, as distinct from the controller failing to drive it.
#: CONTRACTS section 9 requires the two to be reported apart because they want opposite next
#: actions. Added 2026-09-18: a stage that is still making progress but cannot finish inside
#: the budget is not a control failure, and reporting it as one sends the reader to the
#: controller when the answer is the machine.
STOP_INFRA_TIMEOUT = "infra_timeout"
#: The driver found itself stopped inside a region it has no permit for, issued the
#: retreat that D-P14-02 requires, and the retreat was refused. Distinct from every reason
#: above because the next action is different again: it is not the dispatcher's budget or
#: this driver's budget or a slow box, it is that the PERMIT half of D-P14-02 is unwritten.
#: A robot inside a region it may not occupy may be permitted to move only if the motion
#: STRICTLY reduces its overlap, that rule is not implemented, so the gate can only refuse
#: -- and the refusal zeroes the velocity, which is the state this project has repeatedly
#: read as a navigation fault. Added 2026-09-19 with the retreat emitter itself.
STOP_RETREAT_REFUSED = "retreat_refused"

#: 8 s was measured; 15 s clears it with room for the cancel request itself.
DEFAULT_RELEASE_MARGIN_S = 15.0
#: Longer than any observed orphan lifetime, and bounded, because an unbounded wait for
#: someone else's goal is a driver that never returns.
DEFAULT_BUSY_BUDGET_S = 60.0
DEFAULT_RETRY_DELAY_S = 1.0
#: How long a driver may spend QUEUED for a permit without it counting against the time it
#: needs to drive. Zero by default, which reproduces the previous behaviour exactly; a
#: scenario that expects a queue declares its own (CONTRACTS section 9).
DEFAULT_QUEUE_ALLOWANCE_S = 0.0


@dataclass(frozen=True)
class ReleasePolicy:
    """The three numbers, and the sentences that report them."""

    release_margin_s: float = DEFAULT_RELEASE_MARGIN_S
    busy_budget_s: float = DEFAULT_BUSY_BUDGET_S
    retry_delay_s: float = DEFAULT_RETRY_DELAY_S
    queue_allowance_s: float = DEFAULT_QUEUE_ALLOWANCE_S

    def hard_deadline(self, started_at: float, timeout_s: float) -> float:
        """The latest this driver may still be alive.

        Strictly before ``started_at + timeout_s``, so the dispatcher never has to kill it -- a
        kill is precisely what leaves an uncancelled goal behind. The margin is capped at half
        the budget so a small configured timeout cannot produce a deadline in the past.

        Nothing may move this outward, not even the queue allowance: the margin belongs to the
        dispatcher, not to the driver.
        """
        margin = min(max(1.0, self.release_margin_s), max(1.0, timeout_s / 2.0))
        return started_at + timeout_s - margin

    def run_deadline(self, started_at: float, timeout_s: float) -> float:
        """When this driver must stop if it never had to queue.

        ``queue_allowance_s`` is held back here rather than spent, so a driver that is queued
        gets it back (``after_queue``).

        **It is NOT true that an unqueued driver is unaffected**, and the previous version of
        this docstring said it was. The allowance is subtracted unconditionally, so a driver
        that never queued runs under ``hard - allowance`` and loses the whole allowance:
        measured on N07, unqueued drivers reported ``run_deadline_s = 195.0`` against a hard
        deadline of 285.0, i.e. 90 s withheld. The behaviour is deliberate -- a driver may not
        borrow the reserve it has not earned -- but the arithmetic deserves to be stated
        plainly rather than described as having no effect.

        Charging queue time to the movement budget is what failed N04: r02 spent 42.9 s of its
        165 s waiting for r01, then ran out of budget 2.6 s into its last stage with the permit
        in hand.
        """
        hard = self.hard_deadline(started_at, timeout_s)
        return max(started_at, hard - max(0.0, self.queue_allowance_s))

    def after_queue(self, deadline: float, queued_s: float, started_at: float,
                    timeout_s: float) -> float:
        """Give back the time spent queued, capped twice.

        By the allowance (a scenario says how long a queue it expects) and by the hard deadline
        (so this can never become a driver that outlives its dispatcher). Returns a deadline,
        never a duration, because a caller that has to remember what it is holding is a caller
        that will one day hold the wrong thing.
        """
        credit = min(max(0.0, queued_s), max(0.0, self.queue_allowance_s))
        return min(self.hard_deadline(started_at, timeout_s), deadline + credit)

    def stage_deadline(self, started_at: float, timeout_s: float,
                       run_deadline: float) -> float:
        """One stage may not outlive the run budget, which is what the parameter means.

        The dispatcher's ``crossing_timeout_s`` docstring already says this value is
        "handed to the passage driver AND used as this node's own deadline, so the two
        cannot disagree about when it failed". The driver was using it per stage
        instead, so a six-stage crossing could take six times the budget and only the
        dispatcher's kill ever stopped it.
        """
        return min(started_at + timeout_s, run_deadline)

    # ------------------------------------------------------------------ #
    # the reserve check (D-P17-08)
    # ------------------------------------------------------------------ #

    def reserve_ok(self, expected_remaining_s: float, now: float,
                   run_deadline: float) -> bool:
        """May this driver ENTER the passage, given what is left of its budget?

        The budget exists for one purpose: to get a robot through a capacity-1 passage and out
        the far side. A driver that enters with less time than the passage takes does not fail
        on its own -- it fails while holding a resource every other robot is queued behind.

        Measured N05 seed2 (2026-09-18): r02 arrived at its last stage with the permit in hand
        and 2.2 s of a 195.0 s budget left. It could not finish, and no check had asked.

        ``expected_remaining_s`` is the scenario's own estimate of the passage (declared, not
        inferred -- see CONTRACTS section 9), so this refuses an entry the caller already
        expects to overrun, and does not second-guess a scenario that declared a short one.
        """
        return (run_deadline - now) >= max(0.0, expected_remaining_s)

    def reserve_detail(self, expected_remaining_s: float, now: float,
                       run_deadline: float, stage: str) -> str:
        """Say the two numbers. A refusal that does not is a refusal nobody can act on."""
        return (
            f"refused to enter {stage}: {run_deadline - now:.1f}s of budget left against an "
            f"expected {expected_remaining_s:.1f}s to clear the passage. Entering would fail "
            f"*inside* a capacity-1 resource that every other robot is queued behind, which "
            f"is worse than not entering -- the queue would inherit the failure."
        )

    def classify_stop(self, stage: str, stage_elapsed_s: float, stage_expected_s: float,
                      run_deadline: float, now: float) -> str:
        """``infra_timeout`` or ``run_deadline``? Decided from evidence, not from the clock.

        A stage that overran its own estimate while the box was merely slow is an
        infrastructure result. A stage that ran out of run budget after a long queue is a
        budget result. The difference is whether the stage was still making progress at the
        rate it was expected to.

        CONTRACTS section 9 requires this distinction because the reader's next action
        depends on it: a slow machine is re-run, a control failure is debugged.
        """
        if now < run_deadline:
            return STOP_RUN_DEADLINE
        if stage_expected_s > 0 and stage_elapsed_s > stage_expected_s:
            return STOP_INFRA_TIMEOUT
        return STOP_RUN_DEADLINE

    def stopped_detail_infra(self, stage: str, elapsed_s: float,
                             expected_s: float) -> str:
        return (f"stopped during {stage} after {elapsed_s:.1f}s against an expected "
                f"{expected_s:.1f}s -- the stage was still the bottleneck when the budget "
                f"ended, so this is INFRA_TIMEOUT, not a control failure. Re-run on a "
                f"faster machine before reading it as a driver fault.")

    def busy_exhausted(self, busy_since: float, now: float) -> bool:
        return (now - busy_since) >= self.busy_budget_s

    def busy_detail(self, name: str, waited_s: float, attempts: int) -> str:
        """A reader has to be able to tell "busy" from "absent" without the log."""
        return (
            f"goal to {name} was rejected {attempts} time(s) over {waited_s:.1f}s: "
            f"another navigator held this robot's server for the whole "
            f"{self.busy_budget_s:.0f}s budget. The holder need not be this driver -- a "
            f"goal whose driver was killed stays active until Nav2 abandons it, which "
            f"took 8 s in the run this budget came from."
        )

    def stopped_detail(self, stopped_by: str, stage: str, elapsed_s: float) -> str:
        if stopped_by == STOP_SIGNAL:
            return (f"stopped by signal during {stage} after {elapsed_s:.1f}s; the active "
                    f"nav goal was cancelled before exit")
        if stopped_by == STOP_INFRA_TIMEOUT:
            return (f"stopped during {stage} after {elapsed_s:.1f}s with the stage still "
                    f"unfinished: INFRA_TIMEOUT. The machine could not run this scenario in "
                    f"the time the scenario declared, which is not a statement about the "
                    f"controller.")
        if stopped_by == STOP_RUN_DEADLINE:
            return (f"reached this driver's own budget during {stage} after "
                    f"{elapsed_s:.1f}s and stopped early, so the dispatcher would not "
                    f"have to kill it; the active nav goal was cancelled")
        return f"stopped during {stage} after {elapsed_s:.1f}s ({stopped_by})"


def stop_is_early_enough(run_deadline: float, dispatcher_deadline: float) -> bool:
    """The invariant that failed. Stated as a function so a test can assert it directly.

    If this is False for a driver, that driver will be killed rather than stop itself,
    and a killed driver leaves its goal behind.
    """
    return run_deadline < dispatcher_deadline
