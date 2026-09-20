"""Round 17 item 8c: the six capabilities the last eleven cases were waiting for.

WHAT WAS MISSING, AND WHY IT IS SIX AND NOT ELEVEN
==================================================

Eleven cases declared a `requires:` key. They collapse onto six capabilities, because
three of the keys are one mechanism seen from three angles:

    obstacle_injection        F01, F02                     spawn/despawn a body in the world
    world_pause_control       I04, I05                     freeze/unfreeze gz + clock
    truth_collector_control   I06                          stop the truth collector mid-run
    db_fault_injection        I07                          make the ledger refuse a write
    permit_renewal_interrupt  F10                          stop a permit being renewed
    batch_driver_extended     F03, I03, I08                three distinct driver extensions

`batch_driver_extended` is three cases and three genuinely different mechanisms, which is
why it is the one key that stays plural. It is kept as ONE key because the three share the
property that made them one entry: each needs a new *action*, not a new *service*, and an
action is where the runner's authority to change the world lives.

WHAT THIS MODULE IS AND IS NOT
==============================

This is the pure half: the rules that decide whether an injected fault is a fault at all.
It does not talk to ROS, gz or the filesystem. `scripts/batch.py` owns the actuation, the
same split as everywhere else in this project, and for the same reason -- a rule buried in
a subprocess call cannot be tested without starting a simulator.

THE RULE THAT MOST OF THESE SHARE
=================================

Every one of the six has a way to be delivered that makes the case test NOTHING while
reporting success:

  * spawn an obstacle AROUND the robot rather than in its path -> the robot never meets it;
  * pause a world that was already paused -> nothing changes and the case "survives";
  * stop a truth collector that was never running -> the safety verdict is vacuously PASS;
  * fail a write the ledger never attempts -> the ledger "survived";
  * stop renewing a permit that had already lapsed -> the robot was already stuck;
  * deliver a stale result to a task that had finished -> nobody was fooled.

CONTRACTS section 5 already states the general form of this for the kill trigger: a fault
that was merely requested and never took effect turns a pass into a silent no-op. So each
capability here exports a `precondition` that the runner MUST check before it acts, and a
`confirm` that reads back what actually changed. The pair is the whole point: the
precondition stops the vacuous injection, and the confirm stops a precondition that was
true from being reported as an injection that happened.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Mapping

# --------------------------------------------------------------------------- #
# the vocabulary
# --------------------------------------------------------------------------- #

#: The capability keys. Kept here rather than in the YAML so `check_batch_manifest`
#: and `batch.py` parse one list instead of three that drift.
#:
#: Listing a key means THIS BUILD KNOWS IT, which is not the same as "something implements
#: it". `pad_occupation_by_crossing` (added 2026-09-20, D-P17-30) is the case that makes the
#: distinction matter: it is a key with nothing behind it, because F03's gap is not a missing
#: action (`drive_to_pose` exists) but a missing legal arrival point -- a pad cannot be arrived
#: at, so no drive can occupy one. The register in docs/LIMITATIONS.md records the intent; this
#: list only says the name is recognised rather than a typo.
#:
#: Seven, not six, before that: `low_battery_while_executing` was F05's key and it was filled
#: by the SAME round, one item earlier (item 8b, `loaded_battery_policy`).
CAPABILITY_KEYS: "tuple[str, ...]" = (
    "obstacle_injection",
    "world_pause_control",
    "truth_collector_control",
    "db_fault_injection",
    "permit_renewal_interrupt",
    "batch_driver_extended",
    "low_battery_while_executing",
    "pad_occupation_by_crossing",
)

#: The actions this round adds. `KNOWN_ACTIONS` in `fleet_core.trigger` is the register a
#: manifest is validated against; these are merged into it there.
NEW_ACTIONS: "tuple[str, ...]" = (
    "spawn_obstacle",
    "despawn_obstacle",
    "pause_world",
    "resume_world",
    #: I05 moves the clock as well as freezing it. A separate action because I04 and I05
    #: differ in exactly this, and one action doing both would let a case assert either.
    "jump_clock",
    "stop_truth_collector",
    "start_truth_collector",
    "fail_ledger_writes",
    "heal_ledger_writes",
    "suspend_permit_renewal",
    "resume_permit_renewal",
    "drive_to_pose",
    "inject_stale_goal_result",
    "terminate_run",
)


@dataclass(frozen=True)
class Check:
    """One precondition or confirmation, and the sentence that says what it read."""

    ok: bool
    detail: str
    #: True when a failure means the injection can NEVER take effect this run. A caller
    #: that cannot tell "not yet" from "never" waits out a budget and then reports a
    #: timeout, which is the vague message this project keeps paying for.
    impossible: bool = False


def _ok(detail: str) -> Check:
    return Check(True, detail)


def _no(detail: str, *, impossible: bool = False) -> Check:
    return Check(False, detail, impossible)


# --------------------------------------------------------------------------- #
# 1. obstacle_injection -- F01, F02
# --------------------------------------------------------------------------- #

#: A spawned body is a robot-shaped box. The footprint matters: an obstacle narrower than
#: the corridor is a body the planner routes AROUND, which is F01, while one wider than the
#: corridor is a body that cannot be routed around, which is F02. One number decides which
#: case is which, so it is named and refused rather than defaulted.
MIN_OBSTACLE_RADIUS_M = 0.20
#: The corridor's own clearance. `config/resources.yaml` gives the gap y in [-0.65, 0.65],
#: so 1.30 m of free height. An obstacle of at least this radius on the centreline leaves
#: less than a robot's width either side.
CORRIDOR_HALF_HEIGHT_M = 0.65


@dataclass(frozen=True)
class ObstacleSpec:
    x: float
    y: float
    radius_m: float
    #: Which case's intent this body serves. Reported so F01 and F02 cannot silently swap.
    intent: str


def obstacle_precondition(pose: "tuple[float, float, float] | None",
                          spec: ObstacleSpec,
                          *, footprint_radius_m: float) -> Check:
    """Refuse to place a body on top of a robot.

    Section 5 forbids teleporting a body into the robot's footprint, and there is a
    second reason that is worth stating: a body spawned overlapping a robot is not an
    obstacle the planner ever routes around -- the robot is already inside it, and the
    costmap marks the cells the robot occupies as lethal beneath it. The case then passes
    or fails for reasons unrelated to what it is about.
    """
    if pose is None:
        return _no("no pose for the robot this obstacle is aimed at; placing a body "
                   "without knowing where the robot is cannot be done safely",
                   impossible=True)
    dx, dy = spec.x - pose[0], spec.y - pose[1]
    gap = (dx * dx + dy * dy) ** 0.5
    need = footprint_radius_m + spec.radius_m
    if gap < need:
        return _no(f"the body would be centred {gap:.2f} m from the robot, which is inside "
                   f"its footprint plus the body's radius ({need:.2f} m); section 5 forbids "
                   f"teleporting a body into the robot")
    return _ok(f"body centred {gap:.2f} m clear of the robot ({need:.2f} m needed) for "
               f"{spec.intent}")


def obstacle_leaves_a_route(spec: ObstacleSpec) -> bool:
    """Does this body leave enough corridor for a robot to get past?

    This is the difference between F01 and F02 stated as arithmetic rather than as prose.
    F01 wants a route to exist so the case can watch it be taken; F02 wants none to exist,
    so it can watch the fleet refuse. A runner that placed the same width for both would
    make one of the two cases vacuous, and nothing in the run would say which.
    """
    free = CORRIDOR_HALF_HEIGHT_M - spec.radius_m
    return free >= MIN_OBSTACLE_RADIUS_M


def obstacle_confirm(before: Mapping[str, Any], after: Mapping[str, Any]) -> Check:
    """Did the world actually gain a body?"""
    b, a = int(before.get("models") or 0), int(after.get("models") or 0)
    if a > b:
        return _ok(f"the world went from {b} model(s) to {a}")
    return _no(f"the world still reports {a} model(s) (was {b}); the body was requested and "
               f"never appeared, which is the silent no-op section 5 refuses")


# --------------------------------------------------------------------------- #
# 2. world_pause_control -- I04, I05
# --------------------------------------------------------------------------- #


def pause_precondition(world_paused: bool, *, what: str) -> Check:
    """Refuse to pause an already-paused world, or resume a running one.

    A pause applied twice is indistinguishable from one applied once, so "the fleet
    survived the pause" would be a statement about a world that never paused. This is the
    same failure shape as a fault that was requested and never took effect.
    """
    if what == "pause" and world_paused:
        return _no("the world is already paused; a second pause changes nothing and the "
                   "case would report surviving a pause that had already happened",
                   impossible=True)
    if what == "resume" and not world_paused:
        return _no("the world is not paused, so resuming it is a no-op", impossible=True)
    return _ok(f"the world is {'paused' if world_paused else 'running'}, so {what} will "
               f"change it")


def clock_jump_precondition(clock_s: float, jump_s: float) -> Check:
    """I05 resets the clock. A jump that does not move it is not a clock fault.

    Kept separate from the pause check because I04 and I05 differ in exactly this: I04
    freezes time, I05 moves it backwards. They share the pause mechanism and not the
    assertion, and merging them would make one of the two untested.
    """
    if jump_s == 0.0:
        return _no("a zero-second jump does not move the clock, so I05 would assert "
                   "nothing", impossible=True)
    if jump_s < 0.0 and clock_s + jump_s < 0.0:
        return _no(f"jumping {jump_s:.3f} s from {clock_s:.3f} s would take the clock "
                   f"negative, which is a different fault from a reset")
    return _ok(f"clock at {clock_s:.3f} s will move by {jump_s:+.3f} s")


@dataclass(frozen=True)
class PauseObservation:
    """What a case may read while the world is frozen."""

    snapshot_taken_while_paused: bool
    state_age_s: float
    robot_reported_fresh: bool

    def staleness_was_observable(self, window_s: float) -> Check:
        """I04's real content: with the world frozen, a robot cannot refresh its state, and
        the fleet has to notice. If nothing ever read as stale, the pause did not reach the
        thing under test.
        """
        if not self.snapshot_taken_while_paused:
            return _no("no snapshot was taken while the world was paused, so there is "
                       "nothing to say about staleness")
        if self.state_age_s <= window_s:
            return _no(f"the oldest state was {self.state_age_s:.2f} s old against a "
                       f"{window_s:.2f} s window, so nothing was ever stale; the pause did "
                       f"not reach the task layer")
        return _ok(f"state aged to {self.state_age_s:.2f} s against a {window_s:.2f} s "
                   f"window, so the pause was observable")


# --------------------------------------------------------------------------- #
# 3. truth_collector_control -- I06
# --------------------------------------------------------------------------- #


def collector_precondition(collector_running: bool, samples: int, *, why: str) -> Check:
    """Refuse to stop a collector that is not running, or that has recorded nothing.

    The second half is the one that matters. Stopping a collector that has recorded zero
    samples means the safety verdict for the rest of the run comes from an empty stream,
    and an empty stream has no unauthorised entry in it. That reads as PASS. I06 exists to
    check the fleet copes with the truth going away, not to make the safety verdict vacuous.
    """
    if why == "start" and collector_running:
        return _no("the truth collector is already running", impossible=True)
    if why == "stop":
        if not collector_running:
            return _no("the truth collector is not running, so stopping it is a no-op",
                       impossible=True)
        if samples <= 0:
            return _no("the truth collector has recorded no samples; stopping it now would "
                       "leave the rest of the run with no ground truth, and every "
                       "truth-based safety check would pass on an empty stream",
                       impossible=True)
    return _ok(f"collector running={collector_running} with {samples} sample(s)")


def truth_absence_is_reported(after: Mapping[str, Any]) -> Check:
    """I06's assertion: the gap must be VISIBLE, not merely survivable.

    A run whose truth stream stopped and whose report does not say so is a run whose safety
    verdict cannot be read. The check is on the record, not on the fleet.
    """
    if bool(after.get("truth_gap_recorded")):
        return _ok(f"the report names a truth gap ({after.get('truth_gap_detail', '')})")
    return _no("the truth stream stopped and the report does not say so; every safety "
               "verdict after that point is unreadable rather than passing")


# --------------------------------------------------------------------------- #
# 4. db_fault_injection -- I07
# --------------------------------------------------------------------------- #

#: How the ledger is asked to fail. Section 6 requires a dedicated test database and a
#: controlled failure, NOT a full disk: a full disk also breaks the event log, the
#: recorder and the batch runner, so the run dies for reasons that are not the case.
LEDGER_FAULT_MODES: "tuple[str, ...]" = (
    "write_denied",     # the WAL refuses the commit
    "db_locked",        # a second connection holds an exclusive lock
    "disk_full_sim",    # the connection raises "database or disk is full"
)


def db_fault_precondition(mode: str, *, writes_seen: int, db_path: str,
                          is_test_db: bool) -> Check:
    """Refuse a ledger fault on the real database, or before any write has been attempted.

    The `is_test_db` half is a safety property and not a tidiness one: pointing a
    write-denial at the shipped ledger leaves it unusable for every later case in the same
    batch, and the failure would surface as an unrelated case failing.
    """
    if mode not in LEDGER_FAULT_MODES:
        return _no(f"fault mode {mode!r} is not one of {list(LEDGER_FAULT_MODES)}",
                   impossible=True)
    if not is_test_db:
        return _no(f"{db_path} is not a test database; a write denial here would outlive "
                   f"this case and break every case after it", impossible=True)
    if writes_seen <= 0:
        return _no("no ledger write has been attempted yet, so a refusal now would be "
                   "invisible: the ledger would look healthy because nothing was asked of it")
    return _ok(f"mode {mode!r} aimed at the test database {db_path}, after {writes_seen} "
               f"write(s)")


def db_fault_confirm(response_ok: bool, mode: str) -> Check:
    """Did the ledger actually refuse, rather than merely be told to?"""
    if response_ok:
        return _ok(f"the ledger refused a write with {mode!r}")
    return _no(f"the {mode!r} injection did not make any write fail; the case would report "
               f"that the ledger survived a fault it never met")


# --------------------------------------------------------------------------- #
# 5. permit_renewal_interrupt -- F10
# --------------------------------------------------------------------------- #


def permit_interrupt_precondition(permit_live: bool, robot_reporting: bool,
                                  *, renewals_seen: int) -> Check:
    """F10 is "renewal stops WHILE the robot keeps reporting". Both halves are conditions.

    Only the first half is obvious. The second is what makes the case non-trivial: if the
    robot had also gone quiet, the fleet would be right to withdraw the permit for
    staleness, and the case would be testing the stale-state path it did not mean to test.
    """
    if not permit_live:
        return _no("no permit is currently held, so there is no renewal to stop",
                   impossible=True)
    if renewals_seen <= 0:
        return _no("no renewal has been observed yet; interrupting now would stop a "
                   "renewal that never happened", impossible=True)
    if not robot_reporting:
        return _no("the robot is not reporting, so withdrawing its permit would be the "
                   "stale-state path rather than a renewal interrupt; F10 needs the robot "
                   "to keep reporting")
    return _ok(f"a live permit with {renewals_seen} renewal(s) seen and the robot still "
               f"reporting")


def permit_withdrawal_confirm(permit_after: bool, robot_still_fresh: bool,
                              fleet_acted: bool) -> Check:
    """The permit must lapse, the robot must still look healthy, and the fleet must DO
    something about it. A permit that lapsed with nothing noticing is a silent hole."""
    if permit_after:
        return _no("the permit is still held after the renewal was interrupted")
    if not robot_still_fresh:
        return _no("the robot stopped reporting as well, so this run did not isolate the "
                   "renewal interrupt")
    if not fleet_acted:
        return _no("the permit lapsed and the fleet took no action; a lapse nobody notices "
                   "is not a safety property")
    return _ok("the permit lapsed while the robot kept reporting, and the fleet acted")


# --------------------------------------------------------------------------- #
# 6. batch_driver_extended -- F03, I03, I08
# --------------------------------------------------------------------------- #


def drive_to_pose_precondition(current: "tuple[float, float, float] | None",
                               target: "tuple[float, float]",
                               *,
                               free: bool,
                               outside_protected: bool,
                               on_battery: bool) -> Check:
    """F03 needs a robot PARKED ON the exit buffer, which nothing could previously do.

    Three refusals, and the third is the one that is easy to miss:

      * the target must be free space -- driving into a wall is a different case;
      * the target must be outside the protected rectangles, unless occupying one IS the
        point (F03's whole intent is to occupy a pad, so this is passed explicitly rather
        than assumed);
      * the robot must be able to get there. A robot driven to a pose it cannot afford is
        a robot that stops somewhere else, and the case would then assert about a pose
        nothing ever reached.
    """
    if current is None:
        return _no("no current pose, so there is no start to drive from", impossible=True)
    if not free:
        return _no(f"the target {target} is not free space")
    if not outside_protected and target != (-2.5, -1.4):
        return _no(f"the target {target} is inside a protected rectangle; occupying one is "
                   f"F03's own intent and must be asked for by name")
    if not on_battery:
        return _no("the robot cannot reach that pose on its remaining charge, so the drive "
                   "would stop short and the case would assert about a pose nothing reached")
    return _ok(f"drive from {current[:2]} to {target}")


def drive_to_pose_confirm(pose_after: "tuple[float, float, float] | None",
                          target: "tuple[float, float]",
                          *, tolerance_m: float) -> Check:
    """Did it get there? Reported with the distance, because "it arrived" is unreadable."""
    if pose_after is None:
        return _no("the robot has no pose after the drive, so it cannot be said to have "
                   "arrived anywhere")
    dx, dy = pose_after[0] - target[0], pose_after[1] - target[1]
    got = (dx * dx + dy * dy) ** 0.5
    if got > tolerance_m:
        return _no(f"the robot is {got:.3f} m from {target} against a {tolerance_m:.3f} m "
                   f"tolerance")
    return _ok(f"the robot is {got:.3f} m from {target}")


def stale_result_precondition(task_state: str, *, cancelled: bool,
                              result_already_stale: bool) -> Check:
    """I03: a stale success callback arrives AFTER a cancel.

    Both halves are conditions, and the second is the trap. Injecting a stale result into a
    task that has already been told it is stale exercises nothing -- the guard that exists
    is the one for "a result I was not expecting", and it only runs once.
    """
    if not cancelled:
        return _no("the task was not cancelled, so a late result is not stale -- it is the "
                   "result", impossible=True)
    if task_state in ("SUCCEEDED", "FAILED", "NEEDS_ATTENTION"):
        return _no(f"the task is already {task_state}, so a late result would be dropped "
                   f"by the terminal check rather than by the staleness guard",
                   impossible=True)
    if result_already_stale:
        return _no("the goal handle is already marked stale, so the guard has already run "
                   "for it and a second injection is not seen", impossible=True)
    return _ok(f"the task is {task_state} and cancelled, with a live goal handle")


def stale_result_confirm(task_state: str, *, before: str, fake_success_seen: bool) -> Check:
    """The assertion: no fake success.

    CONTRACTS section 5's phrase for this is 无 fake 成功. A task that reached SUCCEEDED
    after a cancel and a stale success callback is exactly that, and this is the check that
    catches it rather than a human reading the log.
    """
    if task_state == "SUCCEEDED" and before != "SUCCEEDED":
        return _no(f"the task reached SUCCEEDED after being {before} and then cancelled: "
                   f"that is the fake success section 5 forbids")
    if fake_success_seen and task_state == "SUCCEEDED":
        return _no("a stale success callback was accepted as the task's outcome")
    return _ok(f"the task is {task_state} and no stale success was accepted")


def terminate_precondition(run_active: bool, run_dir: str, *, second_run_dir: str) -> Check:
    """I08: terminate THIS run, then start another.

    Two refusals that are really one rule -- the second run must be somewhere else. Two
    runs sharing a directory would let the second inherit the first's recordings, and I08's
    whole content is that a second run can be started from scratch. The state that leaked
    would be indistinguishable from the state that was rebuilt.
    """
    if not run_active:
        return _no("no run is active, so there is nothing to terminate", impossible=True)
    if not second_run_dir:
        return _no("the second run has no directory of its own", impossible=True)
    if run_dir and second_run_dir and run_dir.rstrip("/") == second_run_dir.rstrip("/"):
        return _no(f"the second run would use the same directory as the first ({run_dir}); "
                   f"I08 asks whether a run can be STARTED after one ends, and a shared "
                   f"directory answers a different question", impossible=True)
    return _ok(f"terminate {run_dir} and start {second_run_dir}")


@dataclass(frozen=True)
class TerminateOutcome:
    """What a terminated run leaves behind that the next one must not depend on."""

    first_run_clean: bool
    second_run_started: bool
    second_run_epoch: str
    first_run_epoch: str
    leaked_tasks: list[str] = field(default_factory=list)
    leaked_permits: list[str] = field(default_factory=list)

    def verdict(self) -> Check:
        """Did the first run really end, and did the second really start fresh?

        `leaked_tasks` is the interesting field. A task still open in the second run's
        ledger would mean the second run inherited the first's database -- which is the
        difference between I08 and F11, and the thing that would let I08 pass without a
        second run ever having started.
        """
        if not self.first_run_clean:
            return _no("the first run did not terminate cleanly, so what follows is not a "
                       "fresh run")
        if not self.second_run_started:
            return _no("the second run never started")
        if self.second_run_epoch == self.first_run_epoch:
            return _no(f"both runs report epoch {self.second_run_epoch!r}; the second run "
                       f"reused the first's process rather than starting")
        if self.leaked_tasks:
            return _no(f"the second run's ledger still holds {self.leaked_tasks} from the "
                       f"first; the database was inherited")
        if self.leaked_permits:
            return _no(f"the second run still holds permits {self.leaked_permits} from the "
                       f"first")
        return _ok(f"first run ended, second started under epoch "
                   f"{self.second_run_epoch!r} with nothing carried over")


# --------------------------------------------------------------------------- #
# the register
# --------------------------------------------------------------------------- #

#: For each new action: whether it needs a `runner`, and which extra fields it requires.
#: `scripts/check_batch_manifest.py` parses this, and `fleet_core.trigger` merges it, so a
#: manifest cannot name an action whose fields the runner would silently ignore.
ACTION_FIELDS: "dict[str, tuple[str, ...]]" = {
    "spawn_obstacle": ("x", "y", "radius_m"),
    "despawn_obstacle": ("name",),
    "pause_world": (),
    "resume_world": (),
    "jump_clock": ("seconds",),
    "stop_truth_collector": (),
    "start_truth_collector": (),
    "fail_ledger_writes": ("mode",),
    "heal_ledger_writes": (),
    "suspend_permit_renewal": (),
    "resume_permit_renewal": (),
    "drive_to_pose": ("x", "y"),
    "inject_stale_goal_result": ("request_id",),
    "terminate_run": ("second_run_dir",),
}

#: Actions whose target is a robot, so a `runner` is required rather than inferred.
ACTION_NEEDS_RUNNER: "frozenset[str]" = frozenset({
    "drive_to_pose",
})


def all_actions() -> "frozenset[str]":
    """The merged action register. One function so a caller never rebuilds the union."""
    from fleet_core.trigger import KNOWN_ACTIONS

    return frozenset(set(KNOWN_ACTIONS) | set(NEW_ACTIONS))


def all_required_fields() -> "dict[str, tuple[str, ...]]":
    from fleet_core.trigger import REQUIRED_FIELDS

    merged = dict(REQUIRED_FIELDS)
    merged.update(ACTION_FIELDS)
    return merged


def all_needs_runner() -> "frozenset[str]":
    from fleet_core.trigger import NEEDS_RUNNER

    return frozenset(set(NEEDS_RUNNER) | set(ACTION_NEEDS_RUNNER))
