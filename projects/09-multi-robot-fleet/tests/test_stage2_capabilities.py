"""Round 17 item 8c: the six capabilities the last eleven cases were waiting for.

THE SHAPE EVERY ONE OF THESE SHARES
===================================

Each capability has a way to be delivered that makes its case test NOTHING while reporting
success, and each of those ways is a REFUSAL rather than a delay:

  * spawn the obstacle around the robot instead of in its path;
  * pause a world that was already paused;
  * stop a truth collector that never ran, so every safety verdict is vacuously PASS;
  * deny a ledger write the ledger never attempts;
  * interrupt a permit that had already lapsed;
  * hand a stale result to a task that had already finished.

CONTRACTS section 5 states the general rule for the kill trigger: a fault that was merely
requested and never took effect turns a pass into a silent no-op. These tests are the same
rule applied five more times, and they assert the refusal rather than the happy path,
because the happy path is the half that already works.
"""
from __future__ import annotations

import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent / "src/fleet_core"))

from fleet_core.stage2_capabilities import (  # noqa: E402
    ACTION_FIELDS,
    ACTION_NEEDS_RUNNER,
    CAPABILITY_KEYS,
    CORRIDOR_HALF_HEIGHT_M,
    LEDGER_FAULT_MODES,
    MIN_OBSTACLE_RADIUS_M,
    NEW_ACTIONS,
    ObstacleSpec,
    PauseObservation,
    TerminateOutcome,
    clock_jump_precondition,
    collector_precondition,
    db_fault_confirm,
    db_fault_precondition,
    drive_to_pose_confirm,
    drive_to_pose_precondition,
    obstacle_confirm,
    obstacle_leaves_a_route,
    obstacle_precondition,
    pause_precondition,
    permit_interrupt_precondition,
    permit_withdrawal_confirm,
    stale_result_confirm,
    stale_result_precondition,
    terminate_precondition,
    truth_absence_is_reported,
)
from fleet_core.trigger import (  # noqa: E402
    KNOWN_ACTIONS,
    NEEDS_RUNNER,
    REQUIRED_FIELDS,
    validate,
)

ROBOT_AT = (2.26, 0.05, 0.0)
BODY_IN_PATH = ObstacleSpec(x=1.0, y=0.0, radius_m=0.30, intent="F01")
BODY_ON_ROBOT = ObstacleSpec(x=2.30, y=0.10, radius_m=0.30, intent="F01")


# --------------------------------------------------------------------------- #
# the register itself
# --------------------------------------------------------------------------- #


def test_all_capability_keys_are_named():
    """Seven keys, not six: `low_battery_while_executing` was F05's and it was filled by
    the same round, one item earlier. The guard's question is "does this build know this
    key", and for that one the answer is yes."""
    assert set(CAPABILITY_KEYS) == {
        "obstacle_injection",
        "world_pause_control",
        "truth_collector_control",
        "db_fault_injection",
        "permit_renewal_interrupt",
        "batch_driver_extended",
        "low_battery_while_executing",
        # Known, not implemented: F03's gap is a missing legal arrival point, not a missing
        # action. See D-P17-30 and the register in docs/LIMITATIONS.md.
        "pad_occupation_by_crossing",
    }, "a capability key was renamed; the manifest guards parse this list"


def test_every_new_action_is_known_to_the_validator():
    """The merge is the thing under test, not the constant.

    `NEW_ACTIONS` existing is not the same as `validate()` accepting it: the manifest guard
    parses `KNOWN_ACTIONS`, so an action missing from the merged set would be refused at
    validation time and the case would be reported BLOCKED for a capability that exists.
    """
    missing = sorted(set(NEW_ACTIONS) - KNOWN_ACTIONS)
    assert missing == [], f"these actions are not accepted by validate(): {missing}"


def test_known_actions_is_a_LITERAL_the_guard_can_parse():
    """The reason `KNOWN_ACTIONS` is spelled out rather than computed, asserted.

    `scripts/check_batch_manifest.py` reads that name out of the source with `ast` and
    requires a literal collection of string constants. A computed union
    (`frozenset(set(a) | set(b))`) is not visible to that parse, so the guard returned
    "could not read KNOWN_ACTIONS" and -- correctly -- failed every step in the manifest.

    This test is ALSO the drift check that the literal spelling costs. Without it, adding an
    action to `NEW_ACTIONS` and forgetting the literal would pass every test except the
    guard, which reports it as an unreadable vocabulary rather than as a missing action.
    """
    import ast
    import pathlib as _p

    trigger = (_p.Path(__file__).resolve().parent.parent
               / "src/fleet_core/fleet_core/trigger.py")
    tree = ast.parse(trigger.read_text(encoding="utf-8"))
    literal = None
    for node in ast.walk(tree):
        if (isinstance(node, ast.AnnAssign) and isinstance(node.target, ast.Name)
                and node.target.id == "KNOWN_ACTIONS"):
            if isinstance(node.value, ast.Call) and node.value.args \
                    and isinstance(node.value.args[0], ast.Set):
                literal = sorted(e.value for e in node.value.args[0].elts
                                 if isinstance(e, ast.Constant))
    assert literal is not None, (
        "KNOWN_ACTIONS is no longer an AST-readable literal set; "
        "scripts/check_batch_manifest.py will refuse the whole manifest"
    )
    assert set(literal) == set(KNOWN_ACTIONS), (
        f"the literal and the runtime value disagree: "
        f"literal-only {sorted(set(literal) - set(KNOWN_ACTIONS))}, "
        f"runtime-only {sorted(set(KNOWN_ACTIONS) - set(literal))}"
    )
    assert set(NEW_ACTIONS) <= set(literal), (
        f"a stage-2 action is missing from the literal: "
        f"{sorted(set(NEW_ACTIONS) - set(literal))}"
    )


def test_every_new_action_declares_its_required_fields():
    missing = sorted(set(NEW_ACTIONS) - set(REQUIRED_FIELDS))
    assert missing == [], f"no field spec for {missing}"


def test_a_new_action_validates_end_to_end():
    ok = validate({"do": {"action": "spawn_obstacle", "x": 1.0, "y": 0.0, "radius_m": 0.3}})
    assert ok == [], ok
    bad = validate({"do": {"action": "spawn_obstacle", "x": 1.0}})
    assert any("radius_m" in f for f in bad), bad


def test_drive_to_pose_needs_a_runner():
    """The only new action that acts on one robot, so the only one that needs naming."""
    assert "drive_to_pose" in NEEDS_RUNNER
    assert ACTION_NEEDS_RUNNER == frozenset({"drive_to_pose"})
    findings = validate({"do": {"action": "drive_to_pose", "x": 1.0, "y": 0.0}})
    assert any("runner" in f for f in findings), findings


def test_a_step_naming_an_absent_runner_is_refused():
    findings = validate(
        {"do": {"action": "drive_to_pose", "x": 1.0, "y": 0.0, "runner": "r03"}},
        runners=["r01"],
    )
    assert any("r03" in f for f in findings), findings


# --------------------------------------------------------------------------- #
# 1. obstacle_injection
# --------------------------------------------------------------------------- #


def test_obstacle_in_the_robots_path_is_allowed():
    assert obstacle_precondition(ROBOT_AT, BODY_IN_PATH, footprint_radius_m=0.30).ok


def test_obstacle_on_top_of_the_robot_is_refused():
    """Section 5 forbids teleporting a body into the robot's footprint. The second reason
    is sharper: the robot is already inside the body, so the planner never routes around it
    and the case passes or fails for reasons unrelated to what it asks."""
    c = obstacle_precondition(ROBOT_AT, BODY_ON_ROBOT, footprint_radius_m=0.30)
    assert c.ok is False
    assert "footprint" in c.detail


def test_the_clearance_uses_both_radii_not_just_one():
    """A point exactly one radius away is still illegal, because the body has one too."""
    spec = ObstacleSpec(x=ROBOT_AT[0] + 0.45, y=ROBOT_AT[1], radius_m=0.30, intent="F01")
    c = obstacle_precondition(ROBOT_AT, spec, footprint_radius_m=0.30)
    assert c.ok is False, "0.45 m is one and a half radii, not two"


def test_obstacle_without_a_robot_pose_is_refused_as_impossible():
    c = obstacle_precondition(None, BODY_IN_PATH, footprint_radius_m=0.30)
    assert c.ok is False
    assert c.impossible is True, "not-yet and never must stay apart"


def test_a_body_narrower_than_the_corridor_leaves_a_route():
    """F01: the case watches the route be taken, so a route must exist."""
    assert obstacle_leaves_a_route(ObstacleSpec(0.0, 0.0, 0.25, "F01")) is True


def test_a_body_that_fills_the_corridor_leaves_none():
    """F02: the only corridor is blocked FOR GOOD. The same width for both cases would make
    one of the two vacuous and nothing in the run would say which."""
    assert obstacle_leaves_a_route(ObstacleSpec(0.0, 0.0, 0.60, "F02")) is False


def test_the_two_cases_are_distinguished_by_width_and_cannot_swap():
    narrow = ObstacleSpec(0.0, 0.0, MIN_OBSTACLE_RADIUS_M, "F01")
    wide = ObstacleSpec(0.0, 0.0, CORRIDOR_HALF_HEIGHT_M, "F02")
    assert obstacle_leaves_a_route(narrow) != obstacle_leaves_a_route(wide)


def test_obstacle_confirm_needs_the_world_to_actually_gain_a_body():
    assert obstacle_confirm({"models": 4}, {"models": 5}).ok
    c = obstacle_confirm({"models": 4}, {"models": 4})
    assert c.ok is False
    assert "silent no-op" in c.detail


# --------------------------------------------------------------------------- #
# 2. world_pause_control
# --------------------------------------------------------------------------- #


def test_pausing_a_running_world_is_allowed():
    assert pause_precondition(False, what="pause").ok


def test_pausing_an_already_paused_world_is_refused_as_impossible():
    c = pause_precondition(True, what="pause")
    assert c.ok is False and c.impossible is True
    assert "already paused" in c.detail


def test_resuming_a_running_world_is_refused():
    c = pause_precondition(False, what="resume")
    assert c.ok is False and c.impossible is True


def test_a_zero_second_clock_jump_is_not_a_clock_fault():
    """I05 moves the clock. A jump of zero is I05 asserting nothing at all."""
    c = clock_jump_precondition(100.0, 0.0)
    assert c.ok is False and c.impossible is True


def test_a_backwards_jump_below_zero_is_a_different_fault():
    c = clock_jump_precondition(10.0, -50.0)
    assert c.ok is False
    assert "negative" in c.detail


def test_a_real_clock_reset_is_allowed():
    assert clock_jump_precondition(300.0, -250.0).ok


def test_the_clock_jump_is_a_registered_ACTION_not_a_side_effect_of_pausing():
    """The manifest guard caught this the first time it ran: I05 named `jump_clock` and no
    such action existed, so the guard refused the whole manifest rather than passing a step
    that would have done nothing.

    Kept as a test because the guard's failure message is the only other place this is
    visible, and a guard that refuses a file is easy to "fix" by editing the file.
    """
    assert "jump_clock" in NEW_ACTIONS
    assert "jump_clock" in KNOWN_ACTIONS
    assert REQUIRED_FIELDS["jump_clock"] == ("seconds",), (
        "a jump with no amount would be an action that does nothing"
    )


def test_the_pause_is_only_proven_if_something_went_stale():
    """I04's actual content. A frozen world where nothing was ever late means the pause did
    not reach the task layer, and the case would report surviving a fault it never met."""
    late = PauseObservation(True, 12.0, False)
    assert late.staleness_was_observable(3.0).ok

    early = PauseObservation(True, 0.5, True)
    c = early.staleness_was_observable(3.0)
    assert c.ok is False
    assert "did not reach the task layer" in c.detail


def test_a_pause_with_no_snapshot_proves_nothing():
    c = PauseObservation(False, 0.0, True).staleness_was_observable(3.0)
    assert c.ok is False


# --------------------------------------------------------------------------- #
# 3. truth_collector_control
# --------------------------------------------------------------------------- #


def test_stopping_a_running_collector_with_samples_is_allowed():
    assert collector_precondition(True, 500, why="stop").ok


def test_stopping_a_dead_collector_is_refused():
    c = collector_precondition(False, 0, why="stop")
    assert c.ok is False and c.impossible is True


def test_stopping_an_empty_collector_is_refused_because_it_makes_safety_vacuous():
    """The important one. An empty truth stream has no unauthorised entry in it, so every
    truth-based safety check passes. Stopping a collector with zero samples would turn the
    rest of the run's safety verdict into a statement about nothing."""
    c = collector_precondition(True, 0, why="stop")
    assert c.ok is False and c.impossible is True
    assert "empty stream" in c.detail


def test_starting_an_already_running_collector_is_refused():
    c = collector_precondition(True, 100, why="start")
    assert c.ok is False and c.impossible is True


def test_the_truth_gap_must_be_visible_in_the_report():
    """I06's assertion is on the RECORD, not on the fleet: a run whose truth stopped and
    whose report does not say so has an unreadable safety verdict, not a passing one."""
    assert truth_absence_is_reported({"truth_gap_recorded": True,
                                      "truth_gap_detail": "t=412s"}).ok
    c = truth_absence_is_reported({"truth_gap_recorded": False})
    assert c.ok is False
    assert "unreadable" in c.detail


# --------------------------------------------------------------------------- #
# 4. db_fault_injection
# --------------------------------------------------------------------------- #


def test_a_write_denial_on_the_real_database_is_refused():
    """A safety property, not tidiness: a denial pointed at the shipped ledger outlives the
    case and surfaces as an UNRELATED case failing later in the batch."""
    c = db_fault_precondition("write_denied", writes_seen=10,
                              db_path="/var/lib/fleet/tasks.db", is_test_db=False)
    assert c.ok is False and c.impossible is True
    assert "outlive" in c.detail


def test_a_refusal_before_any_write_is_refused():
    c = db_fault_precondition("write_denied", writes_seen=0,
                              db_path="/tmp/i07.db", is_test_db=True)
    assert c.ok is False
    assert "nothing was asked of it" in c.detail


def test_an_unknown_fault_mode_is_refused():
    c = db_fault_precondition("rm_the_file", writes_seen=5,
                              db_path="/tmp/i07.db", is_test_db=True)
    assert c.ok is False and c.impossible is True


def test_a_valid_ledger_fault_is_allowed():
    assert db_fault_precondition("write_denied", writes_seen=5,
                                 db_path="/tmp/i07.db", is_test_db=True).ok


def test_section_6_names_a_controlled_failure_not_a_full_disk():
    """`disk_full_sim` is a simulated mode for exactly this reason: a real full disk also
    breaks the event log, the recorder and the batch runner, so the run dies for reasons
    that are not the case."""
    assert "disk_full_sim" in LEDGER_FAULT_MODES
    assert all("sim" in m or m in ("write_denied", "db_locked")
               for m in LEDGER_FAULT_MODES)


def test_the_injection_must_make_a_write_actually_fail():
    assert db_fault_confirm(True, "write_denied").ok
    c = db_fault_confirm(False, "write_denied")
    assert c.ok is False
    assert "never met" in c.detail


# --------------------------------------------------------------------------- #
# 5. permit_renewal_interrupt
# --------------------------------------------------------------------------- #


def test_the_normal_f10_setup_is_allowed():
    assert permit_interrupt_precondition(True, True, renewals_seen=3).ok


def test_no_live_permit_is_refused():
    c = permit_interrupt_precondition(False, True, renewals_seen=3)
    assert c.ok is False and c.impossible is True


def test_no_renewal_seen_yet_is_refused():
    c = permit_interrupt_precondition(True, True, renewals_seen=0)
    assert c.ok is False


def test_a_quiet_robot_is_refused_because_it_is_a_DIFFERENT_case():
    """The trap. If the robot had also gone quiet the fleet would be right to withdraw the
    permit for staleness, so the case would be testing the stale-state path it did not mean
    to test. F10 says 机器人持续上报 -- the robot keeps reporting."""
    c = permit_interrupt_precondition(True, False, renewals_seen=3)
    assert c.ok is False
    assert "stale-state path" in c.detail


def test_the_permit_must_lapse_while_the_robot_stays_fresh():
    assert permit_withdrawal_confirm(False, True, True).ok

    assert permit_withdrawal_confirm(True, True, True).ok is False
    assert permit_withdrawal_confirm(False, False, True).ok is False


def test_a_lapse_nobody_noticed_is_not_a_safety_property():
    c = permit_withdrawal_confirm(False, True, False)
    assert c.ok is False
    assert "nobody notices" in c.detail


# --------------------------------------------------------------------------- #
# 6. batch_driver_extended -- F03
# --------------------------------------------------------------------------- #

EXIT_PAD = (-2.5, -1.4)


def test_f03_may_occupy_the_pad_by_name():
    """F03's whole intent IS to occupy a pad, so the protected-rectangle refusal is passed
    explicitly rather than removed -- an accident and an intention must not look alike."""
    assert drive_to_pose_precondition(ROBOT_AT, EXIT_PAD, free=True,
                                      outside_protected=False, on_battery=True).ok


def test_an_accidental_protected_target_is_still_refused():
    elsewhere = (0.0, 0.0)  # the corridor's centre
    c = drive_to_pose_precondition(ROBOT_AT, elsewhere, free=True,
                                   outside_protected=False, on_battery=True)
    assert c.ok is False
    assert "by name" in c.detail


def test_a_target_inside_a_wall_is_refused():
    c = drive_to_pose_precondition(ROBOT_AT, (0.0, 0.0), free=False,
                                   outside_protected=True, on_battery=True)
    assert c.ok is False


def test_a_target_the_robot_cannot_afford_is_refused():
    """Otherwise the robot stops somewhere else and the case asserts about a pose nothing
    ever reached."""
    c = drive_to_pose_precondition(ROBOT_AT, EXIT_PAD, free=True,
                                   outside_protected=False, on_battery=False)
    assert c.ok is False
    assert "nothing reached" in c.detail


def test_arrival_is_reported_with_its_distance():
    assert drive_to_pose_confirm((-2.51, -1.41, 0.0), EXIT_PAD, tolerance_m=0.10).ok
    c = drive_to_pose_confirm((-2.0, -1.4, 0.0), EXIT_PAD, tolerance_m=0.10)
    assert c.ok is False
    assert "0.500" in c.detail


# --------------------------------------------------------------------------- #
# 6b. batch_driver_extended -- I03
# --------------------------------------------------------------------------- #


def test_the_normal_stale_result_setup_is_allowed():
    assert stale_result_precondition("CANCELED", cancelled=True,
                                     result_already_stale=False).ok


def test_a_late_result_on_a_task_that_was_never_cancelled_is_the_result():
    c = stale_result_precondition("EXECUTING", cancelled=False,
                                  result_already_stale=False)
    assert c.ok is False and c.impossible is True


def test_a_terminal_task_is_refused_because_the_wrong_guard_would_catch_it():
    """Injecting a stale result into a task that already succeeded exercises the TERMINAL
    check, not the staleness guard -- so the guard I03 is about never runs."""
    c = stale_result_precondition("SUCCEEDED", cancelled=True,
                                  result_already_stale=False)
    assert c.ok is False and c.impossible is True
    assert "terminal check" in c.detail


def test_a_second_injection_into_the_same_goal_is_refused():
    c = stale_result_precondition("CANCELED", cancelled=True, result_already_stale=True)
    assert c.ok is False and c.impossible is True


def test_no_fake_success_is_the_assertion():
    assert stale_result_confirm("NEEDS_ATTENTION", before="CANCELED",
                                fake_success_seen=True).ok
    c = stale_result_confirm("SUCCEEDED", before="CANCELED", fake_success_seen=True)
    assert c.ok is False
    assert "fake success" in c.detail


def test_succeeding_after_a_cancel_is_a_fake_success_even_without_the_callback():
    c = stale_result_confirm("SUCCEEDED", before="EXECUTING", fake_success_seen=False)
    assert c.ok is False


# --------------------------------------------------------------------------- #
# 6c. batch_driver_extended -- I08
# --------------------------------------------------------------------------- #


def test_terminating_and_starting_elsewhere_is_allowed():
    assert terminate_precondition(True, "/runs/a", second_run_dir="/runs/b").ok


def test_the_second_run_must_not_share_the_first_directory():
    """Two runs sharing a directory let the second inherit the first's recordings, and the
    state that leaked is indistinguishable from the state that was rebuilt."""
    c = terminate_precondition(True, "/runs/a", second_run_dir="/runs/a")
    assert c.ok is False and c.impossible is True


def test_no_active_run_is_refused():
    c = terminate_precondition(False, "/runs/a", second_run_dir="/runs/b")
    assert c.ok is False and c.impossible is True


def test_a_clean_handover_passes():
    v = TerminateOutcome(True, True, "epoch-2", "epoch-1").verdict()
    assert v.ok


def test_the_same_epoch_means_the_process_never_restarted():
    v = TerminateOutcome(True, True, "epoch-1", "epoch-1").verdict()
    assert v.ok is False
    assert "reused the first's process" in v.detail


def test_an_inherited_ledger_is_caught():
    """The field that makes I08 different from F11: a task still open in the second run's
    ledger means the database was inherited rather than rebuilt."""
    v = TerminateOutcome(True, True, "epoch-2", "epoch-1",
                         leaked_tasks=["p7-I08-s1"]).verdict()
    assert v.ok is False
    assert "inherited" in v.detail


def test_inherited_permits_are_caught_too():
    v = TerminateOutcome(True, True, "epoch-2", "epoch-1",
                         leaked_permits=["mid@r01"]).verdict()
    assert v.ok is False


def test_a_dirty_termination_is_not_a_fresh_run():
    v = TerminateOutcome(False, True, "epoch-2", "epoch-1").verdict()
    assert v.ok is False


def test_every_check_is_frozen_and_carries_a_sentence():
    """`D-P17-03` is the record of a check that could report an outcome but not a reason."""
    for c in (pause_precondition(False, what="pause"),
              collector_precondition(True, 1, why="stop"),
              db_fault_confirm(True, "write_denied"),
              truth_absence_is_reported({"truth_gap_recorded": True})):
        assert c.detail and len(c.detail) > 10
        try:
            c.ok = False  # type: ignore[misc]
        except Exception:
            continue
        raise AssertionError("Check must be frozen")


# --------------------------------------------------------------------------- #
# the manifest no longer blocks these
# --------------------------------------------------------------------------- #


#: The cases that cannot run, and what each is waiting for. See D-P17-26 and D-P17-30.
#: F03 joined on 2026-09-20: `drive_to_pose` exists, but a pad is not a legal ARRIVAL point
#: (config/resources.yaml: arriving there is refused with `RESOURCE_UNKNOWN` on `mid`, while
#: standing there is allowed), so both candidate targets timed out and the fault was never
#: delivered. That is a missing legal arrival point, not a missing action.
EXPECTED_BLOCKED = {
    'F03': ['pad_occupation_by_crossing'],
    'F10': ['permit_renewal_interrupt'],
    'I03': ['batch_driver_extended'],
    'I06': ['truth_collector_control'],
    'I07': ['db_fault_injection'],
    'I08': ['batch_driver_extended'],
}


def test_the_blocked_cases_are_exactly_the_registered_set():
    """A blocked case is a DECISION, not a leftover, and the decision has to be registered.

    Round 17 promised these eleven would be unblocked. `D-P17-26` established that most of
    them cannot be: they need a node-side service -- a ledger fault injector, a permit
    renewal control, a stale-goal injector, a run terminator -- that does not exist, and
    writing the actuator for them would be writing an actuator with nothing behind it. F03
    joined them for a different reason again (`D-P17-30`): its action exists but there is no
    legal arrival point to aim it at.

    So the assertion is an EQUALITY against the intended set, and that every capability it
    names appears in `prerequisites`. The equality is the point and the number is not: it
    catches a case that was blocked and then quietly unblocked, and a case that was blocked
    without anyone writing down why. Asserting the set is empty would be asserting an
    aspiration, and an aspiration is not a contract. This test used to be named after the
    count it happened to hold, which made every legitimate change look like a regression."""
    import yaml

    manifest = (
        pathlib.Path(__file__).resolve().parent.parent
        / "config/scenarios/regression_v1.yaml"
    )
    data = yaml.safe_load(manifest.read_text(encoding="utf-8"))
    still = {cid: case.get("requires") for cid, case in (data.get("cases") or {}).items()
             if case.get("requires")}
    registered = set((data.get("prerequisites") or {}).keys())
    for cid, needs in still.items():
        for need in needs:
            assert need in registered, (
                f"{cid} is blocked on {need!r}, which is not in `prerequisites` -- a "
                f"blocked case whose reason is not written down is indistinguishable from "
                f"a case somebody forgot")
    assert still == EXPECTED_BLOCKED, (
        f"the blocked set changed: {still}")


def test_every_prerequisite_key_is_declared():
    import yaml

    manifest = (
        pathlib.Path(__file__).resolve().parent.parent
        / "config/scenarios/regression_v1.yaml"
    )
    data = yaml.safe_load(manifest.read_text(encoding="utf-8"))
    declared = set((data.get("prerequisites") or {}).keys())
    assert declared <= set(CAPABILITY_KEYS), (
        f"the manifest declares prerequisite keys this round does not know: "
        f"{sorted(declared - set(CAPABILITY_KEYS))}"
    )
