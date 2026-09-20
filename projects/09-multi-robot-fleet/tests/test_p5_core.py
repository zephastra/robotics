"""P5 core tests: simulated battery, charge pads, leg planning, fault takeover.

No ROS, no simulator -- the same rule every other file in tests/ follows. The
whole suite runs in about a second, which is the point: these pin the rules that
decide whether a robot gets sent somewhere it cannot come back from, and a rule
like that should not need a twelve-minute simulation before anyone can check it.

The three numeric tests worth reading carefully are the battery band boundaries.
They are written with `==` against the config fractions rather than against the
literals 0.20 / 0.08, because the fractions are config and a test that hard codes
them stops testing the config the moment somebody tunes it.
"""

from __future__ import annotations

from pathlib import Path

import pytest
import yaml

from fleet_core import (
    Band,
    BatteryConfig,
    BatteryModel,
    ChargerAllocator,
    ConfigError,
    InCorridorKnowledge,
    PayloadMode,
    PayloadState,
    PlanError,
    ReasonCode,
    Task,
    TaskKind,
    TaskSpec,
    TaskState,
    decide,
    parse_task_request,
    plan_legs,
    total_distance_m,
    validate_fleet_config,
    validate_traffic_config,
)

ROOT = Path(__file__).resolve().parents[1]
L, W = 0.60, 0.45


# --------------------------------------------------------------------------- #
# fixtures: the real config files, because a synthetic config would pass while
# the shipped one is broken
# --------------------------------------------------------------------------- #


@pytest.fixture(scope="module")
def fleet():
    return validate_fleet_config(yaml.safe_load((ROOT / "config" / "fleet.yaml").read_text()))


@pytest.fixture(scope="module")
def traffic():
    return validate_traffic_config(yaml.safe_load((ROOT / "config" / "resources.yaml").read_text()))


def four_robot_fleet() -> "object":
    """A synthetic 4-robot fleet, so a charger queue can actually form."""
    robots = {}
    for rid in ("r01", "r02", "r03", "r04"):
        robots[rid] = {
            "capabilities": ["CARRY"],
            "battery_capacity_wh": 100.0,
            "discharge_wh_per_m": 0.5,
            "discharge_wh_per_s": 0.01,
            "max_speed_mps": 0.35,
            "footprint_length_m": L,
            "footprint_width_m": W,
        }
    return validate_fleet_config(
        {
            "robots": robots,
            "stations": {"A": {"x": 0.0, "y": 0.0}, "B": {"x": 1.0, "y": 0.0}},
            "chargers": {"C1": {}, "C2": {}},
            "charger_capacity": 1,
            "safety": {
                "permit_ttl_s": 30.0,
                "battery_reserve_wh": 5.0,
                "low_battery_wh": 15.0,
            },
        }
    )


# --------------------------------------------------------------------------- #
# battery: the model is simulated, and it says so
# --------------------------------------------------------------------------- #


def test_the_battery_model_announces_that_it_is_simulated(fleet):
    """A number that could be mistaken for measured data must carry its own label.

    CONTRACTS section 5 calls these demo rules. If the label is only in a
    docstring, a report can quote `battery_fraction` without ever meeting the
    caveat, so `simulated` travels with the value and with the snapshot.
    """
    bm = BatteryModel(fleet)
    assert fleet.battery.simulated is True
    assert bm.snapshot()["simulated"] is True
    assert bm.simulate("r01", charge_wh=50.0, dt_s=1.0).simulated is True


def test_thresholds_must_be_ordered(fleet):
    with pytest.raises(ValueError):
        BatteryConfig(low_fraction=0.10, critical_fraction=0.50)
    with pytest.raises(ValueError):
        BatteryConfig(resume_fraction=0.10, low_fraction=0.20)
    with pytest.raises(ValueError):
        BatteryConfig(charge_wh_per_s=0.0)


def test_bands_are_inclusive_at_their_thresholds(fleet):
    """At exactly the threshold counts as in it.

    The alternative -- `f > low_fraction` -- has no defensible reading: a robot
    sitting precisely on the low mark is at least as much a low-battery robot as
    one a hair under it.
    """
    bm = BatteryModel(fleet)
    cap = bm.capacity_wh("r01")
    low = fleet.battery.low_fraction
    crit = fleet.battery.critical_fraction
    assert bm.band("r01", cap * (low + 0.001)) is Band.OK
    assert bm.band("r01", cap * low) is Band.LOW
    assert bm.band("r01", cap * (crit + 0.001)) is Band.LOW
    assert bm.band("r01", cap * crit) is Band.CRITICAL
    assert bm.band("r01", 0.0) is Band.CRITICAL


def test_resume_needs_hysteresis_not_just_clearing_low(fleet):
    """Above LOW is not enough to resume work; the recovery threshold is higher.

    Without the gap a robot oscillates: it clears LOW, takes a task, drops under
    LOW again and turns straight back. The hysteresis is the whole reason the
    third fraction exists.
    """
    bm = BatteryModel(fleet)
    cap = bm.capacity_wh("r01")
    just_above_low = cap * (fleet.battery.low_fraction + 0.01)
    assert bm.needs_charge("r01", just_above_low) is False
    assert bm.may_resume_service("r01", just_above_low) is False
    assert bm.may_resume_service("r01", cap * fleet.battery.resume_fraction) is True


def test_an_overdraw_is_clamped_and_reported(fleet):
    """Going negative is a modelling error, not a very empty battery.

    Letting a negative number flow on would make the allocator's finish-ability
    test agree with a value that cannot exist, and the agreement would look like
    evidence. `clamped_empty` is what lets a caller notice.
    """
    bm = BatteryModel(fleet)
    step = bm.simulate("r01", charge_wh=0.5, dt_s=60.0, distance_m=10.0)
    assert step.charge_wh == 0.0
    assert step.clamped_empty is True
    # 0.01 Wh/s * 60 s + 0.5 Wh/m * 10 m = 5.6 Wh of modelled demand against 0.5 Wh
    # available. The figure reported must be the demand: reporting what was
    # available would hide the shortfall this branch exists to expose.
    assert step.consumed_wh == pytest.approx(5.6, abs=1e-9)

    ok = bm.simulate("r01", charge_wh=90.0, dt_s=1.0)
    assert ok.clamped_empty is False


def test_charging_is_its_own_rate_and_stops_at_capacity(fleet):
    bm = BatteryModel(fleet)
    step = bm.charge("r01", charge_wh=99.0, dt_s=10.0)
    assert step.charge_wh == bm.capacity_wh("r01")
    assert step.consumed_wh < 0.0, "charging is reported as negative consumption"


def test_finishability_is_judged_against_the_reserve_not_zero(fleet):
    """The floor is the configured reserve, not zero.

    A trip that lands the robot on 4.9 Wh against a 5.0 Wh reserve is refused even
    though it "arrives". Both sides are asserted, because a check that refuses
    everything would also pass the negative half.
    """
    bm = BatteryModel(fleet)
    reach = bm.can_finish("r01", charge_wh=5.4, distance_m=1.0)
    assert reach.ok is False
    assert reach.margin_wh < 0.0
    assert reach.charge_after_wh == pytest.approx(5.4 - 0.5, abs=1e-9)

    fine = bm.can_finish("r01", charge_wh=6.0, distance_m=1.0)
    assert fine.ok is True
    assert fine.margin_wh == pytest.approx(0.5, abs=1e-9)


def test_reaching_a_charger_budgets_the_turn(fleet):
    """Arriving with nothing left is not arrival, it is a robot stopped en route.

    The turn budget is the difference between "5.4 m away with 7 Wh" being
    reachable at zero turn cost and not being reachable once the heading change
    is paid for. `turn_wh_per_rad` defaults to 0.0 because nobody has measured
    it, so this test sets it explicitly rather than pretending it is known.
    """
    cfg = validate_fleet_config(
        {
            "robots": {
                "r01": {
                    "capabilities": ["CARRY"],
                    "battery_capacity_wh": 100.0,
                    "discharge_wh_per_m": 0.5,
                    "discharge_wh_per_s": 0.01,
                    "max_speed_mps": 0.35,
                    "footprint_length_m": L,
                    "footprint_width_m": W,
                    "turn_wh_per_rad": 2.0,
                }
            },
            "stations": {"A": {"x": 0.0, "y": 0.0}},
            "safety": {
                "permit_ttl_s": 30.0,
                "battery_reserve_wh": 5.0,
                "low_battery_wh": 15.0,
            },
        }
    )
    bm = BatteryModel(cfg)
    assert bm.can_reach_charger("r01", charge_wh=8.0, distance_m=5.0, turnaround_rad=0.0).ok
    assert not bm.can_reach_charger("r01", charge_wh=8.0, distance_m=5.0, turnaround_rad=3.0).ok


# --------------------------------------------------------------------------- #
# charge pads: one occupant, and a queue that cannot be jumped
# --------------------------------------------------------------------------- #


def test_a_taken_pad_sends_the_second_robot_to_the_other_pad(fleet):
    """Two pads, two robots: both charge, nobody waits.

    Queueing when a free pad exists would be a worse outcome than the one the
    requirement asks for. "排队，不叠在一起" forbids stacking on a pad; it does not
    ask for artificial waiting.
    """
    ca = ChargerAllocator(fleet)
    first = ca.request("r01", now=0.0, preferred="C_left")
    second = ca.request("r02", now=0.0, preferred="C_left")
    assert first is not None and first.charger_id == "C_left"
    assert second is not None and second.charger_id == "C_right"
    assert ca.occupant("C_left") == "r01"
    assert ca.occupant("C_right") == "r02"
    assert ca.queue() == ()


def test_the_fleet_config_has_exactly_two_pads_so_the_third_robot_waits(fleet):
    ca = ChargerAllocator(fleet)
    assert ca.request("r01", now=0.0, preferred="C_left") is not None
    assert ca.request("r02", now=0.0, preferred="C_right") is not None
    third = ca.request("r03", now=5.0, preferred="C_left")
    assert third is None, "a full charging area must not admit a third robot"
    assert ca.queue() == ("r03",)
    assert ca.refusals.get(ReasonCode.CHARGER_BUSY.value, 0) >= 1


def test_a_queue_cannot_be_jumped_when_a_pad_frees_up():
    """FIFO survives the pad becoming free, which is the only case that matters.

    The naive implementation grants the pad to whoever asks next, so a robot that
    has been waiting since t=0 loses to one that asks at t=99. The second robot
    here is that interposer.
    """
    cfg = four_robot_fleet()
    ca = ChargerAllocator(cfg, capacity_override=1)
    assert ca.request("r01", now=0.0) is not None
    assert ca.request("r02", now=0.0) is not None
    assert ca.request("r03", now=1.0, preferred="C1") is None
    assert ca.request("r04", now=2.0, preferred="C1") is None
    assert ca.queue() == ("r03", "r04")

    ca.release("r01", now=10.0)
    assert ca.request("r04", now=10.0, preferred="C1") is None, "r04 must not jump r03"
    granted = ca.request("r03", now=11.0, preferred="C1")
    assert granted is not None and granted.charger_id == "C1"
    assert granted.waited_s == pytest.approx(10.0, abs=1e-9)


def test_one_robot_cannot_hold_two_pads(fleet):
    ca = ChargerAllocator(fleet)
    assert ca.request("r01", now=0.0, preferred="C_left") is not None
    assert ca.request("r01", now=0.0, preferred="C_right") is None
    assert ca.holder("r01") == "C_left"
    assert ca.refusals.get(ReasonCode.RESOURCE_BUSY.value, 0) >= 1


def test_release_actually_frees_the_pad(fleet):
    ca = ChargerAllocator(fleet)
    ca.request("r01", now=0.0, preferred="C_left")
    assert ca.release("r01", now=1.0) is True
    assert ca.occupant("C_left") is None
    assert ca.release("r01", now=2.0) is False
    assert ca.request("r02", now=3.0, preferred="C_left") is not None


def test_an_offline_robot_does_not_keep_a_pad_forever(fleet):
    ca = ChargerAllocator(fleet)
    ca.request("r01", now=0.0, preferred="C_left")
    ca.request("r02", now=0.0, preferred="C_right")
    ca.request("r03", now=0.0, preferred="C_left")
    assert ca.queue() == ("r03",)
    assert ca.drop_offline("r01", now=50.0) is True
    assert ca.holder("r01") is None
    assert ca.occupant("C_left") is None


def test_an_unknown_robot_is_refused_not_created(fleet):
    ca = ChargerAllocator(fleet)
    assert ca.request("r99", now=0.0) is None
    assert ca.refusals.get(ReasonCode.INVALID_INPUT.value, 0) == 1


# --------------------------------------------------------------------------- #
# leg planning
# --------------------------------------------------------------------------- #


def transfer(fleet, request_id, pick, drop, **kw):
    raw = {"request_id": request_id, "pick_station": pick, "drop_station": drop}
    raw.update(kw)
    return parse_task_request(raw, fleet)


def test_a_payload_transfer_has_four_legs(fleet, traffic):
    plan = plan_legs(transfer(fleet, "t1", "S_left_a", "S_left_b", payload_id="p1"), fleet, traffic)
    assert not isinstance(plan, PlanError)
    assert [leg.role.value for leg in plan] == [
        "TO_PICK",
        "PICK_SERVICE",
        "TO_DROP",
        "DROP_SERVICE",
    ]
    assert all(leg.corridor_direction is None for leg in plan)


def test_a_payload_free_task_has_no_service_legs(fleet, traffic):
    """A drive-and-report must not be dressed up as a transfer.

    Empty service steps would make a task that moved nothing look like one that
    moved something, and the report would then need a second rule to undo it.
    """
    plan = plan_legs(transfer(fleet, "t2", "S_left_a", "S_left_b"), fleet, traffic)
    assert [leg.role.value for leg in plan] == ["TO_PICK", "TO_DROP"]


def test_a_cross_corridor_transfer_is_tagged_with_a_direction(fleet, traffic):
    """The dispatcher has to know which direction to ask for before it drives."""
    plan = plan_legs(
        transfer(fleet, "t3", "S_left_a", "S_right_a", payload_id="p2"), fleet, traffic
    )
    tagged = [leg for leg in plan if leg.corridor_direction is not None]
    assert len(tagged) == 1
    assert tagged[0].corridor_direction == "west_to_east"
    assert tagged[0].role.value == "TO_DROP"

    back = plan_legs(
        transfer(fleet, "t4", "S_right_a", "S_left_a", payload_id="p3"), fleet, traffic
    )
    assert [leg.corridor_direction for leg in back if leg.corridor_direction] == ["east_to_west"]


def test_return_to_charge_has_no_source(fleet, traffic):
    spec = parse_task_request(
        {"request_id": "t5", "kind": "return_to_charge", "drop_station": "C_left"}, fleet
    )
    assert spec.pick_station == ""
    plan = plan_legs(spec, fleet, traffic)
    assert [leg.role.value for leg in plan] == ["TO_CHARGER", "CHARGE_SERVICE"]


def test_a_charger_that_is_not_configured_is_refused(fleet):
    with pytest.raises(ConfigError):
        parse_task_request(
            {"request_id": "t6", "kind": "return_to_charge", "drop_station": "C_middle"}, fleet
        )


def test_return_to_charge_may_not_name_a_source(fleet):
    with pytest.raises(ConfigError):
        parse_task_request(
            {
                "request_id": "t7",
                "kind": "return_to_charge",
                "pick_station": "S_left_a",
                "drop_station": "C_left",
            },
            fleet,
        )


def test_physical_payload_mode_is_refused_at_plan_time(fleet, traffic):
    """CONTRACTS section 2: return UNSUPPORTED_PAYLOAD_MODE, do not downgrade.

    The refusal is checked here rather than in validation because validation is
    about shape and this is about capability -- and it must carry the contract's
    code, not a generic UNSUPPORTED, or a report cannot tell the two apart.
    """
    spec = transfer(fleet, "t8", "S_left_a", "S_left_b", payload_id="p4")
    physical = TaskSpec(
        request_id=spec.request_id,
        pick_station=spec.pick_station,
        drop_station=spec.drop_station,
        payload_id=spec.payload_id,
        payload_mode=PayloadMode.PHYSICAL,
    )
    plan = plan_legs(physical, fleet, traffic)
    assert isinstance(plan, PlanError)
    assert plan.reason is ReasonCode.UNSUPPORTED_PAYLOAD_MODE
    assert "downgrad" in plan.detail


def test_visit_station_is_refused_by_name(fleet):
    with pytest.raises(ConfigError) as exc:
        parse_task_request(
            {"request_id": "t9", "kind": "visit_station", "drop_station": "S_left_a"}, fleet
        )
    assert "visit_station" in str(exc.value)


def test_an_unknown_kind_is_not_silently_a_transfer(fleet):
    with pytest.raises(ConfigError) as exc:
        parse_task_request(
            {"request_id": "t10", "kind": "teleport", "pick_station": "S_left_a",
             "drop_station": "S_left_b"}, fleet
        )
    assert "teleport" in str(exc.value)


def test_distance_budget_sums_the_legs(fleet, traffic):
    plan = plan_legs(transfer(fleet, "t11", "S_left_a", "S_left_b"), fleet, traffic)
    d = total_distance_m(plan, (-5.0, 2.5))
    assert d == pytest.approx(5.0, abs=1e-6)


# --------------------------------------------------------------------------- #
# fingerprint / idempotency
# --------------------------------------------------------------------------- #


def test_two_kinds_over_the_same_stations_are_not_the_same_request(fleet):
    """Kind is identity, so a repeat of one must not dedupe into the other."""
    a = transfer(fleet, "same-id", "S_left_a", "S_left_b")
    b = TaskSpec(
        request_id="same-id",
        pick_station="S_left_a",
        drop_station="S_left_b",
        kind=TaskKind.STATION_TRANSFER,
        payload_mode=PayloadMode.PHYSICAL,
    )
    assert a.fingerprint() != b.fingerprint()


def test_service_policy_is_not_identity(fleet):
    """A retried submission with a different retry budget is the same request.

    Otherwise the CLI's own retry would be reported as an idempotency conflict,
    and the operator would be told they submitted two different things.
    """
    a = transfer(fleet, "same-2", "S_left_a", "S_left_b", payload_id="p5")
    b = transfer(
        fleet, "same-2", "S_left_a", "S_left_b", payload_id="p5",
        max_attempts=5, timeout_sim_s=99.0, priority=1,
    )
    assert a.fingerprint() == b.fingerprint()


# --------------------------------------------------------------------------- #
# fault takeover: CONTRACTS section 5, including the prohibitions
# --------------------------------------------------------------------------- #


def task(state=TaskState.EXECUTING, robot="r01", payload="p1") -> Task:
    return Task(
        task_id="task-x",
        spec=TaskSpec(
            request_id="x", pick_station="S_left_a", drop_station="S_left_b", payload_id=payload
        ),
        state=state,
        robot_id=robot,
    )


def test_an_unassigned_task_is_simply_reallocated():
    d = decide(
        task(state=TaskState.SUBMITTED, robot=None, payload=None),
        payload_state=None,
        payload_holder=None,
        stopped_confirmed=False,
    )
    assert d.allowed is True
    assert d.action == "REASSIGN"


def test_nothing_is_taken_over_until_the_old_command_is_proven_stopped():
    """A quiet heartbeat is not proof of a stopped robot.

    This is the rule that stops two robots racing for one payload, and the only
    input that can satisfy it is `stopped_confirmed`.
    """
    d = decide(
        task(),
        payload_state=PayloadState.RESERVED,
        payload_holder=None,
        stopped_confirmed=False,
        in_corridor=InCorridorKnowledge.NO,
    )
    assert d.allowed is False
    assert d.reason is ReasonCode.CANCEL_NOT_CONFIRMED
    assert d.action == "WAIT_FOR_STOP"

    ok = decide(
        task(),
        payload_state=PayloadState.RESERVED,
        payload_holder=None,
        stopped_confirmed=True,
        in_corridor=InCorridorKnowledge.NO,
    )
    assert ok.allowed is True
    assert ok.action == "BUMP_REVISION_AND_REASSIGN"


def test_a_held_payload_is_flagged_never_re_homed():
    d = decide(
        task(),
        payload_state=PayloadState.HELD,
        payload_holder="r01",
        stopped_confirmed=True,
        in_corridor=InCorridorKnowledge.NO,
    )
    assert d.allowed is False
    assert d.reason is ReasonCode.PAYLOAD_HELD_ELSEWHERE
    assert d.action == "MARK_NEEDS_ATTENTION"
    assert d.action_required is True


def test_unknown_corridor_knowledge_is_treated_as_possibly_inside():
    """`we do not know` must not collapse into `not in there`.

    A robot whose last known position is stale may be sitting in the gap. The
    conservative reading blocks the resource; the optimistic one would free it on
    a TTL and send the next robot into a passage that is still occupied.
    """
    d = decide(
        task(),
        payload_state=PayloadState.RESERVED,
        payload_holder=None,
        stopped_confirmed=True,
        in_corridor=InCorridorKnowledge.UNKNOWN,
    )
    assert d.allowed is False
    assert d.reason is ReasonCode.BLOCKED
    assert d.action == "BLOCK_RESOURCE"
    assert "TTL" in d.detail


def test_a_lost_delivery_ack_is_answered_from_the_ledger():
    d = decide(
        task(state=TaskState.EXECUTING),
        payload_state=PayloadState.DELIVERED,
        payload_holder=None,
        stopped_confirmed=False,
        in_corridor=InCorridorKnowledge.NO,
    )
    assert d.allowed is False
    assert d.action == "REPLY_FROM_LEDGER"


def test_a_terminal_task_has_nothing_to_take_over():
    d = decide(
        task(state=TaskState.SUCCEEDED),
        payload_state=PayloadState.DELIVERED,
        payload_holder=None,
        stopped_confirmed=True,
    )
    assert d.allowed is False
    assert d.action == "REPORT_TERMINAL"


# --------------------------------------------------------------------------- #
# route connectivity (CONTRACTS section 6)
# --------------------------------------------------------------------------- #


def test_the_approach_to_the_first_leg_can_need_the_corridor(fleet, traffic):
    """Route connectivity is an allocation filter, and this is why.

    A plan whose stations all sit in one half is corridor-free *for a robot in that
    half*. A three-robot run gave a west-side robot an east-side task and the leg
    died on a 180 s Nav2 timeout, which reads as a navigation fault. The plan is not
    wrong; the assignment was.
    """
    from fleet_core import approach_direction

    plan = plan_legs(
        transfer(fleet, "rc-1", "S_right_a", "S_right_b", payload_id="p_rc"), fleet, traffic
    )
    assert not isinstance(plan, PlanError)
    assert all(leg.corridor_direction is None for leg in plan), "the plan itself is local"

    first = plan[0]
    assert approach_direction(6.0, first, traffic) is None, "r02's side: no crossing"
    assert approach_direction(-6.0, first, traffic) == "west_to_east", "r01's side: crossing"
    assert approach_direction(6.0, plan[0], traffic) is None
