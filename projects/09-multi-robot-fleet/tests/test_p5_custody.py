"""Custody and fault-classification rules -- the four defects the 07:57Z run exposed.

That run SUCCEEDED: p5-A picked cargo_A up and delivered it, p5-B succeeded, p5-D was
cancelled and confirmed stopped. Reading it is what produced these tests, because the
reasons it succeeded were partly luck:

  * a fault anywhere in the world was classified as a corridor loss, because
    `_detect_faults` passed a hardcoded `InCorridorKnowledge.UNKNOWN` and `classify`
    reads UNKNOWN as "possibly in the corridor". So the convoy of CONTRACTS section 5
    rows for "payload already held" and "assigned, nothing picked yet" were unreachable
    in production -- the module was written, unit-tested and effectively dead in the
    live path.
  * every parked task recorded `CANCEL_NOT_CONFIRMED`, whatever had actually happened.
    008 lost a week to exactly that shape: a machine-readable failure label that was a
    constant.
  * `PAYLOAD_HELD_NEEDS_ATTENTION` is in the contract's reason list and appeared nowhere
    in this codebase.
  * r02 was parked by a fault while holding cargo_C, was reallocated, picked cargo_D, and
    ended up holding TWO payloads. Both rows read HELD, the ledger looked consistent, and
    cargo_C was never delivered.

The last one is why the custody guard exists, and it is the reason these tests assert on
behaviour rather than on the presence of a function.
"""

from __future__ import annotations

from pathlib import Path

import pytest
import yaml

from fleet_core import (
    InCorridorKnowledge,
    Ledger,
    PayloadState,
    ReasonCode,
    TaskMachine,
    TaskSpec,
    decide,
    regions_overlapping,
    validate_traffic_config,
)

ROOT = Path(__file__).resolve().parents[1]
DOMAIN = ROOT / "src" / "fleet_core" / "fleet_core" / "domain.py"
SERVICE = ROOT / "src" / "fleet_ros" / "fleet_ros" / "task_service_node.py"
L, W = 0.60, 0.45


@pytest.fixture(scope="module")
def traffic():
    return validate_traffic_config(
        yaml.safe_load((ROOT / "config" / "resources.yaml").read_text(encoding="utf-8"))
    )


@pytest.fixture
def ledger(tmp_path):
    led = Ledger(tmp_path / "fleet.sqlite3")
    yield led
    led.close()


# --------------------------------------------------------------------------- #
# ownership: a release must not clear a live task's robot
# --------------------------------------------------------------------------- #


def test_release_refuses_to_clear_a_live_tasks_ownership():
    """The measured defect: a late release for an older task cleared a newer task's
    ownership, and the newer task then sat in ASSIGNED for 213 s because `_active()` yields
    only robots whose `executing` is set, so nothing looked at it again."""
    body = _function_body(SERVICE.read_text(encoding="utf-8"), "_release_robot")
    assert "self.ownership_refusals += 1" in body, (
        "the refusal must be counted: a release that silently declines is a leak")
    assert "not self.ledger.get(owner).state.terminal" in body, (
        "the guard must ask the LEDGER whether the current owner is still live; asking the "
        "caller would require every call site to pass the right name, and twelve chances to "
        "miss one is how this happened")
    assert "return" in body, "a refused release must not fall through to the clear"


def test_a_task_that_is_assigned_but_tracked_by_nobody_is_parked_with_a_reason():
    """MASTER_PLAN section 9: diagnosable wait policy. Waiting for ever with `reason: OK`
    is indistinguishable from a task that is about to start."""
    source = SERVICE.read_text(encoding="utf-8")
    body = _function_body(source, "_watch_orphans")
    assert "TaskState.ASSIGNED" in body
    assert "ReasonCode.ORPHANED_ASSIGNMENT" in body, (
        "it must be parked under a code that means this, not NAV_FAILED: the task never "
        "navigated")
    assert "self.orphan_grace_s" in body, (
        "the grace must be a declared budget, not a literal")
    assert "_watch_orphans(now)" in source, "the watchdog has to be called by the tick"
    assert "ORPHANED_ASSIGNMENT" in (
        DOMAIN.read_text(encoding="utf-8")), (
        "the reason code has to exist in the enum the ledger stores")


def _function_body(source: str, name: str) -> str:
    """Slice one method out of a file, so an assertion cannot be satisfied elsewhere."""
    marker = f"    def {name}("
    start = source.index(marker)
    end = source.find("\n    def ", start + len(marker))
    return source[start:] if end < 0 else source[start:end]


# --------------------------------------------------------------------------- #
# A. the in-corridor question is answered from geometry, not hardcoded
# --------------------------------------------------------------------------- #

def test_a_robot_in_the_open_is_not_in_a_protected_region(traffic):
    """r02 was at S_right_b when it faulted. That is the open, not the corridor."""
    assert regions_overlapping(traffic, (5.0, -2.5, 0.0), length_m=L, width_m=W) == ()


def test_a_robot_in_the_gap_is_in_the_corridor(traffic):
    hits = regions_overlapping(traffic, (0.0, 0.0, 0.0), length_m=L, width_m=W)
    assert "mid" in hits, hits


def test_an_exit_buffer_is_its_own_region(traffic):
    """The bundle is corridor + exit buffer, so the dispatcher must see both."""
    hits = regions_overlapping(traffic, (2.5, -1.4, 0.0), length_m=L, width_m=W)
    assert hits == ("mid_right",), hits


def test_the_footprint_margin_can_only_grow_the_hit_set(traffic):
    """A margin is an expansion; it must never remove a region that already overlapped."""
    pose = (3.2, -1.4, 0.0)
    tight = regions_overlapping(traffic, pose, length_m=L, width_m=W, margin_m=0.0)
    loose = regions_overlapping(traffic, pose, length_m=L, width_m=W, margin_m=1.0)
    assert tight == (), tight
    assert "mid_right" in loose, loose


def test_unknown_corridor_knowledge_still_blocks_the_resource(executing_task):
    """The fail-closed default must survive the fix.

    UNKNOWN is not NO. Removing the hardcoded UNKNOWN must not turn "we cannot tell"
    into "the corridor is clear" -- that is the exact prohibition in the contract's
    recovery table.
    """
    unknown = decide(
        executing_task,
        payload_state=PayloadState.HELD,
        payload_holder="r02",
        stopped_confirmed=True,
        in_corridor=InCorridorKnowledge.UNKNOWN,
    )
    assert unknown.action == "BLOCK_RESOURCE"
    assert unknown.reason is ReasonCode.BLOCKED


def test_a_known_clear_corridor_reaches_the_payload_held_row(executing_task):
    """With real geometry this becomes the contract's PAYLOAD_HELD row.

    This is the row that was unreachable in production and it is the one that says "mark
    NEEDS_ATTENTION, let unrelated tasks continue" instead of freezing the corridor.
    """
    decision = decide(
        executing_task,
        payload_state=PayloadState.HELD,
        payload_holder="r02",
        stopped_confirmed=True,
        in_corridor=InCorridorKnowledge.NO,
    )
    assert decision.action == "MARK_NEEDS_ATTENTION"
    assert decision.allowed is False
    assert "held" in decision.detail.lower()


def test_a_known_clear_corridor_reaches_the_unpicked_row(executing_task):
    """Nothing picked up and the corridor is known clear: reassign, do not block."""
    decision = decide(
        executing_task,
        payload_state=None,
        payload_holder=None,
        stopped_confirmed=True,
        in_corridor=InCorridorKnowledge.NO,
    )
    assert decision.action == "BUMP_REVISION_AND_REASSIGN"
    assert decision.allowed is True


@pytest.fixture
def executing_task(ledger):
    machine = TaskMachine(ledger, epoch=0)
    ledger.submit(TaskSpec(request_id="custody-probe", pick_station="S_left_a",
                           drop_station="S_left_b"), now=0.0, epoch=0)
    task_id = ledger.find_by_request_id("custody-probe").task_id
    machine.accept(task_id, now=0.0)
    machine.assign(task_id, "r02", now=0.0)
    return machine.begin_execution(task_id, "r02", now=0.0)


# --------------------------------------------------------------------------- #
# B. the ledger can answer "who is carrying something?"
# --------------------------------------------------------------------------- #

def test_payload_holders_is_empty_before_anything_is_picked(ledger):
    assert ledger.payload_holders() == {}


def _submit_carrying(ledger, request_id: str, payload_id: str) -> str:
    """Submit a task that carries a payload. The row is created by submit(), not by pick()."""
    ledger.submit(
        TaskSpec(request_id=request_id, pick_station="S_left_a", drop_station="S_left_b",
                 payload_id=payload_id),
        now=0.0, epoch=0,
    )
    return ledger.find_by_request_id(request_id).task_id


def test_payload_holders_names_the_robot_and_clears_on_delivery(ledger):
    task_id = _submit_carrying(ledger, "carry-1", "cargo_A")
    assert ledger.payload_holders() == {}
    ledger.pick("cargo_A", task_id=task_id, robot_id="r02")
    assert ledger.payload_holders() == {"cargo_A": "r02"}
    assert ledger.deliver("cargo_A", task_id=task_id, robot_id="r02") is True
    assert ledger.payload_holders() == {}


def test_payload_holders_shows_two_cargos_on_one_robot_which_is_the_bug(ledger):
    """The guard in the dispatcher exists because this state is reachable.

    Recording it here rather than only asserting the guard keeps the reason for the
    guard visible: the ledger will happily let one robot hold two cargos, so somebody
    upstream has to refuse.
    """
    first = _submit_carrying(ledger, "carry-2", "cargo_C")
    second = _submit_carrying(ledger, "carry-3", "cargo_D")
    ledger.pick("cargo_C", task_id=first, robot_id="r02")
    ledger.pick("cargo_D", task_id=second, robot_id="r02")
    assert ledger.payload_holders() == {"cargo_C": "r02", "cargo_D": "r02"}


# --------------------------------------------------------------------------- #
# C. the contract's code for "parked while still holding the goods" exists
# --------------------------------------------------------------------------- #

def test_the_custody_reason_code_exists_and_is_not_the_same_as_held_elsewhere():
    assert ReasonCode.PAYLOAD_HELD_NEEDS_ATTENTION.value == "PAYLOAD_HELD_NEEDS_ATTENTION"
    assert ReasonCode.PAYLOAD_HELD_NEEDS_ATTENTION is not ReasonCode.PAYLOAD_HELD_ELSEWHERE


def test_needs_attention_records_the_detail_it_was_given(ledger):
    """A reason code alone cannot say which robot holds which cargo."""
    machine = TaskMachine(ledger, epoch=0)
    ledger.submit(TaskSpec(request_id="park-me", pick_station="S_left_a",
                           drop_station="S_left_b"), now=0.0, epoch=0)
    task_id = ledger.find_by_request_id("park-me").task_id
    machine.accept(task_id, now=0.0)
    task = machine.needs_attention(
        task_id, now=1.0, reason=ReasonCode.PAYLOAD_HELD_NEEDS_ATTENTION,
        detail="r02 still holds cargo_C",
    )
    assert task.reason is ReasonCode.PAYLOAD_HELD_NEEDS_ATTENTION
    assert "cargo_C" in task.detail


# --------------------------------------------------------------------------- #
# D. the dispatcher wires the above together (source-level, since it needs ROS)
# --------------------------------------------------------------------------- #

def test_detect_faults_does_not_hardcode_unknown_corridor_knowledge():
    body = _function_body(SERVICE.read_text(encoding="utf-8"), "_detect_faults")
    assert "in_corridor=InCorridorKnowledge.UNKNOWN" not in body, (
        "a hardcoded UNKNOWN makes every fault a corridor loss, which parks tasks as "
        "BLOCK_RESOURCE and freezes the corridor for a robot that is nowhere near it")
    assert "_corridor_knowledge(" in body


def test_corridor_knowledge_fails_closed_when_the_pose_is_not_trustworthy():
    body = _function_body(SERVICE.read_text(encoding="utf-8"), "_corridor_knowledge")
    assert "InCorridorKnowledge.UNKNOWN" in body
    assert "localization_valid" in body
    assert "pose_age_s" in body


def test_every_field_the_corridor_guard_reads_is_written_by_the_mirror():
    """A guard that reads a field nobody fills is a guard that cannot fail.

    `_corridor_knowledge` reads `localization_valid` and `pose_age_s`. The state mirror
    set `position_known` only, so both kept their defaults -- True and 0.0 -- and two of
    the guard's three conditions were unsatisfiable. This asserts the coupling in both
    directions, so deleting either half turns the test red.
    """
    source = SERVICE.read_text(encoding="utf-8")
    guard = _function_body(source, "_corridor_knowledge")
    for field in ("localization_valid", "pose_age_s"):
        assert f"state.{field}" in guard, f"the guard stopped reading {field}"
        assert f"core.{field} = " in source, (
            f"the mirror never writes {field}, so the guard would be reading a default "
            "and could not fail")


def test_park_does_not_write_a_constant_reason():
    """`reason` used to be hardcoded at every call site. 008 lost a week to this."""
    source = SERVICE.read_text(encoding="utf-8")
    body = _function_body(source, "_park")
    assert "reason=ReasonCode.CANCEL_NOT_CONFIRMED" not in body, (
        "a park reason that is a constant is a failure label nobody can trust")
    assert "detail=why" in body, "the human text must reach the ledger, not just a log line"


def test_every_park_call_site_supplies_a_reason():
    """Otherwise the reason becomes an argument with no value and the fix is cosmetic."""
    source = SERVICE.read_text(encoding="utf-8")
    lines = source.splitlines()
    missing: list[str] = []
    for index, line in enumerate(lines):
        if "self._park(" not in line:
            continue
        window = "\n".join(lines[index:index + 3])
        if "reason=" not in window:
            missing.append(f"line {index + 1}: {line.strip()}")
    assert missing == [], f"park calls without a reason: {missing}"


def test_eligibility_consults_payload_custody():
    body = _function_body(SERVICE.read_text(encoding="utf-8"), "_eligible_robots")
    assert "payload_holders()" in body, (
        "without this a robot holding an undelivered cargo is reallocated, picks a "
        "second one, and the first is never delivered")
    assert "PAYLOAD_HELD_ELSEWHERE" in body, "the refusal must be counted under a real code"


# --------------------------------------------------------------------------- #
# E. a cancel has to actually cancel
# --------------------------------------------------------------------------- #

ADAPTER = ROOT / "src" / "fleet_ros" / "fleet_ros" / "nav2_adapter_node.py"


def _declared_float(source: str, name: str) -> float:
    """Read a module-level numeric constant out of a file, the way a config reader would."""
    for line in source.splitlines():
        stripped = line.strip()
        if stripped.startswith(f"{name} = "):
            return float(stripped.split("=", 1)[1].strip())
    raise AssertionError(f"{name} is not declared in the file")


def test_cancel_is_transmitted_not_merely_recorded():
    """The 08:26Z run took 306 seconds to reach CANCELED because nothing told the robot.

    `_srv_cancel` wrote the request into the ledger and returned. The adapter's cancel
    handling was fully implemented -- it cancels the Nav2 goal and waits for the measured
    speed to settle -- but the dispatcher never sent it one, so the leg ran to its own
    natural end, retries and aborts included.
    """
    body = _function_body(SERVICE.read_text(encoding="utf-8"), "_srv_cancel")
    assert "cancel_goal_async()" in body, (
        "accepting a cancel without transmitting it means the robot keeps driving")
    assert "_cancel_deadlines[" in body, "a cancel with no deadline never times out"


def test_cancel_has_an_enforced_deadline_with_the_contract_reason():
    source = SERVICE.read_text(encoding="utf-8")
    assert "def _enforce_cancel_deadlines(" in source
    body = _function_body(source, "_enforce_cancel_deadlines")
    assert "ReasonCode.CANCEL_UNCONFIRMED" in body, (
        "CONTRACTS section 4 item 6 names CANCEL_UNCONFIRMED for a cancel that does not "
        "confirm in time")
    tick = _function_body(source, "_tick")
    assert "_enforce_cancel_deadlines(" in tick, (
        "an enforcement that is never called is not an enforcement")


def test_the_cancel_budget_exceeds_the_adapters_own_settle_window():
    """The conflict, recorded as an assertion rather than a comment.

    CONTRACTS section 4 item 6 allows 3 wall seconds for a cancel to confirm. The adapter
    waits up to CANCEL_SETTLE_S to see the measured speed settle after Nav2 stops, so a
    3-second budget would mark every cancel CANCEL_UNCONFIRMED -- the rule's letter with
    the opposite of its intent. The budget here is deliberately larger than the settle
    window, and this test is what stops someone "fixing" it back to 3 by accident.
    """
    service = SERVICE.read_text(encoding="utf-8")
    settle = _declared_float(ADAPTER.read_text(encoding="utf-8"), "CANCEL_SETTLE_S")
    budget = _declared_float(service, "CANCEL_CONFIRM_S")
    assert budget > settle, (
        f"cancel budget {budget}s is not larger than the adapter's settle window "
        f"{settle}s, so every cancel would be reported CANCEL_UNCONFIRMED")


def test_the_dispatcher_does_not_use_the_server_side_goal_handle_api():
    """`is_cancel_requested` is ServerGoalHandle-only; the dispatcher holds a client handle.

    Asking anyway raised AttributeError inside `_tick`. The blanket `except` in `_tick`
    swallowed it, so the cancel still had no deadline *and* the other five dispatching
    steps stopped running for the rest of the run -- with one ERROR line per tick as the
    only trace. Comments are allowed to mention the name; code is not.
    """
    offenders = []
    for number, line in enumerate(SERVICE.read_text(encoding="utf-8").splitlines(), 1):
        code = line.split("#", 1)[0]
        if "is_cancel_requested" in code:
            offenders.append(f"line {number}: {line.strip()}")
    assert offenders == [], (
        "the dispatcher is using a server-side goal-handle API: " + "; ".join(offenders))


def test_a_tick_failure_is_surfaced_not_swallowed():
    """A dispatcher that has stopped dispatching must not be able to look healthy."""
    source = SERVICE.read_text(encoding="utf-8")
    tick = _function_body(source, "_tick")
    assert "tick_errors += 1" in tick, "a raising tick must be counted"
    assert "traceback" in tick, "and the first occurrence must carry a traceback"
    assert '"tick_errors"' in source, (
        "the count must reach the status snapshot, or a wedged dispatcher reports healthy")
