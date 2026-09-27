"""W4's protocol tests: one physical action per command, and every refusal says which one it is.

These are the SHORT tests the guidance asks for before any fault is made physical. They use the
fake plant, so what they prove is the PROTOCOL -- idempotency, boot and revision staleness, the
gate as the only authority, and the evidence table. They prove nothing about stopping distance or
about a tray staying on rollers, and the module docstring of the judge says so too.
"""
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src'))
sys.path.insert(0, str(ROOT / 'experiments'))

import math  # noqa: E402

import mujoco  # noqa: E402

import w4_plant as wp  # noqa: E402
from w4_fake_plant import (  # noqa: E402
    FAKE_ENVELOPE, FAKE_STATIONS, FAKE_TRANSFERS, FakePlant)
from workcell.adapters.sim import SkillAdapter  # noqa: E402
from workcell.orchestration.sequence import (  # noqa: E402
    STATE_EVIDENCE, SkillSequence, missing_evidence)
from workcell.safety import CommandGate  # noqa: E402

BOOT = 'boot-001'
ORDER = 'order-001'


def build(*, plant=None, epoch=7, **gate_kwargs):
    plant = plant or FakePlant(stations=FAKE_STATIONS)
    gate = CommandGate(ttl_s=gate_kwargs.pop('ttl_s', 0.5), silence_s=0.5, v_max=0.6, w_max=1.2,
                       source='safety_gate')
    adapter = SkillAdapter(plant=plant, boot_id=BOOT, stations=FAKE_STATIONS,
                           envelope=FAKE_ENVELOPE, transfers=FAKE_TRANSFERS)
    sequence = SkillSequence(adapter=adapter, gate=gate, boot_id=BOOT, epoch=epoch,
                             order_id=ORDER, wall_clock=lambda: 100.0)
    return plant, gate, sequence


def request(command_id, skill, *, arguments=None, revision=None, boot=BOOT, order=ORDER,
            now=100.0):
    revision = 1 if revision is None else revision
    return ({'schema_version': 1, 'command_id': command_id, 'order_id': order,
             'revision': revision, 'expected_boot_id': boot, 'skill': skill,
             'arguments': arguments if arguments is not None else {},
             'resource_generation': 0, 'deadline_s': 30.0}, now)


# ---------------------------------------------------------------- the happy path
def test_one_command_does_one_physical_action():
    plant, _gate, sequence = build()
    raw, now = request('c1', 'MOVE_TO_STATION', arguments={'station_id': 'station_c'})
    answer = sequence.submit(raw, now_s=now)
    assert answer['accepted'] and not answer['idempotent']
    assert plant.calls == [('drive_to', 4.0, False)]
    assert answer['record'].result['status'] == 'SUCCEEDED'


def test_a_repeated_command_id_returns_the_SAME_record_and_acts_once():
    plant, _gate, sequence = build()
    raw, now = request('c1', 'MOVE_TO_STATION', arguments={'station_id': 'station_c'})
    first = sequence.submit(raw, now_s=now)
    second = sequence.submit(raw, now_s=now + 1.0)
    assert second['idempotent'] is True
    assert second['record'] is first['record']
    assert len(plant.calls) == 1, 'a repeated command_id repeated a physical action'


# ---------------------------------------------------------------- identity and staleness
def test_a_stale_boot_is_refused_with_its_own_reason():
    _plant, _gate, sequence = build()
    raw, now = request('c1', 'MOVE_TO_STATION', arguments={'station_id': 'station_c'},
                       boot='boot-999')
    answer = sequence.submit(raw, now_s=now)
    assert not answer['accepted'] and answer['record'].refusal == 'REFUSED_STALE_BOOT'
    assert answer['record'].attempts == 0


def test_a_stale_revision_cannot_change_the_current_task():
    plant, _gate, sequence = build()
    raw, now = request('c1', 'MOVE_TO_STATION', arguments={'station_id': 'station_c'}, revision=5)
    assert sequence.submit(raw, now_s=now)['accepted']
    raw2, _ = request('c2', 'MOVE_TO_STATION', arguments={'station_id': 'station_b'}, revision=4)
    answer = sequence.submit(raw2, now_s=now + 1.0)
    assert answer['record'].refusal == 'REFUSED_STALE_REVISION'
    assert len(plant.calls) == 1


# ---------------------------------------------------------------- the gate is the only authority
def test_the_gate_is_offered_before_the_plant_is_touched():
    plant, gate, sequence = build()
    raw, now = request('c1', 'DOCK', arguments={'station_id': 'station_c', 'transfer_id': 't1'})
    sequence.submit(raw, now_s=now)
    assert gate.counters['accepted'] == 1
    assert plant.calls, 'the plant was never driven'


def test_an_unbound_gate_refuses_and_nothing_moves():
    plant = FakePlant(stations=FAKE_STATIONS)
    gate = CommandGate(ttl_s=0.5, silence_s=0.5, v_max=0.6, w_max=1.2, source='safety_gate')
    adapter = SkillAdapter(plant=plant, boot_id=BOOT, stations=FAKE_STATIONS,
                           envelope=FAKE_ENVELOPE, transfers=FAKE_TRANSFERS)
    sequence = SkillSequence(adapter=adapter, gate=gate, boot_id=BOOT, epoch=7, order_id=ORDER,
                             wall_clock=lambda: 100.0)
    gate.epoch = None                     # simulate a gate that was never bound
    raw, now = request('c1', 'MOVE_TO_STATION', arguments={'station_id': 'station_c'})
    answer = sequence.submit(raw, now_s=now)
    assert not answer['accepted']
    assert answer['record'].refusal.startswith('GATE_')
    assert plant.calls == []
    assert answer['record'].attempts == 0


# ---------------------------------------------------------------- IT-01 .. IT-08
def test_IT01_a_stale_observation_stops_the_dock_at_DOCK_OUT_OF_TOLERANCE():
    plant = FakePlant(stations=FAKE_STATIONS, dock=(0.020, 0.0, 0.0))
    _p, _g, sequence = build(plant=plant)
    raw, now = request('c1', 'DOCK', arguments={'station_id': 'station_b', 'transfer_id': 't1'})
    answer = sequence.submit(raw, now_s=now)
    assert answer['record'].result['status'] == 'FAILED'
    assert answer['record'].result['reason_code'] == 'DOCK_OUT_OF_TOLERANCE'


def test_IT03_a_receiver_that_is_not_empty_is_visible_as_a_payload_fault():
    plant = FakePlant(stations=FAKE_STATIONS, tray_zone='unknown')
    _p, _g, sequence = build(plant=plant)
    raw, now = request('c1', 'VERIFY_TRANSFER',
                       arguments={'transfer_id': 'xfer_source_to_deck'})
    answer = sequence.submit(raw, now_s=now)
    assert answer['record'].result['reason_code'] == 'PAYLOAD_LOST'


def test_IT04_an_interrupted_transfer_is_NOT_reported_as_a_failure():
    plant = FakePlant(stations=FAKE_STATIONS, transfer_state='UNKNOWN')
    _p, _g, sequence = build(plant=plant)
    raw, now = request('c1', 'START_TRANSFER',
                       arguments={'transfer_id': 'xfer_source_to_deck'})
    answer = sequence.submit(raw, now_s=now)
    assert answer['record'].result['reason_code'] == 'TRANSFER_UNKNOWN'


def test_IT05_a_duplicate_start_transfer_neither_acts_nor_charges_twice():
    plant, _gate, sequence = build()
    raw, now = request('c1', 'START_TRANSFER',
                       arguments={'transfer_id': 'xfer_source_to_deck'})
    sequence.submit(raw, now_s=now)
    sequence.submit(raw, now_s=now + 0.5)
    # the plant is ALSO probed for the tray's zone (that is how the adapter derives the transfer
    # direction), so the claim is not "the call list is exactly one entry" but "the transfer
    # happened exactly once"
    assert [call[0] for call in plant.calls].count('transfer') == 1


def test_IT06_a_late_callback_from_a_previous_boot_cannot_finish_a_new_task():
    plant, _gate, sequence = build()
    raw, now = request('c1', 'VERIFY_DELIVERY', arguments={'order_id': ORDER}, boot='boot-000')
    answer = sequence.submit(raw, now_s=now)
    assert answer['record'].refusal == 'REFUSED_STALE_BOOT'
    assert plant.calls == []


def test_IT07_a_STOP_is_a_stop_and_not_a_delivery():
    plant = FakePlant(stations=FAKE_STATIONS)
    _p, _g, sequence = build(plant=plant)
    raw, now = request('c1', 'STOP')
    answer = sequence.submit(raw, now_s=now)
    assert answer['record'].result['status'] == 'SUCCEEDED'
    assert answer['record'].result['final_state']['stopped_confirmed'] is True
    assert 'VERIFY_DELIVERY' not in sequence.adapter.calls


def test_IT08_a_success_with_no_evidence_is_not_success():
    _plant, _gate, sequence = build()
    raw, now = request('c1', 'DOCK', arguments={'station_id': 'station_c', 'transfer_id': 't1'})
    answer = sequence.submit(raw, now_s=now)
    assert answer['record'].result['evidence'], 'a SUCCEEDED result carried no evidence'


# ---------------------------------------------------------------- the evidence table
def test_a_state_cannot_be_claimed_without_its_evidence():
    assert missing_evidence('DOCK_VERIFIED', {}) == list(STATE_EVIDENCE['DOCK_VERIFIED'])
    assert missing_evidence('DOCK_VERIFIED',
                            {'relative_pose': 1, 'stopped_confirmed': True,
                             'motion_permitted': True}) == []


def test_a_correctly_stopped_fault_is_not_a_delivery():
    """The guidance's own sentence, as an executable assertion."""
    _plant, _gate, sequence = build()
    raw, now = request('c1', 'STOP')
    sequence.submit(raw, now_s=now)
    builder = sequence.evaluations(run_id='w4-unit')
    # every check a group names must have been RECORDED: `build` marks an unrecorded one UNKNOWN
    # rather than absent, which is deliberate -- so a test that names a check it never ran gets
    # UNKNOWN, and that is what happened here first.
    for name, value in (('delivered', 'FAIL'), ('tray_at_receiver', 'FAIL'),
                        ('stopped', 'PASS'), ('stopped_before_contact', 'PASS')):
        builder.check(name, value)
    evaluation = builder.build(task_checks={'delivered': 'FAIL'},
                               physical_checks={'tray_at_receiver': 'FAIL'},
                               safety_checks={'stopped': 'PASS'},
                               expected_checks={'stopped_before_contact': 'PASS'},
                               task_outcome='FAILED')
    assert evaluation['task_outcome'] != 'SUCCEEDED'
    assert evaluation['safety_result'] == 'PASS'


def test_an_unknown_skill_cannot_reach_the_plant():
    _plant, _gate, sequence = build()
    with pytest.raises(Exception):
        sequence.submit({'schema_version': 1, 'command_id': 'c1', 'order_id': ORDER,
                         'revision': 1, 'expected_boot_id': BOOT, 'skill': 'RETURN_TO_CHARGE',
                         'arguments': {}, 'resource_generation': 0, 'deadline_s': 1.0},
                        now_s=100.0)


# -------------------------------------------------------------------------------------------
# What the W4 PHYSICAL chain found. These use the real world, unlike everything above: the
# protocol rows are deliberately fake-plant, and a fake cannot prove a stopping distance.
# -------------------------------------------------------------------------------------------

def test_the_drive_direction_does_not_depend_on_how_far_the_wheel_has_spun():
    """★ The first version returned `cross(R @ jnt_axis, R @ (0, 0, -radius))`, and that arm is a
    point on the RIM in body coordinates -- so it turns with the wheel while the contact patch does
    not. The returned sign therefore flipped once a wheel had accumulated a quarter turn of spin.
    Measured consequence in `reports/w4-skills-04`: `cmd-004 DOCK` had a target 0.603 m to the EAST
    and drove 1.7766 m WEST at full dock speed until its timeout.

    This sets the spin to four phases, two of which are in the flipping half, and asserts that the
    signs AND the resulting motion are the same at every phase.
    """
    plant = wp.LogisticsPlant()
    signs = plant._wheel_signs()
    assert set(signs) == {"left", "right"}, signs
    for phase in (0.0, 1.2, math.pi, 4.9):
        for side in signs:
            address = int(plant.model.jnt_qposadr[plant.model.joint(f"n_wheel_{side}_joint").id])
            plant.data.qpos[address] = phase
        mujoco.mj_forward(plant.model, plant.data)
        assert plant._wheel_signs() == signs, f"the drive sign changed at spin phase {phase}"
        start = plant.chassis_x()
        for _ in range(20):
            plant._tick(lambda s: plant._wheel_rate_torque(s, 1.0 * signs[s]))
        assert plant.chassis_x() > start, (
            f"a positive wheel command did not move the chassis +x at spin phase {phase}: the sign "
            f"is being read off a point on the rim again")


def test_zero_torque_coasts_and_the_rate_servo_brakes():
    """★ Zero torque is not a brake. The wheel joints carry `damping 0.02` and `frictionloss 0`,
    so cutting the torque leaves a free-wheeling wheel and the vehicle coasts; measured, it decays
    with a time constant near 0.9 s and was still at 43 mm/s inside the contract window, against a
    10 mm/s limit, having covered about 40 mm.

    The control arm is ASSERTED TO FAIL, so this test cannot pass by the vehicle happening to stop
    for some other reason.
    """
    plant = wp.LogisticsPlant()
    signs = plant._wheel_signs()
    for _ in range(60):
        plant._tick(lambda s: plant._wheel_rate_torque(s, 2.0 * signs[s]))
    assert abs(plant.chassis_speed()) > 3 * wp.STOPPED_SPEED_MPS, (
        "the fixture never got moving, so it cannot show what stopping means")
    saved = (plant.data.qpos.copy(), plant.data.qvel.copy(), plant.data.time)

    def settle(brake):
        plant.data.qpos[:], plant.data.qvel[:], plant.data.time = saved[0], saved[1], saved[2]
        mujoco.mj_forward(plant.model, plant.data)
        start = plant.chassis_x()
        for _ in range(100):
            if brake:
                plant._tick(lambda s: plant._wheel_rate_torque(s, 0.0))
            else:
                plant._tick(lambda s: 0.0)
        return abs(plant.chassis_speed()), abs(plant.chassis_x() - start)

    cut_speed, cut_travel = settle(brake=False)
    brake_speed, brake_travel = settle(brake=True)
    assert brake_speed <= wp.STOPPED_SPEED_MPS, (
        f"the brake left {brake_speed * 1000:.3f} mm/s, over the contract limit "
        f"{wp.STOPPED_SPEED_MPS * 1000:.1f} mm/s")
    assert cut_speed > wp.STOPPED_SPEED_MPS, (
        f"the CUT-TORQUE arm was expected to fail and instead held {cut_speed * 1000:.3f} mm/s: if "
        f"that arm passes, this test no longer demonstrates that the brake is what fixes the creep")
    assert brake_travel < cut_travel, (
        f"the brake travelled {brake_travel * 1000:.4f} mm and the coast {cut_travel * 1000:.4f} mm")


def test_the_drift_limit_is_derived_from_the_contract_not_chosen():
    """The drift row exists because the speed row cannot see a creep whose every sample is under
    the limit. Its threshold must therefore be the same two contract numbers, multiplied -- a
    typed constant here would be a second, unowned copy of the limit."""
    assert wp.DRIFT_LIMIT_M == pytest.approx(wp.STOPPED_SPEED_MPS * wp.STOPPED_HOLD_S)


def test_an_undeclared_transfer_id_is_refused_by_the_SCENARIO_not_inferred():
    """The direction of a leg is a property of the transaction. An id the scenario never declared
    must be refused, not guessed at from wherever the tray happens to be standing.

    This is why the three protocol tests above had to change: they were written against an earlier
    adapter that inferred the direction, and they passed an id no scenario declares.
    """
    from workcell.schema import SchemaRefused

    plant, _gate, sequence = build()
    raw, now = request('c1', 'START_TRANSFER', arguments={'transfer_id': 't1'})
    with pytest.raises(SchemaRefused) as caught:
        sequence.submit(raw, now_s=now)
    assert 'REFUSED_ENTITY_UNKNOWN' in str(caught.value)
    assert [call[0] for call in plant.calls].count('transfer') == 0, (
        'the refusal must happen before anything is commanded')
