"""W3 tests: the contract, the observations, and the docking machine.

Each test is here because a specific rule from the guidance would be violated without it:
  * a default instead of a refusal (`missing/stale/wrong station` must be UNKNOWN, never 0);
  * a threshold with no source, or an unknown field, silently accepted;
  * the command handover skipped;
  * `SETTLE` satisfied by a sleep rather than by a measured interval;
  * `DOCKED` that never expires.
"""
import json
import math
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src'))

from workcell import schema as S  # noqa: E402
from workcell.docking import contract as C  # noqa: E402
from workcell.docking import machine as M  # noqa: E402
from workcell.docking import observations as O  # noqa: E402

CONTRACT_PATH = ROOT / 'config' / 'docking_contract.json'


@pytest.fixture(scope='module')
def contract():
    return C.load_contract(CONTRACT_PATH)


def observation(**overrides):
    doc = {'sensor_id': 'n_lidar_site', 'station_id': 'station_c', 'frame_id': 'world',
           'sequence': 1, 'sim_stamp': 10.0, 'wall_receive_time': 100.0,
           'relative_pose': {'longitudinal_m': 0.07, 'lateral_m': 0.0, 'height_m': 0.0,
                             'yaw_rad': 0.0,
                             'measurement_dimensions': ['longitudinal', 'lateral', 'yaw']},
           'uncertainty': {'longitudinal_m': 0.005}, 'validity': True, 'rejection_reason': None,
           'source_method': 'lidar_scan_match', 'calibration_version': 'gate-1'}
    doc.update(overrides)
    return doc


def refuses(reason, fn, *args, **kwargs):
    with pytest.raises((C.ContractRefused, O.ObservationRefused, M.DockRefused)) as caught:
        fn(*args, **kwargs)
    assert caught.value.reason == reason, f'expected {reason}, got {caught.value.reason}'


# ---------------------------------------------------------------- the contract ----
def test_the_committed_contract_loads(contract):
    assert contract['frozen'] is True
    assert contract['stations']['station_c']['trains_along'] == '+x'
    assert contract['observability']['certifies_tight_side'] is False, (
        'the observability gate could not certify the tight side, and the contract must not pretend '
        'otherwise')


def test_an_unfrozen_contract_is_refused(contract):
    raw = json.loads(CONTRACT_PATH.read_text(encoding='utf-8'))
    raw['frozen'] = False
    refuses('REFUSED_UNFROZEN_THRESHOLD', C.validate_contract, raw)


def test_an_unknown_field_is_refused(contract):
    raw = json.loads(CONTRACT_PATH.read_text(encoding='utf-8'))
    raw['tolerance']['slack_m'] = 0.01
    refuses('REFUSED_UNKNOWN_FIELD', C.validate_contract, raw)


def test_a_budget_entry_with_no_source_is_refused(contract):
    raw = json.loads(CONTRACT_PATH.read_text(encoding='utf-8'))
    raw['tolerance']['budget']['settling_drift_m'] = {'value_m': 0.002, 'source': 'invented'}
    refuses('REFUSED_UNSOURCED_THRESHOLD', C.validate_contract, raw)


def test_the_window_is_a_sum_of_worst_cases(contract):
    window = C.window_of(contract)
    assert window['worst_case_sum_m'] == pytest.approx(0.009)
    assert 'not a quadrature sum' in window['basis']
    assert window['within_declaration'] is True, 'the sum of the worst cases fits the loose side'
    tight = abs(contract['tolerance']['measured_bracket']['last_passing_negative_dx_m'])
    assert window['worst_case_sum_m'] > tight, (
        'the budget it declares, summed the conservative way, is larger than the tight side of the '
        'window C actually measured -- so the contract cannot be cleared by construction and says so')


def test_check_window_reports_each_component(contract):
    result = C.check_window(contract, longitudinal_m=0.07, lateral_m=0.001, height_m=0.0005,
                            yaw_rad=0.001)
    assert result['fits'] is True
    assert result['components']['total_translation_m']['ok'] is True
    outside = C.check_window(contract, longitudinal_m=0.10, lateral_m=0.0, height_m=0.0, yaw_rad=0.0)
    assert outside['fits'] is False


# ---------------------------------------------------------------- observations ----
def test_missing_measurement_is_unknown_not_zero(contract):
    verdict = O.accept_observation(observation(relative_pose=None), now_wall_s=100.0,
                                   contract=contract, expected_station_id='station_c')
    assert verdict['status'] == 'UNKNOWN'
    assert verdict['reason'] == 'REFUSED_NO_MEASUREMENT'


def test_a_stale_observation_is_stale(contract):
    verdict = O.accept_observation(observation(), now_wall_s=101.0, contract=contract,
                                   expected_station_id='station_c')
    assert verdict['status'] == 'STALE'
    assert verdict['reason'] == 'STALE_OBSERVATION'


def test_another_station_is_refused(contract):
    verdict = O.accept_observation(observation(station_id='station_a'), now_wall_s=100.0,
                                   contract=contract, expected_station_id='station_c')
    assert verdict['reason'] == 'REFUSED_WRONG_STATION'


def test_an_undeclared_frame_is_refused(contract):
    verdict = O.accept_observation(observation(frame_id='map_ish'), now_wall_s=100.0,
                                   contract=contract, expected_station_id='station_c')
    assert verdict['reason'] == 'REFUSED_UNKNOWN_FRAME'


def test_an_invalid_sensor_read_is_refused(contract):
    verdict = O.accept_observation(observation(validity=False, rejection_reason='glare'),
                                   now_wall_s=100.0, contract=contract,
                                   expected_station_id='station_c')
    assert verdict['reason'] == 'REFUSED_INVALID'


def test_an_empty_dimension_list_is_refused(contract):
    doc = observation()
    doc['relative_pose']['measurement_dimensions'] = []
    verdict = O.accept_observation(doc, now_wall_s=100.0, contract=contract,
                                   expected_station_id='station_c')
    assert verdict['status'] == 'UNKNOWN'


def test_a_fresh_observation_says_how_fresh(contract):
    verdict = O.accept_observation(observation(), now_wall_s=100.2, contract=contract,
                                   expected_station_id='station_c')
    assert verdict['status'] == 'FRESH'
    assert verdict['age_s'] == pytest.approx(0.2, abs=1e-9)


# ---------------------------------------------------------------- the machine ----
def walk_to(machine, _contract, *, state, now=100.0):
    seq = {'RESERVE': 0, 'APPROACH': 1, 'ACQUIRE_TARGET': 2}[state]
    machine.reserve(resources_reserved=True, now_wall_s=now)
    if seq >= 1:
        machine.permits.grant('navigation')
        machine.approach(now_wall_s=now)
    if seq >= 2:
        machine.acquire_target(in_pre_dock_area=True, now_wall_s=now)
    return machine


def test_the_happy_path_reaches_docked(contract):
    machine = walk_to(M.DockMachine(contract, epoch=3, station_id='station_c'), contract,
                      state='ACQUIRE_TARGET')
    machine.permits.release()
    machine.hand_over_to_align(observation(wall_receive_time=100.0), now_wall_s=100.0)
    assert machine.state == 'ALIGN'
    assert machine.permits.holder == 'docking'
    machine.settle(observation(wall_receive_time=100.1), speed_mps=0.0, now_wall_s=100.1)
    assert machine.state == 'SETTLE'
    machine.verify_dock(observation(wall_receive_time=100.8), speed_mps=0.0, now_wall_s=100.8)
    assert machine.state == 'VERIFY_DOCK'
    machine.dock(observation(wall_receive_time=100.9), now_wall_s=100.9)
    assert machine.state == 'DOCKED'


def test_align_without_releasing_the_navigation_permit_is_refused(contract):
    machine = walk_to(M.DockMachine(contract, epoch=3, station_id='station_c'), contract,
                      state='ACQUIRE_TARGET')
    refuses('REFUSED_WRONG_PERMIT_HOLDER', machine.hand_over_to_align, observation(),
            now_wall_s=100.0)


def test_align_without_a_fresh_observation_is_refused(contract):
    machine = walk_to(M.DockMachine(contract, epoch=3, station_id='station_c'), contract,
                      state='ACQUIRE_TARGET')
    machine.permits.release()
    refuses('REFUSED_OBSERVATION_NOT_FRESH', machine.hand_over_to_align,
            observation(relative_pose=None), now_wall_s=100.0)


def test_settle_requires_the_vehicle_to_be_stopped(contract):
    machine = walk_to(M.DockMachine(contract, epoch=3, station_id='station_c'), contract,
                      state='ACQUIRE_TARGET')
    machine.permits.release()
    machine.hand_over_to_align(observation(wall_receive_time=100.0), now_wall_s=100.0)
    refuses('REFUSED_NOT_AT_REST', machine.settle, observation(), speed_mps=0.5, now_wall_s=100.1)


def test_settle_requires_the_window_to_fit(contract):
    machine = walk_to(M.DockMachine(contract, epoch=3, station_id='station_c'), contract,
                      state='ACQUIRE_TARGET')
    machine.permits.release()
    machine.hand_over_to_align(observation(wall_receive_time=100.0), now_wall_s=100.0)
    far = observation()
    far['relative_pose']['longitudinal_m'] = 0.30
    refuses('REFUSED_OUT_OF_WINDOW', machine.settle, far, speed_mps=0.0, now_wall_s=100.1)


def test_verify_dock_needs_the_settle_interval_not_a_sleep(contract):
    machine = walk_to(M.DockMachine(contract, epoch=3, station_id='station_c'), contract,
                      state='ACQUIRE_TARGET')
    machine.permits.release()
    machine.hand_over_to_align(observation(wall_receive_time=100.0), now_wall_s=100.0)
    machine.settle(observation(wall_receive_time=100.1), speed_mps=0.0, now_wall_s=100.1)
    refuses('REFUSED_SETTLE_NOT_HELD', machine.verify_dock, observation(), speed_mps=0.0,
            now_wall_s=100.2)


def test_docked_expires_when_its_evidence_ages_out(contract):
    machine = walk_to(M.DockMachine(contract, epoch=3, station_id='station_c'), contract,
                      state='ACQUIRE_TARGET')
    machine.permits.release()
    machine.hand_over_to_align(observation(wall_receive_time=100.0), now_wall_s=100.0)
    machine.settle(observation(wall_receive_time=100.1), speed_mps=0.0, now_wall_s=100.1)
    machine.verify_dock(observation(wall_receive_time=100.8), speed_mps=0.0, now_wall_s=100.8)
    machine.dock(observation(wall_receive_time=100.9), now_wall_s=100.9)
    assert machine.step(now_wall_s=101.0)['valid'] is True
    late = machine.step(now_wall_s=103.5)
    assert late['valid'] is False
    assert machine.state == 'NEEDS_ATTENTION'
    assert machine.reason_code == 'DOCK_INVALIDATED'


def test_docked_is_invalidated_by_movement(contract):
    machine = walk_to(M.DockMachine(contract, epoch=3, station_id='station_c'), contract,
                      state='ACQUIRE_TARGET')
    machine.permits.release()
    machine.hand_over_to_align(observation(wall_receive_time=100.0), now_wall_s=100.0)
    machine.settle(observation(wall_receive_time=100.1), speed_mps=0.0, now_wall_s=100.1)
    machine.verify_dock(observation(wall_receive_time=100.8), speed_mps=0.0, now_wall_s=100.8)
    machine.dock(observation(wall_receive_time=100.9), now_wall_s=100.9)
    assert machine.step(now_wall_s=101.0, vehicle_moved=True)['valid'] is False


def test_an_interrupt_keeps_the_reason_and_a_resolve_needs_a_bool(contract):
    machine = walk_to(M.DockMachine(contract, epoch=3, station_id='station_c'), contract,
                      state='APPROACH')
    machine.interrupt(reason_code='TARGET_NOT_VISIBLE', now_wall_s=100.0)
    assert machine.state == 'STOPPING'
    assert machine.return_to == 'APPROACH'
    refuses('REFUSED_BAD_TYPE', machine.resolve, non_transfer_proven=None, now_wall_s=100.1)
    machine.resolve(non_transfer_proven=True, now_wall_s=100.2)
    assert machine.state == 'ABORTED'


def test_an_undeclared_reason_code_is_refused(contract):
    machine = walk_to(M.DockMachine(contract, epoch=3, station_id='station_c'), contract,
                      state='APPROACH')
    refuses('REFUSED_UNDECLARED_REASON', machine.interrupt, reason_code='BECAUSE',
            now_wall_s=100.0)


def test_a_stage_cannot_be_skipped(contract):
    machine = M.DockMachine(contract, epoch=3, station_id='station_c')
    refuses('REFUSED_BAD_TRANSITION', machine.approach, now_wall_s=100.0)


def test_every_refusal_this_module_raises_is_declared():
    source = (ROOT / 'src' / 'workcell' / 'docking' / 'machine.py').read_text(encoding='utf-8')
    import re
    raised = set(re.findall(r"self\._refuse\('([A-Z_]+)'", source))
    raised |= set(re.findall(r"DockRefused\('([A-Z_]+)'", source))
    assert raised <= M.DOCK_REFUSALS, f'undeclared: {sorted(raised - M.DOCK_REFUSALS)}'


def test_the_dock_vocabulary_is_disjoint_from_the_transfer_vocabulary():
    from workcell import transfer as T
    shared = M.DOCK_REFUSALS & T.TRANSFER_REFUSALS
    assert shared == {'REFUSED_STALE_EPOCH'} or not shared or shared == {'REFUSED_STALE_EPOCH'}, (
        'the docking refusals should not silently reuse the transfer state machine\'s vocabulary '
        f'beyond the shared epoch check; shared: {sorted(shared)}')


# ---------------------------------------------------------------------------------------------
# W3: the catch is a COMBINED budget. Found by measurement -- the first run of the corrected
# lattice contained a trial whose deck frame entered at exactly -35.0 mm, equal to the declared
# CATCH_M, and it FAILED. A square frame of half-width h yawed by theta puts its extreme corner at
# |offset| + h*(cos+sin), and the taper opens to h + CATCH, so offset and yaw spend one budget.
# ---------------------------------------------------------------------------------------------

def test_the_catch_is_a_combined_budget_of_offset_and_yaw():
    sys.path.insert(0, str(ROOT / "experiments"))
    import probe_w3_dock_mechanism as judge

    half = 0.300
    assert judge.catch_budget_m(0.0, 0.0, half) == pytest.approx(0.0)
    assert judge.catch_budget_m(0.035, 0.0, half) == pytest.approx(0.035)
    yaw_only = judge.catch_budget_m(0.0, math.radians(1.0), half)
    assert 0.005 < yaw_only < 0.0055, (
        f"one degree of yaw alone should spend about 5.2 mm of a 35 mm catch, got {yaw_only}")
    assert judge.catch_budget_m(0.030, math.radians(1.0), half) > 0.035, (
        "30 mm of offset plus 1 deg must be OUTSIDE a 35 mm catch even though each alone is "
        "inside: that is the whole point of the combined form")
    # ★ THE SYMMETRY. A test that only exercises +1 deg cannot see a sign error, and this one had
    # exactly that: the term was `h*(cos + sin - 1)`, which is NEGATIVE for a negative yaw, so the
    # budget fell below the offset it was charging and outside entries were reported inside.
    for theta in (0.25, 0.5, 1.0, 1.5):
        plus = judge.catch_budget_m(0.020, math.radians(theta), half)
        minus = judge.catch_budget_m(0.020, math.radians(-theta), half)
        assert plus == pytest.approx(minus), (
            f"the budget is not symmetric in the sign of the yaw at {theta} deg: {plus} vs {minus}")
        assert plus > 0.020, (
            f"yaw alone must ADD to the budget at any sign, got {plus} for {theta} deg")
