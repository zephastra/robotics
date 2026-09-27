"""Stage criteria for the H tray task.

Every stage must be provably reachable in both directions: a criterion that has only
ever returned PASS is not a criterion. These tests feed synthetic samples, so they need
no simulator and they pin the thresholds the probe uses.
"""
from pathlib import Path
import sys

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src'))

from humanoid007 import tray_task  # noqa: E402

Z0 = 0.85
LIFT = tray_task.THRESHOLDS['lift_clearance_m']
ALL_STAGES = set(tray_task.STAGES)


def sample(t, z=Z0, left=0, right=0, speed=0.0, arm=0.4, x=0.23, y=0.0):
    return {'t': t, 'payload_z': z, 'payload_x': x, 'payload_y': y,
            'payload_speed': speed,
            'hand_contacts': {'left': left, 'right': right}, 'arm_dev_rad': arm}


ALL_STAGES = set(tray_task.STAGES)


def test_the_six_stages_are_exactly_the_planned_ones():
    """抓取/离台/稳定持有/放下/松手后稳定/退出, from P1_FEASIBILITY."""
    assert tray_task.STAGES == ('grasp', 'lift', 'hold', 'place', 'release', 'exit')


def test_every_stage_is_not_run_when_the_run_did_not_drive_it():
    result = tray_task.evaluate([sample(0.0), sample(0.1)], set(), Z0)
    assert result['not_run'] == list(tray_task.STAGES)
    assert result['overall'].startswith('NOT_RUN')
    for name in tray_task.STAGES:
        assert result['stages'][name]['verdict'] == 'NOT_RUN'


def test_grasp_passes_on_two_handed_contact_and_fails_on_one_handed():
    both = [sample(0.0), sample(0.1, left=2, right=3)]
    assert tray_task.evaluate(both, {'grasp'}, Z0)['stages']['grasp']['verdict'] == 'PASS'

    one = [sample(0.0), sample(0.1, left=4, right=0)]
    verdict = tray_task.evaluate(one, {'grasp'}, Z0)['stages']['grasp']
    assert verdict['verdict'] == 'FAIL'
    assert 'both hands never made contact' in verdict['detail']


def test_lift_needs_real_clearance_not_a_wobble():
    enough = [sample(0.0), sample(0.1), sample(0.2, z=Z0 + LIFT)]
    assert tray_task.evaluate(enough, {'lift'}, Z0)['stages']['lift']['verdict'] == 'PASS'

    wobble = [sample(0.0), sample(0.1), sample(0.2, z=Z0 + LIFT / 2)]
    verdict = tray_task.evaluate(wobble, {'lift'}, Z0)['stages']['lift']
    assert verdict['verdict'] == 'FAIL'
    assert verdict['numbers']['rise_m'] == pytest.approx(LIFT / 2)


def test_hold_needs_an_unbroken_two_handed_airborne_run():
    need = tray_task.THRESHOLDS['hold_seconds']
    step = 0.1
    count = int(need / step) + 5
    good = [sample(0.0, left=2, right=2)]
    good += [sample(0.1 + i * step, z=Z0 + LIFT, left=2, right=2) for i in range(count)]
    assert tray_task.evaluate(good, {'hold'}, Z0)['stages']['hold']['verdict'] == 'PASS'

    # Same duration, but the contact drops out halfway: not a hold.
    broken = [sample(0.0, left=2, right=2)]
    for i in range(count):
        contact = 2 if i < count // 2 else 0
        broken.append(sample(0.1 + i * step, z=Z0 + LIFT, left=contact, right=contact))
    verdict = tray_task.evaluate(broken, {'hold'}, Z0)['stages']['hold']
    assert verdict['verdict'] == 'FAIL'
    assert verdict['numbers']['held_seconds'] < need


def test_hold_does_not_count_time_while_the_tray_is_still_on_the_table():
    step = 0.1
    count = int(tray_task.THRESHOLDS['hold_seconds'] / step) + 5
    on_table = [sample(0.1 + i * step, z=Z0, left=2, right=2) for i in range(count)]
    verdict = tray_task.evaluate(on_table, {'hold'}, Z0)['stages']['hold']
    assert verdict['verdict'] == 'FAIL'


def test_place_passes_only_when_the_tray_comes_back_down_while_held():
    good = [sample(0.0), sample(0.1, left=2, right=2),
            sample(0.2, z=Z0 + LIFT, left=2, right=2),
            sample(0.3, z=Z0 + 0.002, left=2, right=2)]
    assert tray_task.evaluate(good, {'place'}, Z0)['stages']['place']['verdict'] == 'PASS'

    stayed_up = [sample(0.0), sample(0.1, left=2, right=2),
                 sample(0.2, z=Z0 + LIFT, left=2, right=2),
                 sample(0.3, z=Z0 + LIFT, left=2, right=2)]
    verdict = tray_task.evaluate(stayed_up, {'place'}, Z0)['stages']['place']
    assert verdict['verdict'] == 'FAIL'
    assert verdict['numbers']['closest_m'] == pytest.approx(LIFT)


def test_release_needs_the_hands_off_and_the_tray_still():
    step = 0.1
    count = int(tray_task.THRESHOLDS['rest_seconds'] / step) + 5
    good = [sample(0.0, left=2, right=2), sample(0.1, z=Z0 + 0.002, left=2, right=2)]
    good += [sample(0.2 + i * step, z=Z0, speed=0.001) for i in range(count)]
    assert tray_task.evaluate(good, {'release'}, Z0)['stages']['release']['verdict'] == 'PASS'

    still_holding = [sample(0.0, left=2, right=2),
                     sample(0.1, z=Z0, left=2, right=2),
                     sample(0.2, z=Z0, left=1, right=1, speed=0.001)]
    verdict = tray_task.evaluate(still_holding, {'release'}, Z0)['stages']['release']
    assert verdict['verdict'] == 'FAIL'
    assert verdict['numbers']['cleared_t'] is None

    drifting = [sample(0.0, z=Z0, left=0, right=0, speed=0.4),
                sample(0.1, z=Z0, left=0, right=0, speed=0.4)]
    verdict = tray_task.evaluate(drifting, {'release'}, Z0)['stages']['release']
    assert verdict['verdict'] == 'FAIL'
    assert verdict['numbers']['settled_seconds'] < tray_task.THRESHOLDS['rest_seconds']


def test_release_is_judged_after_the_grasp_not_before_it():
    """Regression: the first version anchored on the run's opening samples.

    A run starts with the tray untouched; if `release` counts that as "hands cleared",
    every run passes the stage without ever letting go. Caught by running the real
    sequence, which reported cleared_t = 0.10 s.
    """
    step = 0.1
    count = int(tray_task.THRESHOLDS['rest_seconds'] / step) + 5
    rows = [sample(0.0, z=Z0, left=0, right=0, speed=0.001)]
    rows.append(sample(0.1, left=2, right=2))
    rows.append(sample(0.2, z=Z0 + LIFT, left=2, right=2))
    rows.append(sample(0.3, z=Z0, left=2, right=2))
    for i in range(count):
        rows.append(sample(0.4 + i * step, z=Z0, speed=0.001))
    verdict = tray_task.evaluate(rows, {'release'}, Z0)['stages']['release']
    assert verdict['verdict'] == 'PASS'
    assert verdict['numbers']['cleared_t'] >= 0.4, verdict['numbers']


def test_release_fails_when_the_run_ends_in_the_hands():
    rows = [sample(0.0), sample(0.1, left=2, right=2), sample(0.2, z=Z0, left=2, right=2)]
    verdict = tray_task.evaluate(rows, {'release'}, Z0)['stages']['release']
    assert verdict['verdict'] == 'FAIL'
    assert 'ended while the hands were still on the tray' in verdict['detail']


def test_exit_requires_the_arms_to_leave_the_work_zone():
    limit = tray_task.THRESHOLDS['exit_arm_rad']
    good = [sample(0.0, left=2, right=2, arm=0.40),
            sample(0.1, left=0, right=0, arm=limit / 2),
            sample(0.2, left=0, right=0, arm=0.0)]
    assert tray_task.evaluate(good, {'exit'}, Z0)['stages']['exit']['verdict'] == 'PASS'

    stayed_out = [sample(0.0, left=2, right=2, arm=0.40),
                  sample(0.1, left=0, right=0, arm=0.40),
                  sample(0.2, left=0, right=0, arm=0.40)]
    verdict = tray_task.evaluate(stayed_out, {'exit'}, Z0)['stages']['exit']
    assert verdict['verdict'] == 'FAIL'
    assert verdict['numbers']['final_arm_dev_rad'] == pytest.approx(0.40)


def test_overall_reports_fail_before_not_run():
    only_some = [sample(0.0), sample(0.1, z=Z0 + LIFT)]
    result = tray_task.evaluate(only_some, {'lift'}, Z0)
    assert result['failed'] == []
    assert result['overall'].startswith('NOT_RUN')

    # Grasp is satisfied, lift is not: the run is a FAIL, not a NOT_RUN.
    broken = [sample(0.0), sample(0.1, left=2, right=2), sample(0.2, z=Z0 + LIFT / 2)]
    result = tray_task.evaluate(broken, {'grasp', 'lift'}, Z0)
    assert result['failed'] == ['lift']
    assert result['overall'].startswith('FAIL')
    assert 'grasp' in [n for n in tray_task.STAGES if n not in result['failed']]


def test_exit_fails_if_the_arms_come_back_and_then_leave_again():
    """Reaching the posture is not the same as holding it."""
    limit = tray_task.THRESHOLDS['exit_arm_rad']
    rows = [sample(0.0, left=2, right=2, arm=0.40),
            sample(0.1, left=0, right=0, arm=limit / 2),
            sample(0.2, left=0, right=0, arm=0.90)]
    verdict = tray_task.evaluate(rows, {'exit'}, Z0)['stages']['exit']
    assert verdict['verdict'] == 'FAIL'
    assert 'left the posture again' in verdict['detail']


def test_exit_fails_when_the_run_ends_with_the_hands_still_on_the_tray():
    rows = [sample(0.0, left=2, right=2, arm=0.40),
            sample(0.1, left=2, right=2, arm=0.35)]
    verdict = tray_task.evaluate(rows, {'exit'}, Z0)['stages']['exit']
    assert verdict['verdict'] == 'FAIL'
    assert 'still on the tray' in verdict['detail']


def test_a_fully_driven_healthy_run_passes():
    """One timeline that satisfies all six criteria, on monotonic timestamps."""
    step = 0.1
    rows = [sample(0.0)]                                   # settle, arms out

    def append(t, **kwargs):
        rows.append(sample(t, **kwargs))

    t = 0.1
    append(t, left=2, right=2, arm=0.43)                   # grasp
    t = 0.2
    append(t, z=Z0 + LIFT, left=2, right=2, arm=0.62)      # lift begins
    t += step
    while t < 3.0:                                          # hold: 2.8 s airborne
        append(t, z=Z0 + LIFT, left=2, right=2, arm=0.62)
        t += step
    append(t, z=Z0 + 0.002, left=2, right=2, arm=0.43)     # place
    t += step
    for _ in range(4):                                      # release: hands travel back
        append(t, z=Z0, left=0, right=0, speed=0.001, arm=0.30)
        t += step
    while t < 6.0:                                          # exit: settled, held to the end
        append(t, z=Z0, left=0, right=0, speed=0.001, arm=0.02)
        t += step

    stamps = [row['t'] for row in rows]
    assert stamps == sorted(stamps), 'the synthetic timeline must be monotonic'

    result = tray_task.evaluate(rows, ALL_STAGES, Z0)
    assert result['failed'] == [], result['failed']
    assert result['not_run'] == [], result['not_run']
    assert result['overall'].startswith('PASS'), result['overall']


def test_a_timeline_that_never_grasps_fails_rather_than_passing_quietly():
    """The most likely way this whole test set could be worthless."""
    rows = [sample(i * 0.1, z=Z0 + LIFT) for i in range(40)]   # airborne, never touched
    result = tray_task.evaluate(rows, ALL_STAGES, Z0)
    assert 'grasp' in result['failed']
    assert 'release' in result['failed']
    assert result['overall'].startswith('FAIL')


def test_exit_fails_if_the_withdrawal_shoves_the_tray():
    """The defect that motivated the limit: withdrawing brushed the tray, which slid."""
    limit = tray_task.THRESHOLDS['exit_tray_travel_m']
    rows = [sample(0.0, left=2, right=2, arm=0.40, x=0.23),
            sample(0.1, left=0, right=0, arm=0.02, x=0.23),
            sample(0.2, left=0, right=0, arm=0.02, x=0.23 + limit * 3)]
    verdict = tray_task.evaluate(rows, {'exit'}, Z0)['stages']['exit']
    assert verdict['verdict'] == 'FAIL'
    assert 'moved the tray' in verdict['detail']
    assert verdict['numbers']['tray_travel_m'] == pytest.approx(limit * 3)


def test_exit_fails_if_the_withdrawal_knocks_the_tray_without_moving_it_far():
    speed_limit = tray_task.THRESHOLDS['exit_tray_speed_mps']
    rows = [sample(0.0, left=2, right=2, arm=0.40),
            sample(0.1, left=0, right=0, arm=0.02),
            sample(0.2, left=0, right=0, arm=0.02, speed=speed_limit * 5)]
    verdict = tray_task.evaluate(rows, {'exit'}, Z0)['stages']['exit']
    assert verdict['verdict'] == 'FAIL'
    assert 'knocked the tray' in verdict['detail']


def test_exit_reports_the_settled_deviation_not_the_crossing_value():
    """The old wording reported the deviation at the instant it first qualified.

    For a monotonically converging arm that number is always just under the limit, so it
    reads as "passed by 1.3%" -- wrong, and it hid a 13 mm tray shove.
    """
    limit = tray_task.THRESHOLDS['exit_arm_rad']
    rows = [sample(0.0, left=2, right=2, arm=0.40),
            sample(0.1, left=0, right=0, arm=limit * 0.99),
            sample(0.2, left=0, right=0, arm=0.002)]
    verdict = tray_task.evaluate(rows, {'exit'}, Z0)['stages']['exit']
    assert verdict['verdict'] == 'PASS'
    assert verdict['numbers']['final_arm_dev_rad'] == pytest.approx(0.002)
    assert 'settling to 0.0020 rad' in verdict['detail']
    assert 'moved 0.00 mm' in verdict['detail']


def test_exit_says_so_when_the_recording_carries_no_tray_position():
    rows = [{'t': 0.0, 'payload_z': Z0, 'payload_speed': 0.0, 'arm_dev_rad': 0.4,
             'hand_contacts': {'left': 0, 'right': 0}}]
    verdict = tray_task.evaluate(rows, {'exit'}, Z0)['stages']['exit']
    assert verdict['verdict'] == 'FAIL'
    assert 'no tray position' in verdict['detail']
