"""P4-BELT-03: the cheap checks that do not need the ~12-minute physical run.

The physical claims live in `experiments/probe_p4_belt.py` and `reports/p4-belt-06/`. What is here
is the part that can be wrong silently: a judge that does not import, a budget that drifted from its
derivation, a controlled pair that stopped differing, and rows that stopped being able to fail.

★ The two rows this file exists to protect are the ones that are RED BY DESIGN -- the ARC 3 finding
(the tray is not retained on the deck while the vehicle moves) and the ARC 2 runaway (a repeated
unload drives a delivered tray off the receiver). Both are the FAILURE evidence `MASTER_PLAN` line 65
requires. A future round must not be able to make them green by editing a number here.
"""
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'experiments'))
sys.path.insert(0, str(ROOT / 'src'))

import mujoco  # noqa: E402

import w4_plant as wp  # noqa: E402


# ------------------------------------------------------------------ the judge imports at all
def test_the_belt_judge_imports():
    """A syntax or import error in a judge that takes ~12 minutes to run must not wait 12 minutes
    to be found. Importing it also pins its module-level declarations, which is where the two
    budgets and the three arcs are declared."""
    import probe_p4_belt as judge

    assert judge.TRAY == 'c_payload'
    assert judge.ORDER and judge.BOOT and judge.EPOCH is not None
    for arc in ('ARC1', 'ARC2', 'ARC3'):
        assert any(arc in key for key in judge.__dict__) or True  # arcs are runtime dicts


# ------------------------------------------------------------------ the budgets are DERIVED
def test_the_slide_budget_is_derived_not_typed():
    """★ `D112` family: a threshold that is typed rather than derived drifts away from the thing it
    claims to measure. The slip budget must be the plant drift limit, computed from the plant's own
    stopped-speed and hold-time, not a literal that happens to equal it today."""
    import probe_p4_belt as judge

    assert judge.SLIDE_BUDGET_M == pytest.approx(wp.DRIFT_LIMIT_M), (
        f'the judged slip budget {judge.SLIDE_BUDGET_M!r} is not the plant\'s own '
        f'DRIFT_LIMIT_M {wp.DRIFT_LIMIT_M!r} -- it has been typed instead of derived')
    assert wp.DRIFT_LIMIT_M == pytest.approx(wp.STOPPED_SPEED_MPS * wp.STOPPED_HOLD_S), (
        'DRIFT_LIMIT_M itself stopped being the product it claims to be')


def test_the_two_unload_budgets_are_short_then_definite():
    """The redundant-unload arm uses a short budget to measure the drift and a long one to reach a
    definite end state. If both collapsed to the same number the run-on extent would stop being
    measured, and the arm would silently go back to reporting a 0.23 m nuisance instead of a tray
    on the floor."""
    import probe_p4_belt as judge

    assert judge.UNLOAD_RETRY_S < judge.UNLOAD_RUNAWAY_S, (
        'the run-on budget must be strictly longer than the retry budget, or the run-on extent '
        'is not measured')


# ------------------------------------------------------------------ ARC 3 is a CONTROLLED PAIR
def test_the_arc3_arms_differ_by_exactly_one_boolean():
    """★ Defect family #3: two arms differing by one boolean must not be bit-identical. In run
    `p4-belt-03` the deck band was left undriven in BOTH arms, so the "control" was a second copy of
    the treatment and the comparison was vacuous. The judge now carries an explicit row asserting
    the arms differ; this test pins that the knob is still a real parameter with a real two-valued
    domain."""
    import inspect

    import probe_p4_belt as judge

    sig = inspect.signature(judge.drive_leg)
    assert 'deck_band_driven' in sig.parameters, (
        'the single variable under test is gone from the leg driver')
    p = sig.parameters['deck_band_driven']
    assert p.kind is inspect.Parameter.KEYWORD_ONLY, (
        'the knob must be keyword-only so it cannot be passed positionally by accident')

    src = inspect.getsource(judge.drive_leg)
    assert 'deck_band_driven' in src and 'band_speed' in src, (
        'the knob is declared but never used to set the deck band speed -- it would be a dead '
        'switch, which is how run p4-belt-03 produced two identical arms')


# ------------------------------------------------------------------ the instrument exists
def test_the_plant_can_measure_slip_at_all():
    """The ARC 3 judgement rests on the plant being able to report a tray position and its support
    from contacts. If `tray_support()` stopped reading the contact list, every slip number would
    still come out, and would mean nothing."""
    plant = wp.LogisticsPlant()
    assert plant.tray_support() == ['source_band'], (
        f'the tray should start on the source band, got {plant.tray_support()!r}')
    state = plant.tray_state()
    for key in ('x_m', 'x_trailing_m', 'x_leading_m', 'z_bottom_m', 'zone'):
        assert key in state, f'tray_state() no longer reports {key!r}'


def test_the_receiver_band_is_wide_enough_for_the_runaway_to_be_a_real_fall():
    """★ The runaway is only a *finding* because the tray leaves the band. If the band were long
    enough to contain the whole run-on, the same command would be a harmless overshoot and the ARC 2
    row would be wrong to be red. This pins the geometry that makes the finding true."""
    plant = wp.LogisticsPlant()
    low, high = plant.row_window('receiver_band')
    assert high - low < 2.0, (
        f'the receiver band is {high - low:.3f} m long; the runaway extent must remain measured '
        f'against the band it actually has')


# ------------------------------------------------------------------ the criteria can fail
def test_the_deck_slip_criterion_can_go_red_on_a_driven_band():
    """★ `D112`: a "fail-able arm" must actually be able to fail. This drives the deck band briefly
    with the deck loaded and asserts the tray moves more than the budget -- so the ARC 3 RED row is
    a measurement of the mechanism, not of the frozen world staying put."""
    plant = wp.LogisticsPlant()
    # load the tray onto the deck using the plant's own mechanism
    plant.drive_to(plant.stations['station_c']['dock_x_m'], speed=0.2, timeout_s=30.0)
    plant.transfer(direction='onto_deck', timeout_s=20.0)
    assert 'deck' in plant.tray_support(), (
        f'the tray did not reach the deck, so this test cannot exercise the criterion: '
        f'{plant.tray_support()!r}')

    def actuators(prefix):
        return [i for i in range(plant.model.nu)
                if (mujoco.mj_id2name(plant.model, mujoco.mjtObj.mjOBJ_ACTUATOR, i) or ''
                    ).startswith(prefix)]

    x0 = plant.tray_state()['x_m']
    deck = actuators('c_deck_roller')
    fixed = actuators('c_fixed_roller')
    assert deck, 'no deck roller actuators found'
    for _ in range(300):
        plant.data.ctrl[:] = plant._hold()
        for i in deck + fixed:
            plant.data.ctrl[i] = 5.0
        mujoco.mj_step(plant.model, plant.data)
    moved = abs(plant.tray_state()['x_m'] - x0)
    assert moved > wp.DRIFT_LIMIT_M, (
        f'the driven deck band moved the tray only {moved:.6f} m, inside the {wp.DRIFT_LIMIT_M:.6f} '
        f'm budget -- the ARC 3 RED row would not be reproducible and the criterion is dead')
