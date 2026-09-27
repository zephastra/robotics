"""W5: the cheap checks that do not need the five-minute physical run.

The physical claims live in `experiments/probe_w5_loop.py` and its report. What is here is the part
that can be wrong silently: an instrument that never moves, and a judge that does not import.
"""
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'experiments'))
sys.path.insert(0, str(ROOT / 'src'))

import mujoco  # noqa: E402

import w4_plant as wp  # noqa: E402


def test_the_w5_judge_imports():
    """A syntax or import error in a judge that takes five minutes to run must not wait five
    minutes to be found. Importing it also pins its module-level declarations."""
    import probe_w5_loop as judge

    assert judge.SOURCE_RES and judge.DECK_RES and judge.RECEIVER_RES
    skills = {skill for skill, _args in judge.SCRIPT}
    assert 'START_TRANSFER' in skills and 'VERIFY_TRANSFER' in skills
    assert 'VERIFY_DELIVERY' in skills


def test_the_sub_scenario_is_not_an_order():
    """The loop must not do picking. Calling the sub-scenario N01 would claim a fulfilment that was
    never attempted, so the absence is asserted rather than left to a reader."""
    import probe_w5_loop as judge

    kitting = {'PRESENT_TRAY', 'PICK_PART', 'PLACE_PART', 'VERIFY_KIT'}
    assert not ({skill for skill, _args in judge.SCRIPT} & kitting)
    assert judge.SCRIPT[0][0] == 'MOVE_TO_STATION' and judge.SCRIPT[-1][0] == 'VERIFY_DELIVERY'


def test_the_qpos_write_counter_is_live():
    """★ An instrument that cannot move is not an instrument. If `_set_state` stopped counting,
    every "the tray was never teleported" claim in the W5 report would become vacuous while still
    reading as a pass -- which is this project\'s most common defect family."""
    plant = wp.LogisticsPlant()
    assert plant.qpos_writes.get('initialise') == 1, plant.qpos_writes
    plant.reset()
    assert plant.qpos_writes['initialise'] == 2, (
        'the counter did not move on a reset, so it is not measuring writes')


def test_a_wheel_driven_move_writes_no_qpos():
    """The claim the loop makes about the tray depends on this: the wheels are the only thing that
    moves the vehicle, and no phase writes the state."""
    plant = wp.LogisticsPlant()
    plant.qpos_writes.pop('run', None)
    start = plant.chassis_x()
    target = plant.stations['station_c']['dock_x_m'] + 0.06
    plant.drive_to(target, speed=0.2, timeout_s=20.0)
    assert plant.qpos_writes.get('run') is None, (
        f'a drive wrote qpos: {plant.qpos_writes}')
    assert abs(plant.chassis_x() - target) < 0.01, (
        f'the vehicle ended {plant.chassis_x() - target:+.4f} m from its target, so this test did '
        f'not exercise a real move (started {start:.4f})')


def test_the_tray_is_a_free_body_with_no_weld():
    """A weld is one of the three ways to fake "the tray is held". Read from the compiled model."""
    plant = wp.LogisticsPlant()
    tray = plant.model.body('c_payload').id
    hits = []
    for e in range(plant.model.neq):
        if tray in (int(plant.model.eq_obj1id[e]), int(plant.model.eq_obj2id[e])):
            hits.append(int(plant.model.eq_type[e]))
    assert not hits, f'equality constraints touching the tray: {hits}'


# ------------------------------------------------------- the evidence classification (regression)
def test_every_stage_the_ledger_can_enter_is_classified():
    """The partition must be closed by construction, not by luck.

    A stage in neither `STATE_EVIDENCE` nor `NO_PHYSICAL_CLAIM` is a stage nothing can judge. The
    first W5 judge crashed on one; a judge that merely SKIPPED unknown stages would instead have
    let it pass unjudged, which is worse because it is silent.
    """
    from workcell import transfer as X
    from workcell.orchestration import sequence as SEQ

    assert SEQ.unclassified_stages() == [], (
        f'the ledger can enter {sorted(X.STAGES)} and these are in neither declared set: '
        f'{SEQ.unclassified_stages()}')
    assert set(X.STAGES) == set(SEQ.STATE_EVIDENCE) | set(SEQ.NO_PHYSICAL_CLAIM)


def test_a_booking_stage_needs_no_measured_evidence_and_an_unknown_stage_is_reported():
    """Both halves of the classification, each shown able to fail.

    `REQUESTED` is a booking stage: the run enters it before anything is measured, so demanding
    measured evidence there would fail a correct run -- which is exactly what happened. A stage in
    neither set must come back under `unclassified` rather than being skipped, because skipping is
    how a future stage would escape judgement.
    """
    from workcell.orchestration.sequence import classify_claims

    booking = classify_claims(['REQUESTED', 'BOTH_RESERVED'], {})
    assert booking['booking'] == ['BOTH_RESERVED', 'REQUESTED']
    assert booking['missing'] == {} and booking['unclassified'] == []

    unknown = classify_claims(['INVENTED_STAGE'], {})
    assert unknown['unclassified'] == ['INVENTED_STAGE'], (
        'a stage in neither declared set must be reported, not skipped')
    assert unknown['missing'] == {} and unknown['booking'] == []

    absent = classify_claims(['DOCK_VERIFIED'], {'relative_pose': True})
    assert absent['missing'] == {'DOCK_VERIFIED': ['stopped_confirmed', 'motion_permitted']}, (
        'a claimed state with evidence absent must be named, item by item')


def test_missing_evidence_still_refuses_an_undeclared_stage():
    """`classify_claims` tolerates undeclared stages by REPORTING them; `missing_evidence` must
    still refuse. If it quietly returned an empty list, an undeclared stage would read as fully
    evidenced -- the opposite of what the table means."""
    from workcell.orchestration.sequence import missing_evidence
    from workcell.schema import SchemaRefused

    with pytest.raises(SchemaRefused):
        missing_evidence('INVENTED_STAGE', {})


def test_the_tray_support_vocabulary_is_the_row_name_vocabulary():
    """The W5 judge once compared `plant.tray_support()` (ROW names) against a GEOM PREFIX.

    `LogisticsPlant.ROWS` is `(row_name, geom_prefix)` pairs, and the left element is what
    `tray_support()` and `zone_of()` return. Mixing the two makes the comparison unsatisfiable --
    not wrong sometimes, unsatisfiable always -- and the row it broke was the one whose job is to
    catch a claim made without evidence. This test pins the two vocabularies as DISJOINT so the
    mix cannot be reintroduced silently.
    """
    row_names = {row for row, _prefix in wp.LogisticsPlant.ROWS}
    prefixes = {prefix for _row, prefix in wp.LogisticsPlant.ROWS}

    assert row_names == {'source_band', 'deck', 'receiver_band'}, row_names

    # ★ THE DIRECTION MATTERS, and I got it wrong first. A row name is SHORT and the geom prefix is
    # long and CONTAINS it, so `prefix in row_name` is the question that can never be answered yes
    # -- which is why the judge's row failed on every run instead of sometimes. (`row in prefix` is
    # true and always was; asserting that would have proven nothing.)
    for row in row_names:
        for prefix in prefixes:
            assert prefix not in row, (
                f'{prefix!r} is a substring of the row name {row!r}, so the two vocabularies are '
                f'not distinguishable here and a cross-comparison would start matching by accident')

    # MEASURED, not just declared: what `tray_support()` actually hands the judge.
    plant = wp.LogisticsPlant()
    touched = plant.tray_support()
    assert touched, 'the tray is resting on its source row at construction, so this must be non-empty'
    assert set(touched) <= row_names, (
        f'tray_support() returned {touched!r}, which is not row names -- the judge consumes this '
        f'value, so its vocabulary is the one that matters')

    # and the zones the judge uses must be exactly those row names
    import probe_w5_loop as judge
    judge_zones = {spec['source_zone'] for spec in judge.TRANSFERS.values()}
    judge_zones |= {spec['destination_zone'] for spec in judge.TRANSFERS.values()}
    assert judge_zones <= row_names, (
        f'the judge names zones the plant does not have as rows: {sorted(judge_zones - row_names)}')


def test_a_plant_row_name_is_not_a_geom_name():
    """The other direction of the same fact, measured on the real model rather than declared: the
    strings `tray_support()` returns must not appear among the model's geom names, because if they
    did, "row name vs geom name" would be a distinction without a difference and the pinning test
    above would prove nothing."""
    plant = wp.LogisticsPlant()
    geom_names = {mujoco.mj_id2name(plant.model, mujoco.mjtObj.mjOBJ_GEOM, g) or ''
                  for g in range(plant.model.ngeom)}
    for row, _prefix in wp.LogisticsPlant.ROWS:
        assert row not in geom_names, (
            f'{row!r} is also a geom name, so a row name and a geom name are not distinguishable '
            f'here and the judge could regress without any test noticing')
