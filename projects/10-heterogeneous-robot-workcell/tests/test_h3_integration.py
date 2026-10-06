"""H3: the cheap checks that do not need the ten-minute physical run.

The physical claims live in `experiments/probe_h3_w5.py` and `reports/p4-h3-w5-01`. What is here is
the part that can be wrong silently:

  * a judge that raises `NameError` AFTER its run has already measured everything -- which is
    exactly what happened (`XFER` imported in one function, used in another), and it turned a
    six-minute run into `POST_RUN_ERROR` with no rows;
  * a world whose two files differ somewhere nobody declared;
  * a threshold that no row reads, i.e. a number frozen for show;
  * the negative arm losing one of its two required substitutions, which would make it a
    "failure" that still delivers the tray.
"""
import hashlib
import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'experiments'))
sys.path.insert(0, str(ROOT / 'src'))

import mujoco  # noqa: E402

import check_globals as cg  # noqa: E402


#: the files the H3 chain is made of. Each is frozen in `config/p3_freeze.json`.
H3_CHAIN = (
    'experiments/probe_h3_w5.py',
    'experiments/w5_h085_plan.py',
    'experiments/probe_h2_w5.py',
    'experiments/probe_h_w5.py',
    'experiments/w4_plant.py',
    'src/humanoid007/runtime.py',
)


def test_the_h3_judge_imports():
    """A syntax or import error in a judge that takes ten minutes to run must not wait ten minutes
    to be found. Importing it also pins its module-level declarations."""
    import probe_h3_w5 as judge

    assert judge.H3_THRESHOLDS
    assert judge.BAND_CLEARANCE_BUDGET_M > 0
    assert judge.WORLD.name == 'world_w5_h085_loop.xml'
    assert judge.TRAY == 'c_payload'


def test_no_chain_file_reads_an_unbound_global():
    """★ THE DEFECT THIS CATCHES, MEASURED. `judge_h3` used `XFER.STAGES` while `XFER` is imported
    inside `run()`. The judge runs LAST, so the failure arrived after every physical claim had been
    measured, and the report came back `POST_RUN_ERROR` with zero rows. `check_globals` resolves
    scopes with `symtable`, so it sees this in a second.
    """
    problems = [p for name in H3_CHAIN for p in cg.report(ROOT / name)]
    assert problems == [], 'unbound global reads: %r' % problems


def test_the_globals_checker_can_fail():
    """★ A check that cannot fail is not a check. Two of this project's defect families are exactly
    that, so the instrument is exercised on a module that HAS the defect."""
    import tempfile

    broken = (
        'def load():\n'
        '    import json as J\n'
        '    return J.dumps({})\n'
        '\n'
        'def use():\n'
        '    return J.dumps({})\n'
    )
    path = Path(tempfile.mkdtemp()) / 'broken.py'
    path.write_text(broken, encoding='utf-8')
    found = cg.report(path)
    assert any(name == 'J' for *_rest, name in found), \
        'the checker did not see an import used from another scope'


def test_the_h3_judge_runs_on_a_fixture_report_positive_arm():
    """★ THE JUDGE IS TESTABLE IN A SECOND, WHICH IT WAS NOT.

    `judge_h3` runs LAST, after every physical claim of a ~2-minute run has been measured. Two runs
    were lost to defects that exist only there -- `XFER` imported in another function, then `tx`
    defined after a row that reads it -- and neither an import check nor a scope check catches an
    ORDERING error inside one function. Calling the judge on a fixture turns both into a
    one-second failure.
    """
    import argparse

    import probe_h3_w5 as judge
    from _h3_fixture import report as fixture

    args = argparse.Namespace(negative_grasp=False)
    rows = judge.judge_h3(fixture(negative=False), 0.2295, 'h_lh_rf_tip', 0.1262,
                          'h_lh_rf_tip', args)
    assert rows, 'the judge produced no rows'
    # every declared threshold-backed check is present, and none is silently missing
    for name in ('one_world_one_data', 'same_tray_entity', 'tray_on_source_band_at_handoff',
                 'hands_clear_of_tray_at_handoff', 'vehicle_parked_while_humanoid_works',
                 'tray_still_between_release_and_the_chain', 'downstream_waits_for_release',
                 'the_chain_moved_the_tray', 'chain_reached_receiver',
                 'every_chain_step_succeeded', 'dock_residuals_within_tolerance',
                 'custody_transferred_by_the_ledger',
                 'each_leg_destination_matched_by_contacts', 'no_weld_on_the_tray',
                 'no_runtime_qpos_write', 'downstream_gated_on_handoff',
                 'no_abnormal_collision', 'humanoid_withdrew_clear'):
        assert name in rows, 'the judge no longer reports %r' % name
    assert 'PASS' in rows['chain_reached_receiver']['verdict']


def test_the_h3_judge_runs_on_a_fixture_report_negative_arm():
    """And the negative arm must be judged too, with the rows it cannot exercise marked NOT_RUN
    rather than scored against the positive arm's expectations."""
    import argparse

    import probe_h3_w5 as judge
    from _h3_fixture import report as fixture

    args = argparse.Namespace(negative_grasp=True)
    rows = judge.judge_h3(fixture(negative=True), 0.2295, 'h_lh_rf_tip', 0.1262,
                          'h_lh_rf_tip', args)
    not_run = [k for k, v in rows.items() if v['verdict'] == 'NOT_RUN']
    assert not_run, 'the negative arm marked no row NOT_RUN'
    # the gating row is the negative arm's own claim, and the fixture satisfies it
    assert 'PASS' in rows['downstream_gated_on_handoff']['verdict'], \
        rows['downstream_gated_on_handoff']['detail']
    # and a row that could not be exercised is not counted as a pass
    assert rows['chain_reached_receiver']['verdict'] == 'NOT_RUN'


def test_no_row_detail_has_an_unformatted_placeholder():
    """A detail containing `%s` prints the judge's own source instead of its numbers. The judge
    asserts this itself; this checks the assertion is still there."""
    text = (ROOT / 'experiments' / 'probe_h3_w5.py').read_text(encoding='utf-8')
    assert 'unformatted placeholders' in text


def test_every_row_declares_the_arm_it_is_judged_in():
    """★ THE STRUCTURAL FIX for a defect that happened FOUR TIMES.

    Each time, a row written for the positive arm was scored in the negative arm, where its claim is
    not the claim the run makes -- `chain_reached_receiver`, `tray_on_source_band_at_handoff`,
    `custody_transferred_by_the_ledger`, `each_leg_destination_matched_by_contacts`. Each one turned
    a correct run into a red mark, and each was found only by RUNNING the arm (two minutes).

    `judge_h3` now refuses to return if a row judged in this arm is not declared for it, so this
    test proves the guard is reachable and the mapping covers every row.
    """
    import argparse

    import probe_h3_w5 as judge
    from _h3_fixture import report as fixture

    assert set(judge.ROW_ARMS), 'the arm mapping is empty'
    for arm, negative in (('positive', False), ('negative', True)):
        args = argparse.Namespace(negative_grasp=negative)
        rows = judge.judge_h3(fixture(negative=negative), 0.2295, 'h_lh_rf_tip', 0.1262,
                              'h_lh_rf_tip', args)
        # the guard's own invariant, restated here so the mapping cannot drift from the rows
        for name, verdict in ((n, r['verdict']) for n, r in rows.items()):
            if arm not in judge.ROW_ARMS[name]:
                assert verdict == 'NOT_RUN', \
                    '%s is not judged in the %s arm but returned %s' % (name, arm, verdict)
        assert set(rows) == set(judge.ROW_ARMS)


def test_the_arm_guard_can_fail():
    """★ A check that cannot fail is not a check. The guard is exercised by breaking a row's
    arm-awareness and requiring the judge to raise."""
    import argparse

    import probe_h3_w5 as judge
    from _h3_fixture import report as fixture

    # `no_abnormal_collision` is judged in BOTH arms and returns PASS in the negative one. Declaring
    # it positive-only therefore makes it a row that "is not judged in the negative arm but returned
    # a verdict anyway" -- exactly the shape the guard exists for. (Picking a row that already
    # reports NOT_RUN in the negative arm, as the first version of this test did, proves nothing:
    # a NOT_RUN row satisfies the guard whether or not it is declared for the arm.)
    name = 'no_abnormal_collision'
    saved = judge.ROW_ARMS[name]
    judge.ROW_ARMS[name] = ('positive',)
    try:
        args = argparse.Namespace(negative_grasp=True)
        try:
            judge.judge_h3(fixture(negative=True), 0.2295, 'h_lh_rf_tip', 0.1262,
                           'h_lh_rf_tip', args)
        except RuntimeError as refusal:
            assert 'not judged in the negative arm' in str(refusal), str(refusal)
        else:
            raise AssertionError('the arm guard did not fire on a row that lost its arm-awareness')
    finally:
        judge.ROW_ARMS[name] = saved


def test_the_h3_world_differs_from_candidate_b_in_exactly_the_declared_way():
    """The builder's whole claim is that it changed ONE line plus the keyframe.

    Asserted on the text, so a silent extra edit fails here as well as in the builder itself -- and
    asserted on the COMPILED sizes, because a text that does not load must not be able to pass.
    """
    import w5_h085_plan as plan

    source = plan.SOURCE.read_text(encoding='utf-8')
    built = plan.OUT.read_text(encoding='utf-8')
    src_lines, out_lines = source.split('\n'), built.split('\n')
    assert len(src_lines) == len(out_lines), 'the builder changed the line count'

    changed = [(n, a, b) for n, (a, b) in enumerate(zip(src_lines, out_lines), 1) if a != b]
    assert len(changed) == 2, 'expected exactly 2 differing lines, saw %r' % [c[0] for c in changed]
    anchors = {name: [n for n, _a, b in changed if name in b]
               for name in ('h_LINK_BASE', '<key name="home"')}
    assert all(len(v) == 1 for v in anchors.values()), anchors
    # and the humanoid base is the only BODY that moved
    body_changes = [n for n, _a, b in changed if 'name="h_LINK_BASE"' in b]
    assert body_changes == [anchors['h_LINK_BASE'][0]]

    before = mujoco.MjModel.from_xml_path(str(plan.SOURCE))
    after = mujoco.MjModel.from_xml_path(str(plan.OUT))
    assert (after.nq, after.nbody, after.ngeom) == (before.nq, before.nbody, before.ngeom), \
        'the H3 world changed size, so it added or removed something'
    assert after.nq == 151


def test_the_h3_world_places_the_humanoid_where_the_world_says():
    """The station must be the WORLD's, not a probe's. The keyframe and the body must agree, and
    the humanoid must actually be next to the presentation station rather than at the far end of
    the cell -- which is what candidate B shipped (`x 0.660132` against a table at `x 4.31`)."""
    import numpy as np
    import w5_h085_plan as plan

    model = mujoco.MjModel.from_xml_path(str(plan.OUT))
    data = mujoco.MjData(model)
    data.qpos[:] = model.key_qpos[0]
    mujoco.mj_forward(model, data)

    base = data.xpos[model.body('h_LINK_BASE').id]
    table = data.geom_xpos[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM,
                                            'w5_h085_station')]
    tray = data.xpos[model.body('c_payload').id]
    assert abs(float(base[0]) - plan.STATION_X) < 1e-6, base
    assert abs(float(base[0]) - float(model.key_qpos[0][0])) < 1e-9, \
        'the keyframe and the body disagree about the station'
    # the robot is at the workcell, not four metres away from it
    assert abs(float(base[0]) - float(table[0])) < 0.5, (base, table)
    # and the tray sits between the robot's hands and the band, i.e. the supply is reachable
    assert float(base[0]) < float(tray[0]) < float(band_first_roller_x(model, data))


def band_first_roller_x(model, data):
    obj = mujoco.mjtObj.mjOBJ_GEOM
    row = sorted(float(data.geom_xpos[g][0]) for g in range(model.ngeom)
                 if (mujoco.mj_id2name(model, obj, g) or '').startswith('c_fixed_roller'))
    return row[0]


def test_the_negative_arm_keeps_both_of_its_substitutions():
    """★ MEASURED, AND IT NEEDS TWO. The first negative arm only opened the fingers; the OPEN hand
    then swept the `place` translation and BULLDOZED the tray off the fixture onto the source band
    (`_diag/out_h3_neg1.txt`: handoff support `source_band`, tray x 4.4869), and the chain
    delivered it. A failure that delivers is not a failure.

    This asserts the source still computes BOTH from the negative flag. It is a source-level check
    on purpose: the physical arm is what proves it, but losing one substitution is a one-line edit
    that nothing else would notice.
    """
    text = (ROOT / 'experiments' / 'probe_h3_w5.py').read_text(encoding='utf-8')
    assert re.search(r'grasp_posture\s*=\s*OPEN\s+if\s+negative\s+else\s+GRASP', text), \
        'the negative arm no longer substitutes the grasp posture'
    assert re.search(r'carry_target\s*=\s*float\(tray0\[0\]\)\s+if\s+negative\s+else\s+'
                     r'H2\.PLACE_X', text), \
        'the negative arm no longer pins the carry target, so the open hand can bulldoze the tray'
    # and it must actually reach the call site. Counted on the call, not on a `split`, because the
    # first version of this assertion split on the string `stage_driver` and indexed element 2 --
    # which is a claim about how many times the NAME appears in the file, not about the argument.
    call = re.search(r'H2\.stage_driver\((.*?)\)\n', text, re.S)
    assert call is not None, 'no stage_driver call site found'
    assert 'carry_target' in call.group(1), \
        'the carry target is computed but never passed to the stage driver'


def test_the_rows_are_arm_aware():
    """A stage the run did not drive is NOT_RUN, never PASS and never FAIL -- and that applies to
    ROWS. The first negative arm judged `chain_reached_receiver` with the POSITIVE arm's
    expectation, so a run that correctly refused to deliver was scored FAIL on a claim it was
    never meant to make."""
    text = (ROOT / 'experiments' / 'probe_h3_w5.py').read_text(encoding='utf-8')
    for name in ('chain_reached_receiver', 'tray_on_source_band_at_handoff',
                 'each_leg_destination_matched_by_contacts'):
        assert "verdict='NOT_RUN'" in text, 'no row can be NOT_RUN'
        assert name in text
    # the NOT_RUN rows must be excluded from the PASS count
    assert 'not_run' in text and 'judged H3 rows' in text


def test_the_ledger_is_actually_driven():
    """★ `TransferLedger` WAS CONSTRUCTED AND NEVER USED in the first H3 probe, so the custody
    transfer -- a named part of the H3 definition -- was recorded nowhere. That is the
    "a required field that was never written" defect wearing a different costume.

    Asserted by requiring the calls the ledger has to receive, not by checking that an object
    exists.
    """
    text = (ROOT / 'experiments' / 'probe_h3_w5.py').read_text(encoding='utf-8')
    assert 'ledger.request(' in text, 'the transfer transaction is never launched'
    assert 'ledger.advance(' in text, 'the transaction stages are never advanced'
    assert "report['chain_transactions']" in text, 'the ledger record is never reported'
    assert 'chain_transactions' in text.split('def judge_h3')[1], \
        'no row reads the ledger record'


def test_every_declared_h3_threshold_is_read_by_a_row():
    """A threshold nobody reads is a number frozen for show, and the freeze contract would happily
    publish it. Each key of `H3_THRESHOLDS` has to appear as `T['...']` somewhere."""
    import probe_h3_w5 as judge

    text = (ROOT / 'experiments' / 'probe_h3_w5.py').read_text(encoding='utf-8')
    unread = [k for k in judge.H3_THRESHOLDS if "T['%s']" % k not in text]
    assert unread == [], 'declared but never read: %r' % unread


def test_the_plant_refuses_to_reset_when_it_is_a_guest():
    """The "no runtime qpos writes" claim is meant to be STRUCTURAL, not promised. A guest plant
    must refuse the one write site it has."""
    import w4_plant as wp

    plant = wp.LogisticsPlant()
    assert plant._owns_world is True
    plant._owns_world = False                      # simulate the guest case without a second load
    try:
        plant.reset()
    except RuntimeError as refusal:
        assert 'qpos write' in str(refusal)
    else:
        raise AssertionError('a guest plant performed a reset, which would be a runtime qpos write')


def test_the_h2_stage_driver_defaults_are_unchanged():
    """H3 added `place_x` and the window/approach overrides to H2's stage driver. H2's own report
    must keep coming from the code it came from, so the defaults have to be the module's values."""
    import inspect

    import probe_h2_w5 as H2

    signature = inspect.signature(H2.stage_driver)
    for name in ('phase_windows', 'approach_start', 'place_x'):
        assert signature.parameters[name].default is None, name
    assert signature.parameters['handle_y'].default == H2.HANDLE_Y
    assert signature.parameters['hand_roll'].default == H2.HAND_ROLL_BIAS
    text = (ROOT / 'experiments' / 'probe_h2_w5.py').read_text(encoding='utf-8')
    assert 'target_x = PLACE_X if place_x is None else float(place_x)' in text, \
        'the place target no longer defaults to PLACE_X'


def test_the_candidate_b_world_was_not_touched():
    """H3 reads candidate B and writes a separate file. The source's hash is recorded in the
    manifest; this checks the file on disk still matches it."""
    import json

    manifest = json.loads((ROOT / 'assets' / 'world_w5_h085_loop.manifest.json')
                          .read_text(encoding='utf-8'))
    source = ROOT / 'assets' / manifest['source']
    digest = hashlib.sha256(source.read_bytes()).hexdigest()
    assert digest == manifest['source_sha256'], \
        'candidate B moved: %s vs manifest %s' % (digest[:16], manifest['source_sha256'][:16])
    assert digest == '702e22a6bbfb08984171f3a2e0829e1547761b029fc7902e0ff6b5a740818a12'


# =============================================================================================
# THE CUSTODY GUARD -- "a transfer must not be authorised when the supply failed"
# =============================================================================================
#
# These tests exist because the H3 negative arm recorded a real defect, found on 2026-09-29:
# leg 1 ended RELEASED / AT_DESTINATION while the tray never left the source band. The ledger
# moved custody from a declared stage sequence rather than from the receiver's confirmation, and
# nothing in the machine refused it. These tests walk the LEDGER DIRECTLY, so they fail on the
# behaviour rather than on a stored fixture.

import json  # noqa: E402

import pytest  # noqa: E402

from workcell.resources import ResourceTable  # noqa: E402
from workcell.schema import SchemaRefused  # noqa: E402
from workcell.transfer import TransferLedger  # noqa: E402

SOURCE_RES, RECEIVER_RES = 'c_source_band', 'c_receiver_band'
POSITIVE_REPORT = ROOT / 'reports' / 'p4-h3-w5-01' / 'report.json'
NEGATIVE_REPORT = ROOT / 'reports' / 'p4-h3-w5-neg-01' / 'report.json'


def _open_transfer(transfer_id='xfer_source_to_deck'):
    """A transfer opened on a source that CLAIMS to hold the tray.

    The claim is what the contract checks at launch; whether the tray is really there is what
    the receiver's confirmation is for. Keeping the two separate is the point of the test.
    """
    resources = ResourceTable(epoch=13, resources=[SOURCE_RES, RECEIVER_RES])
    for rid in (SOURCE_RES, RECEIVER_RES):
        resources.mark_cleared(rid, evidence=['runs/init/%s.json' % rid], now_s=0.0)
    ledger = TransferLedger(epoch=13)
    preconditions = {
        'source_capacity_free': True, 'receiver_capacity_free': True,
        'height_within_tolerance': True, 'lateral_within_tolerance': True,
        'yaw_within_tolerance': True, 'clearance_ok': True, 'amr_at_rest': True,
        'source_has_tray': True, 'receiver_empty': True, 'arms_retracted': True,
        'stop_chain_healthy': True, 'evidence_age_s': 0.05,
        'max_evidence_age_s': 0.5, 'ttl_s': 60.0,
    }
    ledger.request({'transfer_id': transfer_id, 'order_id': 'h3-test', 'epoch': 13,
                    'source': SOURCE_RES, 'receiver': RECEIVER_RES, 'tray_id': 'c_payload',
                    'preconditions': preconditions},
                   resources=resources, now_s=0.0)
    return ledger


def test_committed_is_refused_when_the_receiver_never_confirmed():
    """★ THE CENTRAL TEST: COMMITTED/AT_DESTINATION is rejected while the supply failed.

    Before the guard this exact walk reached RELEASED / AT_DESTINATION -- the ledger reporting a
    delivery for a tray that never moved. That is the defect the H3 negative arm shipped with.
    """
    ledger = _open_transfer()
    record = ledger.advance('xfer_source_to_deck')          # DOCK_VERIFIED
    record = ledger.advance('xfer_source_to_deck')          # BOTH_READY
    record = ledger.advance('xfer_source_to_deck')          # TRANSFERRING
    assert record.custody == 'TRANSFERRING'
    with pytest.raises(SchemaRefused) as refusal:
        ledger.advance('xfer_source_to_deck')               # -> RECEIVER_CONFIRMED, no evidence
    assert refusal.value.reason == 'REFUSED_MISSING_FIELD'
    assert record.stage == 'TRANSFERRING', 'the ledger advanced past a missing confirmation'
    assert record.custody != 'AT_DESTINATION'
    assert 'COMMITTED' not in record.history


def test_committed_is_refused_when_the_confirmation_is_missing():
    """Exercises the NEW rule, not the pre-existing evidence check.

    The transaction reaches RECEIVER_CONFIRMED *with* evidence, the confirmation then goes
    missing, and the commit must still be refused by the ledger's own guard.
    """
    ledger = _open_transfer()
    record = ledger.advance('xfer_source_to_deck')
    record = ledger.advance('xfer_source_to_deck')
    record = ledger.advance('xfer_source_to_deck')
    record = ledger.advance('xfer_source_to_deck', evidence=['contacts:deck'])
    assert record.stage == 'RECEIVER_CONFIRMED'
    assert record.evidence.get('receiver_confirmed') == ['contacts:deck']
    record.evidence.pop('receiver_confirmed')
    with pytest.raises(SchemaRefused) as refusal:
        ledger.advance('xfer_source_to_deck')
    assert refusal.value.reason == 'REFUSED_MISSING_FIELD'
    assert refusal.value.field == 'receiver_confirmed'
    assert record.stage == 'RECEIVER_CONFIRMED'
    assert record.custody != 'AT_DESTINATION'


def test_custody_never_reaches_at_destination_without_a_confirmation():
    """The invariant, stated once: no blind walk reaches AT_DESTINATION."""
    for attempts in range(1, 9):
        tid = 'blind-%d' % attempts
        ledger = _open_transfer(transfer_id=tid)
        record = None
        for _ in range(attempts):
            try:
                record = ledger.advance(tid)
            except SchemaRefused:
                break
        assert record is not None, 'a transfer with no confirmation advanced nothing at all'
        assert record.custody != 'AT_DESTINATION', (
            'a blind walk of %d advances reached AT_DESTINATION' % attempts)
        assert 'COMMITTED' not in record.history


def test_the_healthy_path_still_commits():
    """The guard must not break the case it exists to permit."""
    ledger = _open_transfer()
    record = ledger.advance('xfer_source_to_deck')
    record = ledger.advance('xfer_source_to_deck')
    record = ledger.advance('xfer_source_to_deck')
    record = ledger.advance('xfer_source_to_deck', evidence=['contacts:deck'])
    record = ledger.advance('xfer_source_to_deck', evidence=['contacts:deck'])
    record = ledger.advance('xfer_source_to_deck')
    assert record.stage == 'RELEASED'
    assert record.custody == 'AT_DESTINATION'
    assert record.evidence['committed_on'] == ['contacts:deck']


def test_the_stored_reports_do_not_claim_unsupported_custody():
    """On both stored reports, no transaction may claim custody the physics cannot support.

    This is the regression lock on the shipped evidence: it reads what the runs actually wrote.
    """
    for report_path in (POSITIVE_REPORT, NEGATIVE_REPORT):
        report = json.loads(report_path.read_text(encoding='utf-8'))
        tx = report['chain_transactions']
        for leg, body in tx.items():
            if body['final_custody'] == 'AT_DESTINATION':
                support = body['support_after'] or []
                assert body['destination_zone'] in support, (
                    '%s: leg %s claims AT_DESTINATION but the tray is on %s'
                    % (report_path.name, leg, support))
        row = report['h3']['ledger_agrees_with_physics']
        assert row['verdict'] == 'PASS', '%s: %s' % (report_path.name, row['detail'])


def test_the_cross_check_row_is_scored_in_both_arms():
    """The cross-check must not be arm-blind: 'the ledger told the truth' is claimed by both."""
    import probe_h3_w5 as judge

    assert judge.ROW_ARMS['ledger_agrees_with_physics'] == ('positive', 'negative')
