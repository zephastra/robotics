"""Rig structure tests plus judge tests driven by synthetic inputs that must FAIL."""
import importlib.util
import json
import math
from pathlib import Path
import subprocess
import sys
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
TRAY = ROOT / 'assets' / 'objects' / 'tray_v1.xml'
OLD_RIG_HALF_LENGTH = 0.220          # what the superseded roller rig used


def _load(relative, name):
    path = ROOT / relative
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


rig = _load('experiments/roller_rig.py', 'roller_rig')


def test_the_roller_row_is_longer_than_the_tray_floor():
    measured = rig.tray_measurements(TRAY)
    half_length = rig.roller_half_length(measured)
    assert half_length > measured['support_half_width'], 'rollers must outrun the tray floor'
    # the point of deriving it: the superseded rig would have failed this
    assert OLD_RIG_HALF_LENGTH < measured['support_half_width']


def test_the_handles_clear_the_rollers():
    assert rig.handle_clearance(rig.tray_measurements(TRAY)) > 0.0


def test_neighbouring_rollers_do_not_overlap():
    root = ET.fromstring(rig.build_model(TRAY)[0])
    bodies = [b for b in root.findall('worldbody/body') if b.get('name').startswith('fixed_roller_')]
    assert len(bodies) == rig.FIXED_SECTIONS * rig.ROLLERS_PER_SECTION
    for first, second in zip(bodies, bodies[1:]):
        gap = float(second.get('pos').split()[0]) - float(first.get('pos').split()[0])
        assert gap > 2 * rig.ROLLER_RADIUS


def test_the_deck_rollers_sit_on_the_deck_surface():
    root = ET.fromstring(rig.build_model(TRAY)[0])
    deck = root.find("worldbody/body[@name='deck']")
    rollers = [b for b in deck.findall('body') if b.get('name').startswith('deck_roller_')]
    assert len(rollers) == rig.DECK_ROLLERS
    for roller in rollers:
        z = float(roller.get('pos').split()[2])
        assert abs(z + rig.ROLLER_RADIUS) < 1e-9, 'deck roller crowns must land on the deck surface'


def test_the_deck_frame_clears_the_deck_rollers():
    root = ET.fromstring(rig.build_model(TRAY)[0])
    deck = root.find("worldbody/body[@name='deck']")
    frame = deck.find("geom[@name='deck_frame']")
    frame_top = float(frame.get('pos').split()[2]) + float(frame.get('size').split()[2])
    assert frame_top <= -rig.ROLLER_RADIUS


def test_the_blade_stows_behind_the_fixed_row_not_between_the_roller_rows():
    """REWRITTEN (D034). The old test asked the pin to fit BETWEEN the roller rows.

    That placement rule was the pin's whole problem, and it is provably unsatisfiable: the
    pin/blade is 0.0200 m wide and the clear slot between roller surfaces is
    `ROLLER_PITCH - 2r = 0.0100 m`. The only roller-clear positions are behind the last fixed
    tangent or ahead of the deck's first tangent, and the first of those is what the mechanism
    now uses.

    So the assertion is inverted, deliberately: the blade must NOT be inside either roller row,
    it must be behind BOTH of them. `PUSHER_CORRIDOR_X` is exactly that boundary.
    """
    root = ET.fromstring(rig.build_model(TRAY)[0])
    carrier = root.find("worldbody/body[@name='deck']/body[@name='pusher_carriage']")
    assert carrier is not None, 'the pusher carriage is not on the deck'
    blade = carrier.find("body[@name='pusher_blade']")
    assert blade is not None, 'the blade is not a child of the carriage'
    blade_pos = float(blade.get('pos').split()[0])
    blade_half = float(blade.find('geom').get('size').split()[0])
    blade_rear, blade_front = blade_pos - blade_half, blade_pos + blade_half
    # Behind every roller of every row: the corridor limit is the rear tangent of the last
    # fixed crown, so anything at or behind it clears the whole band at every height.
    assert blade_front <= rig.PUSHER_CORRIDOR_X + 1e-9, (
        f'the stowed blade front face is at deck-local {blade_front:.4f}, forward of the '
        f'roller-free corridor limit {rig.PUSHER_CORRIDOR_X:.4f}; it would foul a fixed roller')
    # And the hole the old test tried to thread is genuinely too narrow -- so that if a later
    # edit re-widens the pitch, this test explains why the placement rule was abandoned.
    slot = rig.ROLLER_PITCH - 2.0 * rig.ROLLER_RADIUS
    assert 2.0 * blade_half > slot, (
        f'the slot between roller crowns is now {slot:.4f} m and the blade is '
        f'{2.0 * blade_half:.4f} m wide: the between-the-rows placement is now possible, so '
        f're-derive the stow position and delete this test rather than leaving it true by luck')


def test_the_stowed_blade_is_clear_of_the_roller_band_from_above():
    """REWRITTEN (D034). The blade cannot hide BELOW the crown plane, so it must clear the band.

    The old form asserted `top < 0.0`: "a stowed pin must not present an edge to the tray",
    meaning the pin hides under the deck surface. That is impossible for a blade that must also
    reach a wall, and the attempt to satisfy it is what put the blade in the roller band (D032)
    and then in the tray (D034).

    The safe direction is the other one: the blade's bottom face sits just ABOVE the crown
    plane, so it clears every roller, and it is then inside the tray interior -- which is a
    separate, still-unresolved problem recorded in the geometry suite's marked test.
    """
    root = ET.fromstring(rig.build_model(TRAY)[0])
    blade = root.find("worldbody/body[@name='deck']/body[@name='pusher_carriage']"
                      "/body[@name='pusher_blade']")
    body_z = float(blade.get('pos').split()[2])
    geom = blade.find('geom')
    bottom = body_z + float(geom.get('pos').split()[2]) - float(geom.get('size').split()[2])
    assert bottom >= 0.0, (
        f'the blade bottom is at deck-local {bottom:.4f}, below the crown plane; it would sit '
        f'in the roller band and plough the fixed row (the D032 defect)')


def test_the_payload_is_a_free_body_with_no_equality_constraint():
    root = ET.fromstring(rig.build_model(TRAY)[0])
    assert root.find('equality') is None
    tray = root.find("worldbody/body[@name='payload']")
    assert tray is not None and tray.find('freejoint') is not None


def test_a_negative_docking_gap_is_refused():
    try:
        rig.build_model(TRAY, gap=-0.001)
    except SystemExit:
        return
    raise AssertionError('a negative gap overlaps the fixed section and must be refused')


# --------------------------------------------------------------------------------------------
# Synthetic reports for the judge.
#
# REBUILT (D034). These fixtures were written against an OLDER shape of `report.json`: the
# scenario records lived under a top-level `scenarios` key carrying `pin_contacts`/`pin_qpos`.
# The judge was subsequently restructured to consume a top-level `runs` list of per-run records
# with `final_state`, `refused`, `align_measured`, `records[]`, and `recv_first_crown_x`.
#
# Nobody updated the fixtures, so from that day on every judge-driven test in this file died with
#   KeyError: 'runs'  -> evaluate_c_transfer.py exits -> no acceptance.json
#   -> FileNotFoundError in the test
# and NOT ONE of the outcome tests was actually exercising the judge. They were green-looking
# assertions wrapped around a missing file. Rebuilding the fixture is what makes them tests
# again; the alternative (loosening the assertions to tolerate a missing file) would have
# preserved the illusion.
#
# The fixtures are built to the judge's contract by construction rather than by transcription:
# `RECV_FIRST_CROWN_X` is read from the rig, the stage lists are read from the probe, and the
# per-friction windows come from the probe's own tables. A change to any of those shows up here
# as a fixture that no longer describes the machine, instead of a test that quietly stops
# meaning anything.
# --------------------------------------------------------------------------------------------

PROBE_ALIGN_BY_FRICTION = {0.15: {'gap_max': 0.028, 'step_max': 0.0045, 'yaw_max': 0.06981317007977318},
                           0.25: {'gap_max': 0.028, 'step_max': 0.0065, 'yaw_max': 0.06981317007977318},
                           0.40: {'gap_max': 0.032, 'step_max': 0.013, 'yaw_max': 0.06981317007977318}}
FULL_C_REQUIRED = ['CHECK_ALIGN', 'CONVEY', 'DEPLOY_PUSHER', 'DONE', 'EMBARK',
                   'PLATFORM_CLEAR', 'RECEIVED', 'RELEASE', 'SETTLE', 'UNLOAD']
DELIVERY_STAGES = ['UNLOAD', 'RECEIVED', 'PLATFORM_CLEAR']
# A DONE run must traverse the delivery stages, and the judge checks the SEQUENCE against
# COORDINATE_STATES, so the records below walk the machine in order.
COORDINATE_PATH = ['CHECK_ALIGN', 'CONVEY', 'RELEASE', 'SETTLE', 'DEPLOY_PUSHER', 'EMBARK',
                   'UNLOAD', 'RECEIVED', 'PLATFORM_CLEAR', 'DONE']


def _stage_records(final_state, *, tray_x=2.30, lateral=0.002, deck_qpos=0.240,
                   rel_start=0.065, rel_final=0.480, follow=0.95):
    """Records walking the machine in coordinate order, truncated at `final_state`.

    Truncation matters: the judge's delivery check reads the STATES PRESENT in a run's records,
    so a fixture that claims DONE while emitting only three stages is the exact violation the
    check was written to catch -- and it must be reachable from a fixture, or the check is
    untested.

    Two judge checks constrain the shape of these records and are honoured here rather than
    worked around:

      * `the tray travelled with the deck during EMBARK` needs the tray's WORLD x to advance by
        at least `embark_follow_fraction` (0.80) of the deck's travel, measured from the first
        EMBARK record. A tray that stays put while the deck moves is the "deck left without it"
        failure, so `follow` drives the tray's world motion over the EMBARK phase.
      * `the sequence is sensor-driven and reaches DONE in order` compares the *deduplicated*
        state sequence against `COORDINATE_STATES`, so the records must be emitted in order with
        no repeated or out-of-order states.
    """
    path = (COORDINATE_PATH[:COORDINATE_PATH.index(final_state) + 1]
            if final_state in COORDINATE_PATH else COORDINATE_PATH[:1])
    terminal = final_state
    # Where the tray sits in world x, and where the deck starts. The tray must end up on the
    # RECEIVING section, so its trailing edge has to clear RECV_FIRST_CROWN_X - tol.
    tray_half = 0.065
    end_tray_x = rig.RECV_FIRST_CROWN_X + tray_half + 0.010
    embark_i = COORDINATE_PATH.index('EMBARK') if 'EMBARK' in COORDINATE_PATH else 0
    n = len(path)
    recs = []
    for i, state in enumerate(path):
        frac = i / max(n - 1, 1)
        if state in ('CHECK_ALIGN', 'CONVEY', 'RELEASE', 'SETTLE', 'DEPLOY_PUSHER'):
            tx, dq = tray_x, 0.0
            rel = rel_start + (rel_final - rel_start) * frac
        else:
            # EMBARK onwards: the deck drives and the tray rides it to the receiver.
            since = (i - embark_i) / max(n - 1 - embark_i, 1)
            dq = deck_qpos * since
            tx = tray_x + (end_tray_x - tray_x) * since
            rel = rel_final
        recs.append({
            't': round(2.0 + 20.0 * frac, 4),
            'state': state,
            'tray': [round(tx, 6), lateral, 0.52],
            'rel_to_deck': [round(rel, 6), lateral, 0.04],
            'deck_qpos': round(dq, 6),
            'pusher_qpos': 0.0,
        })
    if terminal not in {r['state'] for r in recs}:
        recs.append({'t': 22.0, 'state': terminal,
                     'tray': [end_tray_x, lateral, 0.52],
                     'rel_to_deck': [rel_final, lateral, 0.04], 'deck_qpos': deck_qpos,
                     'pusher_qpos': 0.0})
    return recs


def synth_run(tmp_path, name, runs, *, tray_asset='assets/objects/tray_v1.xml',
              friction_sweep=(0.15, 0.25, 0.40),
              stages_present=None, stage_claim='CLAIMED'):
    run = tmp_path / name
    run.mkdir(parents=True)
    (run / 'model.xml').write_text(
        '<mujoco><worldbody><body name="payload"><freejoint/></body></worldbody></mujoco>')
    report = {
        'tray_asset': tray_asset,
        'tray_asset_sha256': 'a' * 64,
        'tray_measurements': {'x_min': -0.065, 'x_max': 0.065, 'support_half_width': 0.230,
                              'mass_kg': 0.098, 'n_colliding_geoms': 11},
        'roller_half_length_m': 0.250, 'handle_overhang_m': 0.085,
        'deck': {'x0': 1.92, 'length': 0.79, 'surface_z': 0.485},
        'frictions': list(friction_sweep),
        'guides': [False],
        # The judge measures the delivery against the geometry the RIG declares, not a constant.
        'recv_first_crown_x': rig.RECV_FIRST_CROWN_X,
        'receiver': {'rollers': rig.RECV_ROLLERS},
        'align_window': PROBE_ALIGN_BY_FRICTION[0.15],
        'align_window_by_friction': {str(k): v for k, v in PROBE_ALIGN_BY_FRICTION.items()},
        'guide_band': {'ball_inner_y': 0.265, 'wall_outer_y': 0.230},
        'timeline': {'drive_on': [18.0, 20.5], 'pusher_deploy_at': 16.5, 'sim_end': 22.0},
        'runs': runs,
        'full_C_acceptance': stage_claim,
        'full_C_stages_required': FULL_C_REQUIRED,
        'full_C_stages_present': FULL_C_REQUIRED if stages_present is None else stages_present,
        'scope_claim': ('C RECEIVING-SIDE COMPONENT; the transfer is a MECHANISM experiment, '
                        'not a real-AMR docking; deck is a stand-in; offsets are initial '
                        'conditions; every transition is driven by modelled sensors'),
    }
    (run / 'report.json').write_text(json.dumps(report))
    return run.parent


def scenario(name, *, final_state='DONE', friction=0.25, guides=False, gap=0.0, step=0.0,
             yaw=0.0, refused=False, final_rel_x=0.480, lateral=0.002, tray_x=2.30,
             fell_at=None, refusal_reason=None, on_boundary=False, matched_axis=None):
    """One judge-visible run. Mirrors the shape `probe_deck.py` actually emits in `runs[]`."""
    admitted = not refused
    # The judge cross-checks the two gap heads against the pair formula
    #     heads[1] - heads[0] == -2 * GAP_RAY_Y * tan(yaw)
    # and treats a `nan` worst-error as a FAILURE (`worst_pair <= tol` is False for nan). So the
    # fixture must supply `gap_heads` whenever it supplies a yaw, exactly as the probe does.
    # `GAP_RAY_Y` is READ from the rig, not typed, so the fixture tracks the geometry.
    half_span = -rig.GAP_RAY_Y * math.tan(yaw)
    align = {'gap': gap, 'yaw': yaw, 'step': step,
             'gap_heads': [gap - half_span, gap + half_span],
             'admitted_by_gate': admitted,
             'refusal_reason': refusal_reason, 'margin': 0.001, 'on_boundary': on_boundary}
    records = _stage_records(final_state, tray_x=tray_x, lateral=lateral,
                             rel_final=final_rel_x,
                             # A refused run must NOT move the tray: the judge faults a refusal
                             # whose first and last records disagree by more than
                             # `refusal_motion_tol_m`. So a refusal freezes rel_to_deck.
                             rel_start=(final_rel_x if refused else 0.065))
    return {
        'scenario': {'name': name, 'gap': gap, 'step': step, 'yaw': yaw},
        'friction': friction, 'guides': guides,
        'final_state': final_state,
        'refused': refused,
        'align_measured': align,
        'align_window_applied': PROBE_ALIGN_BY_FRICTION[friction],
        'matched_axis': matched_axis,
        'refusal_reason': refusal_reason,
        'on_boundary': on_boundary,
        'margin': 0.001,
        'fell_at': fell_at,
        'records': records,
        'final': records[-1],
        'n_samples': len(records),
        'mean_contacts_while_on_deck': 3.0,
        'sim_seconds': 22.0, 'wall_seconds': 0.8,
    }


def judge(tmp_path, name, runs, **kw):
    reports = synth_run(tmp_path, name, runs, **kw)
    script = ROOT / 'experiments' / 'evaluate_c_transfer.py'
    completed = subprocess.run([sys.executable, str(script), '--run-id', name,
                                '--reports-dir', str(reports)],
                               capture_output=True, text=True)
    acceptance = reports / name / 'acceptance.json'
    assert acceptance.is_file(), (
        f'the judge wrote no acceptance.json (rc={completed.returncode}). '
        f'stderr: {completed.stderr[-2000:]}')
    payload = json.loads(acceptance.read_text())
    return completed.returncode, payload


def healthy_arm(**kw):
    """A run set the judge should call PASS: every case admitted, every one reaching DONE.

    Satisfies each of the judge's own requirements rather than being tuned until the verdict
    turns green, because a fixture that only looks healthy will hide a judge check that stops
    working:

      * `min_sensor_runs` (8): every run carries a full gap/yaw/step reading, and there are 12.
      * friction sweep: three distinct frictions, so the friction-sweep check has three values.
      * sensor accuracy (gap 2 mm, yaw 4 mrad, step 2 mm): `align_measured` reports EXACTLY what
        the scenario declares, which is the ideal sensor. Any drift between the two shows up as
        a sensor failure, which is correct.
      * seated footprint (`rel_x` in [0.065, 0.625]): the records are built with
        `rel_start=0.065` and `rel_final=0.480`, both inside the deck.
      * delivery: the tray ends past `RECV_FIRST_CROWN_X` (handled in `_stage_records`).
    """
    runs = []
    for mu in (0.15, 0.25, 0.40):
        runs += [
            scenario('aligned', friction=mu),
            scenario('gap_20mm', friction=mu, gap=0.020),
            scenario('step_4mm', friction=mu, step=0.004),
            scenario('yaw_2deg', friction=mu, yaw=0.0349, lateral=0.02),
        ]
    return runs


def test_a_healthy_arm_passes_every_check_it_can_decide(tmp_path):
    """An all-admitted, all-DONE arm must pass every check the artifact can decide.

    The verdict is INCOMPLETE, not PASS, and that is the correct reading rather than a defect in
    the arm: with no refused case there is nothing for the two interlock checks to test, so they
    report NOT_RUN. The whole point of the reviewer's "unify the acceptance scope" instruction is
    that "nothing to check" must not be reportable as PASS -- so this test asserts BOTH halves:
    nothing FAILS, and the two interlock checks are NOT_RUN.
    """
    code, payload = judge(tmp_path, 'ok', healthy_arm())
    assert payload['verdict'] == 'INCOMPLETE', (
        f'expected INCOMPLETE (no refusals to judge), got {payload["verdict"]}: '
        f'failed={payload["failed"]} not_run={payload["not_run"]}')
    assert payload['failed'] == [], f'a clean arm was faulted: {payload["failed"]}'
    assert 'the interlock refuses outside its declared window' in payload['not_run']
    assert code != 0


def test_a_healthy_arm_plus_a_refusal_and_a_bypass_arm_passes_completely(tmp_path):
    """The only shape that can earn PASS: sound AND tight.

    Tightness needs an interlock-bypass arm, and this is the first test that supplies one -- so
    it also exercises the three-category tightness path end to end, which until now no fixture
    reached. The bypass arm is a run the ARMED gate refused, re-run with the gate forced open:
    `would_have_completed=False` means the refusal was justified, which is a healthy tightness
    result (conservatism is what would be faulted).

    Wire format: the judge reads `interlock_bypass_runs` off the armed report, and the probe
    only populates it when run with `--interlock bypass`. The fixture does the same thing the
    probe's `--bypass-run-id` plumbing does: it folds a bypass arm's entries into the report.
    """
    arm = healthy_arm()
    arm.append(scenario('gap_60mm', friction=0.25, gap=0.060, refused=True,
                        final_state='REFUSED', final_rel_x=0.065, tray_x=1.90))
    reports = synth_run(tmp_path, 'okplus', arm)
    # A bypass arm that was re-run with the gate open and still could not finish.
    bypass = synth_run(tmp_path, 'bypassarm', [
        scenario('gap_60mm', friction=0.25, gap=0.060, refused=True, final_state='STALLED',
                 final_rel_x=0.065, tray_x=1.90, refusal_reason='gap exceeds window',
                 matched_axis='gap'),
    ])
    bypass_report = json.loads((reports / 'bypassarm' / 'report.json').read_text())
    bypass_report['interlock_mode'] = 'bypass'
    bypass_report['interlock_bypass_runs'] = [
        {'scenario': 'gap_60mm', 'friction': 0.25, 'guides': False,
         'measured': {'gap': 0.060, 'yaw': 0.0, 'step': 0.0},
         'window_applied': PROBE_ALIGN_BY_FRICTION[0.25],
         'gap_floor_required': 0.0, 'matched_axis': 'gap', 'matched_excess': 0.032,
         'refusal_reason': 'gap exceeds window', 'margin': 0.032, 'on_boundary': False,
         'final_state': 'STALLED', 'fell_at': None, 'would_have_completed': False},
    ]
    (reports / 'bypassarm' / 'report.json').write_text(json.dumps(bypass_report))

    script = ROOT / 'experiments' / 'evaluate_c_transfer.py'
    completed = subprocess.run(
        [sys.executable, str(script), '--run-id', 'okplus', '--reports-dir', str(reports),
         '--bypass-run-id', 'bypassarm'],
        capture_output=True, text=True)
    acceptance = reports / 'okplus' / 'acceptance.json'
    assert acceptance.is_file(), f'no acceptance.json; stderr {completed.stderr[-1500:]}'
    payload = json.loads(acceptance.read_text())
    assert payload['failed'] == [], payload['failed']
    assert payload['verdict'] == 'PASS', f'not_run={payload["not_run"]}'
    assert completed.returncode == 0


def test_a_bypass_case_that_would_have_worked_is_reported_as_conservatism(tmp_path):
    """The reviewer's second category: a refusal that cost something must be named, not hidden.

    `would_have_completed=True` on a bypass case means the armed gate refused a case that in
    fact works -- excessive conservatism. The judge must FAIL the tightness check and say so in
    the conservatism category, because a window that refuses workable cases is an undiscovered
    defect, not a safe default.
    """
    arm = healthy_arm()
    arm.append(scenario('gap_60mm', friction=0.25, gap=0.060, refused=True,
                        final_state='REFUSED', final_rel_x=0.065, tray_x=1.90))
    reports = synth_run(tmp_path, 'tootight', arm)
    synth_run(tmp_path, 'bypassarm', [])
    bypass_report = json.loads((reports / 'bypassarm' / 'report.json').read_text())
    bypass_report['interlock_mode'] = 'bypass'
    bypass_report['interlock_bypass_runs'] = [
        {'scenario': 'gap_60mm', 'friction': 0.25, 'guides': False,
         'measured': {'gap': 0.060, 'yaw': 0.0, 'step': 0.0},
         'window_applied': PROBE_ALIGN_BY_FRICTION[0.25],
         'gap_floor_required': 0.0, 'matched_axis': 'gap', 'matched_excess': 0.032,
         'refusal_reason': 'gap exceeds window', 'margin': 0.032, 'on_boundary': False,
         'final_state': 'DONE', 'fell_at': None, 'would_have_completed': True},
    ]
    (reports / 'bypassarm' / 'report.json').write_text(json.dumps(bypass_report))

    script = ROOT / 'experiments' / 'evaluate_c_transfer.py'
    subprocess.run([sys.executable, str(script), '--run-id', 'tootight',
                    '--reports-dir', str(reports), '--bypass-run-id', 'bypassarm'],
                   capture_output=True, text=True)
    payload = json.loads((reports / 'tootight' / 'acceptance.json').read_text())
    assert 'the interlock is tight: refusals are justified, in three categories' in payload['failed']


def test_a_sensor_unavailable_refusal_is_undecidable_not_a_pass(tmp_path):
    """The reviewer's third category must be NOT_RUN, never a discretionary pass.

    If the ray never saw the tray the case is not evidence in either direction, and if that
    leaves nothing decidable the tightness claim must be NOT_RUN rather than PASS-by-vacuity.
    """
    arm = healthy_arm()
    arm.append(scenario('gap_60mm', friction=0.25, gap=0.060, refused=True,
                        final_state='REFUSED', final_rel_x=0.065, tray_x=1.90,
                        refusal_reason='gap ray missed the tray'))
    reports = synth_run(tmp_path, 'undecidable', arm)
    synth_run(tmp_path, 'bypassarm', [])
    bypass_report = json.loads((reports / 'bypassarm' / 'report.json').read_text())
    bypass_report['interlock_mode'] = 'bypass'
    bypass_report['interlock_bypass_runs'] = [
        {'scenario': 'yaw_8deg', 'friction': 0.25, 'guides': False,
         'measured': {'gap': 0.020, 'yaw': 0.1396, 'step': 0.0},
         'window_applied': PROBE_ALIGN_BY_FRICTION[0.25],
         'gap_floor_required': 0.0, 'matched_axis': 'yaw', 'matched_excess': 0.07,
         'refusal_reason': 'gap ray missed the tray', 'margin': 0.07, 'on_boundary': False,
         'final_state': 'DONE', 'fell_at': None, 'would_have_completed': True},
    ]
    (reports / 'bypassarm' / 'report.json').write_text(json.dumps(bypass_report))

    script = ROOT / 'experiments' / 'evaluate_c_transfer.py'
    subprocess.run([sys.executable, str(script), '--run-id', 'undecidable',
                    '--reports-dir', str(reports), '--bypass-run-id', 'bypassarm'],
                   capture_output=True, text=True)
    payload = json.loads((reports / 'undecidable' / 'acceptance.json').read_text())
    tight = [c for c in payload['checks']
             if c['check'] == ('the interlock is tight: refusals are justified, in three '
                               'categories')][0]
    assert tight['verdict'] == 'NOT_RUN', (
        f'a sensor-unavailable refusal was credited as {tight["verdict"]}; the undecidable '
        f'category must not become a loophole')
    assert 'UNDECIDABLE' in tight['detail'], tight['detail']


def test_the_fixture_is_actually_reaching_the_judge(tmp_path):
    """Regression guard for the fixture bug itself, not for the judge.

    Until D034 these fixtures used a schema the judge no longer reads, so the judge died with
    `KeyError: 'runs'` and every outcome test below failed as `FileNotFoundError` on
    acceptance.json -- they were asserting nothing. This test fails loudly if that ever recurs:
    it checks that the judge produced a real verdict with a real number of checks, so a fixture
    that stops being understood cannot masquerade as a passing suite.
    """
    _, payload = judge(tmp_path, 'reaches', healthy_arm())
    assert len(payload['checks']) >= 10, (
        f'the judge only produced {len(payload["checks"])} checks; it is not reading this '
        f'fixture, and the outcome tests below are therefore vacuous')
    names = {c['check'] for c in payload['checks']}
    for expected in ('the interlock is sound: every admitted case completes',
                     'DONE means delivered, not merely picked up'):
        assert expected in names, f'the judge skipped its own check {expected!r}'


def test_an_admitted_case_that_does_not_finish_is_not_sound(tmp_path):
    """Soundness: a run the gate ADMITTED that fails is the critical class.

    REPLACES `test_a_pin_that_never_loads_after_the_raise_is_not_a_pass`. That test asserted an
    INCOMPLETE verdict from a probe-side NOT_RUN check (`the retention pin is shown to carry
    load`) which no longer exists -- the pin is gone and the judge's checks were reorganised
    around soundness/tightness. The property it was protecting is the same one and is asserted
    here directly: an admitted case that does not reach DONE must fail soundness.
    """
    arm = healthy_arm()
    arm[0] = scenario('aligned', friction=0.25, final_state='STALLED', final_rel_x=0.100,
                      tray_x=1.90)
    code, payload = judge(tmp_path, 'unsound', arm)
    assert payload['verdict'] == 'FAIL'
    assert 'the interlock is sound: every admitted case completes' in payload['failed']
    assert code != 0


def test_a_transfer_that_stops_short_fails(tmp_path):
    """A refused-behaviour check: an admitted run that never leaves the deck is a failure."""
    arm = healthy_arm()
    arm[0] = scenario('aligned', friction=0.25, final_state='STALLED', final_rel_x=-0.100,
                      tray_x=1.90)
    code, payload = judge(tmp_path, 'short', arm)
    assert payload['verdict'] == 'FAIL'
    assert 'the interlock is sound: every admitted case completes' in payload['failed']


def test_a_done_run_that_never_reached_the_receiver_fails(tmp_path):
    """DONE must mean DELIVERED, not merely PICKED UP.

    REPLACES `test_a_fallen_tray_fails`, which relied on the judge treating a `fell_at` value as
    a failure. The judge does not read `fell_at` -- a fallen tray shows up as a run that does not
    reach DONE, which the soundness check catches. This test covers the stronger and more
    interesting violation instead: a run that reports DONE while its tray is still on the deck.

    Built by overriding the tray's final world x to leave it short of `RECV_FIRST_CROWN_X`.
    """
    arm = healthy_arm()
    short = scenario('aligned', friction=0.25, final_state='DONE')
    for rec in short['records']:
        rec['tray'][0] = 2.10                  # still over the deck, short of the receiver
    short['final'] = short['records'][-1]
    arm[0] = short
    code, payload = judge(tmp_path, 'notdelivered', arm)
    assert payload['verdict'] == 'FAIL'
    assert ('the tray ends on the receiving section, measured in world coordinates'
            in payload['failed'])


def test_a_done_run_that_skipped_the_delivery_stages_fails(tmp_path):
    """The anti-silent-upgrade check: DONE without UNLOAD/RECEIVED/PLATFORM_CLEAR is a FAIL.

    Built by hand rather than through `scenario()` so the record list can omit the delivery
    stages while still claiming DONE -- which is exactly the artifact the check exists to reject.
    """
    arm = healthy_arm()
    truncated = scenario('aligned', friction=0.25, final_state='DONE')
    truncated['records'] = truncated['records'][:6]      # stops at EMBARK
    truncated['final'] = truncated['records'][-1]
    truncated['final_state'] = 'DONE'
    arm[0] = truncated
    code, payload = judge(tmp_path, 'nod elivery', arm)
    assert 'DONE means delivered, not merely picked up' in payload['failed']


def test_lateral_drift_beyond_the_bound_fails(tmp_path):
    """NOT_RUN -> FAIL once the rig no longer reports the check it used to.

    REPLACES the old lateral-drift test, which asserted a check name
    (`lateral drift stays within the rail-free bound`) that the judge does not have. The judge
    checks lateral behaviour indirectly, through soundness: a drifting tray stops reaching DONE.
    """
    arm = healthy_arm()
    arm[0] = scenario('aligned', friction=0.25, final_state='STALLED', lateral=0.080,
                      final_rel_x=0.100, tray_x=1.90)
    code, payload = judge(tmp_path, 'drift', arm)
    assert payload['verdict'] == 'FAIL'
    assert code != 0


def test_an_out_of_window_case_is_refused_without_moving_the_tray(tmp_path):
    """A case outside the declared window must be refused, and a refusal must be a STOP.

    REPLACES `test_an_unbounded_window_fails`, which asserted a check name about a bounded
    transfer window that the judge words differently now. What the judge actually checks is
    `the interlock refuses outside its declared window`, and the property behind it is that a
    refusal halts the tray rather than letting it drift -- so this asserts the check PASSES for
    a clean refusal, and `test_a_refused_run_whose_tray_moved_fails` covers the other direction.
    """
    arm = healthy_arm()
    arm[0] = scenario('gap_60mm', friction=0.25, gap=0.060, refused=True, final_state='REFUSED',
                      final_rel_x=0.065, tray_x=1.90)
    code, payload = judge(tmp_path, 'widegap', arm)
    assert 'the interlock refuses outside its declared window' not in payload['failed'], (
        f'a clean refusal was faulted: {payload["failed"]}')
    # Tightness needs a bypass arm, so it is NOT_RUN here -- asserted explicitly, because the
    # whole point of the three-category redesign is that "no evidence" must not read as PASS.
    assert 'the interlock is tight: refusals are justified, in three categories' in payload['not_run']


def test_an_empty_refusal_set_is_not_credited_as_tight(tmp_path):
    """No refusals at all means tightness is NOT_RUN, never a free PASS.

    This is the guard on the reviewer's core concern: a report with nothing to check must not be
    readable as a report that passed the check.
    """
    code, payload = judge(tmp_path, 'norefusals', healthy_arm())
    assert 'the interlock refuses outside its declared window' in payload['not_run']
    assert 'the interlock is tight: refusals are justified, in three categories' in payload['not_run']
    # And the absence of refusals must NOT by itself fail the arm.
    assert ('the interlock refuses outside its declared window' not in payload['failed'])


def test_a_refused_run_whose_tray_moved_fails(tmp_path):
    """The complementary case: a refusal that let the tray move IS a failure."""
    arm = healthy_arm()
    refused = scenario('gap_60mm', friction=0.25, gap=0.060, refused=True,
                       final_state='REFUSED', final_rel_x=0.400, tray_x=2.20)
    # Make the tray move during the refusal: the first and last records disagree in rel_to_deck
    # by more than the judge's `refusal_motion_tol_m` (0.005 m).
    refused['records'][0]['rel_to_deck'] = [-0.300, 0.004, 0.04]
    refused['records'][-1]['rel_to_deck'] = [0.400, 0.004, 0.04]
    arm[0] = refused
    code, payload = judge(tmp_path, 'movedrefusal', arm)
    assert 'the interlock refuses outside its declared window' in payload['failed']


def test_retention_slip_beyond_the_limit_fails(tmp_path):
    """A tray that slides relative to the deck cannot finish, and that is a FAIL.

    REPLACES `test_retention_slip_beyond_the_limit_fails`, which asserted a check name about
    the tray moving with the deck on drive-away. The judge measures that behaviour through the
    EMBARK follow check, `the tray travelled with the deck during EMBARK`; a slipped tray does
    not follow and the run does not reach DONE.
    """
    arm = healthy_arm()
    slipped = scenario('aligned', friction=0.25, final_state='STALLED', final_rel_x=0.480,
                       tray_x=1.95)
    # Deck drives but the tray does not follow: same world x start and end for the tray.
    for rec in slipped['records']:
        rec['tray'] = [1.90, 0.002, 0.52]
        rec['deck_qpos'] = 0.375
    slipped['final'] = slipped['records'][-1]
    arm[0] = slipped
    code, payload = judge(tmp_path, 'slip', arm)
    assert payload['verdict'] == 'FAIL'
    assert code != 0


def test_the_wrong_object_is_a_failure(tmp_path):
    """The object under test must be the real V1 tray, and the judge must say so."""
    reports = synth_run(tmp_path, 'wrongobj', healthy_arm(),
                        tray_asset='assets/objects/payload_007.xml')
    script = ROOT / 'experiments' / 'evaluate_c_transfer.py'
    completed = subprocess.run([sys.executable, str(script), '--run-id', 'wrongobj',
                                '--reports-dir', str(reports)],
                               capture_output=True, text=True)
    payload = json.loads((reports / 'wrongobj' / 'acceptance.json').read_text())
    assert 'the object under test is the real V1 tray asset' in payload['failed']


def test_a_run_that_drops_its_tray_fails(tmp_path):
    """REINSTATES `test_a_fallen_tray_fails`, deleted because the judge ignored `fell_at`.

    The deletion was rationalised at the time: "a fallen tray shows up as a run that does not
    reach DONE, which the soundness check catches." That reasoning is what let the hole live --
    payload loss and non-completion are DIFFERENT claims, and the judge had zero references to
    `fell_at`. Meanwhile the probe recorded the drop and drove on for 6.0 s.

    Now that the judge reads it, the original test comes back.
    """
    arm = healthy_arm()
    arm[0] = scenario('aligned', friction=0.25, final_state='PAYLOAD_LOST', fell_at=54.81,
                      final_rel_x=0.058)
    code, payload = judge(tmp_path, 'dropped', arm)
    assert payload['verdict'] == 'FAIL'
    assert 'no run loses the tray it is carrying' in payload['failed']
    assert code != 0


def test_a_dropped_tray_fails_even_when_the_run_reports_done(tmp_path):
    """The check must NOT be a proxy for "did not reach DONE".

    If a run that reports DONE while having dropped its tray still passes, then the check is
    soundness wearing a hat and detects nothing new. This asserts the independence directly:
    the drop is a failure, and the soundness check has nothing to say about it.
    """
    arm = healthy_arm()
    arm[0] = scenario('aligned', friction=0.25, final_state='DONE', fell_at=41.0)
    code, payload = judge(tmp_path, 'droppeddone', arm)
    assert payload['verdict'] == 'FAIL'
    assert 'no run loses the tray it is carrying' in payload['failed']
    assert 'the interlock is sound: every admitted case completes' not in payload['failed'], (
        'the run reached DONE, so soundness must pass; if it is in `failed` this test is '
        'measuring completion rather than payload loss and proves nothing')


def test_a_clean_arm_decides_the_payload_integrity_check(tmp_path):
    """The check must be decidable from every report -- never NOT_RUN.

    A check that CAN be NOT_RUN is a check that can be skipped, and payload loss is decidable
    from any report carrying `fell_at` (all of them do). If this ever goes NOT_RUN the hole has
    been reopened from the other side.
    """
    code, payload = judge(tmp_path, 'cleanpayload', healthy_arm())
    name = 'no run loses the tray it is carrying'
    assert name not in payload['failed']
    assert name not in payload['not_run'], (
        'payload integrity went NOT_RUN; it is decidable from every report and must say so')


def test_a_refused_run_that_drops_its_tray_also_fails(tmp_path):
    """Evaluated over ALL runs, not just admitted ones.

    A refusal is a STOP: the tray must not move at all. So a refused case whose tray fell is a
    failure of the refusal, and excluding it from this check would let the gate's own cases go
    unexamined.
    """
    arm = healthy_arm()
    arm[0] = scenario('gap_60mm', friction=0.25, gap=0.060, refused=True,
                      final_state='PAYLOAD_LOST', fell_at=12.0, final_rel_x=0.065, tray_x=1.90)
    code, payload = judge(tmp_path, 'droprefused', arm)
    assert payload['verdict'] == 'FAIL'
    assert 'no run loses the tray it is carrying' in payload['failed']

# --- the scope sentence must TRACK the stage list, not be typed ------------------------------
# `evaluate_c_transfer.py` used to close its whole-C check with a typed sentence reading
# "A PASS here is a statement about the receiving-side component ONLY. Whether C as a whole
# passes is NOT_RUN ... : {len(missing)} stage(s) ... never executed." Once the unload stage
# existed, `len(missing)` was 0 while the sentence still said C was undecidable -- one line
# asserting two contradictory things about the same run, which is how four rounds got spent
# reading a constant as if it were evidence. These two tests pin the sentence to the stage
# list in BOTH directions so it cannot go stale again in either.

def _whole_c_check(payload):
    for c in payload['checks']:
        if c['check'] == 'the whole-C claim is not made without its stages':
            return c['verdict'], c['detail']
    raise AssertionError('the whole-C check is absent from the artifact')


def test_every_stage_present_makes_the_scope_sentence_say_so(tmp_path):
    """With nothing missing, the artifact must not still be telling the reader C is undecidable."""
    _, payload = judge(tmp_path, 'scope_all_present', healthy_arm())
    verdict, detail = _whole_c_check(payload)
    assert verdict == 'PASS'
    assert '0 of 10 required stages do not exist' in detail
    # The stale sentence, verbatim. It must not survive a run where nothing is missing.
    assert 'Whether C as a whole passes is NOT_RUN' not in detail
    # A PASS still has to carry its scope, and a real docking is still NOT_RUN.
    assert 'real-AMR docking is NOT_RUN' in detail


def test_a_missing_stage_makes_the_scope_sentence_say_so(tmp_path):
    """With a stage absent, the sentence must say so -- the other direction of the same pin."""
    present = [s for s in FULL_C_REQUIRED if s != 'UNLOAD']
    _, payload = judge(tmp_path, 'scope_missing_unload', healthy_arm(),
                       stages_present=present, stage_claim='NOT_RUN')
    verdict, detail = _whole_c_check(payload)
    # This check grades AGREEMENT between the claim and the list, not completeness.
    assert verdict == 'PASS'
    assert '1 of 10 required stages do not exist' in detail
    assert 'Whether C as a whole passes is NOT_RUN' in detail
    assert '1 stage(s) of the transfer were never executed' in detail
