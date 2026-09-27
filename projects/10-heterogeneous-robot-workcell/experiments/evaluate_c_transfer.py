"""Judge for the C receiving-side rig: sensors, interlock, sequence, and the mu sweep.

Pure Python over the probe's records, so every check can be driven by synthetic input.

Four claims are kept apart, because conflating them is how the previous round produced a
meaningless pass:
  1. the sensor chain reads what it claims to read (measured vs commanded);
  2. the interlock REFUSES outside its declared window (and did so without driving);
  3. the interlock is SOUND -- every case it admits must actually complete;
  4. the interlock is TIGHT -- no case it refuses would have succeeded.
Claim 3 and 4 are deliberately separate: a window can be sound but useless, or tight but
unsafe. Claim 4 is measured to a separate report produced by the interlock-bypass arm
(`probe_deck.py --interlock bypass`), which re-runs each case the ARMED gate refused with the
gate forced open; the judge fails tightness if any of those cases reaches DONE. If no bypass
report is supplied the claim is NOT_RUN, never a silent pass.
"""
import argparse
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]

THRESHOLDS = {
    # The yaw estimate from the head PAIR DIFFERENCE is exact: measured error 0.000000 rad at 1,
    # 3 and 8 degrees. The gap estimate from the pair MEAN is not, and its failure is not a bug
    # to calibrate away -- it is the head pair running out of range:
    #     yaw 1 deg -> gap error 0.000106 m
    #     yaw 3 deg -> gap error 0.000959 m
    #     yaw 8 deg -> gap error 0.041560 m
    # The cause is visible in the raw heads. Each one shifts by GAP_RAY_Y * tan(theta), which is
    # the yaw signal and cancels exactly in the difference. But the ray is no longer perpendicular
    # to the roller axis, so at 8 degrees it walks 0.0141 m ALONG the roller (0.1 * tan(8 deg)),
    # and past a few degrees it leaves the cylindrical region it was calibrated on. So the gap
    # head has a NARROWER angular range than the yaw pair, and the check below is evaluated only
    # inside the yaw window the interlock actually admits.
    #
    # This is why the gap bound is expressed against `yaw_max` rather than as a flat tolerance:
    # a flat tolerance would either hide the 8 degree failure or fail the 1 degree passes.
    'sensor_gap_tol_at_yaw_max_m': 0.002,
    'sensor_yaw_tol_rad': 0.004,
    'sensor_step_tol_m': 0.002,
    'sensor_pair_tol_m': 0.002,
    # GAP_RAY_Y, restated as a literal so this file stays pure Python over the report and imports
    # nothing. The observer records; the judge decides; the judge can be driven by synthetic input.
    'gap_ray_y_m': 0.150,
    'min_sensor_runs': 8,
    'refusal_motion_tol_m': 0.05,
    'seated_rel_x_min_m': 0.065,
    'seated_rel_x_max_margin_m': 0.065,
    # EMBARK's DONE is only meaningful if the tray really travelled WITH the deck. Measured in
    # the deck's own frame the two are indistinguishable: when the deck slides out from under a
    # stationary tray, the tray's deck-local x DECREASES back toward the seating range, so a
    # "fully aboard" check on deck-local coordinates passes on a tray that never boarded. The
    # previous revision passed exactly that way. The trajectory is therefore measured in WORLD
    # coordinates: the tray must have moved forward by at least this fraction of the deck travel.
    'embark_follow_fraction': 0.80,
    # The tray's trailing edge must clear the first receiving crown by no more than this.
    # Expressed as a tolerance on the DELIVERY, not on the mechanism: the mechanism's own
    # clear margin is RECEIVED_CLEAR_MARGIN in the probe, and this is the judge's allowance
    # for the tray settling after the rollers stop.
    'received_tol_m': 0.030,
}
COORDINATE_STATES = ('CHECK_ALIGN', 'CONVEY', 'RELEASE', 'SETTLE', 'DEPLOY_PUSHER', 'EMBARK',
                     'UNLOAD', 'RECEIVED', 'PLATFORM_CLEAR', 'DONE')
# The stage list must match `probe_deck.py`'s state machine EXACTLY, and it did not for
# one revision: the pin-to-pusher redesign (D032/D034) renamed `RAISE_PIN` to
# `DEPLOY_PUSHER` in the probe and left this tuple stale, so the sequence check compared
# every real run against a stage the machine no longer has. Nothing caught it because the
# only tests exercising this path were failing on a fixture-schema error (see
# tests/test_p1_c_transfer.py) and had been asserting nothing for several rounds.
# `tests/test_p1_c_rig_geometry.py::test_the_stage_detector_sees_every_transition_form`
# now pins the probe side; this comment is the judge side's half of that pairing.
# The stages that make DONE mean DELIVERED rather than PICKED UP. A run that reaches DONE
# without traversing these has not transferred anything to the receiving end, and it is the
# specific silent upgrade this round exists to close.
DELIVERY_STAGES = ('UNLOAD', 'RECEIVED', 'PLATFORM_CLEAR')


def model_has_no_weld(path):
    if not path.is_file():
        return None
    root = ET.parse(path).getroot()
    if root.find('equality') is not None:
        return False
    tray = root.find("worldbody/body[@name='payload']")
    return tray is not None and tray.find('freejoint') is not None


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--run-id', required=True)
    parser.add_argument('--reports-dir', default=str(ROOT / 'reports'))
    parser.add_argument('--bypass-run-id', default='',
                        help='run id of the interlock-bypass arm; its `interlock_bypass_runs` is '
                             'folded into this report so the tightness claim can be decided')
    args = parser.parse_args()
    run_dir = Path(args.reports_dir) / args.run_id
    report = json.loads((run_dir / 'report.json').read_text(encoding='utf-8'))
    if args.bypass_run_id:
        bypass_dir = Path(args.reports_dir) / args.bypass_run_id
        bypass_report = json.loads((bypass_dir / 'report.json').read_text(encoding='utf-8'))
        if bypass_report.get('interlock_mode') != 'bypass':
            raise SystemExit(f'{args.bypass_run_id} is not a bypass arm '
                             f"(interlock_mode={bypass_report.get('interlock_mode')!r})")
        report['interlock_bypass_runs'] = bypass_report.get('interlock_bypass_runs')
        report['interlock_bypass_run_id'] = args.bypass_run_id
    runs = report['runs']
    deck_length = report['deck']['length']
    # The receiving geometry is READ from the artifact, never re-derived here: if the rig's
    # placement changes, the judge must measure against the new placement rather than against
    # a number typed into the judge.
    recv_first = report.get('recv_first_crown_x')

    checks = []

    def check(name, verdict, detail):
        checks.append({'check': name, 'verdict': verdict, 'detail': detail})

    tray = report['tray_measurements']
    half_length = round((tray['x_max'] - tray['x_min']) / 2.0, 6)

    check('the object under test is the real V1 tray asset',
          'PASS' if report['tray_asset'].endswith('tray_v1.xml') and tray['mass_kg'] > 0 else 'FAIL',
          f"{report['tray_asset']} sha256 {report['tray_asset_sha256'][:12]}, "
          f"mass {tray['mass_kg']} kg")

    supported = 2 * report['roller_half_length_m'] >= 2 * tray['support_half_width']
    check('the roller row is wide enough to carry the tray floor',
          'PASS' if supported else 'FAIL',
          f"roller length {2 * report['roller_half_length_m']:.3f} m vs support width "
          f"{2 * tray['support_half_width']:.3f} m")

    no_weld = model_has_no_weld(run_dir / 'model.xml')
    check('the payload is a free body with no equality constraint',
          'NOT_RUN' if no_weld is None else ('PASS' if no_weld else 'FAIL'),
          'model.xml has no <equality> and payload carries a freejoint')

    friction_reported = sorted({r['friction'] for r in runs})
    check('the interface friction is a swept, overridden parameter',
          'PASS' if len(friction_reported) >= 3 and all(f > 0 for f in friction_reported) else 'FAIL',
          f'swept over {friction_reported}; the tray asset declares 1.0 with no material basis '
          f'anywhere in the project, and both sides of every interface are overwritten with the '
          f'swept value so the effective coefficient is unambiguous')

    # The sensor claim is about the range the sensor chain is exercised over. It must be evaluated
    # PER RUN against the window that actually applies to that run, NOT against the widest window
    # across all frictions. The earlier version took the maximum, which let a case refused at
    # mu = 0.15 (yaw_max 4 deg) be judged as if the 6 deg counter of mu = 0.25 governed it -- the
    # same "one scalar where the physics is a curve" error the probe had, in the judge this time.
    # It surfaced as yaw_5deg@0.15 being counted INSIDE the window and failing the gap tolerance,
    # when that run had in fact been refused and was outside its own friction's admitted range.
    by_friction = report.get('align_window_by_friction') or {}
    yaw_max = max([report['align_window']['yaw_max']]
                  + [w['yaw_max'] for w in by_friction.values()])

    def window_for(friction):
        """The window applying to a run, by friction, falling back to the declared fallback."""
        key = str(friction)
        for k, w in by_friction.items():
            if abs(float(k) - friction) < 1e-9:
                return w
        return report['align_window']

    inside, outside, pair_err = [], [], []
    for r in runs:
        a = r['align_measured'] or {}
        if a.get('gap') is None or a.get('yaw') is None or a.get('step') is None:
            continue
        heads = a.get('gap_heads')
        if heads is not None:
            # The claim about the pair: the DIFFERENCE recovers 2 * GAP_RAY_Y * tan(theta).
            pair_err.append(abs((heads[1] - heads[0])
                                - (-2.0 * THRESHOLDS['gap_ray_y_m'] * math.tan(a['yaw']))))
        row = (r['scenario']['name'], r['friction'], a['yaw'],
               abs(a['gap'] - r['scenario']['gap']),
               abs(a['yaw'] - r['scenario']['yaw']),
               abs(a['step'] - r['scenario']['step']))
        # The run is inside the sensor's usable range only if it is inside ITS OWN friction's
        # window. A run refused by its own gate is excluded from the gap claim, not forgiven --
        # the interlock is what keeps the system inside the head pair's angular range.
        own = window_for(r['friction'])
        (inside if abs(a['yaw']) <= own['yaw_max'] + 1e-9 else outside).append(row)

    if len(inside) + len(outside) < THRESHOLDS['min_sensor_runs']:
        check('the sensor chain reads what it claims to read', 'NOT_RUN',
              f'only {len(inside) + len(outside)} runs carried a full sensor reading')
    else:
        worst_gap = max(t[3] for t in inside) if inside else float('nan')
        breach = [f'{t[0]}@{t[1]}: gap err {t[3]:.6f} > '
                  f'{THRESHOLDS["sensor_gap_tol_at_yaw_max_m"]:.4f}'
                  for t in inside if t[3] > THRESHOLDS['sensor_gap_tol_at_yaw_max_m']]
        worst_yaw = max(t[4] for t in inside + outside)
        worst_step = max(t[5] for t in inside + outside)
        worst_pair = max(pair_err) if pair_err else float('nan')
        ok = (not breach
              and worst_yaw <= THRESHOLDS['sensor_yaw_tol_rad']
              and worst_step <= THRESHOLDS['sensor_step_tol_m']
              and worst_pair <= THRESHOLDS['sensor_pair_tol_m'])
        check('the sensor chain reads what it claims to read', 'PASS' if ok else 'FAIL',
              f'{len(inside)} run(s) inside their OWN friction\'s admitted yaw window '
              f'(widest is |yaw| <= {yaw_max:.6f} rad = {math.degrees(yaw_max):.1f} deg): '
              f'gap worst {worst_gap:.6f} m vs tolerance '
              f'{THRESHOLDS["sensor_gap_tol_at_yaw_max_m"]:.4f} m; '
              f'yaw worst {worst_yaw:.6f} rad, step worst {worst_step:.6f} m, '
              f'pair-difference worst {worst_pair:.6f} m. '
              f'{len(outside)} run(s) fall outside their own window and are excluded from the gap '
              f'claim, NOT forgiven: the head pair loses range there (measured 0.0416 m at '
              f'8 deg), so the gap head is narrower-ranged than the yaw pair and the interlock '
              f'is what keeps the system inside its range'
              + (f'; breaches: {breach}' if breach else ''))

    refused = [r for r in runs if r['refused']]
    admitted = [r for r in runs if not r['refused']]
    if not refused:
        check('the interlock refuses outside its declared window', 'NOT_RUN',
              'no run was outside the window')
    else:
        moved = []
        for r in refused:
            final = r['final'] or {}
            rel = (final.get('rel_to_deck') or [0.0])[0]
            start = (r['records'][0].get('rel_to_deck') or [0.0])[0] if r['records'] else 0.0
            if abs(rel - start) > THRESHOLDS['refusal_motion_tol_m']:
                moved.append(r['scenario']['name'])
        check('the interlock refuses outside its declared window',
              'PASS' if not moved else 'FAIL',
              f'{len(refused)} run(s) refused; tray moved in {moved or "none"} '
              f'(tolerance {THRESHOLDS["refusal_motion_tol_m"]:.3f} m)')

    # -- payload integrity: losing the object ends the discussion ------------------------
    # Every run records `fell_at`, the first simulated time the tray's world height fell
    # `FALL_DEPTH` below its own resting height. UNTIL THIS CHECK EXISTED NOTHING READ IT: this
    # file had zero references to `fell_at`, so payload loss was only ever caught INDIRECTLY, as
    # a run that failed to reach DONE. Those are different claims, and the difference is not
    # academic -- `p1-c-f1-01` lost its tray at 54.808 s while the machine kept commanding the
    # deck, the rollers and the pusher for another 6.0 s.
    #
    # A run that drops the object it is carrying is a FAILURE IN ITS OWN RIGHT, whatever it
    # reports afterwards, and it is decidable from every report, so this check is never NOT_RUN.
    # It is deliberately evaluated over ALL runs, not just the admitted ones: a refused case
    # must not move the tray at all, so a fall there is a failure too.
    dropped = [f"{r['scenario']['name']}@{r['friction']}" for r in runs if r.get('fell_at')]
    check('no run loses the tray it is carrying',
          'PASS' if not dropped else 'FAIL',
          f'`fell_at` is the first simulated time the tray fell below its resting height; '
          f'runs that dropped their tray: {dropped or "none"}')

    sound = [f"{r['scenario']['name']}@{r['friction']}={r['final_state']}" for r in admitted
             if r['final_state'] != 'DONE']
    check('the interlock is sound: every admitted case completes',
          'PASS' if not sound else 'FAIL',
          f'{len(admitted)} run(s) admitted; did not reach DONE: {sound or "none"} '
          f'(the state after "=" is where the run actually stopped. An earlier revision printed '
          f'a FIXED gloss here -- "STALLED means the deck ran out of budget without carrying the '
          f'tray" -- which named a mechanism that had not happened: `p1-c-f1-01` was reported '
          f'under that gloss while its tray had in fact fallen off the side of the deck. Do not '
          f'summarise a per-run outcome with a constant. Read the state.)')

    over = []
    for r in admitted:
        if r['final_state'] != 'DONE':
            continue
        # EVALUATED AT EMBARK, NOT AT THE END -- because the two are now mutually exclusive.
        # Since DONE means "delivered and clear of the deck", a DONE run's FINAL position is
        # necessarily OUTSIDE the deck footprint, so testing it against a "seated on the deck"
        # bound can never pass:
        #     DONE needs     rel_x >  DECK_LENGTH + PLATFORM_CLEAR_MARGIN   = 0.7900 + 0.05 = 0.8400
        #     seated needs   rel_x <= deck_length - seated_rel_x_max_margin = 0.7900 - 0.065 = 0.7250
        # Measured on `p1-c-fix-01`: 0.8415 and 0.8415, both DONE and both "outside the footprint".
        # The claim that was always meant is "the tray was FULLY ABOARD when EMBARK handed it over",
        # which is the last EMBARK record. Testing the terminal state was testing a moment at which
        # the property is false by design.
        embark = [rec for rec in (r.get('records') or []) if rec['state'] == 'EMBARK']
        if not embark:
            over.append(f"{r['scenario']['name']}@{r['friction']}: no EMBARK record")
            continue
        rel = embark[-1].get('rel_to_deck', [None])[0]
        if rel is None or not (THRESHOLDS['seated_rel_x_min_m']
                               <= rel <= deck_length - THRESHOLDS['seated_rel_x_max_margin_m']):
            over.append(f"{r['scenario']['name']}@{r['friction']}={rel}")
    check('an admitted transfer seats the tray fully on the deck',
          'PASS' if not over else 'FAIL',
          f'tray half-length {half_length:.3f} m on a {deck_length:.3f} m deck; '
          f'outside the footprint: {over or "none"}')

    # -- and it must have actually gone along for the ride --------------------------------
    # See `embark_follow_fraction`: deck-local coordinates cannot tell "the tray boarded" from
    # "the deck left without it". This is the check that closes that hole.
    lag = []
    for r in admitted:
        if r['final_state'] != 'DONE':
            continue
        recs = r['records']
        if not recs:
            continue
        entered = False
        tray_start = deck_start = None
        for rec in recs:
            if rec['state'] == 'EMBARK' and not entered:
                entered = True
                tray_start = rec['tray'][0]
                deck_start = rec['deck_qpos']
        if tray_start is None:
            lag.append(f"{r['scenario']['name']}@{r['friction']}: no EMBARK record")
            continue
        final = r['final']
        d_deck = final['deck_qpos'] - deck_start
        d_tray = final['tray'][0] - tray_start
        if d_deck <= 1e-6 or d_tray < THRESHOLDS['embark_follow_fraction'] * d_deck:
            lag.append(f"{r['scenario']['name']}@{r['friction']}: the deck moved "
                       f"{d_deck:.4f} m but the tray only {d_tray:.4f} m")
    check('the tray travelled with the deck during EMBARK',
          'PASS' if not lag else 'FAIL',
          f'measured in WORLD coordinates over EMBARK; the tray must move at least '
          f'{THRESHOLDS["embark_follow_fraction"]:.0%} of the deck travel. '
          f'Failures: {lag or "none"}')

    bypass = report.get('interlock_bypass_runs')
    if not bypass:
        check('the interlock is tight: refusals are justified, in three categories', 'NOT_RUN',
              'no interlock-bypass arm in this report; only the armed gate was run, so a refused '
              'case was never attempted and tightness cannot be measured (needs '
              '`probe_deck.py --interlock bypass`)')
    else:
        # Each case is judged against the window declared FOR ITS OWN friction (see
        # `align_window_by_friction`), so no friction needs a privileged carve-out and a win at
        # any swept friction is genuine conservatism. The earlier "only the design floor counts"
        # rule existed solely because one window was being applied to three different envelopes,
        # and it let real refusals of workable cases through.
        #
        # Refusal reason. `sensor_unavailable` is the case the reviewer named as undecidable: the
        # ray missed the tray, so the rig could not even see the situation, and reaching DONE or
        # not says nothing about whether the gate was right. It is identified from the probe's
        # own record of WHY the case was refused, never guessed from the outcome -- guessing from
        # the outcome is how an undecidable case would quietly become evidence.
        def reason_of(b):
            return (b.get('refusal_reason') or b.get('matched_axis') or '').lower()

        def is_sensor(b):
            r = reason_of(b)
            return 'sensor' in r or 'miss' in r or 'beam' in r or 'ray' in r

        # Cases decided by rounding are evidence in neither direction and are excluded from all
        # three categories, not silently from whichever side is convenient.
        #
        # ORIENTATION, stated because getting it backwards is a real trap here: every entry in
        # `bypass` is a case the ARMED gate REFUSED (the probe skips the ones it would have
        # admitted, since re-running those tests nothing). So within this set
        #     would_have_completed     -> the refusal cost us something   = CONSERVATISM
        #     not would_have_completed -> the refusal was correct        = healthy
        decided = [b for b in bypass if not b.get('on_boundary')]
        boundary = [b for b in bypass if b.get('on_boundary')]
        sensor_unavailable = [b for b in decided if is_sensor(b)]
        decidable = [b for b in decided if not is_sensor(b)]
        refused_but_completable = [b for b in decidable if b.get('would_have_completed')]
        refusal_justified = [b for b in decidable if not b.get('would_have_completed')]
        # The reviewer's critical class -- admitted by the gate and THEN failed -- is not
        # observable in a bypass set (nothing here was admitted). It is owned by the soundness
        # check on the armed arm; the count is repeated here so the three categories read as a
        # complete partition, and marked as owned elsewhere so it cannot be double-counted.
        armed_refused = sum(1 for r in runs if r['refused'])

        parts = [
            f'{len(bypass)} case(s) the armed gate refused were re-run with the gate forced open, '
            f'each judged against the window declared for its own friction; reported in three '
            f'categories, because a safety interlock is allowed to refuse conservatively and '
            f'collapsing conservatism into "unsafe" makes the word FAIL unreadable',
            f'CONSERVATISM / refused-but-completable: {len(refused_but_completable)} '
            f'(cost, not hazard: the gate refused work something that in fact worked; a number '
            f'above zero means the window is narrower than the mechanism)',
            f'JUSTIFIED / refused-and-also-failed: {len(refusal_justified)} '
            f'(the refusal was correct -- opening the gate changed nothing, the case still could '
            f'not finish)',
            f'UNDECIDABLE / sensor-unavailable: {len(sensor_unavailable)} '
            f'(the ray did not see the tray, so the outcome is not evidence either way)',
            f'CRITICAL / admitted-then-failed: not measurable in a bypass set, and owned by the '
            f'soundness check on the armed arm (see `the interlock is sound`); '
            f'{armed_refused} case(s) were refused there',
        ]
        if boundary:
            names = sorted({f"{b['scenario']}@{b['friction']} (margin {b.get('margin')})"
                            for b in boundary})
            parts.append(f'{len(boundary)} excluded as on-boundary (decided by rounding, not by '
                         f'the interface): {"; ".join(names)}')
        if refused_but_completable:
            parts.append('the window is TOO TIGHT at these frictions -- the gate refused cases '
                         'that would have worked: '
                         + '; '.join(f"{b['scenario']}@{b['friction']} "
                                     f"(matched {b.get('matched_axis')}, "
                                     f"final {b.get('final_state')})"
                                     for b in refused_but_completable))
        if not decidable:
            # Every bypass case was undecidable. There is no evidence here at all, so the claim
            # is NOT_RUN -- emphatically not a pass earned by having nothing to check.
            verdict = 'NOT_RUN'
            parts.append('no bypass case was decidable: every refusal was caused by the sensor '
                         'not seeing the tray, so tightness is NOT_RUN rather than PASS')
        else:
            # Both defect classes fail. Conservatism fails because shipping a window that
            # refuses workable cases means an undiscovered defect is being shipped -- the
            # report names it conservatism so the diagnosis is not confused with "unsafe".
            verdict = 'FAIL' if refused_but_completable else 'PASS'
        check('the interlock is tight: refusals are justified, in three categories',
              verdict, '; '.join(parts))

    sequenced = []
    for r in runs:
        if r['final_state'] != 'DONE':
            continue
        seen = []
        for rec in r['records']:
            if not seen or seen[-1] != rec['state']:
                seen.append(rec['state'])
        expect = [s for s in COORDINATE_STATES if s in seen]
        if seen != expect:
            sequenced.append({'run': f"{r['scenario']['name']}@{r['friction']}", 'seen': seen})
    check('the sequence is sensor-driven and reaches DONE in order',
          'PASS' if not sequenced else 'FAIL',
          f'expected order {list(COORDINATE_STATES)}; out of order: {sequenced or "none"}')

    guided = [r for r in runs if r['guides']]
    plain = [r for r in runs if not r['guides']]
    if guided and plain:
        # Raw counts would charge the guide for the friction curve, since mu=0.15 cannot finish
        # regardless of guides. The question is whether a guide CHANGES the outcome on the SAME
        # (mu, scenario) pair, so the comparison is paired.
        by_plain = {(r['friction'], r['scenario']['name']): r['final_state'] for r in plain}
        pairs = [(r, by_plain[(r['friction'], r['scenario']['name'])]) for r in guided
                 if (r['friction'], r['scenario']['name']) in by_plain]
        worse = [f"{r['scenario']['name']}@{r['friction']}: "
                 f"{r['final_state']} with guides vs {p} without"
                 for r, p in pairs
                 if r['final_state'] != 'DONE' and p == 'DONE']
        better = sum(1 for r, p in pairs if r['final_state'] == 'DONE' and p != 'DONE')
        check('the guide rail is at least not harmful',
              'PASS' if not worse else 'FAIL',
              f'{len(pairs)} paired (mu, scenario) comparisons: guides lose {len(worse)}, '
              f'win {better}; a straight rail captures at most '
              f'{(report["guide_band"]["ball_inner_y"] - report["guide_band"]["wall_outer_y"]) * 1000:.0f} '
              f'mm because a flared mouth would be swept by the handle ball'
              + (f'; regressions: {worse}' if worse else ''))

    # -- the receiving end ------------------------------------------------------------------
    # A DONE that never went through UNLOAD/RECEIVED/PLATFORM_CLEAR is a DONE that means "the
    # tray was picked up". That is the exact claim this check refuses to let pass.
    no_delivery = []
    for r in runs:
        if r['final_state'] != 'DONE':
            continue
        seen = {rec['state'] for rec in r['records']}
        absent = [s for s in DELIVERY_STAGES if s not in seen]
        if absent:
            no_delivery.append(f"{r['scenario']['name']}@{r['friction']} missing "
                               f"{'+'.join(absent)}")
    reached = [r for r in runs if r['final_state'] == 'DONE']
    check('DONE means delivered, not merely picked up',
          'PASS' if (reached and not no_delivery) else ('NOT_RUN' if not reached else 'FAIL'),
          f'{len(reached)} run(s) reached DONE; every one of them must have traversed '
          f'{list(DELIVERY_STAGES)}. ' + (f'Violations: {no_delivery}' if no_delivery
                                          else 'All DONE runs traversed the delivery stages.') +
          ' Without this check a machine that stops at EMBARK reports DONE on every transfer '
          'while having delivered nothing.')

    # The tray must actually END on the receiving section, measured in WORLD coordinates
    # against the geometry the rig itself reports. Deck-local coordinates cannot show this:
    # after the deck has driven out, a tray still aboard and a tray on the receiver can have
    # the same deck-local x.
    recv_meta = report.get('rig', {}).get('receiver') if report.get('rig') else None
    recv_first = report.get('recv_first_crown_x')
    half = half_length
    put = []
    for r in runs:
        if r['final_state'] != 'DONE':
            continue
        final = r['final'] or {}
        tray = final.get('tray')
        if not tray or recv_first is None:
            put.append(f"{r['scenario']['name']}@{r['friction']}: no final tray pose")
            continue
        trailing = tray[0] - half
        if trailing < recv_first - THRESHOLDS['received_tol_m']:
            put.append(f"{r['scenario']['name']}@{r['friction']}: trailing edge "
                       f"{trailing:.4f} m is short of the first receiving crown "
                       f"{recv_first:.4f} m")
    check('the tray ends on the receiving section, measured in world coordinates',
          'PASS' if (reached and not put) else ('NOT_RUN' if not reached else 'FAIL'),
          f'the receiving section starts at world x {recv_first if recv_first is not None else "?"} '
          f'(declared by the rig, not by this file); the tray\'s trailing edge must reach it '
          f'within {THRESHOLDS["received_tol_m"]:.3f} m. '
          + (f'Short: {put}' if put else 'Every DONE run finished on the receiving section.'))

    # -- the whole-C claim, and the anti-silent-upgrade check ------------------------------
    # The failure this guards against is specific and has happened in this project: a partial
    # experiment being read as the complete one. Two ways to get there are closed here.
    #   (a) `full_C_acceptance` must AGREE with the stage list in the same artifact. If the
    #       probe ever claims CLAIMED while a required stage is missing, that is a FAIL, not a
    #       note -- the claim and the evidence would disagree inside one file.
    #   (b) A PASS in this report must never be quotable as "C is done" WITHOUT its scope. The
    #       check spells out, in its own detail string, which stages are absent -- or, when none
    #       are, what scope remains unestablished. The scope travels with the number instead of
    #       living in someone's memory, and it is DERIVED so that it cannot go stale.
    required = report.get('full_C_stages_required')
    present = report.get('full_C_stages_present')
    claim = report.get('full_C_acceptance')
    if not required or not present:
        check('the whole-C claim is not made without its stages', 'NOT_RUN',
              'the probe did not record its stage lists, so the claim cannot be checked')
    else:
        missing = sorted(set(required) - set(present))
        consistent = ((claim == 'NOT_RUN' and missing) or (claim == 'CLAIMED' and not missing))
        delivered = [s for s in present
                     if any(rec['state'] == s for r in runs for rec in r['records'])]
        # The scope sentence is DERIVED for the same reason the claim is. An earlier revision
        # typed it: "Whether C as a whole passes is NOT_RUN" sat beside a computed
        # "N stage(s) ... never executed", so as soon as every stage existed the SAME LINE said
        # both "0 were never executed" and "C is undecidable". A reader cannot tell a stale
        # constant from a real limit -- and this project has already spent rounds doing exactly
        # that. The sentence now follows the stage list in both directions.
        if missing:
            scope_note = (
                f"A PASS here is a statement about the stages this rig has. Whether C as a "
                f"whole passes is NOT_RUN and cannot be decided from this artifact: "
                f"{len(missing)} stage(s) of the transfer were never executed.")
        else:
            scope_note = (
                f"Every required stage exists AND ran in this report, so the stage-list half of "
                f"the whole-C claim is satisfied here. What this artifact still does NOT "
                f"establish is its declared scope: {report.get('scope_claim')}. Read this PASS "
                f"with that scope attached; a real-AMR docking is NOT_RUN.")
        check('the whole-C claim is not made without its stages',
              'PASS' if consistent else 'FAIL',
              f"full_C_acceptance is {claim!r}, derived by the probe from its own stage names, "
              f"and it is consistent with the stage list; {len(missing)} of "
              f"{len(required)} required stages do not exist in this rig: "
              f"{', '.join(missing) or 'none'}. Present: {', '.join(sorted(present))}. "
              f"Stages that actually ran in this report: {', '.join(sorted(delivered))}. "
              f"*** {scope_note} ***")

    failed = [c for c in checks if c['verdict'] == 'FAIL']
    not_run = [c for c in checks if c['verdict'] == 'NOT_RUN']
    verdict = 'FAIL' if failed else ('INCOMPLETE' if not_run else 'PASS')

    payload = {'run_id': args.run_id, 'verdict': verdict, 'checks': checks,
               'failed': [c['check'] for c in failed], 'not_run': [c['check'] for c in not_run],
               'thresholds': THRESHOLDS, 'frictions': report['frictions'],
               'interlock_bypass_run_id': report.get('interlock_bypass_run_id'),
               # Carried through from the probe, NOT re-asserted here. The judge's job is to
               # check the claim, not to make it; a hard-coded value here could disagree with
               # the report without anyone noticing.
               'full_C_acceptance': report.get('full_C_acceptance'),
               'full_C_stages_required': report.get('full_C_stages_required'),
               'full_C_stages_present': report.get('full_C_stages_present'),
               'scope_claim': report.get('scope_claim')}
    (run_dir / 'acceptance.json').write_text(json.dumps(payload, indent=2) + '\n',
                                            encoding='utf-8')
    lines = [f'# C receiving-side rig -- `{args.run_id}`', '',
             f'**verdict: {verdict}**  ({len(checks)} checks: '
             f'{sum(1 for c in checks if c["verdict"] == "PASS")} PASS, {len(failed)} FAIL, '
             f'{len(not_run)} NOT_RUN)', '', '## Checks', '']
    for item in checks:
        lines.append(f'- `{item["verdict"]}` **{item["check"]}** -- {item["detail"]}')
    lines += ['', '## Runs by state', '',
              '| mu | guides | scenario | measured gap | yaw | step | state | rel_x |',
              '| --- | --- | --- | --- | --- | --- | --- | --- |']
    for r in runs:
        a = r['align_measured'] or {}
        rel = (r['final'] or {}).get('rel_to_deck', [None])[0]
        lines.append(f"| {r['friction']} | {r['guides']} | {r['scenario']['name']} | "
                     f"{a.get('gap')} | {a.get('yaw')} | {a.get('step')} | "
                     f"{r['final_state']} | {rel} |")
    (run_dir / 'acceptance.md').write_text('\n'.join(lines) + '\n', encoding='utf-8')

    print(json.dumps({'run_id': args.run_id, 'verdict': verdict, 'failed': payload['failed'],
                      'not_run': payload['not_run']}, ensure_ascii=False))
    for item in checks:
        print(f"  {item['verdict']:<8} {item['check']}")
    return 0 if verdict == 'PASS' else 1


if __name__ == '__main__':
    raise SystemExit(main())
