#!/usr/bin/env python3
"""P4 vision prerequisite: a perception DECISION that can say UNKNOWN.

WHY THIS EXISTS
---------------
`reports/p3-vision-02` calibrated a camera and localized two parts from rendered depth + colour,
blind, cross-validated on a held-out part. That work stands. What it does NOT provide is the thing
`P4-ARM-02` needs before an arm can be allowed to move:

    a decision that REFUSES when its own evidence is not good enough.

P3's selection rule is "exactly one chromatic component resting on the bench". That rule has no
notion of confidence: under depth noise a component can still be unique and on-bench while its
centroid is wrong, and the rule would hand the arm a confidently wrong pose. MASTER_PLAN section 2
step 5 is explicit -- "look unclear -> return UNKNOWN; do not read the order to infer the actual
count" -- so UNKNOWN has to be an OUTPUT of the decision, not a verdict written by a judge.

WHAT THIS FILE ADDS OVER P3
---------------------------
* `decide(per, g)` -- depth + colour -> {class: pose | 'UNKNOWN'}, taking ONLY the observation. It
  refuses when the candidate count is not 1, when there are too few pixels, or when the component's
  own world-space SIZE disagrees with the part's known size beyond a declared tolerance. That last
  gate is the confidence measure P3 lacked, and it is what makes noise degrade to UNKNOWN.
* Cases: clear, moved target, absent part, out-of-view part, and four declared depth-noise levels.
* Every observation carries its frame index, timestamp, age and intrinsics.
* Every non-structural row carries a FALSIFIABILITY probe: an input that makes that row fail. A row
  that cannot be made to fail is not evidence.

WHAT IS REUSED, AND WHAT THAT MEANS
-----------------------------------
`probe_p3_vision` is IMPORTED, not copied: `build_scene`, `prepare`, `View`, `calibrate`, `analyze`,
`mean_rgb`, `chroma_class`, `rests_on_bench`, `bench_geometry`, `part_size`, `pose_of`. The scene,
the camera convention and the segmentation are therefore the SAME code the P3 report was judged on,
which is the point -- a new file cannot silently restyle the calibration it inherits. P3's own
artefacts are untouched: `assets/world_p3_cell.xml` stays byte-identical and `reports/p3-vision-02`
keeps citing its own hashes.

Truth is read ONLY by the scoring code at the bottom. `decide()` never sees it, and the
`decision_is_blind` row demonstrates that behaviourally rather than by promise: it feeds the same
decision a scene whose parts have MOVED and requires the answer to follow the physics.

DECLARED, NOT FROZEN. This is the prerequisite probe for P4-ARM-02; it does not claim P4-ARM-02.
"""
import argparse
import hashlib
import json
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src'))
sys.path.insert(0, str(ROOT / 'experiments'))

import mujoco  # noqa: E402
import numpy as np  # noqa: E402

import probe_p3_vision as PV  # noqa: E402 -- the SAME perception code, imported not copied

WORLD = PV.WORLD
RED, BLUE = PV.RED, PV.BLUE
CLASSES = (('red', RED), ('blue', BLUE))

#: DECLARED design parameters (chosen, not measured).
#: `LOC_TOL` is P3's own localization tolerance, inherited unchanged so the claims are comparable.
LOC_TOL = PV.LOC_TOL
#: A component whose world-space size disagrees with the part's known size by more than this is not
#: trusted. It is applied to the **x and y axes only**, and that is a MEASUREMENT, not a preference:
#: the camera looks DOWN at the bench, so it sees a part's top and never its full height. Measured in
#: the clear case, the z extent comes up 8.9 mm (red) and 10.6 mm (blue) short of 2*g_z, while the
#: worst x/y deficit is 3.7 mm. Gating on z therefore rejects every good detection -- which is
#: exactly what the first version of this file did, and it reported UNKNOWN for a scene it could
#: see perfectly. The declared 6 mm sits at 1.6x the measured clear-case worst x/y deficit: margin,
#: but not a restatement of the position tolerance.
EXTENT_TOL_M = 0.006
#: Which extent components carry shape information. z is excluded for the reason above.
EXTENT_AXES = (0, 1)
#: Depth noise levels in metres. Declared, and swept so the transition from "resolves" to "UNKNOWN"
#: is shown as a curve rather than asserted at one point.
NOISE_SIGMA_M = (0.001, 0.005, 0.015, 0.050)
NOISE_SEED = 20260929

#: How far the target is moved for the moved-target case, in metres.
MOVE_DY_M = 0.120
#: How far a part is lifted to take it off the bench, in metres.
LIFT_M = 0.300
#: How far a part is taken out of the camera's view, in metres.
AWAY_DY_M = -1.200


def decide(per, g):
    """Depth + colour -> {class: pose or UNKNOWN}. Takes ONLY the observation.

    The three refusals, in order of what they protect against:
      * not exactly one chromatic, on-bench candidate -> nothing to hand over, or an ambiguous one
      * too few pixels -> the blob is not a part
      * the component's own world size disagrees with the part's -> the pixels are there but the
        geometry is not, which is how depth noise shows up before the centroid looks wrong
    """
    out = {}
    reasons = {}
    for cls, _body in CLASSES:
        cands = [c for c in per['comps'] if c['cls'] == cls and c['on_bench']]
        if len(cands) != 1:
            out[cls] = None
            reasons[cls] = 'UNKNOWN: %d candidate(s) for a %s part' % (len(cands), cls)
            continue
        c = cands[0]
        if int(c['n_pixels']) < PV.MIN_PART_PIXELS:
            out[cls] = None
            reasons[cls] = 'UNKNOWN: only %d pixels (need %d)' % (c['n_pixels'], PV.MIN_PART_PIXELS)
            continue
        ext = np.asarray(c['bbox_extent_m'], float)
        want = 2.0 * np.asarray(g, float)
        ax = list(EXTENT_AXES)
        worst = float(np.max(np.abs(ext[ax] - want[ax])))
        if worst > EXTENT_TOL_M:
            out[cls] = None
            reasons[cls] = ('UNKNOWN: world x/y size disagrees with the part by %.4f m (limit %.4f)'
                            % (worst, EXTENT_TOL_M))
            continue
        out[cls] = np.asarray(c['centroid'], float)
        reasons[cls] = 'resolved (%d px, size error %.4f m)' % (c['n_pixels'], worst)
    return out, reasons


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--run-id', default='p4-vision-01')
    args = ap.parse_args(argv)

    out = ROOT / 'reports' / args.run_id
    if out.exists() and any(out.iterdir()):
        raise SystemExit('REFUSED: %s already holds evidence; pick a new --run-id' % out)
    out.mkdir(parents=True, exist_ok=True)

    started = time.monotonic()
    report = {
        'probe': 'P4 prerequisite: a perception decision that can return UNKNOWN',
        'reuses': 'experiments/probe_p3_vision.py (imported, not copied)',
        'p3_probe_sha256': hashlib.sha256(
            (ROOT / 'experiments' / 'probe_p3_vision.py').read_bytes()).hexdigest(),
        'world': WORLD.name,
        'world_sha256': hashlib.sha256(WORLD.read_bytes()).hexdigest(),
        'declared': {'loc_tol_m': LOC_TOL, 'extent_tol_m': EXTENT_TOL_M,
                     'extent_axes': list(EXTENT_AXES),
                     'noise_sigma_m': list(NOISE_SIGMA_M), 'noise_seed': NOISE_SEED,
                     'move_dy_m': MOVE_DY_M, 'lift_m': LIFT_M, 'away_dy_m': AWAY_DY_M},
        'status': 'ERROR',
    }

    model, bench_z, blue_pose = PV.build_scene()
    d = mujoco.MjData(model)
    # The keyframe alone is NOT enough to place the parts: with a free-jointed body added afterwards,
    # MuJoCo pads the keyframe's qpos with ZERO, so the blue part would sit at the world origin (P3
    # documents this as the cause of a whole wasted run). So the keyframe is read for RED's own
    # placement, and then BOTH parts are placed explicitly, by name.
    PV.prepare(model, d)
    red_pose = PV.pose_of(model, d, RED)
    g = np.asarray(PV.part_size(model), float)

    def reset(red_xyz, blue_xyz):
        PV.prepare(model, d, poses={RED: np.asarray(red_xyz, float),
                                    BLUE: np.asarray(blue_xyz, float)})
        return {'red': PV.pose_of(model, d, RED), 'blue': PV.pose_of(model, d, BLUE)}

    landed = reset(red_pose, blue_pose)
    report['bench_z_m'] = float(bench_z)
    report['part_size_m'] = [float(v) for v in g]
    report['home_pose'] = {k: [float(x) for x in v] for k, v in landed.items()}

    view = PV.View(model)
    frames = {'n': 0}
    rng = np.random.default_rng(NOISE_SEED)

    def observe(depth_transform=None):
        """One observation. Carries its own frame index, timestamp, age and intrinsics."""
        frames['n'] += 1
        t = time.monotonic()
        rgb = view.capture(d, 'rgb')
        depth = view.capture(d, 'depth')
        depth = np.asarray(depth, float)
        if depth_transform is not None:
            depth = depth_transform(depth)
        return {'frame': frames['n'], 't_monotonic': t, 'depth': depth, 'rgb': rgb}, t

    # calibration: P3's own two-target fit, on this scene
    seg = view.capture(d, 'seg')
    _e, conv, _ranked = PV.calibrate(model, d, np.asarray(view.capture(d, 'depth'), float),
                                     seg, bench_z, g, landed['red'])
    report['camera_convention'] = {k: float(v) for k, v in conv.items()}

    def perceive(depth_transform=None, red_xyz=None, blue_xyz=None):
        if red_xyz is not None and blue_xyz is not None:
            reset(red_xyz, blue_xyz)
        obs, t0 = observe(depth_transform)
        per = PV.analyze(model, d, obs['rgb'], obs['depth'], bench_z, g, conv)
        dec, why = decide(per, g)
        obs['age_s'] = time.monotonic() - t0
        obs['fx'], obs['fy'] = per['fx'], per['fy']
        return dec, why, per, obs

    def score(dec, truth):
        """The ONLY place truth is read. Returns per-class error, or None where the decision
        refused. `dec` and `truth` are BOTH keyed by class ('red'/'blue') -- keying one by class and
        the other by body name is what a KeyError caught, rather than a silently missing row."""
        return {cls: (None if dec[cls] is None
                      else float(np.linalg.norm(np.asarray(dec[cls])[:2]
                                                - np.asarray(truth[cls])[:2])))
                for cls, _body in CLASSES}

    cases = {}

    # -- 1. clear ------------------------------------------------------------------------------
    dec, why, per, obs = perceive(red_xyz=landed['red'], blue_xyz=landed['blue'])
    cases['clear'] = {'decision': {k: (None if v is None else [float(x) for x in v])
                                   for k, v in dec.items()},
                      'why': why, 'raw_error_m': score(dec, landed), 'n_comps': len(per['comps']),
                      'components': [{'cls': c['cls'], 'on_bench': bool(c['on_bench']),
                                      'n_pixels': int(c['n_pixels']),
                                      'bbox_extent_m': [float(v) for v in c['bbox_extent_m']],
                                      'centroid': [float(v) for v in c['centroid']]}
                                     for c in per['comps']],
                      'observation': {'frame': obs['frame'], 'age_s': obs['age_s'],
                                      'fx': obs['fx'], 'fy': obs['fy']}}
    report['clear_raw_error_m'] = cases['clear']['raw_error_m']

    # -- 2. moved target ------------------------------------------------------------------------
    moved = {'red': np.asarray(landed['red'], float),
             'blue': np.asarray(landed['blue'], float) + np.array([0.0, MOVE_DY_M, 0.0])}
    dec_m, why_m, _per_m, _o_m = perceive(red_xyz=moved['red'], blue_xyz=moved['blue'])
    err_m = score(dec_m, moved)
    # distance from the decision to where the part USED to be, so a stale answer is visible
    stale = {cls: (None if dec_m[cls] is None else
                   float(np.linalg.norm(np.asarray(dec_m[cls])[:2]
                                        - np.asarray(landed[cls])[:2])))
             for cls, _body in CLASSES}
    cases['moved_target'] = {'shift_m': MOVE_DY_M, 'error_to_new_m': err_m,
                             'distance_to_old_m': stale, 'why': why_m,
                             'truth_new': {k: [float(x) for x in v] for k, v in moved.items()}}

    # -- 3. absent part (lifted off the bench) --------------------------------------------------
    lifted = {'red': np.asarray(landed['red'], float),
              'blue': np.asarray(landed['blue'], float) + np.array([0.0, 0.0, LIFT_M])}
    dec_l, why_l, _per_l, _o_l = perceive(red_xyz=lifted['red'], blue_xyz=lifted['blue'])
    cases['absent_part'] = {'decision': {k: (None if v is None else 'pose') for k, v in dec_l.items()},
                            'why': why_l, 'blue_lift_m': LIFT_M}

    # -- 4. out of view -------------------------------------------------------------------------
    away = {'red': np.asarray(landed['red'], float),
            'blue': np.asarray(landed['blue'], float) + np.array([0.0, AWAY_DY_M, 0.0])}
    dec_a, why_a, _per_a, _o_a = perceive(red_xyz=away['red'], blue_xyz=away['blue'])
    cases['out_of_view'] = {'decision': {k: (None if v is None else 'pose') for k, v in dec_a.items()},
                            'why': why_a, 'away_dy_m': AWAY_DY_M}

    # -- 5. depth noise sweep -------------------------------------------------------------------
    noise = []
    for sigma in NOISE_SIGMA_M:
        reset(landed['red'], landed['blue'])
        rng_local = np.random.default_rng(NOISE_SEED)

        def add_noise(depth, sigma=sigma, rng=rng_local):
            return np.where(np.isfinite(depth), depth + rng.normal(0.0, sigma, depth.shape),
                            depth)

        dec_n, why_n, _per_n, obs_n = perceive(depth_transform=add_noise)
        err_n = score(dec_n, landed)
        noise.append({'sigma_m': sigma, 'error_m': err_n, 'why': why_n,
                      'verdicts': {k: ('UNKNOWN' if dec_n[k] is None else 'pose') for k in dec_n}})
    cases['depth_noise'] = noise

    report['cases'] = cases

    # -- rows -----------------------------------------------------------------------------------
    def err_of(case_err, cls):
        return case_err.get(cls)

    resolved_clear = all(dec is not None for dec in cases['clear']['decision'].values())
    clear_ok = resolved_clear and all((v is not None and v <= LOC_TOL)
                                      for v in cases['clear']['raw_error_m'].values())
    # ONLY the blue part is moved; red must stay resolved where it is. The first version tested both
    # classes against "far from its old pose", which demands that the part that was NOT moved also
    # move -- a test that fails a correct run.
    moved_ok = (err_m['red'] is not None and err_m['red'] <= LOC_TOL
                and err_m['blue'] is not None and err_m['blue'] <= LOC_TOL
                and stale['blue'] is not None and stale['blue'] >= MOVE_DY_M * 0.5)
    absent_ok = dec_l['blue'] is None and dec_l['red'] is not None
    away_ok = dec_a['blue'] is None and dec_a['red'] is not None
    # no noise level may return a pose that is wrong beyond tolerance
    noise_wrong = [n for n in noise
                   if any(v is not None and v > LOC_TOL for v in n['error_m'].values())]
    noise_honest = len(noise_wrong) == 0
    obs_ok = (obs['frame'] >= 1 and obs['age_s'] >= 0.0
              and obs['fx'] > 0 and obs['fy'] > 0)

    rows = []

    def add(name, ok, detail, falsified=None):
        rows.append({'name': name, 'status': 'PASS' if ok else 'FAIL', 'detail': detail,
                     'falsified_by': falsified})

    add('decision_is_blind_to_the_truth', moved_ok,
        'ONLY the blue part was moved %+.3f m in y, and the decision followed it: %.5f m from its '
        'NEW pose and %.5f m from its OLD one, so it cannot be reading a stored or stale pose. The '
        'red part, which was not moved, stayed resolved at %.5f m. decide() takes only (per, g)'
        % (MOVE_DY_M,
           err_m['blue'] if err_m['blue'] is not None else -1.0,
           stale['blue'] if stale['blue'] is not None else -1.0,
           err_m['red'] if err_m['red'] is not None else -1.0),
        falsified='a decision that read a stored pose would report the OLD one and fail the second half')

    add('clear_scene_resolves_both_parts', clear_ok,
        'both parts resolved with x,y error %s m against a declared %.3f m tolerance (%d components '
        'seen)'
        % ({k: (None if v is None else round(v, 5)) for k, v in cases['clear']['raw_error_m'].items()},
           LOC_TOL, cases['clear']['n_comps']),
        falsified='lifting a part off the bench makes this row fail')

    add('an_absent_part_is_unknown_not_a_guess', absent_ok,
        'with the blue part %0.2f m off the bench the decision returned UNKNOWN for it (%s) while '
        'still resolving red (%s)'
        % (LIFT_M, why_l['blue'], why_l['red']),
        falsified='a rule that ignored on_bench would return a pose here')

    add('a_part_out_of_view_is_unknown', away_ok,
        'with the blue part %+.2f m away the decision returned UNKNOWN for it (%s) while still '
        'resolving red (%s)' % (AWAY_DY_M, why_a['blue'], why_a['red']),
        falsified='a rule that matched the nearest blob would return a pose here')

    add('noise_degrades_to_unknown_never_to_a_wrong_pose', noise_honest,
        'over sigma %s m the decision returned %s, and no level produced a pose wrong by more than '
        'the %.3f m tolerance (violations: %d)'
        % ([float(s) for s in NOISE_SIGMA_M],
           [{k: v for k, v in n['verdicts'].items()} for n in noise],
           LOC_TOL, len(noise_wrong)),
        falsified='a decision without the size gate returns a confidently wrong pose at high sigma')

    add('every_observation_carries_time_and_intrinsics', obs_ok,
        'frame %d, age %.4f s, intrinsics fx %.1f fy %.1f -- so an observation can be aged out '
        'rather than silently reused' % (obs['frame'], obs['age_s'], obs['fx'], obs['fy']),
        falsified=None)

    report['rows'] = rows
    report['counts'] = {'pass': sum(1 for r in rows if r['status'] == 'PASS'),
                        'fail': sum(1 for r in rows if r['status'] == 'FAIL')}
    report['falsifiable_rows'] = sum(1 for r in rows if r['falsified_by'])
    report['verdict'] = ('PASS: all %d judged rows' % len(rows)
                         if report['counts']['fail'] == 0
                         else 'FAIL: %d of %d judged rows'
                              % (report['counts']['fail'], len(rows)))
    report['wall_s'] = time.monotonic() - started
    report['status'] = 'DIAGNOSTIC_COMPLETED'

    (out / 'report.json').write_text(json.dumps(report, indent=2) + '\n', encoding='utf-8')

    for r in rows:
        print('%-4s [%-4s] %-46s %s' % ('ok' if r['status'] == 'PASS' else 'FAIL',
                                        r['status'], r['name'], r['detail'][:150]))
    print()
    print(report['verdict'])
    print('world %s (%s)' % (report['world'], report['world_sha256'][:16]))
    print('p3 probe %s' % report['p3_probe_sha256'][:16])
    print('report -> %s' % (out / 'report.json'))
    return 0 if report['counts']['fail'] == 0 else 1


if __name__ == '__main__':
    raise SystemExit(main())
