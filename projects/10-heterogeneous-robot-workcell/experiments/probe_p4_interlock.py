#!/usr/bin/env python3
"""P4-ARM-02a: the shared-zone interlock, judged.

`P4`'s exit door, quoted from `docs/MASTER_PLAN.md`:
    P4 | 人形取盘、机械臂装盘、输送器分模块闭环 | **每项有单技能成功与故障证据**

So this probe carries BOTH: a grant when the zone is clear, and a REFUSAL when it is not. A probe
that only showed the grant would be showing the easy half, and would not distinguish an interlock
from a function that always returns True.

THE CLAIM UNDER TEST
`docs/MASTER_PLAN.md` section 2, step 3: 人形退出共享操作区并证明**确已清空**，固定臂获得区内操作许可.
The word doing the work is 证明 -- PROVE. So the tests below make a PREDICTION from a measurement and
then check the interlock against it, rather than quoting the interlock's own verdict back.

THE MEASUREMENT
Distances are taken over COLLISION-ENABLED geoms only, because this project measured (`D109`) that
`mj_geomDistance` answers about geometry and that visual-only geoms produce phantom collisions --
272 self-penetrating pairs for a robot standing still, a body against itself. And `distmax` is a
RANGE LIMIT, not a ceiling, so pairs at the budget are counted and reported as beyond-budget rather
than quoted as a number. Row `the_measurement_is_collision_enabled` shows the difference is real
here too, not inherited on faith.

DECLARED, NOT FROZEN. The margin is a declared design parameter and its derivation is printed.
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

from workcell.interlock import (Interlock, ACTORS,  # noqa: E402
                                closest_pair, geom_sets)

WORLD = ROOT / 'assets' / 'world_w5_h085_loop.xml'
HUMANOID_BASE = 'h_LINK_BASE'

#: DECLARED. The margin must be strictly BELOW the separation measured at the working station,
#: otherwise the arm could never be permitted to do its job. Measured at the station (humanoid
#: x 4.130): 0.266779 m. The margin is set to 75% of that, so an intrusion of more than
#: (0.266779 - 0.20) = 66.8 mm is refused. Both numbers are printed by the probe.
MARGIN_M = 0.20
BUDGET_M = 1.0

#: The sweep that makes the boundary a PREDICTION rather than a quotation. It walks the humanoid in
#: past the measured home separation until the two actors interpenetrate.
SWEEP_X = (4.130, 4.050, 4.000, 3.950, 3.900, 3.850, 3.800)
STATION_X = 4.130


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--run-id', default='p4-interlock-01')
    args = ap.parse_args(argv)

    out = ROOT / 'reports' / args.run_id
    if out.exists() and any(out.iterdir()):
        raise SystemExit('REFUSED: %s already holds evidence; pick a new --run-id' % out)
    out.mkdir(parents=True, exist_ok=True)

    started = time.monotonic()
    report = {
        'probe': 'P4-ARM-02a: shared-zone interlock -- granted on a measurement, refused on one',
        'world': WORLD.name,
        'world_sha256': hashlib.sha256(WORLD.read_bytes()).hexdigest(),
        'declared': {'margin_m': MARGIN_M, 'budget_m': BUDGET_M, 'station_x': STATION_X,
                     'sweep_x': list(SWEEP_X)},
        'status': 'ERROR',
    }

    model = mujoco.MjModel.from_xml_path(str(WORLD))
    data = mujoco.MjData(model)
    home = np.array(model.key_qpos[0], dtype=float)

    base_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, HUMANOID_BASE)
    base_jnt = int(model.body_jntadr[base_id])
    base_qadr = int(model.jnt_qposadr[base_jnt])
    if base_qadr != 0:
        raise SystemExit('REFUSED: the humanoid base is not the free joint at qpos[0] (adr=%d)'
                         % base_qadr)

    def park(x):
        data.qpos[:] = home
        data.qvel[:] = 0.0
        data.qpos[0] = float(x)
        mujoco.mj_forward(model, data)
        landed = float(data.xpos[base_id][0])
        if abs(landed - x) > 1e-6:
            raise SystemExit('REFUSED: the humanoid did not move to %.4f (at %.4f)' % (x, landed))
        return landed

    def fresh():
        return Interlock(model=model, data=data, margin_m=MARGIN_M, budget_m=BUDGET_M)

    # -- the instrument, over all geoms vs over collision-enabled geoms -------------------------
    park(STATION_X)
    sets = geom_sets(model, {'arm': 'a_', 'humanoid': 'h_'})
    d_all, pair_all, nb_all = closest_pair(model, data, sets['arm']['all'],
                                           sets['humanoid']['all'], budget_m=BUDGET_M)
    d_live, pair_live, nb_live = closest_pair(model, data, sets['arm']['live'],
                                              sets['humanoid']['live'], budget_m=BUDGET_M)

    def pair_names(pair):
        if pair is None:
            return None
        out = []
        for g in pair:
            b = int(model.geom_bodyid[g])
            out.append('%s/geom%d'
                       % (mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, b) or '?', g))
        return ' vs '.join(out)

    report['instrument'] = {
        'all_geoms': {'n': len(sets['arm']['all']) + len(sets['humanoid']['all']),
                      'distance_m': d_all, 'pair': pair_names(pair_all), 'n_at_budget': nb_all},
        'collision_enabled': {'n': len(sets['arm']['live']) + len(sets['humanoid']['live']),
                              'distance_m': d_live, 'pair': pair_names(pair_live),
                              'n_at_budget': nb_live},
        'visual_only_geoms': {'arm': len(sets['arm']['all']) - len(sets['arm']['live']),
                              'humanoid': len(sets['humanoid']['all']) - len(sets['humanoid']['live'])},
    }

    # -- the zone, derived --------------------------------------------------------------------
    il_station = fresh()
    report['zone_at_station'] = {'extent_x': il_station.snapshot()['extent_x'],
                                 'bounds_x': il_station.zone.bounds_x()}
    park(3.950)
    il_near = fresh()
    report['zone_when_intruded'] = {'extent_x': il_near.snapshot()['extent_x'],
                                    'bounds_x': il_near.zone.bounds_x()}

    # -- 1. granted / refused, the two halves -------------------------------------------------
    park(STATION_X)
    il = fresh()
    sep_home = il.separation('arm', 'humanoid')
    granted, reason_g = il.request('arm')
    report['at_station'] = {'separation': sep_home, 'granted': granted, 'reason': reason_g}
    il_release_ok, reason_rel = il.release('arm') if granted else (None, None)

    park(4.050)
    il2 = fresh()
    sep_near = il2.separation('arm', 'humanoid')
    refused, reason_r = il2.request('arm')
    report['at_intrusion'] = {'humanoid_x': 4.050, 'separation': sep_near,
                              'granted': refused, 'reason': reason_r}
    # a release by a non-holder while the zone is held
    park(STATION_X)
    il3 = fresh()
    il3.request('arm')
    ok_nonholder, reason_nh = il3.release('humanoid')
    ok_reverse, reason_rev = il3.request('humanoid')
    il3.release('arm')
    ok_after, reason_after = il3.request('humanoid')
    report['mutual_exclusion'] = {
        'release_by_non_holder': {'ok': ok_nonholder, 'reason': reason_nh},
        'request_while_held_by_arm': {'ok': ok_reverse, 'reason': reason_rev},
        'request_after_the_arm_released': {'ok': ok_after, 'reason': reason_after},
    }

    # -- 2. the boundary as a PREDICTION ------------------------------------------------------
    sweep = []
    for x in SWEEP_X:
        park(x)
        il_s = fresh()
        sep = il_s.separation('arm', 'humanoid')
        d = sep['distance_m']
        predicted = (d is None) or (d >= MARGIN_M)
        got, why = il_s.request('arm')
        sweep.append({'humanoid_x': x, 'separation_m': d,
                      'n_at_budget': sep['n_at_budget'], 'pair': sep['names'],
                      'predicted_grant': bool(predicted), 'granted': bool(got), 'reason': why,
                      'agrees': bool(predicted) == bool(got)})
    report['sweep'] = sweep
    report['margin_derivation'] = {
        'separation_at_station_m': sep_home['distance_m'],
        'margin_m': MARGIN_M,
        'fraction_of_station': (None if sep_home['distance_m'] is None
                                else MARGIN_M / sep_home['distance_m']),
        'intrusion_tolerated_m': (None if sep_home['distance_m'] is None
                                  else sep_home['distance_m'] - MARGIN_M),
    }

    # -- rows ---------------------------------------------------------------------------------
    rows = []

    def add(name, ok, detail, falsified_by=None):
        rows.append({'name': name, 'status': 'PASS' if ok else 'FAIL', 'detail': detail,
                     'falsified_by': falsified_by})

    zone_moved = (report['zone_at_station']['extent_x']['humanoid']
                  != report['zone_when_intruded']['extent_x']['humanoid'])
    add('the_zone_is_derived_from_the_model', zone_moved,
        'the humanoid extent is read from the geometry and MOVES with it: %s at station x %.3f vs '
        '%s when parked at x 3.950. bounds_x is recomputed per call, so no constant can go stale'
        % ([round(v, 5) for v in report['zone_at_station']['extent_x']['humanoid']], STATION_X,
           [round(v, 5) for v in report['zone_when_intruded']['extent_x']['humanoid']]),
        falsified_by='a typed extent would report the same interval at both positions')

    add('the_measurement_is_collision_enabled',
        d_all is not None and d_live is not None and pair_names(pair_all) != pair_names(pair_live),
        'over ALL geoms the minimum is %.6f m (%s); over COLLISION-ENABLED geoms only it is '
        '%.6f m (%s). %d arm and %d humanoid geoms are visual-only and are excluded, because `D109` '
        'measured that such geoms produce phantom near-collisions -- a robot standing still once '
        'reported 272 self-penetrating pairs, a body against itself'
        % (d_all, pair_names(pair_all), d_live, pair_names(pair_live),
           report['instrument']['visual_only_geoms']['arm'],
           report['instrument']['visual_only_geoms']['humanoid']),
        falsified_by='including visual geoms changes the reported pair, which the row detects')

    add('the_zone_is_not_vacuous',
        not report['zone_at_station']['bounds_x']['empty'],
        'padded by the margin, the two actors\' extents still share the interval x[%.4f, %.4f], so '
        'an intrusion is geometrically possible and the interlock is guarding something real'
        % (report['zone_at_station']['bounds_x']['lo'], report['zone_at_station']['bounds_x']['hi']),
        falsified_by='if the extents could not meet, the zone would be empty and the check decoration')

    add('permission_is_granted_when_the_other_actor_is_clear', bool(granted),
        '[SUCCESS EVIDENCE] with the humanoid at its working station (x %.3f) the measured '
        'separation is %.6f m against a declared margin of %.3f m, so the arm\'s request is granted '
        '(%s), and releasing returns %s'
        % (STATION_X, sep_home['distance_m'], MARGIN_M, reason_g,
           reason_rel if granted else 'n/a'),
        falsified_by='park the humanoid inside the margin and the same request is refused')

    add('permission_is_refused_when_the_other_actor_is_inside_the_margin', not refused,
        '[FAILURE EVIDENCE] with the humanoid at x %.3f the measured separation is %.6f m, below '
        'the %.3f m margin, and the arm\'s request is REFUSED (%s). The interlock is therefore not '
        'a function that always returns True'
        % (report['at_intrusion']['humanoid_x'], sep_near['distance_m'], MARGIN_M, reason_r),
        falsified_by='the same request at the station is granted, so the row distinguishes the two')

    add('the_reverse_holds_while_the_zone_is_held',
        (not ok_nonholder) and (not ok_reverse) and bool(ok_after),
        'while the arm holds the zone: a release by the humanoid is refused (%s), the humanoid\'s '
        'own request is refused (%s), and AFTER the arm releases the humanoid\'s request is granted '
        '(%s). So exclusion is mutual and it is released, not leaked'
        % (reason_nh, reason_rev, reason_after),
        falsified_by='if the holder were not tracked, the humanoid would be granted immediately')

    add('the_boundary_matches_the_declared_margin', all(s['agrees'] for s in sweep),
        'the margin makes a PREDICTION and the sweep checks it: at each of %d positions, grant is '
        'predicted iff the measured separation is at least %.3f m, and the interlock agrees at all '
        'of them (%s). Separations %s m'
        % (len(SWEEP_X), MARGIN_M, 'yes' if all(s['agrees'] for s in sweep) else 'NO',
           [None if s['separation_m'] is None else round(s['separation_m'], 5) for s in sweep]),
        falsified_by='a margin outside the measured range would mispredict some position')

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

    print('instrument: all geoms %.6f m (%s) | collision-enabled %.6f m (%s)'
          % (d_all, pair_names(pair_all), d_live, pair_names(pair_live)))
    print('margin derivation: station separation %.6f -> margin %.3f (%.1f%%), tolerating %.1f mm'
          % (sep_home['distance_m'], MARGIN_M,
             100.0 * report['margin_derivation']['fraction_of_station'],
             1000.0 * report['margin_derivation']['intrusion_tolerated_m']))
    print()
    for r in rows:
        print('%-4s [%-4s] %-52s %s' % ('ok' if r['status'] == 'PASS' else 'FAIL',
                                        r['status'], r['name'], r['detail'][:118]))
    print()
    print(report['verdict'])
    print('report -> %s' % (out / 'report.json'))
    return 0 if report['counts']['fail'] == 0 else 1


if __name__ == '__main__':
    raise SystemExit(main())
