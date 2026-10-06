#!/usr/bin/env python3
"""G4 seam probe v2: the AT-REST state at the declared place point (no drop, no hands).

WHY v2 (revision 1 of the seam declaration): v1 (`p4-belt-g4-seam-01`, evidence
preserved) misread the H2 event. H2's own report shows the tray was ALREADY at
rest when the hands left (release-stage speed 4.5e-6 m/s; the exit row passed
with "the tray moved 0.06 mm at most"); the 9.99 mm place gap closed DURING the
hands' lowering, between t=33.5 and 34.4. v1's 10 mm free fall (0.44 m/s impact)
was therefore a sled test, not the H2 landing: it launched the tray (80 m
excursion, idlers to 750 rad) and its FAIL rows measure v1's own overload, not
the world.

What the existing data already established (no new sim needed for these):
  * the 0.176 rad / 1.23 rad/s `source_stopped` spike happened at t=34.1-34.2,
    while the LEFT hand was still in contact (right hand already off) -- it is
    hand-withdrawal disturbance of rollers;
  * the tray's true post-release motion was 30 um (4.44924 -> 4.44921) over
    15.5 s -- the source carried the tray by 30 um, i.e. not at all.

v2 therefore qualifies the state the transfer contract actually cares about:
the tray AT REST on the designated rollers, no hands, no impact -- plus the
seam geometry. Ten seconds, everything free except the pinned humanoid/arm/
station (same declared boundary condition as v1).

ROWS (no frozen machine or threshold is touched; the H2 FAIL stands):
  1. at_rest_state_is_quiet      -- peak band-joint speed < 0.05 rad/s
  2. tray_static_at_declared     -- xy excursion from the declared point <= 5 mm
  3. supported_on_designated     -- BOTH named main rollers in contact at settle
  4. w5_scope_rollers_held       -- every w5-scope roller <= 5 mm own-radius travel
  5. worst_mover_reported        -- attribution listing (expected: negligible)
  6. seam_geometry_continuous    -- same static seam check as v1
"""
import argparse
import hashlib
import json
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT / 'experiments'), str(ROOT / 'src')]

import mujoco  # noqa: E402
import numpy as np  # noqa: E402

WORLD = ROOT / 'assets' / 'world_p5_candidate_v7_hinged_retainer.xml'
W5 = ROOT / 'assets' / 'world_w5_h085.xml'
TRAY = 'c_payload'
SUPPORT_ROLLERS = ('c_fixed_roller_0_0', 'c_fixed_roller_0_1')
PLACE_X = 4.45372
PLACE_Y = 0.0
DROP_M = 0.0            # v2: the tray is placed AT REST -- H2's settled state
CARRY_LIMIT_M = 0.005   # H2's band_carry value, reused verbatim
QUIET_MAX_RAD_S = 0.05
SEAM_HEIGHT_TOL_M = 0.002
SEAM_GAP_MAX_M = 0.020
SIM_SECONDS = 10.0
SAMPLE_HZ = 50
PREFIX = ('c_fixed_roller_', 'c_recv_roller_', 'c_deck')
FREE_EXTRA_PREFIX = ('c_retainer', 'c2_deck', 'c_deck')


def band_collection(model):
    bodies = [b for b in range(model.nbody)
              if (model.body(b).name or '').startswith(PREFIX)]
    joints = []
    for b in bodies:
        for j in range(int(model.body_jntadr[b]), int(model.body_jntadr[b]) + int(model.body_jntnum[b])):
            r = None
            for g in range(int(model.body_geomadr[b]), int(model.body_geomadr[b]) + int(model.body_geomnum[b])):
                if int(model.geom_type[g]) == int(mujoco.mjtGeom.mjGEOM_CYLINDER):
                    r = max(r or 0.0, float(model.geom_size[g][0]))
            joints.append({'joint': j, 'name': model.joint(j).name, 'body': model.body(b).name,
                           'radius_m': r})
    return bodies, joints


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--run-id', required=True)
    args = ap.parse_args()
    out = ROOT / 'reports' / args.run_id
    if out.exists() and any(out.iterdir()):
        raise SystemExit('REFUSED: %s already holds evidence; pick a new --run-id' % out)
    out.mkdir(parents=True)
    started = time.monotonic()
    report = {'probe': 'G4 v7 seam v2: at-rest state at the declared place point',
              'scope': 'DIAGNOSTIC_ONLY', 'status': 'ERROR', 'command': sys.argv,
              'source_sha256': hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
              'world': str(WORLD.relative_to(ROOT)),
              'world_sha256': hashlib.sha256(WORLD.read_bytes()).hexdigest(),
              'w5_ref_sha256': hashlib.sha256(W5.read_bytes()).hexdigest(),
              'constants': dict(drop_m=DROP_M, carry_limit_m=CARRY_LIMIT_M,
                                quiet_max_rad_s=QUIET_MAX_RAD_S, sim_seconds=SIM_SECONDS,
                                seam_height_tol_m=SEAM_HEIGHT_TOL_M, seam_gap_max_m=SEAM_GAP_MAX_M),
              'prior_evidence': ['reports/p4-belt-g4-seam-01 (sled regime, preserved)',
                                 'reports/p4-belt-g4-h2v7-01 (the judged event)']}
    try:
        model = mujoco.MjModel.from_xml_path(str(WORLD))
        data = mujoco.MjData(model)
        mujoco.mj_resetDataKeyframe(model, data, 0)
        data.ctrl[:] = model.key_ctrl[0]
        mujoco.mj_forward(model, data)

        w5_model = mujoco.MjModel.from_xml_path(str(W5))
        w5_bodies, _ = band_collection(w5_model)
        w5_names = {w5_model.body(b).name for b in w5_bodies}
        bodies, joints = band_collection(model)
        report['collection_sizes'] = {'v7_bodies': len(bodies), 'w5_bodies': len(w5_bodies)}
        for jrec in joints:
            jrec['qadr'] = int(model.jnt_qposadr[jrec['joint']])
            jrec['vadr'] = int(model.jnt_dofadr[jrec['joint']])
            jrec['in_w5_scope'] = jrec['body'] in w5_names

        def body_free(bname):
            return bname.startswith(PREFIX) or bname.startswith(FREE_EXTRA_PREFIX)

        tray_bid = model.body(TRAY).id
        tray_jadr = int(model.body_jntadr[tray_bid])
        assert int(model.jnt_type[tray_jadr]) == int(mujoco.mjtJoint.mjJNT_FREE)
        tray_qadr = int(model.jnt_qposadr[tray_jadr])
        tray_vadr = int(model.jnt_dofadr[tray_jadr])

        pinned_qadr, pinned_vadr = [], []
        for j in range(model.njnt):
            b = int(model.jnt_bodyid[j])
            if j == tray_jadr or body_free(model.body(b).name or ''):
                continue
            pinned_qadr.append(int(model.jnt_qposadr[j]))
            pinned_vadr.append(int(model.jnt_dofadr[j]))
        report['pinning'] = {'pinned_joints': len(pinned_qadr),
                             'free_joints': model.njnt - len(pinned_qadr),
                             'note': 'same declared boundary condition as seam-01'}

        surf = None
        support_info = {}
        for nm in SUPPORT_ROLLERS:
            b = model.body(nm).id
            r = max((float(model.geom_size[g][0]) for g in
                     range(int(model.body_geomadr[b]), int(model.body_geomadr[b]) + int(model.body_geomnum[b]))
                     if int(model.geom_type[g]) == int(mujoco.mjtGeom.mjGEOM_CYLINDER)), default=0.0)
            support_info[nm] = {'center_z': float(data.xpos[b][2]), 'radius_m': r}
            surf = max(surf or 0.0, float(data.xpos[b][2]) + r)
        tray_half_z = max(float(model.geom_size[g][2]) for g in
                          range(int(model.body_geomadr[tray_bid]),
                                int(model.body_geomadr[tray_bid]) + int(model.body_geomnum[tray_bid]))
                          if int(model.geom_type[g]) == int(mujoco.mjtGeom.mjGEOM_BOX))
        rest_center_z = surf + tray_half_z
        report['surface'] = {'support_rollers': support_info, 'derived_surface_z': surf,
                             'tray_half_z_m': tray_half_z, 'rest_center_z': rest_center_z}

        home_qpos = np.array(data.qpos, float)
        data.qpos[tray_qadr:tray_qadr + 3] = [PLACE_X, PLACE_Y, rest_center_z + DROP_M]
        data.qvel[tray_vadr:tray_vadr + 6] = 0.0
        mujoco.mj_forward(model, data)
        report['init_qpos_writes'] = {'tray': 1, 'drop_height_m': DROP_M,
                                      'place_xy': [PLACE_X, PLACE_Y],
                                      'note': 'the settled state H2 leaves behind, hands absent'}

        band_qadr = np.array([jrec['qadr'] for jrec in joints])
        band_vadr = np.array([jrec['vadr'] for jrec in joints])
        qpos0 = np.array(data.qpos[band_qadr], float)
        max_rot = np.zeros(len(joints))
        peak_vel = np.zeros(len(joints))
        dt = float(model.opt.timestep)
        every = max(1, int(round((1.0 / SAMPLE_HZ) / dt)))
        steps = int(SIM_SECONDS / dt)
        tray_trace = []
        for k in range(steps):
            data.qpos[pinned_qadr] = home_qpos[pinned_qadr]
            data.qvel[pinned_vadr] = 0.0
            mujoco.mj_step(model, data)
            peak_vel = np.maximum(peak_vel, np.abs(data.qvel[band_vadr]))
            if k % every == 0:
                max_rot = np.maximum(max_rot, np.abs(data.qpos[band_qadr] - qpos0))
                tray_trace.append([round(k * dt, 4), float(data.qpos[tray_qadr]),
                                   float(data.qpos[tray_qadr + 1]), float(data.qpos[tray_qadr + 2])])
        report['tray_trace_hz50'] = tray_trace

        for jrec, rot, vel in zip(joints, max_rot, peak_vel):
            jrec['max_rotation_rad'] = float(rot)
            jrec['peak_speed_rad_s'] = float(vel)
            jrec['own_surface_travel_m'] = (float(rot) * jrec['radius_m']) \
                if jrec['radius_m'] is not None else None
        ranked = sorted(joints, key=lambda r: -r['max_rotation_rad'])
        report['top_movers'] = [
            {k: r[k] for k in ('name', 'body', 'in_w5_scope', 'radius_m',
                               'max_rotation_rad', 'peak_speed_rad_s', 'own_surface_travel_m')}
            for r in ranked[:10]]
        worst = ranked[0]

        tray_geoms = {g for g in range(model.ngeom) if int(model.geom_bodyid[g]) == tray_bid}
        census = {}
        for c in range(data.ncon):
            con = data.contact[c]
            for g1, g2 in ((int(con.geom1), int(con.geom2)), (int(con.geom2), int(con.geom1))):
                if g1 in tray_geoms:
                    bn = model.body(int(model.geom_bodyid[g2])).name
                    if bn.startswith(PREFIX):
                        census[bn] = census.get(bn, 0) + 1
        report['support_census_end'] = census

        excursion = max(float(np.hypot(r[1] - PLACE_X, r[2] - PLACE_Y)) for r in tray_trace)
        report['tray_xy_excursion_from_declared_m'] = excursion

        # seam geometry (identical static method to v1)
        def mains(prefix):
            return [(model.body(b).name, float(data.xpos[model.body(b).id][0]),
                     float(data.xpos[model.body(b).id][2]))
                    for b in bodies
                    if (model.body(b).name or '').startswith(prefix)
                    and '_idler' not in (model.body(b).name or '')]
        fixed = sorted(mains('c_fixed_roller_'), key=lambda r: r[1])
        deck = sorted(mains('c_deck_roller'), key=lambda r: r[1])
        pitches = [b[1] - a[1] for a, b in zip(fixed, fixed[1:])]
        normal_pitch = float(np.median(pitches))
        seam_pitch = deck[0][1] - fixed[-1][1]
        rr = support_info[SUPPORT_ROLLERS[0]]['radius_m']
        report['seam_geometry'] = {
            'last_source_roller': fixed[-1][0], 'first_deck_roller': deck[0][0],
            'normal_pitch_m': normal_pitch, 'seam_pitch_m': seam_pitch,
            'seam_surface_gap_m': seam_pitch - 2.0 * rr,
            'normal_surface_gap_m': normal_pitch - 2.0 * rr,
            'height_last_m': fixed[-1][2], 'height_first_m': deck[0][2],
            'height_delta_m': abs(deck[0][2] - fixed[-1][2])}

        rows = []

        def add(name, ok, detail, falsified_by=None):
            rows.append({'name': name, 'status': 'PASS' if ok else 'FAIL', 'detail': detail,
                         'falsified_by': falsified_by})

        peak_any = float(peak_vel.max())
        add('at_rest_state_is_quiet', peak_any < QUIET_MAX_RAD_S,
            'peak band-joint speed over %d s at rest %.6f rad/s (limit %.2f); the resting '
            'state does not drive anything' % (SIM_SECONDS, peak_any, QUIET_MAX_RAD_S),
            falsified_by='a resting tray that spins rollers would be a live transfer hazard')
        add('tray_static_at_declared_point', excursion <= CARRY_LIMIT_M,
            'tray xy excursion from the declared point %.6f m over the run (limit %.4f m)'
            % (excursion, CARRY_LIMIT_M),
            falsified_by='the tray drifting after settle would be a real transfer defect')
        both_mains = all(census.get(nm, 0) > 0 for nm in SUPPORT_ROLLERS)
        add('supported_on_designated_rollers', bool(both_mains),
            'settled-tray contact census: %s (both designated mains in contact: %s)'
            % (json.dumps(census, sort_keys=True), both_mains),
            falsified_by='resting on anything but the designated pair would change the '
                         'transfer claim')
        w5_scope = [j for j in joints if j['in_w5_scope']]
        worst_w5 = max(w5_scope, key=lambda j: -1.0 if j['own_surface_travel_m'] is None
                       else j['own_surface_travel_m'])
        add('w5_scope_rollers_held',
            worst_w5['own_surface_travel_m'] is not None
            and worst_w5['own_surface_travel_m'] <= CARRY_LIMIT_M,
            'worst W5-scope joint %s: %.6f rad -> %.6f m own-radius surface travel '
            '(limit %.4f m); %d joints in scope'
            % (worst_w5['name'], worst_w5['max_rotation_rad'],
               worst_w5['own_surface_travel_m'] or 0.0, CARRY_LIMIT_M, len(w5_scope)),
            falsified_by='w5-era source hardware moving at rest would be a real source defect')
        add('worst_mover_reported', True,
            'worst full-collection mover %s (w5 scope: %s): %.6f rad, r=%s'
            % (worst['name'], worst['in_w5_scope'], worst['max_rotation_rad'],
               worst['radius_m']),
            falsified_by='reporting row; the falsifier lives in the three rows above')
        seam = report['seam_geometry']
        add('seam_geometry_continuous',
            abs(seam['height_delta_m']) <= SEAM_HEIGHT_TOL_M
            and seam['seam_surface_gap_m'] <= SEAM_GAP_MAX_M,
            'seam pitch %.4f m (normal %.4f), surface gap %.4f m (normal %.4f, limit %.3f); '
            'heights %+.5f m (tol %.3f)'
            % (seam['seam_pitch_m'], seam['normal_pitch_m'], seam['seam_surface_gap_m'],
               seam['normal_surface_gap_m'], SEAM_GAP_MAX_M, seam['height_delta_m'],
               SEAM_HEIGHT_TOL_M),
            falsified_by='a gap or height step beyond bounds would snag a crossing tray')

        report['rows'] = rows
        report['findings'] = [
            {'finding': 'the H2 source_stopped FAIL stands as recorded',
             'detail': 'v1 + the H2 contact log attribute it: the spike happened while the '
                       'left hand was still touching rollers; the tray itself moved 30 um. '
                       'This probe qualifies the at-rest state; it edits no machine.'},
            {'finding': 'v1 was a sled test, preserved as evidence',
             'detail': 'its 10 mm free fall (0.44 m/s) is NOT the H2 release (the tray was '
                       'already at rest when the hands left). Its FAIL rows measure v1\'s '
                       'own overload; the idler-dominance attribution transfers.'},
        ]
        failed = [r['name'] for r in rows if r['status'] == 'FAIL']
        report['overall'] = ('PASS: all %d rows' % len(rows)) if not failed else \
            ('FAIL: %d of %d rows (%s)' % (len(failed), len(rows), ', '.join(failed)))
        report['status'] = 'DIAGNOSTIC_COMPLETED'
    except Exception as exc:
        report['error'] = '%s: %s' % (type(exc).__name__, exc)
        if report.get('status') in (None, 'ERROR'):
            report['status'] = 'POST_RUN_ERROR'
    finally:
        report['wall_seconds'] = time.monotonic() - started
        (out / 'report.json').write_text(json.dumps(report, indent=2) + '\n', encoding='utf-8')
        printable = {k: v for k, v in report.items()
                     if k not in ('rows', 'tray_trace_hz50')}
        print(json.dumps(printable, indent=2, ensure_ascii=False), flush=True)
        for r in report.get('rows') or []:
            print('%-38s %-5s %s' % (r['name'], r['status'], r['detail']), flush=True)
        print('OVERALL: %s' % report.get('overall'), flush=True)
    return 0 if report['status'] == 'DIAGNOSTIC_COMPLETED' else 1


if __name__ == '__main__':
    raise SystemExit(main())
