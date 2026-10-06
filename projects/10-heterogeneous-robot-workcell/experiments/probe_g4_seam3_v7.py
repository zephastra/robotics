#!/usr/bin/env python3
"""G4 seam probe v3: the at-rest state, rest pose derived from TRUE geometry.

WHY v3 (revision 2 of the seam declaration): v1 and v2 (`p4-belt-g4-seam-01/-02`,
evidence preserved) both placed the tray with a rest pose derived from
max(box half-z) = 0.030 -- which is the SIDE WALL's half height. The tray's true
lowest geom is its bottom plate at -0.040 in the body frame, so the true rest
origin is z = 0.810 + 0.040 = 0.850 -- exactly the z H2's tray rests at. v2
therefore BURIED the tray 10 mm into the rollers and MuJoCo's penetration
correction exploded it; v1's "wrong rest + 0.010 drop" coincidentally landed on
the true 0.850 tangent pose, sat ~1.5 s, then sank and launched -- unexplained.

v3 changes exactly two things vs v2 (both declared in the amendment):
  1. rest origin derived from the tray's per-geom LOWEST point, cross-checked
     against H2's observed rest z=0.85 as a judged row;
  2. full instrumentation: tray trace at 50 Hz, band peak speed per 0.5 s
     window, and a contact census per 0.5 s window -- if the tray launches
     again, the census names the contacting hardware at that moment.

The pinned humanoid/arm, free band/deck/retainer/tray, ctrl=key_ctrl[0] --
all identical to v1/v2. DROP = 0: this is the state H2 leaves behind.
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
H2_OBSERVED_REST_Z = 0.85   # p4-belt-g4-h2v7-01: the tray's constant z at rest
DROP_M = 0.0
CARRY_LIMIT_M = 0.005
QUIET_MAX_RAD_S = 0.05
POSE_TOL_M = 0.005
SEAM_HEIGHT_TOL_M = 0.002
SEAM_GAP_MAX_M = 0.020
SIM_SECONDS = 10.0
SAMPLE_HZ = 50
WINDOW_S = 0.5
PREFIX = ('c_fixed_roller_', 'c_recv_roller_', 'c_deck')
FREE_EXTRA_PREFIX = ('c_retainer', 'c2_deck', 'c_deck')


def geom_lowest_z(model, g):
    """Lowest point of geom g in its BODY frame (exact for box/cylinder/sphere)."""
    t = int(model.geom_type[g])
    pz = float(model.geom_pos[g][2])
    s = np.asarray(model.geom_size[g])
    if t == int(mujoco.mjtGeom.mjGEOM_BOX):
        return pz - float(s[2])
    if t == int(mujoco.mjtGeom.mjGEOM_CYLINDER) or t == int(mujoco.mjtGeom.mjGEOM_CAPSULE):
        return pz - float(s[1])
    if t == int(mujoco.mjtGeom.mjGEOM_SPHERE):
        return pz - float(s[0])
    return pz - float(model.geom_rbound[g])   # meshes: conservative bound


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
    report = {'probe': 'G4 v7 seam v3: at-rest state, true-geometry rest pose',
              'scope': 'DIAGNOSTIC_ONLY', 'status': 'ERROR', 'command': sys.argv,
              'source_sha256': hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
              'world': str(WORLD.relative_to(ROOT)),
              'world_sha256': hashlib.sha256(WORLD.read_bytes()).hexdigest(),
              'w5_ref_sha256': hashlib.sha256(W5.read_bytes()).hexdigest(),
              'constants': dict(drop_m=DROP_M, carry_limit_m=CARRY_LIMIT_M,
                                quiet_max_rad_s=QUIET_MAX_RAD_S, pose_tol_m=POSE_TOL_M,
                                sim_seconds=SIM_SECONDS, window_s=WINDOW_S,
                                seam_height_tol_m=SEAM_HEIGHT_TOL_M, seam_gap_max_m=SEAM_GAP_MAX_M),
              'prior_evidence': ['reports/p4-belt-g4-seam-01 (sled regime, preserved)',
                                 'reports/p4-belt-g4-seam-02 (buried pose, preserved)',
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
                             'note': 'same declared boundary condition as v1/v2'}

        surf = None
        support_info = {}
        for nm in SUPPORT_ROLLERS:
            b = model.body(nm).id
            r = max((float(model.geom_size[g][0]) for g in
                     range(int(model.body_geomadr[b]), int(model.body_geomadr[b]) + int(model.body_geomnum[b]))
                     if int(model.geom_type[g]) == int(mujoco.mjtGeom.mjGEOM_CYLINDER)), default=0.0)
            support_info[nm] = {'center_z': float(data.xpos[b][2]), 'radius_m': r}
            surf = max(surf or 0.0, float(data.xpos[b][2]) + r)
        tray_geoms = list(range(int(model.body_geomadr[tray_bid]),
                                int(model.body_geomadr[tray_bid]) + int(model.body_geomnum[tray_bid])))
        lowest = min(geom_lowest_z(model, g) for g in tray_geoms)
        rest_center_z = surf - lowest
        report['surface'] = {'support_rollers': support_info, 'derived_surface_z': surf,
                             'tray_lowest_body_frame_m': lowest,
                             'derived_rest_center_z': rest_center_z,
                             'h2_observed_rest_z': H2_OBSERVED_REST_Z}
        report['tray_geoms'] = [
            {'geom': g, 'type': int(model.geom_type[g]),
             'pos': [float(v) for v in model.geom_pos[g]],
             'size': [float(v) for v in model.geom_size[g]],
             'lowest_body_frame': geom_lowest_z(model, g)} for g in tray_geoms]

        home_qpos = np.array(data.qpos, float)
        data.qpos[tray_qadr:tray_qadr + 3] = [PLACE_X, PLACE_Y, rest_center_z + DROP_M]
        data.qvel[tray_vadr:tray_vadr + 6] = 0.0
        mujoco.mj_forward(model, data)
        report['init_qpos_writes'] = {'tray': 1, 'drop_height_m': DROP_M,
                                      'place_xy': [PLACE_X, PLACE_Y],
                                      'note': 'true tangent rest, hands absent'}

        band_qadr = np.array([jrec['qadr'] for jrec in joints])
        band_vadr = np.array([jrec['vadr'] for jrec in joints])
        qpos0 = np.array(data.qpos[band_qadr], float)
        max_rot = np.zeros(len(joints))
        peak_vel = np.zeros(len(joints))
        dt = float(model.opt.timestep)
        every = max(1, int(round((1.0 / SAMPLE_HZ) / dt)))
        wstep = max(1, int(round(WINDOW_S / dt)))
        steps = int(SIM_SECONDS / dt)
        tray_trace = []
        speed_windows = []
        census_windows = []
        tray_geoms_set = set(tray_geoms)

        def census_now():
            cen = {}
            for c in range(data.ncon):
                con = data.contact[c]
                for g1, g2 in ((int(con.geom1), int(con.geom2)), (int(con.geom2), int(con.geom1))):
                    if g1 in tray_geoms_set:
                        bn = model.body(int(model.geom_bodyid[g2])).name
                        if bn.startswith(PREFIX):
                            cen[bn] = cen.get(bn, 0) + 1
            return cen

        win_peak = 0.0
        for k in range(steps):
            data.qpos[pinned_qadr] = home_qpos[pinned_qadr]
            data.qvel[pinned_vadr] = 0.0
            mujoco.mj_step(model, data)
            pv = float(np.max(np.abs(data.qvel[band_vadr])))
            peak_vel = np.maximum(peak_vel, np.abs(data.qvel[band_vadr]))
            win_peak = max(win_peak, pv)
            if k % every == 0:
                max_rot = np.maximum(max_rot, np.abs(data.qpos[band_qadr] - qpos0))
                tray_trace.append([round(k * dt, 4), float(data.qpos[tray_qadr]),
                                   float(data.qpos[tray_qadr + 1]), float(data.qpos[tray_qadr + 2])])
            if (k + 1) % wstep == 0:
                speed_windows.append({'t_end': round((k + 1) * dt, 3), 'peak_rad_s': win_peak})
                census_windows.append({'t_end': round((k + 1) * dt, 3), 'contacts': census_now()})
                win_peak = 0.0
        report['tray_trace_hz50'] = tray_trace
        report['band_speed_windows'] = speed_windows
        report['support_census_windows'] = census_windows

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

        census_end = census_now()
        report['support_census_end'] = census_end
        excursion = max(float(np.hypot(r[1] - PLACE_X, r[2] - PLACE_Y)) for r in tray_trace)
        report['tray_xy_excursion_from_declared_m'] = excursion

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

        pose_err = abs(rest_center_z - H2_OBSERVED_REST_Z)
        add('tray_pose_matches_h2_rest', pose_err <= POSE_TOL_M,
            'derived rest origin z %.5f m vs H2 observed %.3f m (delta %.5f, tol %.3f); '
            'tray lowest point %.3f m in body frame over %d geoms'
            % (rest_center_z, H2_OBSERVED_REST_Z, pose_err, POSE_TOL_M, lowest, len(tray_geoms)),
            falsified_by='a rest pose that disagrees with the judged run would invalidate '
                         'this instrument')
        peak_any = float(peak_vel.max())
        add('at_rest_state_is_quiet', peak_any < QUIET_MAX_RAD_S,
            'peak band-joint speed over %d s at rest %.6f rad/s (limit %.2f); windows: %s'
            % (SIM_SECONDS, peak_any, QUIET_MAX_RAD_S,
               json.dumps(speed_windows[:6])))
        add('tray_static_at_declared_point', excursion <= CARRY_LIMIT_M,
            'tray xy excursion from the declared point %.6f m over the run (limit %.4f m)'
            % (excursion, CARRY_LIMIT_M),
            falsified_by='the tray drifting or being carried after settle would be a real '
                         'transfer defect')
        both_mains = all(census_end.get(nm, 0) > 0 for nm in SUPPORT_ROLLERS)
        add('supported_on_designated_rollers', bool(both_mains),
            'end contact census: %s (both designated mains: %s)'
            % (json.dumps(census_end, sort_keys=True), both_mains),
            falsified_by='resting on anything but the designated pair would change the claim')
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
            falsified_by='w5-era source hardware moving at rest would be a real defect')
        add('worst_mover_reported', True,
            'worst full-collection mover %s (w5 scope: %s): %.6f rad, r=%s'
            % (worst['name'], worst['in_w5_scope'], worst['max_rotation_rad'],
               worst['radius_m']),
            falsified_by='reporting row; falsifiers live in the rows above')
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
            {'finding': 'v1/v2 placement error, measured and fixed here',
             'detail': 'the tray bottom plate is 0.040 m below its body origin (the walls '
                       'are 0.030); v2 buried it 10 mm and the penetration correction '
                       'exploded it; v1\'s offset drop landed on the same tangent pose and '
                       'its t~1.5 s launch is what this run re-tests with full instrumentation.'},
            {'finding': 'the H2 source_stopped FAIL stands as recorded',
             'detail': 'attributed from the H2 contact log: the spike happened while the '
                       'left hand still touched rollers; the tray moved 30 um. No machine '
                       'or threshold was edited.'},
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
