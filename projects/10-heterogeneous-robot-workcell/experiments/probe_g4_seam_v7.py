#!/usr/bin/env python3
"""G4 seam probe: v7 source-roller/deck seam + attribution of the H2 landing kick.

CONTEXT (measured, 2026-10-04): in `p4-belt-g4-h2v7-01` the frozen H2 machine's
`source_stopped` row FAILED -- worst band-joint rotation 0.176 rad, peak speed
1.2319 rad/s, surface travel 6.17 mm at the MAIN roller radius 0.035 (limit 5 mm).
The mother run `p4-h2-w5-03` measured 0.0006 rad on the same machine. The event is
time-localized: the band is dead still (1e-10 rad/s) until the tray lands/being
released at t=34.1-34.3, then a spike with a slow coast afterwards.

ATTRIBUTION HYPOTHESIS this probe tests: the row collects band bodies by name
prefix (c_fixed_roller_/c_recv_roller_/c_deck); w5 matches 47 bodies, v7 matches
151. The 104 new bodies are almost all small IDLER rollers (y=+-0.10, z=0.80),
and the pair c_fixed_roller_idler_0_side+-1 sits at x=4.4537 -- exactly under the
declared tray place point, where w5 had nothing. The row converts EVERY joint's
rotation at the MAIN roller radius, so a small idler's surface travel is
overestimated ~3.5x. If the kick is taken by idlers, the w5-era source hardware
(which is what the row was written to judge) did not move.

METHOD: reproduce the landing mechanics in isolation. The tray c_payload is put
at the declared place point (x=4.45372, y=0) 10 mm above its rest surface -- the
drop height is H2's own measured place gap (9.99 mm) -- and falls freely onto
the rollers. The humanoid and the arm are PINNED at the home keyframe (declared
boundary condition: without a controller they sag/collapse and would shake the
scene from 0.35 m away; the pinned masses take no part in the impact). Band,
deck, retainer and tray evolve freely with ctrl = key_ctrl[0], exactly as in H2.
Every collected joint is recorded at 50 Hz and attributed BY NAME.

WHAT IS JUDGED (no threshold of any frozen machine is touched; the H2 FAIL stands):
  1. the drop reproduces a landing kick (some joint exceeds 0.05 rad/s);
  2. W5-SCOPE joints (names that exist in w5's 47-body collection) each stay
     within 5 mm OWN-radius surface travel -- the original intent of the row;
  3. the worst full-collection mover is identified and its own-radius surface
     travel reported next to the machine's 0.035-radius conversion;
  4. the tray is not carried by the impact (post-touchdown xy excursion <= 5 mm,
     the H2 band_carry value);
  5. the tray is actually supported at the declared point (contact census);
  6. seam geometry: source-band-end -> deck pitch/surface gap/height alignment.
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
DROP_M = 0.010          # H2's measured place gap, reused verbatim
CARRY_LIMIT_M = 0.005   # H2's band_carry value, reused verbatim
KICK_MIN_RAD_S = 0.05   # below this the drop did not reproduce a landing event
SEAM_HEIGHT_TOL_M = 0.002
SEAM_GAP_MAX_M = 0.020
SIM_SECONDS = 20.0
SAMPLE_HZ = 50
PREFIX = ('c_fixed_roller_', 'c_recv_roller_', 'c_deck')
FREE_EXTRA_PREFIX = ('c_retainer', 'c2_deck', 'c_deck')  # seam hardware stays free


def band_collection(model):
    """(bodies, joints) matching the H2 row's name predicate, with per-joint radius."""
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
    report = {'probe': 'G4 v7 seam + landing-kick attribution (isolated drop)',
              'scope': 'DIAGNOSTIC_ONLY', 'status': 'ERROR', 'command': sys.argv,
              'source_sha256': hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
              'world': str(WORLD.relative_to(ROOT)),
              'world_sha256': hashlib.sha256(WORLD.read_bytes()).hexdigest(),
              'w5_ref_sha256': hashlib.sha256(W5.read_bytes()).hexdigest(),
              'constants': dict(drop_m=DROP_M, carry_limit_m=CARRY_LIMIT_M,
                                kick_min_rad_s=KICK_MIN_RAD_S, sim_seconds=SIM_SECONDS,
                                seam_height_tol_m=SEAM_HEIGHT_TOL_M,
                                seam_gap_max_m=SEAM_GAP_MAX_M)}
    try:
        model = mujoco.MjModel.from_xml_path(str(WORLD))
        data = mujoco.MjData(model)
        mujoco.mj_resetDataKeyframe(model, data, 0)
        data.ctrl[:] = model.key_ctrl[0]
        mujoco.mj_forward(model, data)

        w5_model = mujoco.MjModel.from_xml_path(str(W5))
        w5_bodies, _ = band_collection(w5_model)
        w5_names = {w5_model.body(b).name for b in w5_bodies}
        report['collection_sizes'] = {'v7_bodies': None, 'w5_bodies': len(w5_bodies)}

        bodies, joints = band_collection(model)
        report['collection_sizes']['v7_bodies'] = len(bodies)
        for jrec in joints:
            jrec['qadr'] = int(model.jnt_qposadr[jrec['joint']])
            jrec['vadr'] = int(model.jnt_dofadr[jrec['joint']])
            jrec['in_w5_scope'] = jrec['body'] in w5_names

        # free set = band-collection joints + tray + seam hardware (declared)
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
            bname = model.body(b).name or ''
            if j == tray_jadr or body_free(bname):
                continue
            pinned_qadr.append(int(model.jnt_qposadr[j]))
            pinned_vadr.append(int(model.jnt_dofadr[j]))
        report['pinning'] = {
            'pinned_joints': len(pinned_qadr),
            'free_joints': model.njnt - len(pinned_qadr),
            'note': 'humanoid/arm/station pinned at the home keyframe (qpos+qvel reset each '
                    'step); band, deck, retainer, c2 deck and the tray stay free with '
                    'ctrl=key_ctrl[0], exactly as H2 left them'}

        # tray rest surface, derived from the support rollers' own geometry
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

        # DECLARED initialization qpos write: tray at the place point, DROP_M above rest
        home_qpos = np.array(data.qpos, float)
        data.qpos[tray_qadr:tray_qadr + 3] = [PLACE_X, PLACE_Y, rest_center_z + DROP_M]
        data.qvel[tray_vadr:tray_vadr + 6] = 0.0
        mujoco.mj_forward(model, data)
        report['init_qpos_writes'] = {
            'tray': 1, 'drop_height_m': DROP_M,
            'place_xy': [PLACE_X, PLACE_Y],
            'note': 'the H2-run equivalent of the hands letting go from the measured 9.99 mm gap'}

        band_qadr = np.array([jrec['qadr'] for jrec in joints])
        band_vadr = np.array([jrec['vadr'] for jrec in joints])
        qpos0 = np.array(data.qpos[band_qadr], float)
        max_rot = np.zeros(len(joints))
        peak_vel = np.zeros(len(joints))
        dt = float(model.opt.timestep)
        every = max(1, int(round((1.0 / SAMPLE_HZ) / dt)))
        steps = int(SIM_SECONDS / dt)
        tray_xy = []
        touchdown_t = None
        t = 0.0
        for k in range(steps):
            data.qpos[pinned_qadr] = home_qpos[pinned_qadr]
            data.qvel[pinned_vadr] = 0.0
            mujoco.mj_step(model, data)
            t += dt
            peak_vel = np.maximum(peak_vel, np.abs(data.qvel[band_vadr]))
            if k % every == 0:
                max_rot = np.maximum(max_rot, np.abs(data.qpos[band_qadr] - qpos0))
                tray_xy.append([t, float(data.qpos[tray_qadr]), float(data.qpos[tray_qadr + 1]),
                                float(data.qpos[tray_qadr + 2])])
                if touchdown_t is None and data.qpos[tray_qadr + 2] <= rest_center_z + 0.001:
                    touchdown_t = t

        report['touchdown_t'] = touchdown_t
        for jrec, rot, vel in zip(joints, max_rot, peak_vel):
            jrec['max_rotation_rad'] = float(rot)
            jrec['peak_speed_rad_s'] = float(vel)
            # slide joints (e.g. c_deck) own no cylinder: surface travel is undefined, not zero
            jrec['own_surface_travel_m'] = (float(rot) * jrec['radius_m']) \
                if jrec['radius_m'] is not None else None
        ranked = sorted(joints, key=lambda r: -r['max_rotation_rad'])
        report['top_movers'] = [
            {k: r[k] for k in ('name', 'body', 'in_w5_scope', 'radius_m',
                               'max_rotation_rad', 'peak_speed_rad_s', 'own_surface_travel_m')}
            for r in ranked[:10]]
        worst = ranked[0]
        machine_conversion_m = float(worst['max_rotation_rad']) * 0.035
        report['worst_mover'] = {
            'name': worst['name'], 'in_w5_scope': worst['in_w5_scope'],
            'radius_m': worst['radius_m'],
            'max_rotation_rad': worst['max_rotation_rad'],
            'own_surface_travel_m': worst['own_surface_travel_m'],
            'machine_035_conversion_m': machine_conversion_m,
            'h2_event_reference': {'max_rotation_rad': 0.17637244391944118,
                                   'peak_speed_rad_s': 1.231915746563952,
                                   'machine_035_conversion_m': 0.006173035537180442}}

        # contact census at the end: which band bodies are under the settled tray
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

        # tray drift after touchdown
        drift = None
        if touchdown_t is not None:
            post = [row for row in tray_xy if row[0] >= touchdown_t]
            ref = post[0]
            drift = max(float(np.hypot(r[1] - ref[1], r[2] - ref[2])) for r in post)
        report['tray_post_touchdown_xy_excursion_m'] = drift

        # seam geometry (static, from the same loaded model)
        fixed_x = sorted(float(data.xpos[model.body(b).id][0]) for b in bodies
                         if (model.body(b).name or '').startswith('c_fixed_roller_')
                         and '_idler' not in (model.body(b).name or ''))
        deck_main_x = sorted(float(data.xpos[model.body(b).id][0]) for b in bodies
                             if (model.body(b).name or '').startswith('c_deck_roller')
                             and '_idler' not in (model.body(b).name or ''))
        pitches = [b - a for a, b in zip(fixed_x, fixed_x[1:])]
        normal_pitch = float(np.median(pitches))
        seam_pitch = deck_main_x[0] - fixed_x[-1]
        lastname = [model.body(b).name for b in bodies
                    if (model.body(b).name or '').startswith('c_fixed_roller_')
                    and '_idler' not in (model.body(b).name or '')][-1]
        firstdeck = [model.body(b).name for b in bodies
                     if (model.body(b).name or '').startswith('c_deck_roller')
                     and '_idler' not in (model.body(b).name or '')][0]
        z_last = float(data.xpos[model.body(lastname).id][2])
        z_first = float(data.xpos[model.body(firstdeck).id][2])
        rr = support_info[SUPPORT_ROLLERS[0]]['radius_m']
        report['seam_geometry'] = {
            'last_source_roller': lastname, 'first_deck_roller': firstdeck,
            'normal_pitch_m': normal_pitch, 'seam_pitch_m': seam_pitch,
            'seam_surface_gap_m': seam_pitch - 2.0 * rr,
            'normal_surface_gap_m': normal_pitch - 2.0 * rr,
            'height_last_m': z_last, 'height_first_m': z_first,
            'height_delta_m': abs(z_first - z_last)}

        # -- rows ------------------------------------------------------------------------
        rows = []

        def add(name, ok, detail, falsified_by=None):
            rows.append({'name': name, 'status': 'PASS' if ok else 'FAIL', 'detail': detail,
                         'falsified_by': falsified_by})

        peak_any = float(peak_vel.max())
        add('drop_reproduces_a_landing_kick', peak_any >= KICK_MIN_RAD_S,
            'peak band-joint speed in the isolated drop %.4f rad/s (need >= %.2f to make the '
            'attribution meaningful)' % (peak_any, KICK_MIN_RAD_S),
            falsified_by='a kick-free drop would mean the H2 spike needed the hands, and this '
                         'instrument cannot attribute it')
        w5_scope = [j for j in joints if j['in_w5_scope']]
        worst_w5 = max(w5_scope, key=lambda j: -1.0 if j['own_surface_travel_m'] is None
                       else j['own_surface_travel_m'])
        add('w5_scope_rollers_held', worst_w5['own_surface_travel_m'] <= CARRY_LIMIT_M,
            'worst W5-scope joint %s: %.5f rad -> %.5f m own-radius surface travel '
            '(limit %.4f m); %d joints in scope'
            % (worst_w5['name'], worst_w5['max_rotation_rad'],
               worst_w5['own_surface_travel_m'], CARRY_LIMIT_M, len(w5_scope)),
            falsified_by='a W5-scope main roller exceeding %.3f m means the source hardware '
                         'itself moved under the landing tray' % CARRY_LIMIT_M)
        add('worst_mover_is_not_w5_scope_hardware', not worst['in_w5_scope'],
            'worst full-collection mover is %s (in w5 scope: %s); %.5f rad at r=%.3f = %.5f m '
            'own-radius vs %.5f m at the machine\'s 0.035 conversion'
            % (worst['name'], worst['in_w5_scope'], worst['max_rotation_rad'],
               worst['radius_m'] or 0.0, worst['own_surface_travel_m'], machine_conversion_m),
            falsified_by='if the worst mover WERE w5-scope hardware, the H2 FAIL would be a '
                         'real source defect, not scope growth')
        add('tray_not_carried_by_impact', drift is not None and drift <= CARRY_LIMIT_M,
            'post-touchdown xy excursion %.5f m (limit %.4f m); touchdown at t=%s s'
            % (drift if drift is not None else float('nan'), CARRY_LIMIT_M, touchdown_t),
            falsified_by='the tray sliding after landing would be a real transfer defect')
        mains_under = [nm for nm in SUPPORT_ROLLERS if census.get(nm, 0) > 0]
        add('tray_supported_at_declared_point', bool(mains_under),
            'settled-tray contact census: %s (designated mains in contact: %d of 2)'
            % (json.dumps(census, sort_keys=True), len(mains_under)),
            falsified_by='the tray resting on nothing designated would invalidate the drop')
        seam = report['seam_geometry']
        add('seam_geometry_continuous',
            abs(seam['height_delta_m']) <= SEAM_HEIGHT_TOL_M
            and seam['seam_surface_gap_m'] <= SEAM_GAP_MAX_M,
            'seam pitch %.4f m (normal %.4f), surface gap %.4f m (normal %.4f, limit %.3f); '
            'heights %+.5f m (tol %.3f)'
            % (seam['seam_pitch_m'], seam['normal_pitch_m'], seam['seam_surface_gap_m'],
               seam['normal_surface_gap_m'], SEAM_GAP_MAX_M, seam['height_delta_m'],
               SEAM_HEIGHT_TOL_M),
            falsified_by='a gap or height step beyond the declared bounds would snag a tray '
                         'crossing the seam')

        report['rows'] = rows
        report['findings'] = [
            {'finding': 'the H2 source_stopped FAIL stands as recorded',
             'detail': 'this probe explains it; it does not overturn it. The frozen machine '
                       'and its 5 mm limit are untouched.'},
            {'finding': 'the machine converts every band joint at the main roller radius',
             'detail': 'probe_h2_w5 takes the FIRST band cylinder (0.035 m) as the universal '
                       'radius; v7 added small idlers, so the conversion overstates their '
                       'surface travel. Recorded as an instrument note, not a machine edit.'},
            {'finding': 'the isolated drop is not the hand release',
             'detail': 'the drop reproduces the 10 mm fall but not the hands touching the tray '
                       'during release; attribution (which hardware moves) transfers, exact '
                       'magnitudes need not match.'},
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
        printable = {k: v for k, v in report.items() if k != 'rows'}
        print(json.dumps(printable, indent=2, ensure_ascii=False), flush=True)
        for r in report.get('rows') or []:
            print('%-40s %-5s %s' % (r['name'], r['status'], r['detail']), flush=True)
        print('OVERALL: %s' % report.get('overall'), flush=True)
    return 0 if report['status'] == 'DIAGNOSTIC_COMPLETED' else 1


if __name__ == '__main__':
    raise SystemExit(main())
