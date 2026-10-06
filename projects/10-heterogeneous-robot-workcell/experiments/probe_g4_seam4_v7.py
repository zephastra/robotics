#!/usr/bin/env python3
"""G4 seam probe v4: WHO injects the energy? Three-arm paired experiment.

CONTEXT (revision 3 of the seam declaration): seam-03 (`p4-belt-g4-seam-03`)
reproduced the launch from the true tangent rest pose (excursion 80.67 m,
4/7 rows FAIL) AND proved the rest-pose derivation exact (0.85000 vs H2's
0.850, delta 0.00000). A world dump then showed every band actuator ctrl in
key_ctrl[0] is 0.0 -- the belt is NOT driven -- while the v7-only idlers
(r=0.01, damping/frictionloss/armature all 0.0, no actuator) sit EXACTLY flush
with the mains (tops 0.810) at x=4.4537, y=+-0.10, directly under the tray.
Once spun (peak 92.8 rad/s) they coast at 83.9 rad/s for the remaining 9 s:
an undamped energy ratchet under a metastable 4-line support.

Two suspects remain BEFORE convicting the world:
  (a) the probe's own pinning (85 joints clamped by qpos/qvel reset each step)
      while LARGE actuator forces (hips 231, base actuator 255) act on those
      clamped dofs -- a possible solver-pumping instrument artifact;
  (b) the key_ctrl[0] drives themselves (retainer drives at 1.5708 rad, arm
      joint targets) acting on free bodies.

Three arms, same placement (DROP=0 tangent rest at the H2 place point), 6 s each:
  A  clamp + ctrl=key_ctrl[0]   reproduction control (seam-03 deterministic)
  B  clamp + ctrl=0             drives excluded; launch => contact-gravity world-side
  C  no clamp + ctrl=key_ctrl[0] clamp excluded; launch => world-side realistically
Guard for C: the humanoid must hold standing (base z drop <= 0.10 m) or arm C
is inconclusive (reported, not silently used).
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
H2_OBSERVED_REST_Z = 0.85
DROP_M = 0.0
CARRY_LIMIT_M = 0.005
SIM_SECONDS = 6.0
SAMPLE_HZ = 50
STAND_DROP_MAX_M = 0.10
WATCH_IDLER = 'c_fixed_roller_idler_0_side1_joint'
BASE_JOINT = 'h_base_free'
PREFIX = ('c_fixed_roller_', 'c_recv_roller_', 'c_deck')
FREE_EXTRA_PREFIX = ('c_retainer', 'c2_deck', 'c_deck')


def geom_lowest_z(model, g):
    t = int(model.geom_type[g])
    pz = float(model.geom_pos[g][2])
    s = np.asarray(model.geom_size[g])
    if t == int(mujoco.mjtGeom.mjGEOM_BOX):
        return pz - float(s[2])
    if t == int(mujoco.mjtGeom.mjGEOM_CYLINDER) or t == int(mujoco.mjtGeom.mjGEOM_CAPSULE):
        return pz - float(s[1])
    if t == int(mujoco.mjtGeom.mjGEOM_SPHERE):
        return pz - float(s[0])
    return pz - float(model.geom_rbound[g])


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--run-id', required=True)
    args = ap.parse_args()
    out = ROOT / 'reports' / args.run_id
    if out.exists() and any(out.iterdir()):
        raise SystemExit('REFUSED: %s already holds evidence; pick a new --run-id' % out)
    out.mkdir(parents=True)
    started = time.monotonic()
    report = {'probe': 'G4 v7 seam v4: three-arm energy-source localization',
              'scope': 'DIAGNOSTIC_ONLY', 'status': 'ERROR', 'command': sys.argv,
              'source_sha256': hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
              'world': str(WORLD.relative_to(ROOT)),
              'world_sha256': hashlib.sha256(WORLD.read_bytes()).hexdigest(),
              'w5_ref_sha256': hashlib.sha256(W5.read_bytes()).hexdigest(),
              'constants': dict(drop_m=DROP_M, carry_limit_m=CARRY_LIMIT_M,
                                sim_seconds=SIM_SECONDS, sample_hz=SAMPLE_HZ,
                                stand_drop_max_m=STAND_DROP_MAX_M),
              'prior_evidence': ['reports/p4-belt-g4-seam-03 (launch reproduced, 4/7 FAIL)',
                                 'reports/p4-belt-g4-h2v7-01 (the judged event)',
                                 'world dump: all band ctrl in key_ctrl[0] = 0.0']}
    try:
        model = mujoco.MjModel.from_xml_path(str(WORLD))
        w5_model = mujoco.MjModel.from_xml_path(str(W5))

        tray_bid = model.body(TRAY).id
        tray_jadr = int(model.body_jntadr[tray_bid])
        assert int(model.jnt_type[tray_jadr]) == int(mujoco.mjtJoint.mjJNT_FREE)
        tray_qadr = int(model.jnt_qposadr[tray_jadr])
        tray_vadr = int(model.jnt_dofadr[tray_jadr])
        tray_geoms = list(range(int(model.body_geomadr[tray_bid]),
                                int(model.body_geomadr[tray_bid]) + int(model.body_geomnum[tray_bid])))
        tray_geoms_set = set(tray_geoms)

        band_qadr, band_vadr = [], []
        for j in range(model.njnt):
            b = int(model.jnt_bodyid[j])
            if (model.body(b).name or '').startswith(PREFIX):
                band_qadr.append(int(model.jnt_qposadr[j]))
                band_vadr.append(int(model.jnt_dofadr[j]))
        band_qadr = np.array(band_qadr)
        band_vadr = np.array(band_vadr)
        watch_vadr = int(model.jnt_dofadr[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, WATCH_IDLER)])
        base_bid = int(model.jnt_bodyid[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, BASE_JOINT)])

        pinned_qadr, pinned_vadr = [], []
        for j in range(model.njnt):
            b = int(model.jnt_bodyid[j])
            nm = model.body(b).name or ''
            if j == tray_jadr or nm.startswith(PREFIX) or nm.startswith(FREE_EXTRA_PREFIX):
                continue
            pinned_qadr.append(int(model.jnt_qposadr[j]))
            pinned_vadr.append(int(model.jnt_dofadr[j]))

        surf = None
        for nm in SUPPORT_ROLLERS:
            b = model.body(nm).id
            r = max((float(model.geom_size[g][0]) for g in
                     range(int(model.body_geomadr[b]), int(model.body_geomadr[b]) + int(model.body_geomnum[b]))
                     if int(model.geom_type[g]) == int(mujoco.mjtGeom.mjGEOM_CYLINDER)), default=0.0)
            surf = max(surf or 0.0, 0.775 + r)   # roller centers are z=0.775 (verified seam-03)
        lowest = min(geom_lowest_z(model, g) for g in tray_geoms)
        rest_center_z = surf - lowest
        report['surface'] = {'derived_surface_z': surf, 'tray_lowest_body_frame_m': lowest,
                             'derived_rest_center_z': rest_center_z,
                             'h2_observed_rest_z': H2_OBSERVED_REST_Z,
                             'pose_delta_m': abs(rest_center_z - H2_OBSERVED_REST_Z)}
        report['pinning_declared'] = {'pinned_joints': len(pinned_qadr),
                                      'free_joints': int(model.njnt) - len(pinned_qadr)}

        def run_arm(tag, clamp, ctrl_key0):
            data = mujoco.MjData(model)
            mujoco.mj_resetDataKeyframe(model, data, 0)
            if ctrl_key0:
                data.ctrl[:] = model.key_ctrl[0]
            else:
                data.ctrl[:] = 0.0
            mujoco.mj_forward(model, data)
            home_qpos = np.array(data.qpos, float)
            base_z0 = float(data.xpos[base_bid][2])
            data.qpos[tray_qadr:tray_qadr + 3] = [PLACE_X, PLACE_Y, rest_center_z + DROP_M]
            data.qvel[tray_vadr:tray_vadr + 6] = 0.0
            mujoco.mj_forward(model, data)
            qpos0 = np.array(data.qpos[band_qadr], float)
            dt = float(model.opt.timestep)
            every = max(1, int(round((1.0 / SAMPLE_HZ) / dt)))
            steps = int(SIM_SECONDS / dt)
            trace = []
            peak_band = 0.0
            peak_idler = 0.0
            max_rot_idler = 0.0
            base_min_z = base_z0
            idler_i = list(band_vadr).index(watch_vadr)
            for k in range(steps):
                if clamp:
                    data.qpos[pinned_qadr] = home_qpos[pinned_qadr]
                    data.qvel[pinned_vadr] = 0.0
                mujoco.mj_step(model, data)
                peak_band = max(peak_band, float(np.max(np.abs(data.qvel[band_vadr]))))
                peak_idler = max(peak_idler, abs(float(data.qvel[watch_vadr])))
                base_min_z = min(base_min_z, float(data.xpos[base_bid][2]))
                if k % every == 0:
                    max_rot_idler = max(max_rot_idler,
                                        abs(float(data.qpos[band_qadr[idler_i]]) - qpos0[idler_i]))
                    trace.append([round(k * dt, 4), float(data.qpos[tray_qadr]),
                                  float(data.qpos[tray_qadr + 1]), float(data.qpos[tray_qadr + 2])])
            excursion = max(float(np.hypot(r[1] - PLACE_X, r[2] - PLACE_Y)) for r in trace)
            first_t = None
            for r in trace:
                if float(np.hypot(r[1] - PLACE_X, r[2] - PLACE_Y)) > CARRY_LIMIT_M:
                    first_t = r[0]
                    break
            return {'tag': tag, 'clamp': clamp, 'ctrl_key0': ctrl_key0,
                    'tray_excursion_m': excursion, 'first_launch_t_s': first_t,
                    'peak_band_rad_s': peak_band, 'peak_idler_rad_s': peak_idler,
                    'max_idler_rot_rad': max_rot_idler,
                    'base_z0_m': base_z0, 'base_min_z_m': base_min_z,
                    'base_drop_m': base_z0 - base_min_z,
                    'end_xyz': trace[-1], 'tray_trace_hz50': trace}

        arms = [run_arm('A_clamp_key0', True, True),
                run_arm('B_clamp_zero', True, False),
                run_arm('C_nopin_key0', False, True)]
        report['arms'] = [{k: v for k, v in a.items() if k != 'tray_trace_hz50'} for a in arms]
        report['traces'] = {a['tag']: a['tray_trace_hz50'] for a in arms}

        a, b, c = arms
        rows = []

        def add(name, ok, detail, falsified_by=None):
            rows.append({'name': name, 'status': 'PASS' if ok else 'FAIL', 'detail': detail,
                         'falsified_by': falsified_by})

        def fmt(ar):
            return ('excursion %.6f m (limit %.4f), first>5mm at %s s, peak band %.3f rad/s, '
                    'watch idler peak %.3f rad/s max %.3f rad'
                    % (ar['tray_excursion_m'], CARRY_LIMIT_M,
                       ar['first_launch_t_s'], ar['peak_band_rad_s'],
                       ar['peak_idler_rad_s'], ar['max_idler_rot_rad']))

        add('arm_A_launch_reproduced', a['tray_excursion_m'] > CARRY_LIMIT_M,
            'A clamp+key0: %s' % fmt(a),
            falsified_by='non-reproduction of seam-03 would invalidate the paired comparison')
        add('arm_B_zero_ctrl_still_launches', b['tray_excursion_m'] > CARRY_LIMIT_M,
            'B clamp+zero ctrl: %s' % fmt(b),
            falsified_by='launching at zero ctrl convicts contact-gravity world-side; '
                         'staying quiet acquits the world and convicts drive-coupled clamping')
        stood = c['base_drop_m'] <= STAND_DROP_MAX_M
        add('arm_C_humanoid_stood', stood,
            'C no-pin+key0: humanoid base drop %.4f m over %.1f s (guard %.2f m)'
            % (c['base_drop_m'], SIM_SECONDS, STAND_DROP_MAX_M),
            falsified_by='a collapse makes arm C inconclusive, not exculpatory')
        add('arm_C_still_launches', c['tray_excursion_m'] > CARRY_LIMIT_M,
            'C no-pin+key0: %s; standing held: %s' % (fmt(c), stood),
            falsified_by='launching without clamping convicts the world-side; '
                         'staying quiet convicts the pinning clamp (instrument)')

        if not a['tray_excursion_m'] > CARRY_LIMIT_M:
            verdict = 'A did not reproduce seam-03: engine nondeterminism -- investigate before judging B/C'
        elif b['tray_excursion_m'] > CARRY_LIMIT_M and c['tray_excursion_m'] > CARRY_LIMIT_M and stood:
            verdict = ('WORLD-SIDE: launches with zero drives AND without clamping -- '
                       'contact-gravity instability at the flush undamped idlers')
        elif b['tray_excursion_m'] > CARRY_LIMIT_M and not stood:
            verdict = 'world-side suspected (B launches at zero ctrl) but C inconclusive: humanoid collapsed'
        elif b['tray_excursion_m'] > CARRY_LIMIT_M:
            verdict = 'INSTRUMENT: needs the clamp even with zero drives -- pinning clamp pumps the solver'
        elif c['tray_excursion_m'] > CARRY_LIMIT_M and stood:
            verdict = 'INSTRUMENT: needs the key_ctrl drives under clamp -- actuator-coupled clamp artifact'
        else:
            verdict = 'INSTRUMENT: requires both clamp and drives -- clamp+drive interaction artifact'
        add('energy_source_localized', True, 'decision table: %s' % verdict,
            falsified_by='reporting row; falsifiers live in the arm rows above')

        report['rows'] = rows
        report['findings'] = [
            {'finding': 'why this run exists',
             'detail': 'seam-03 launched the tray from tangent rest while a world dump shows '
                       'every band actuator at ctrl 0.0 and the v7-only idlers undamped, flush, '
                       'unactuated at the place point. Before convicting the world, the two '
                       'probe-side suspects (clamped large actuator forces; key0 drives) must '
                       'be excluded by paired arms.'},
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
                     if k not in ('rows', 'arms', 'traces')}
        print(json.dumps(printable, indent=2, ensure_ascii=False), flush=True)
        for r in report.get('rows') or []:
            print('%-38s %-5s %s' % (r['name'], r['status'], r['detail']), flush=True)
        print('OVERALL: %s' % report.get('overall'), flush=True)
    return 0 if report['status'] == 'DIAGNOSTIC_COMPLETED' else 1


if __name__ == '__main__':
    raise SystemExit(main())
