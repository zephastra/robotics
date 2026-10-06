#!/usr/bin/env python3
"""G4 (section 7): the Panda's reach and grasp, judged IN the v7 candidate world.

RECORD CORRECTION this probe is paired with: an earlier session note said the Panda
was "NOT instantiated" in v7. That was wrong. Measured 2026-10-04: v7 carries the
full arm (a_link0 at (5.21372, -0.58, 0.61), yaw 90 deg, a_actuator1..8 at actuator
slots 57..64 in the same order as w5_h085_loop, all nine joint definitions
attribute-identical to w5, home arm ctrl all zero). What was missing is EXECUTION
evidence in v7 -- the candidate chain's own builder says its capacity/IK checks are
"necessary static checks, NOT execution proof". This probe produces that evidence.

AMENDMENT 1 (seed-state fix, measured before this run): the first attempt failed its
opening IK at best-pos 0.55288 m. A/B measurement (same target, same solver): seeding
the solver with the arm's RAW zeros gets stuck at 0.55288 m while seeding from the
home keyframe's arm pose [0, 0, 0, -1.5708, 0, 1.5708, -0.7853] converges to 3e-5 m.
The zero pose is a solver trap, not a reach limit -- and it is also what the world's
own key_ctrl would drive the arm to, because the arm's position servos are commanded
0 in the home keyframe. So this probe (a) initializes data from the home keyframe,
(b) declares an initialization ctrl write that HOLDS the arm at its home pose (grip
open) instead of letting it sag, and (c) seeds every IK from the arm's CURRENT state.
The initialization is counted and reported like H2's station-qpos writes are.

WHAT IS REUSED (imported, not copied)
  * `arm_bridge.install('a_')` + `arm_bridge.verify` against the standalone arm
    world -- the same drives-the-same-arm proof `probe_p4_arm.py` rows on;
  * `arm_rig` for IK, the grasp constants and the finger-gap instrument;
  * `probe_p4_arm.py`'s eight-phase schedule and per-phase step budgets verbatim
    (inherited constants, each printed in the report);
  * `probe_p3_vision.part_size` as a cross-check instrument only (it reads the red
    box's z extent as 0.025 while the authored pose delta is 0.035 -- reported, and
    the support plane is taken from the world's own table body z, which the authored
    part pose agrees with).

WHAT IS DIFFERENT FROM `probe_p4_arm.py`, AND DECLARED
  * World: v7 directly. No p4 fixture, no D115 loading fixture -- v7's OWN authored
    layout already puts the table inside the arm's measured reach (0.78 m converges,
    0.820 m fails: reports/p1-a-envscan), unlike the p4 world's 1.7264 m gap.
  * Pick pose: the world's OWN authored home for `a_payload` (read from the home
    keyframe, cross-checked against the body pos attribute). Nothing is teleported;
    the probe never reads runtime truth to aim.
  * Place pose: a DECLARED offset on the table surface (want +0.15 m along the
    table's x, clamped to the table's own box extent and reported). NOT the stale
    `a_place_target` site: it still sits at the w5-era (3.4373, 0.13, 0.2), which is
    1.98 m from the moved arm and referenced by no candidate code (grep, 2026-10-04).
    Reported as a finding.
  * Perception: NOT_RUN, declared. v7 authors no arm-side camera; the "refuse on
    UNKNOWN" half of the claim belongs to the vehicle RGB-D gate and is already
    judged there (reports/p4-belt-g4-vision-*).
  * Negative control, replacing the UNKNOWN refusal: IK at 0.850 m must FAIL. If IK
    "converges" there, the measured boundary was never real and the row fails.

Zero runtime qpos writes: the plan lives in its own MjData; the run moves only by
writing arm/gripper ctrl. A dropped payload latches and ends the run (probe_arm's
lesson: a lost payload must not drive on for 79 minutes).
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

import arm_bridge  # noqa: E402
import arm_rig as rig  # noqa: E402
import probe_p3_vision as PV  # noqa: E402

WORLD = ROOT / 'assets' / 'world_p5_candidate_v7_hinged_retainer.xml'
STANDALONE = ROOT / 'assets' / 'world_arm_a.xml'
PICK_PART = 'a_payload'
PART_FREE_JOINT = 'a_payload_free'
TABLE = 'a_table'
ARM_BASE = 'a_link0'

WANT_DX_M = 0.15          # declared carry distance along the table's x, clamped below
MIN_DX_M = 0.05           # below this the "carry" would not be a carry
PLACE_WINDOW_M = 0.020    # H2's declared place window, reused verbatim
REST_SPEED_MPS = 0.010    # H2's declared rest speed, reused verbatim
LIFT_MIN_RISE_M = 0.08    # the part must clear the table by this much at lift end
REACH_BOUND_M = 0.78      # measured boundary: converges here (reports/p1-a-envscan)
REACH_FAIL_M = 0.820      # ...and fails here; cited, not re-measured
NEGATIVE_RADIUS_M = 0.85  # the negative control probes beyond the fail point
PHASE_STEPS = {'approach': 700, 'descend': 400, 'close': 500, 'lift': 500,
               'carry': 900, 'lower': 600, 'release': 400, 'retreat': 600}
GRIP_OPEN = rig.GRIPPER_CTRL_OPEN
GRIP_CLOSED = rig.GRIPPER_CTRL_CLOSED


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--run-id', required=True)
    args = ap.parse_args()
    out = ROOT / 'reports' / args.run_id
    if out.exists() and any(out.iterdir()):
        raise SystemExit('REFUSED: %s already holds evidence; pick a new --run-id' % out)
    out.mkdir(parents=True)
    started = time.monotonic()
    report = {'probe': 'G4 v7 Panda reach and grasp (v7-native, no fixture)',
              'scope': 'DIAGNOSTIC_ONLY', 'status': 'ERROR',
              'command': sys.argv,
              'source_sha256': hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
              'world': str(WORLD.relative_to(ROOT)),
              'world_sha256': hashlib.sha256(WORLD.read_bytes()).hexdigest(),
              'inherited_constants': dict(PHASE_STEPS, want_dx_m=WANT_DX_M,
                                          place_window_m=PLACE_WINDOW_M,
                                          rest_speed_mps=REST_SPEED_MPS,
                                          lift_min_rise_m=LIFT_MIN_RISE_M,
                                          reach_bound_m=REACH_BOUND_M,
                                          reach_fail_m=REACH_FAIL_M,
                                          negative_radius_m=NEGATIVE_RADIUS_M)}
    try:
        model = mujoco.MjModel.from_xml_path(str(WORLD))
        data = mujoco.MjData(model)
        # The world's own home -- not the constructor's raw state. Measured (amendment 1):
        # everything downstream seeds better from here, and the world's declared initial
        # condition is the only honest starting state for a qualification run.
        mujoco.mj_resetDataKeyframe(model, data, 0)
        mujoco.mj_forward(model, data)
        report['data_init'] = 'mj_resetDataKeyframe(0) + mj_forward'

        arm_bridge.install('a_')
        report['bridge'] = arm_bridge.installed()
        standalone = mujoco.MjModel.from_xml_path(str(STANDALONE))
        verify = arm_bridge.verify(model, data, standalone)
        report['bridge_verify'] = verify

        d = dict(rig.ids(model))
        d['arm_ctrl'] = [model.actuator('a_actuator%d' % i).id for i in range(1, 8)]
        grip_ctrl = model.actuator('a_actuator8').id
        home_arm = np.array(data.qpos[d['arm_qadr']], float)

        # Initialization ctrl write, declared and counted: the home keyframe commands the
        # arm's position servos 0, which drives the arm AWAY from its home qpos (measured
        # trap, amendment 1). Hold the home pose instead; open the grip.
        data.ctrl[:] = model.key_ctrl[0]
        data.ctrl[d['arm_ctrl']] = home_arm
        data.ctrl[grip_ctrl] = float(GRIP_OPEN)
        mujoco.mj_forward(model, data)
        report['init_ctrl_writes'] = {'arm_servos': int(len(d['arm_ctrl'])),
                                      'gripper': 1,
                                      'note': ('arm servos hold the home keyframe pose; '
                                               'everything else stays at key_ctrl[0]')}
        report['home_arm_qpos'] = [float(v) for v in home_arm]

        arm_origin = np.asarray(data.xpos[model.body(ARM_BASE).id], float)
        report['arm_frame_origin'] = [float(v) for v in arm_origin]

        part_half_pv = float(np.asarray(PV.part_size(model), float)[2])
        bid = model.body(PICK_PART).id
        jadr = int(model.body_jntadr[bid])
        assert int(model.jnt_type[jadr]) == int(mujoco.mjtJoint.mjJNT_FREE), \
            'the part must keep its free joint'
        qadr = int(model.jnt_qposadr[jadr])
        vadr = int(model.jnt_dofadr[jadr])  # qvel indexes DOFs, not qpos (measured trap)
        part_home = np.array(model.key_qpos[0][qadr:qadr + 7], float) if model.nkey > 0 \
            else np.array(model.qpos0[qadr:qadr + 7], float)
        body_pos = np.asarray(model.body_pos[bid], float)
        pick = np.array([part_home[0], part_home[1], float(part_home[2])])
        bench_z = float(model.body_pos[model.body(TABLE).id][2])
        report['part'] = {
            'authored_key_xyz': [float(v) for v in part_home[:3]],
            'body_pos_xyz': [float(v) for v in body_pos],
            'authored_matches_body_pos': bool(np.allclose(part_home[:3], body_pos, atol=1e-9)),
            'pv_part_half_z': part_half_pv,
            'pose_delta_above_table': float(pick[2] - bench_z),
            'support_plane': 'table body z (the authored pose agrees; PV reads the red box '
                             '0.01 thinner than the pose delta -- reported, not used)'}
        report['table'] = {'bench_z_m': bench_z}

        # the declared place offset, clamped by the table's own surface extent
        tb = model.body(TABLE).id
        hx = None
        for g in range(int(model.body_geomadr[tb]), int(model.body_geomadr[tb])
                       + int(model.body_geomnum[tb])):
            if int(model.geom_type[g]) == int(mujoco.mjtGeom.mjGEOM_BOX):
                hx = max(hx or 0.0, float(model.geom_size[g][0]))
        if hx is None:
            for g in range(int(model.body_geomadr[tb]), int(model.body_geomadr[tb])
                           + int(model.body_geomnum[tb])):
                hx = max(hx or 0.0, float(model.geom_rbound[g]))
            report['table']['extent_source'] = 'rbound (no box geom found)'
        else:
            report['table']['extent_source'] = 'box geom half-x'
        max_dx = hx - 0.035 - 0.005
        dx = min(WANT_DX_M, max_dx)
        place = np.array([pick[0] + dx, pick[1], bench_z + 0.035])
        report['place'] = {'want_dx_m': WANT_DX_M, 'table_half_x_m': hx,
                           'max_dx_m': float(max_dx), 'chosen_dx_m': float(dx),
                           'clamped': bool(dx < WANT_DX_M - 1e-12),
                           'place_xyz': [float(v) for v in place]}

        # -- IK machinery --------------------------------------------------------------------
        plan = mujoco.MjData(model)
        rot = rig.home_hand_rot(model, plan)   # leaves plan AT the home keyframe
        rest_half_z = float(pick[2] - bench_z)  # the part's own rest height, derived

        def arm_q():
            return np.array(data.qpos[d['arm_qadr']], float)

        def ik_to(target):
            # seed from the arm's CURRENT state; the A/B measurement (amendment 1) showed
            # the raw zero pose is a solver trap (best 0.55288 m) while the held home pose
            # converges to 3e-5 m on the same target.
            mujoco.mj_resetDataKeyframe(model, plan, 0)
            plan.qpos[d['arm_qadr']] = data.qpos[d['arm_qadr']]
            q, err_p, _err_r = rig.ik(model, plan, np.asarray(target, float), target_rot=rot,
                                      q_seed=np.array(data.qpos[d['arm_qadr']], float),
                                      iters=600)
            if err_p is None or float(err_p) > 1e-3:
                raise RuntimeError('IK did not converge to %s (residual %s)'
                                   % (np.round(target, 4), err_p))
            return np.array(q, float)

        # NEGATIVE CONTROL FIRST, while the arm is still at its init pose.
        dir_xy = np.array([pick[0] - arm_origin[0], pick[1] - arm_origin[1]])
        norm = float(np.linalg.norm(dir_xy)) or 1.0
        neg = np.array([arm_origin[0] + dir_xy[0] / norm * NEGATIVE_RADIUS_M,
                        arm_origin[1] + dir_xy[1] / norm * NEGATIVE_RADIUS_M,
                        bench_z + rest_half_z])
        neg_rec = {'target': [float(v) for v in neg]}
        try:
            ik_to(neg)
            neg_rec['raised'] = False
            neg_rec['detail'] = 'IK converged at %.2f m; the measured boundary did not hold' \
                                % NEGATIVE_RADIUS_M
        except Exception as exc:
            neg_rec['raised'] = True
            neg_rec['detail'] = '%s: %s' % (type(exc).__name__, exc)
        report['negative_control'] = neg_rec

        pick_radius = float(np.linalg.norm(pick - arm_origin))
        place_radius = float(np.linalg.norm(place - arm_origin))

        q_before_all = arm_q().copy()
        part_z0 = float(pick[2])
        rec = {'phases': {}, 'latch': None}
        seq = [('approach', pick + np.array([0.0, 0.0, rig.PREGRASP_CLEARANCE]), GRIP_OPEN),
               ('descend', pick + np.array([0.0, 0.0, rig.GRASP_INSET]), GRIP_OPEN),
               ('close', None, GRIP_CLOSED),
               ('lift', pick + np.array([0.0, 0.0, rig.LIFT_HEIGHT]), None),
               ('carry', place + np.array([0.0, 0.0, rig.PREGRASP_CLEARANCE]), None),
               ('lower', place + np.array([0.0, 0.0, rig.GRASP_INSET]), None),
               ('release', None, GRIP_OPEN),
               ('retreat', place + np.array([0.0, 0.0, rig.RETREAT_HEIGHT]), None)]
        lost = False
        for phase, target, grip in seq:
            if lost:
                break
            if target is not None:
                q = ik_to(target)
                q0 = arm_q().copy()
                for k in range(1, PHASE_STEPS[phase] + 1):
                    data.ctrl[d['arm_ctrl']] = q0 + (q - q0) * (k / PHASE_STEPS[phase])
                    mujoco.mj_step(model, data)
                    z = float(data.xpos[bid][2])
                    if k % 50 == 0 and phase in ('lift', 'carry') and z < bench_z - 0.005:
                        rec['latch'] = {'phase': phase, 'step': k, 'part_z': z}
                        lost = True
                        break
                rec['phases'][phase] = {'steps': PHASE_STEPS[phase],
                                        'part_z_end': float(data.xpos[bid][2])}
            if grip is not None and not lost:
                data.ctrl[grip_ctrl] = float(grip)
                for _ in range(PHASE_STEPS[phase]):
                    mujoco.mj_step(model, data)
                rec['phases'][phase] = {'grip': float(grip), 'steps': PHASE_STEPS[phase],
                                        'part_z_end': float(data.xpos[bid][2])}
                if phase == 'close' and float(data.xpos[bid][2]) < bench_z - 0.005:
                    rec['latch'] = {'phase': 'close', 'part_z': float(data.xpos[bid][2])}
                    lost = True
        report['run'] = rec
        final = np.asarray(data.xpos[bid], float)
        report['final_part_xyz'] = [float(v) for v in final]
        report['final_gripper_gap_m'] = float(rig.finger_gap(model, data))
        report['arm_moved_rad'] = float(np.max(np.abs(arm_q() - q_before_all)))
        # release stability: the tail of the retreat phase is the "let go and watch" window
        tail_speed = []
        tail_z = []
        for _ in range(500):
            mujoco.mj_step(model, data)
            tail_speed.append(float(np.linalg.norm(data.qvel[vadr:vadr + 3])))
            tail_z.append(float(data.xpos[bid][2]))
        report['release_tail'] = {'max_speed_mps': max(tail_speed),
                                  'z_drift_m': float(max(tail_z) - min(tail_z)),
                                  'z_end': tail_z[-1]}

        # -- rows ---------------------------------------------------------------------------
        rows = []

        def add(name, ok, detail, falsified_by=None):
            rows.append({'name': name, 'status': 'PASS' if ok else 'FAIL', 'detail': detail,
                         'falsified_by': falsified_by})

        add('the_bridge_drives_the_same_arm',
            bool(verify['same_arm']) and verify['joint_ranges_equal']
            and verify['grasp_local_offset_equal_across_worlds'],
            '%d arm joints and %d finger joints resolve through the bridge; joint ranges equal '
            'the standalone world: %s; grasp_local_offset equal across worlds: %s'
            % (verify['n_arm_joints'], verify['n_finger_joints'], verify['joint_ranges_equal'],
               verify['grasp_local_offset_equal_across_worlds']),
            falsified_by='a prefix typo that resolved to another body would change the offset')
        add('pick_and_place_inside_measured_reach',
            pick_radius < REACH_BOUND_M and place_radius < REACH_BOUND_M,
            'pick radius %.4f m, place radius %.4f m from a_link0; measured boundary %.2f m '
            'converges / %.3f m fails (reports/p1-a-envscan, an arm property not a world one)'
            % (pick_radius, place_radius, REACH_BOUND_M, REACH_FAIL_M),
            falsified_by='the same IK sweep at the fail point does not converge, so the '
                         'boundary is real')
        add('ik_refuses_beyond_boundary', bool(neg_rec['raised']), neg_rec['detail'],
            falsified_by='IK converging at %.2f m would mean the boundary was never real'
                         % NEGATIVE_RADIUS_M)
        lift_end = rec['phases'].get('lift', {}).get('part_z_end')
        add('grasp_lifts_part_by_contact_only',
            (not lost) and lift_end is not None
            and lift_end >= part_z0 + LIFT_MIN_RISE_M,
            'part z at lift end %s m vs authored %s m (need >=%.2f m rise); latch=%s; '
            'gripper gap after release %.5f m'
            % (lift_end, part_z0, LIFT_MIN_RISE_M, rec['latch'],
               report['final_gripper_gap_m']),
            falsified_by='a weld or equality could fake the lift; the report carries neq and '
                         'the run writes ctrl only')
        dz_xy = float(np.hypot(final[0] - place[0], final[1] - place[1]))
        dz_z = float(final[2] - place[2])
        add('part_placed_at_declared_target',
            (not lost) and dz_xy <= PLACE_WINDOW_M and abs(dz_z) <= 0.010,
            'final offset from the declared place point: xy %.5f m (window %.3f), z %+.5f m '
            '(window 0.010)' % (dz_xy, PLACE_WINDOW_M, dz_z),
            falsified_by='a runtime teleport could fake it; qpos writes are zero by '
                         'construction (motion is ctrl-only)')
        stable = (max(tail_speed) <= REST_SPEED_MPS
                  and abs(tail_z[-1] - (bench_z + rest_half_z)) <= 0.005)
        add('release_leaves_part_stable', bool(stable and not lost),
            'release tail: max speed %.5f m/s (limit %.3f), z drift %.5f m, final z %.5f m vs '
            'rest z %.5f m' % (max(tail_speed), REST_SPEED_MPS,
                               report['release_tail']['z_drift_m'], tail_z[-1],
                               bench_z + rest_half_z),
            falsified_by='the part sliding off the declared spot after release')
        report['rows'] = rows
        report['findings'] = [
            {'finding': 'a_place_target is stale in v7',
             'detail': 'the site still sits at the w5-era (3.4373, 0.13, 0.2), 1.98 m from the '
                       'moved a_link0; no candidate code references it (grep 2026-10-04). This '
                       'probe used a declared table offset instead and did NOT move the site.'},
            {'finding': 'perception NOT_RUN in this probe',
             'detail': 'v7 authors no arm-side camera; the UNKNOWN-refusal half of the pick '
                       'claim is judged by the vehicle RGB-D gate (p4-belt-g4-vision-*).'},
            {'finding': 'the home keyframe does not hold the arm',
             'detail': 'key_ctrl commands the arm servos 0 while key_qpos holds a bent-home '
                       'pose; left alone the arm sags (amendment 1). This probe declares a '
                       'hold-the-home initialization ctrl write; fixing the WORLD keyframe is '
                       'a separate declared change, not taken here.'},
        ]
        report['neq'] = int(model.neq)
        report['runtime_qpos_writes'] = 0
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
            print('%-38s %-5s %s' % (r['name'], r['status'], r['detail']), flush=True)
        print('OVERALL: %s' % report.get('overall'), flush=True)
    return 0 if report['status'] == 'DIAGNOSTIC_COMPLETED' else 1


if __name__ == '__main__':
    raise SystemExit(main())
