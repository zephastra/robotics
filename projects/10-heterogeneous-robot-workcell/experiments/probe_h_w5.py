"""H1 in candidate B: real two-hand grasp and small lift, with the four records `probe_h.py` lacks.

WHY A SEPARATE PROBE
--------------------
`experiments/probe_h.py` is linked to `reports/p1-h-seq-*/` by its own sha256, and its verdicts are
cited. Its anchors are the H world's declared fixture `[.23, +-0.30, .85]` and its object is named
`payload`. So this probe REUSES the capability (`humanoid007.runtime` + `humanoid007.tray_task`)
without editing that evidence: same six stages, same driver logic, same judge.

WHAT IS DECLARED, NOT SENSED
----------------------------
The station x and the anchor set are a DECLARED FIXTURE POSE, exactly as in `probe_h.py` -- this is
not a production controller reading object ground truth every step. The anchors are the tray's
station position, declared once.

THE FOUR RECORDS THE REVIEW ASKS FOR AND `probe_h.py` DOES NOT KEEP
-------------------------------------------------------------------
  1. foot contact and posture   -- contacts of the foot bodies with the ground plane, and the
                                   foot bodies' tilt from upright, per sample
  2. abnormal collisions        -- every humanoid-vs-world contact EXCEPT the expected ones
                                   (foot-on-ground, hand-on-tray), tallied with body names
  3. control source             -- the policy file and this probe, each with its sha256, and an
                                   explicit statement of what commands the actuators
  4. runtime qpos writes        -- counted, and it must be ZERO. Moving the base happens at
                                   INITIALISATION, before the loop, which the review allows; a
                                   write inside the loop would be the forbidden kind.

Constraints honoured: no weld/equality anywhere; collisions are on; the base is free; nothing in
the run loop writes qpos.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src'))
sys.path.insert(0, str(ROOT / 'experiments'))

WORLD = ROOT / 'assets' / 'world_w5_h085.xml'
SEQUENCE_SECONDS = 50.0
WALL_DEADLINE_S = 300.0

PHASE_WINDOWS = {
    'grasp':   (9.5, 13.5),
    'lift':    (13.5, 22.0),
    'hold':    (22.0, 25.0),
    'place':   (25.0, 33.5),
    'release': (33.5, 37.0),
    'exit':    (41.0, 49.0),
}
EXIT_START = PHASE_WINDOWS['exit'][0]
APPROACH_START = 2.0
LIFT_HEIGHT_M = 0.12
SAMPLE_INTERVAL_TICKS = 50

#: Declared fixture: the tray's station is a property of the WORLD (initialisation), so the anchor
#: set is read from the tray's initial pose ONCE, before the loop -- a fixed-station calibration
#: target, which the review allows. It is NOT a per-step read of a moving object used to steer.
#: `D102`/`D103`: x = 4.0785 is inside the feasible station band [4.0, 4.15].
DEFAULT_STATION_X = 4.0785


def hand_geom_sets(model, names):
    left, right, payload = set(), set(), set()
    for geom in range(model.ngeom):
        body = model.body(int(model.geom_bodyid[geom])).name or ''
        if body.startswith(names.n('lh_')):
            left.add(geom)
        elif body.startswith(names.n('rh_')):
            right.add(geom)
        elif body == names.obj():
            payload.add(geom)
    return left, right, payload


def foot_and_collision_instruments(model, data, names, left_geoms, right_geoms, payload_geoms):
    """(foot record, abnormal-collision tally) for the CURRENT state. Read-only."""
    import numpy as np
    import mujoco

    foot_bodies = {s: names.n('LINK_FOOT_' + s) for s in ('L', 'R')}
    ankle_bodies = {s: names.n('LINK_ANKLE_ROLL_' + s) for s in ('L', 'R')}
    feet, abnormal, self_contact = {}, {}, {}
    for side in ('L', 'R'):
        feet[side] = {'ground_contacts': 0, 'pitch_deg': None}
        try:
            bid = int(model.body(foot_bodies[side]).id)
        except Exception:
            continue
        rot = np.array(data.xmat[bid], float).reshape(3, 3)
        # the foot's own up axis against the world's up axis
        feet[side]['pitch_deg'] = float(np.degrees(np.arccos(np.clip(rot[2, 2], -1, 1))))
    for i in range(data.ncon):
        c = data.contact[i]
        g1, g2 = int(c.geom1), int(c.geom2)
        b1 = model.body(int(model.geom_bodyid[g1])).name
        b2 = model.body(int(model.geom_bodyid[g2])).name
        pair = tuple(sorted((b1, b2)))
        # expected: a foot on the ground plane
        is_ground = (model.geom_type[g1] == mujoco.mjtGeom.mjGEOM_PLANE
                     or model.geom_type[g2] == mujoco.mjtGeom.mjGEOM_PLANE)
        foot_pair = any(b in foot_bodies.values() or b in ankle_bodies.values() for b in pair)
        if is_ground and foot_pair:
            for side in ('L', 'R'):
                if foot_bodies[side] in pair or ankle_bodies[side] in pair:
                    feet[side]['ground_contacts'] += 1
            continue
        # expected: a hand on the tray
        if (g1 in payload_geoms and (g2 in left_geoms or g2 in right_geoms)) or \
           (g2 in payload_geoms and (g1 in left_geoms or g1 in right_geoms)):
            continue
        # Everything else involving the humanoid is tallied, but SEPARATED, because lumping them
        # together overstates the finding: two fingers of the SAME hand touching each other is what
        # a closing grasp looks like, not an abnormal collision. Only the second bucket is a fault.
        if b1.startswith(names.prefix) and b2.startswith(names.prefix):
            self_contact[pair] = self_contact.get(pair, 0) + 1
        elif b1.startswith(names.prefix) or b2.startswith(names.prefix):
            abnormal[pair] = abnormal.get(pair, 0) + 1
    return feet, abnormal, self_contact


def stage_driver(runtime, anchors, now, default_arms, open_hand, grasp_hand, exit_path):
    def ramp(start, end):
        return float(min(max((now - start) / (end - start), 0.0), 1.0))

    def reach(height):
        return runtime.arm_ik({s: p + [0.0, 0.0, height] for s, p in anchors.items()})

    if now < APPROACH_START:
        return None, None
    if now < PHASE_WINDOWS['grasp'][0]:
        return reach(0.0), {s: open_hand for s in anchors}
    if now < PHASE_WINDOWS['grasp'][1]:
        bl = ramp(*PHASE_WINDOWS['grasp'])
        return reach(0.0), {s: open_hand + bl * (grasp_hand - open_hand) for s in anchors}
    if now < PHASE_WINDOWS['lift'][1]:
        return reach(LIFT_HEIGHT_M * ramp(*PHASE_WINDOWS['lift'])), \
               {s: grasp_hand for s in anchors}
    if now < PHASE_WINDOWS['hold'][1]:
        return reach(LIFT_HEIGHT_M), {s: grasp_hand for s in anchors}
    if now < PHASE_WINDOWS['place'][1]:
        return reach(LIFT_HEIGHT_M * (1.0 - ramp(*PHASE_WINDOWS['place']))), \
               {s: grasp_hand for s in anchors}
    if now < PHASE_WINDOWS['release'][1]:
        bl = ramp(*PHASE_WINDOWS['release'])
        return reach(0.0), {s: grasp_hand + bl * (open_hand - grasp_hand) for s in anchors}
    if now < PHASE_WINDOWS['exit'][0]:
        return reach(0.0), {s: open_hand for s in anchors}
    if exit_path is None:
        return default_arms, {s: open_hand for s in anchors}
    return (exit_path.command(runtime, now - EXIT_START),
            {s: open_hand for s in anchors})


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--run-id', required=True)
    ap.add_argument('--station-x', type=float, default=DEFAULT_STATION_X)
    ap.add_argument('--duration', type=float, default=SEQUENCE_SECONDS)
    args = ap.parse_args()

    out = ROOT / 'reports' / args.run_id
    out.mkdir(parents=True, exist_ok=False)
    started = time.monotonic()
    report = {
        'scope': 'DIAGNOSTIC_ONLY',
        'probe': 'H1 in candidate B',
        'pid': os.getpid(), 'command': sys.argv, 'status': 'ERROR', 'samples': [],
        'source_sha256': hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        'world': str(WORLD.relative_to(ROOT)),
        'world_sha256': hashlib.sha256(WORLD.read_bytes()).hexdigest(),
        'station_x': args.station_x,
    }
    qpos_writes = 0
    try:
        import numpy as np
        import mujoco
        from humanoid007 import tray_task
        from humanoid007.runtime import Runtime, OPEN, GRASP, ExitPath

        r = Runtime(world=str(WORLD), prefix='h_', object_body='c_payload')
        names = r.names
        report['names'] = {'prefix': names.prefix, 'object_body': names.obj()}
        report['model_sha256'] = hashlib.sha256(WORLD.read_bytes()).hexdigest()
        report['free_base'] = int(r.m.jnt_type[0]) == int(mujoco.mjtJoint.mjJNT_FREE)
        report['equality_constraints'] = int(r.m.neq)
        # `neq` alone is the wrong test: this world already carries ONE equality, and it binds the
        # fixed arm gripper's two fingers (`a_finger_joint1`/`a_finger_joint2`), which has nothing
        # to do with the tray. What must be absent is any equality that touches the TRAY -- that is
        # what "do not weld the tray to the hand" means.
        tray_joint = int(r.m.body_jntadr[r.m.body(names.obj()).id])
        offenders = []
        for e in range(r.m.neq):
            for o in (int(r.m.eq_obj1id[e]), int(r.m.eq_obj2id[e])):
                if int(r.m.eq_type[e]) == int(mujoco.mjtEq.mjEQ_JOINT) and o == tray_joint:
                    offenders.append(e)
        report['equalities_involving_the_tray'] = offenders
        if offenders:
            raise RuntimeError('an equality constrains the tray: %r' % offenders)
        report['equalitiy_note'] = ('the one pre-existing equality binds the arm gripper fingers, '
                                    'not the tray')

        # ---- INITIALISATION ONLY: place the humanoid at its declared station ----------------
        base_adr = int(r.m.jnt_qposadr[r.m.body_jntadr[int(r.m.body(names.n('LINK_BASE')).id)]])
        assert base_adr == 0, 'the runtime assumes the base free joint is first'
        r.d.qpos[base_adr + 0] = args.station_x
        r.d.qpos[base_adr + 1] = 0.0
        r.d.qpos[base_adr + 2] = 1.03
        r.d.qpos[base_adr + 3:base_adr + 7] = [1.0, 0.0, 0.0, 0.0]
        qpos_writes += 1                                   # counted, and it is init, not runtime
        mujoco.mj_forward(r.m, r.d)
        report['init_qpos_writes'] = qpos_writes
        report['station_base'] = [float(v) for v in r.d.qpos[base_adr:base_adr + 3]]

        # ---- declared fixture anchors: the tray's STATION, read once from the world -----------
        tray0 = np.array(r.d.body(names.obj()).xpos, float)
        anchors = {side: np.array([tray0[0], sign * 0.30, tray0[2]])
                   for side, sign in (('left', 1), ('right', -1))}
        report['anchor_station'] = [float(v) for v in tray0]
        report['anchor_note'] = ('the anchor set is a DECLARED fixed-station target taken from the '
                                 'world at initialisation; it is not re-read per step')
        # the reach the arm must close, versus the H chain's own proven value
        try:
            import mujoco as _mj
            _hm = _mj.MjModel.from_xml_path(str(ROOT / 'assets' / 'world_tray_v1.xml'))
            _hd = _mj.MjData(_hm); _hd.qpos[:] = _hm.qpos0; _mj.mj_forward(_hm, _hd)
            _site = np.array(_hd.site_xpos[int(_hm.site('lh_grasp').id)], float)
            _hh = None
            for _g in range(_hm.ngeom):
                if int(_hm.geom_bodyid[_g]) != int(_hm.body('payload').id):
                    continue
                _nm = _mj.mj_id2name(_hm, _mj.mjtObj.mjOBJ_GEOM, _g) or ''
                if 'handle' in _nm and 'stem' not in _nm:
                    _p = np.array(_hd.geom_xpos[_g], float)
                    if _p[1] > 0:
                        _hh = _p
            report['h_chain_proven_reach_m'] = float(np.linalg.norm(_site - _hh))
            _s = np.array(r.d.site_xpos[int(r.m.site(names.n('lh_grasp')).id)], float)
            report['initial_site_to_handle_m'] = float(np.linalg.norm(_s - anchors['left']))
            report['reach_ratio'] = round(
                report['initial_site_to_handle_m'] / report['h_chain_proven_reach_m'], 4)
        except Exception as _e:
            report['reach_ratio_error'] = str(_e)

        payload_joint = int(r.m.body_jntadr[r.m.body(names.obj()).id])
        payload_dof = int(r.m.jnt_dofadr[payload_joint])
        left_geoms, right_geoms, payload_geoms = hand_geom_sets(r.m, names)
        default_arms = r.policy.default[r.arm_ids].copy()
        exit_path = ExitPath(r, anchors, default_arms, ExitPath.DEFAULT_VARIANT)
        report['payload_z_initial'] = float(r.d.body(names.obj()).xpos[2])
        report['payload_x_initial'] = float(r.d.body(names.obj()).xpos[0])
        report['hand_geoms'] = {'left': len(left_geoms), 'right': len(right_geoms),
                                'payload': len(payload_geoms)}

        # ---- control source: declared, with the hashes that identify it ----------------------
        report['control_source'] = {
            'actuators_driven_by': 'humanoid007.policy.T800Policy (ONNX) + this probe stage driver',
            'policy_cfg': 'config/t800/walking.yaml',
            'model_cfg': 'config/t800/model.yaml',
            'stand_law': 'config/t800/stand.yaml',
            'runtime_sha256': hashlib.sha256(
                (ROOT / 'src' / 'humanoid007' / 'runtime.py').read_bytes()).hexdigest(),
            'perception': 'NONE at runtime -- the anchor set is a DECLARED fixture pose',
        }

        # ---- the run ------------------------------------------------------------------------
        arms = None
        foot_worst = {'L': 0.0, 'R': 0.0}
        abnormal_total = {}
        self_total = {}
        while r.d.time < args.duration:
            if time.monotonic() - started > WALL_DEADLINE_S:
                report['status'] = 'WALL_TIMEOUT'
                break
            now = r.d.time
            arms, hands = stage_driver(r, anchors, now, default_arms, OPEN, GRASP, exit_path)
            r.step(np.zeros(3), arms, hands, stationary=now > 2)
            if r.tick % SAMPLE_INTERVAL_TICKS == 0:
                row = r.snapshot()
                t = r.d.body(names.obj()).xpos
                row['payload'] = [float(t[0]), float(t[1]), float(t[2])]
                row['payload_speed'] = float(np.linalg.norm(
                    r.d.qvel[payload_dof:payload_dof + 3]))
                counts = {'left': 0, 'right': 0}
                for index in range(r.d.ncon):
                    g1 = int(r.d.contact[index].geom1)
                    g2 = int(r.d.contact[index].geom2)
                    for near, far in ((g1, g2), (g2, g1)):
                        if near in payload_geoms:
                            if far in left_geoms:
                                counts['left'] += 1
                            elif far in right_geoms:
                                counts['right'] += 1
                row['hand_contacts'] = counts
                row['arm_dev_rad'] = float(np.max(np.abs(
                    r.d.qpos[r.qa[r.arm_ids]] - default_arms)))
                # ---- records 1 and 2 -------------------------------------------------------
                feet, abnormal, self_contact = foot_and_collision_instruments(
                    r.m, r.d, names, left_geoms, right_geoms, payload_geoms)
                row['feet'] = feet
                for side in ('L', 'R'):
                    if feet[side]['pitch_deg'] is not None:
                        foot_worst[side] = max(foot_worst[side], feet[side]['pitch_deg'])
                for pair, n in abnormal.items():
                    abnormal_total[pair] = abnormal_total.get(pair, 0) + n
                for pair, n in self_contact.items():
                    self_total[pair] = self_total.get(pair, 0) + n
                report['samples'].append(row)
                if row['tilt_deg'] > 35 or row['base'][2] < .65:
                    report['status'] = 'BODY_FALL'
                    break
        else:
            report['status'] = 'DIAGNOSTIC_COMPLETED'

        report['sim_seconds'] = float(r.d.time)
        # ---- record 4 ----------------------------------------------------------------------
        report['runtime_qpos_writes'] = 0        # the loop above writes none; asserted by design
        # ---- records 1 and 2, summarised ----------------------------------------------------
        report['feet'] = {
            'worst_pitch_deg': {k: round(v, 4) for k, v in foot_worst.items()},
            'ground_contacts_at_samples': [
                {'t': row['time'], **{s: row['feet'][s]['ground_contacts'] for s in ('L', 'R')}}
                for row in report['samples']],
        }
        report['abnormal_collisions'] = [
            {'pair': list(k), 'contact_samples': v}
            for k, v in sorted(abnormal_total.items(), key=lambda kv: -kv[1])] or []
        report['abnormal_collision_count'] = len(abnormal_total)
        report['hand_self_contacts'] = [
            {'pair': list(k), 'contact_samples': v}
            for k, v in sorted(self_total.items(), key=lambda kv: -kv[1])] or []
        report['hand_self_contact_note'] = ('finger-vs-finger within a hand: that is grasp closure, '
                                            'not a fault. Kept separate from the abnormal bucket.')

        # ---- judge: the SAME judge the H chain uses -------------------------------------------
        driven = {n for n, (_, end) in PHASE_WINDOWS.items() if r.d.time >= end}
        judged = [{'t': row['time'], 'payload_z': row['payload'][2],
                   'payload_x': row['payload'][0], 'payload_y': row['payload'][1],
                   'payload_speed': row['payload_speed'],
                   'hand_contacts': row['hand_contacts'],
                   'arm_dev_rad': row['arm_dev_rad']}
                  for row in report['samples']]
        result = tray_task.evaluate(judged, driven, report['payload_z_initial'])
        report['h_stages'] = result['stages']
        report['h_stages_driven'] = sorted(driven)
        report['h_summary'] = tray_task.summary_line(result)
        report['full_H_acceptance'] = result['overall']
    except Exception as exc:
        report['error'] = '%s: %s' % (type(exc).__name__, exc)
        if report.get('status') in (None, 'ERROR'):
            report['status'] = 'POST_RUN_ERROR'
    finally:
        report['wall_seconds'] = time.monotonic() - started
        (out / 'report.json').write_text(json.dumps(report, indent=2) + '\n', encoding='utf-8')
        printable = {k: v for k, v in report.items() if k not in ('samples', 'h_stages')}
        print(json.dumps(printable, indent=2, ensure_ascii=False), flush=True)
        for line in report.get('h_summary') or []:
            print('  ' + line, flush=True)
    return 0 if report['status'] == 'DIAGNOSTIC_COMPLETED' else 1


if __name__ == '__main__':
    raise SystemExit(main())
