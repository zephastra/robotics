"""G4 #2: unload-path clearance + both shoe envelopes + second-vehicle separation.

Zero-physics KINEMATIC CLEARANCE SWEEP, judge-only: the tray is stepped along
+x off the deck in two initialized shoe poses (closed preload / open release),
and geom-pair distances are measured. This is a geometric feasibility
instrument, NOT a transport simulation and NOT control-driven; no physics step
runs. Declared expectations: CLOSED shoes BLOCK the exit (retention envelope),
OPEN shoes CLEAR the exit. Second vehicle (n2) separation is measured from
static AABBs, not eyeballed.
"""
import argparse
import hashlib
import json
import sys
from pathlib import Path

import mujoco
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT / 'experiments'), str(ROOT / 'src')]
from build_hinged_retainer_candidate import TARGET as WORLD, PRELOAD_ANGLE, RELEASE_ANGLE

STEP_M = 0.005
MARGIN_M = 0.002
N2_MARGIN_M = 0.05


def free_addr(model, name):
    j = model.joint(name + '_free').id
    return int(model.jnt_qposadr[j])


def geom_ids_of(model, body_name):
    b = model.body(body_name).id
    adr = int(model.body_geomadr[b])
    return list(range(adr, adr + int(model.body_geomnum[b])))


def min_pair_distance(model, data, ids_a, ids_b, distmax=0.5):
    best = distmax
    for ga in ids_a:
        for gb in ids_b:
            fromto = np.zeros(6)
            d = mujoco.mj_geomDistance(model, data, ga, gb, distmax, fromto)
            best = min(best, float(d))
    return best


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run-id', required=True)
    args = parser.parse_args()
    out = ROOT / 'reports' / args.run_id
    out.mkdir(exist_ok=False)
    report = dict(scope='G4_V7_UNLOAD_ENVELOPE_KINEMATIC', physics_steps=0,
                  runtime_cargo_qpos_writes='zero control-driven writes; tray qpos '
                  'repositioned for measurement only (declared)',
                  v1_complete=False,
                  world_sha256=hashlib.sha256(WORLD.read_bytes()).hexdigest(),
                  source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
                  method='zero-physics kinematic sweep, judge-only',
                  expectations=dict(closed_blocks_exit=True, open_clears_exit=True,
                                    open_margin_m=MARGIN_M, n2_margin_m=N2_MARGIN_M))
    try:
        model = mujoco.MjModel.from_xml_path(str(WORLD))
        data = mujoco.MjData(model)
        mujoco.mj_resetDataKeyframe(model, data, 0)
        mujoco.mj_forward(model, data)
        # place tray on deck crown exactly like probe_vehicle_rgbd.scene declares
        base_id = model.body('n_base_link').id
        base_pos = data.xpos[base_id].copy()
        base_rot = data.xmat[base_id].reshape(3, 3).copy()
        crown_id = model.geom('c_deck_roller_2').id
        crown = data.geom_xpos[crown_id].copy()
        crown[2] += model.geom_size[crown_id][0]
        adr = free_addr(model, 'c_payload')
        tray_x0 = (base_pos + base_rot @ np.array([2.19, 0., 0.]))[0]
        data.qpos[adr:adr + 3] = [tray_x0, base_pos[1], crown[2] + .04]
        data.qpos[adr + 3:adr + 7] = [1., 0., 0., 0.]
        shoe_jadr = [int(model.jnt_qposadr[model.joint(n + '_joint').id])
                     for n in ('c_retainer_-1', 'c_retainer_1')]
        tray_ids = geom_ids_of(model, 'c_payload')
        shoe_ids = (geom_ids_of(model, 'c_retainer_-1') + geom_ids_of(model, 'c_retainer_1'))
        mast_ids = [model.geom(n + '_mast').id for n in ('c_retainer_-1', 'c_retainer_1')]
        # deck x-extent in world frame defines the exit span
        deck_ids = [g for g in geom_ids_of(model, 'c_deck')]
        deck_x_max = max(float(data.geom_xpos[g][0]) + float(model.geom_size[g][0])
                         for g in deck_ids if model.geom_type[g] != mujoco.mjtGeom.mjGEOM_PLANE)
        sweep = dict()
        for pose, angle in (('closed', PRELOAD_ANGLE), ('open', RELEASE_ANGLE)):
            for a in shoe_jadr:
                data.qpos[a] = angle
            rows = []
            dx = 0.0
            while dx <= deck_x_max - tray_x0 + 0.45:
                data.qpos[adr] = tray_x0 + dx
                mujoco.mj_forward(model, data)
                d_shoes = min_pair_distance(model, data, tray_ids, shoe_ids + mast_ids)
                rows.append(dict(dx_m=round(dx, 4), min_shoe_distance_m=round(d_shoes, 6)))
                dx += STEP_M
            contact = [r for r in rows if r['min_shoe_distance_m'] <= 0.0]
            sweep[pose] = dict(
                shoe_angle_rad=round(float(angle), 5), samples=len(rows),
                min_distance_m=min(r['min_shoe_distance_m'] for r in rows),
                min_at_dx_m=min(rows, key=lambda r: r['min_shoe_distance_m'])['dx_m'],
                penetration_samples=len(contact),
                first_contact_dx_m=(contact[0]['dx_m'] if contact else None))
        # second vehicle static separation from everything on the first corridor
        mujoco.mj_resetDataKeyframe(model, data, 0)
        mujoco.mj_forward(model, data)
        n2_ids = geom_ids_of(model, 'n2_base_link') + geom_ids_of(model, 'n2_wheel_left') \
            + geom_ids_of(model, 'n2_wheel_right')
        corridor_ids = [g for g in range(model.ngeom)
                        if model.geom_bodyid[g] > 0 and g not in n2_ids
                        and model.geom_contype[g] != 0
                        and abs(float(data.geom_xpos[g][1])) < 0.8
                        and 7.0 < float(data.geom_xpos[g][0]) < 12.0]
        n2_sep = min_pair_distance(model, data, n2_ids, corridor_ids, distmax=1.0)
        report.update(sweep=sweep, n2_min_separation_m=round(n2_sep, 6),
                      n2_corridor_geoms=len(corridor_ids),
                      sweep_repositions=sum(sweep[p]['samples'] for p in sweep))
        open_min = sweep['open']['min_distance_m']
        closed_pen = sweep['closed']['penetration_samples']
        report['result'] = ('PASS' if (open_min >= MARGIN_M and closed_pen > 0
                                       and n2_sep >= N2_MARGIN_M) else 'FAIL')
        report['verdict_reason'] = dict(
            open_clears=bool(open_min >= MARGIN_M),
            closed_blocks=bool(closed_pen > 0),
            n2_clear=bool(n2_sep >= N2_MARGIN_M))
    except Exception as exc:
        report.update(status='ERROR', error=repr(exc))
        report['result'] = 'ERROR'
    (out / 'report.json').write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps({k: report.get(k) for k in
                      ('result', 'verdict_reason', 'sweep', 'n2_min_separation_m',
                       'n2_corridor_geoms', 'error')}, default=str))
    return 0 if report.get('result') == 'PASS' else 1


if __name__ == '__main__':
    raise SystemExit(main())
