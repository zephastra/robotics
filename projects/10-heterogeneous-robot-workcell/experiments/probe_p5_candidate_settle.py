"""Re-run the existing P3 settling criterion on the approved P5 candidate.

Two independently initialized diagnostic arms, not a continuous order. Same 3 s
and 0.10 per-instance DOF speed criterion as evaluate_p3_world.check_settles.
That criterion is NOT the 0.01 m/s mechanical-stop contract. No old PASS inherited.
"""
import argparse
import hashlib
import json
import re
import sys
from pathlib import Path

import numpy as np
import mujoco

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'experiments'))
import evaluate_p3_world as existing

WORLD = ROOT / 'assets/world_p5_candidate.xml'


def explain(world):
    """Separate initialized diagnostic replay; never used to produce control targets."""
    model = world.model
    data = mujoco.MjData(model)
    data.qpos[:] = world.home
    mujoco.mj_forward(model, data)
    initial_contacts = []
    for contact in data.contact[:data.ncon]:
        names = [model.geom(g).name for g in (contact.geom1, contact.geom2)]
        if contact.dist < 0 and any((name or '').startswith('a_') for name in names):
            initial_contacts.append({'geoms': names, 'depth_m': float(-contact.dist)})
    for _ in range(int(existing.SETTLE_SECONDS / model.opt.timestep)):
        data.ctrl[:] = existing.mw.home_hold_ctrl(model, world.home, data.qpos, data.qvel)
        mujoco.mj_step(model, data)
    velocities = []
    for joint in range(model.njnt):
        body = model.body(model.jnt_bodyid[joint]).name or ''
        if not body.startswith('a_'):
            continue
        dof = int(model.jnt_dofadr[joint])
        count = 6 if model.jnt_type[joint] == mujoco.mjtJoint.mjJNT_FREE else 1
        velocities.append({'joint': model.joint(joint).name, 'body': body,
                           'qvel': data.qvel[dof:dof + count].tolist(),
                           'peak_abs': float(np.max(np.abs(data.qvel[dof:dof + count])))})
    return {'scope': 'INITIALIZED_REPLAY_DIAGNOSTIC_ONLY',
            'initial_arm_contacts': initial_contacts,
            'final_arm_velocities': sorted(velocities, key=lambda x: -x['peak_abs']),
            'final_stock_part_xyz': data.xpos[model.body('a_payload').id].tolist()}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run-id', required=True)
    parser.add_argument('--world', default=WORLD.name)
    args = parser.parse_args()
    out = ROOT / 'reports' / args.run_id
    out.mkdir(parents=True, exist_ok=False)
    if not re.fullmatch(r'world_p5_candidate(?:_v[0-9]+)?\.xml', args.world):
        parser.error('world must be a versioned candidate filename')
    path = WORLD.parent / args.world
    world = existing.World(path)
    world.keys += ('c2',)  # cover the new deck, rather than silently ignoring its DOFs
    normal, negative = existing.Results(), existing.Results()
    existing.check_settles(world, normal)
    existing.check_settles(world, negative, qvel0=np.full(world.model.nv, 5.0))
    checks = {'normal_candidate_settles': normal.rows[0]['status'],
              'unstable_initialization_is_rejected':
                  'PASS' if negative.rows[0]['status'] == 'FAIL' else 'FAIL'}
    report = {'scope': 'CANDIDATE_SETTLING_ONLY', 'checks': checks,
              'normal': normal.rows, 'injected_initial_velocity': negative.rows,
              'world': str(path.relative_to(ROOT)),
              'world_sha256': hashlib.sha256(path.read_bytes()).hexdigest(),
              'judge': 'evaluate_p3_world.check_settles (unchanged)',
              'judge_sha256': hashlib.sha256(Path(existing.__file__).read_bytes()).hexdigest(),
              'diagnostic_result': 'PASS' if all(v == 'PASS' for v in checks.values()) else 'FAIL',
              'mechanical_stop': 'NOT_RUN', 'dynamic_grasp': 'NOT_RUN',
              'full_order': 'NOT_RUN', 'v1_complete': False}
    if normal.rows[0]['status'] == 'FAIL':
        report['failure_diagnostics'] = explain(world)
    (out / 'acceptance.json').write_text(json.dumps(report, indent=2) + '\n', encoding='utf-8')
    print(json.dumps(report, indent=2))
    return 0 if report['diagnostic_result'] == 'PASS' else 1


if __name__ == '__main__':
    raise SystemExit(main())
