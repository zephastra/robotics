"""G4 integration qualification: v7 RGB-D vision positive/occluded-negative.

Same judging machine as probe_vehicle_rgbd.py (observe + observed_load_gate,
same four scenes, thresholds inherited unchanged). The ONLY factor changed is
the world: world_p5_candidate_v7_hinged_retainer.xml. Zero physics steps;
initialized photometric feasibility only. The observation contract is declared
as inherited from the v5 loaded profile until G5 mints v7's own config.
"""
import argparse
import hashlib
import json
import sys
import time
from pathlib import Path

import mujoco
import numpy as np
from PIL import Image

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT / 'experiments'), str(ROOT / 'src')]
import probe_p3_vision as pv
import probe_vehicle_rgbd as rgbd
from vehicle_rgbd_reader import observe
from loaded_nav_envelope import observed_load_gate

WORLD = ROOT / 'assets/world_p5_candidate_v7_hinged_retainer.xml'
CAMERA = rgbd.CAMERA
SIZE = rgbd.SIZE


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run-id', required=True)
    args = parser.parse_args()
    out = ROOT / 'reports' / args.run_id
    out.mkdir(exist_ok=False)
    pv.W, pv.H = CAMERA['width'], CAMERA['height']
    contract = json.loads(
        (ROOT / 'config/joint_world_v5_loaded.profile.json').read_text())['observation_contract']
    report = dict(scope='G4_V7_INITIALIZED_RGBD_ZERO_PHYSICS', physics_steps=0,
                  runtime_qpos_writes=0, loaded_navigation='NOT_RUN',
                  full_order='NOT_RUN', v1_complete=False,
                  world_sha256=hashlib.sha256(WORLD.read_bytes()).hexdigest(),
                  source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
                  camera=CAMERA, cases={},
                  contract_source='config/joint_world_v5_loaded.profile.json (inherited)',
                  assumptions=['virtual rigid overhead vehicle camera; no hardware commissioning',
                               'declared planar support height and chromatic catalog',
                               'separate initialized scenes, not runtime retention or fault recovery',
                               'v5 observation contract inherited; v7 own config is G5 scope'])
    for name, shift, occluded in [('reference', (0., 0.), False),
                                  ('shifted', (.035, .03), False),
                                  ('occluded', (0., 0.), True)]:
        model, data, template, floor_z, truth = rgbd.scene(shift, occluded,
                                                           world_path=WORLD)
        with mujoco.Renderer(model, height=pv.H, width=pv.W) as renderer:
            renderer.update_scene(data, camera='vehicle_load_cam')
            rgb = renderer.render().copy()
            renderer.enable_depth_rendering()
            renderer.update_scene(data, camera='vehicle_load_cam')
            depth = renderer.render().copy()
        Image.fromarray(rgb).save(out / (name + '.png'))
        np.save(out / (name + '-depth.npy'), depth)
        result = observe(rgb, depth, CAMERA, template, SIZE, roi=(1.7, 2.7, -.4, .4),
                         floor_z=floor_z, observed_sim_s=float(data.time),
                         observed_wall_s=time.monotonic())
        error = (float(np.linalg.norm(np.array(result['tray_in_base_xy']) - truth))
                 if result['status'] == 'RESOLVED' else None)
        passed = ((result['status'] == 'UNKNOWN') if occluded else
                  result['status'] == 'RESOLVED'
                  and result['counts'] == dict(red=2, blue=1) and error <= .02)
        gated = dict(result, count_verified=result.get('counts') == dict(red=2, blue=1),
                     support='deck')
        gate = observed_load_gate(gated, contract, now_sim_s=data.time,
                                  now_wall_s=time.monotonic())
        report['cases'][name] = dict(observation=result, truth_for_judge=truth,
                                     localization_error_m=error,
                                     result='PASS' if passed else 'FAIL',
                                     synthetic_support_gate=gate, sim_s=float(data.time))
        if name == 'reference':
            missing = observe(rgb, np.zeros_like(depth), CAMERA, template, SIZE,
                              roi=(1.7, 2.7, -.4, .4), floor_z=floor_z,
                              observed_sim_s=data.time, observed_wall_s=time.monotonic())
            report['cases']['missing_depth'] = dict(
                observation=missing,
                result='PASS' if missing['status'] == 'UNKNOWN'
                and missing['counts'] is None else 'FAIL')
    report['result'] = ('PASS' if all(c['result'] == 'PASS'
                                      for c in report['cases'].values()) else 'FAIL')
    (out / 'report.json').write_text(json.dumps(report, indent=2, default=lambda x:
        x.tolist() if isinstance(x, np.ndarray) else float(x)) + '\n')
    print(json.dumps(dict(result=report['result'],
                          cases={k: v['result'] for k, v in report['cases'].items()})))
    return 0 if report['result'] == 'PASS' else 1


if __name__ == '__main__':
    raise SystemExit(main())
