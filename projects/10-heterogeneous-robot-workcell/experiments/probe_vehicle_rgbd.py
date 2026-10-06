"""Initialized photometric feasibility only. ZERO physics, not loaded transport.

Camera is a declared virtual vehicle-mounted rig. Each scene is initialized
separately; cargo positions never feed the reader. Truth only judges its output.
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
sys.path[:0] = [str(ROOT/'experiments'), str(ROOT/'src')]
import probe_p3_vision as pv
from workcell.tray import derive_cells, cell_centre_world
from vehicle_rgbd_reader import observe
from loaded_nav_envelope import observed_load_gate

WORLD = ROOT/'assets/world_p5_candidate_v5.xml'
SIZE = dict(red=np.array([.02, .015, .025]), blue=np.array([.015, .015, .025]))
CAMERA_POS = [2.19, -.55, 1.8]
CAMERA_QUAT = pv.look_at(CAMERA_POS, [2.19, 0., .80])
CAMERA_ROT = np.empty(9)
mujoco.mju_quat2Mat(CAMERA_ROT, np.array(CAMERA_QUAT))
CAMERA = dict(width=320, height=240, fovy_deg=30., position_in_base=CAMERA_POS,
              rotation_camera_to_base=CAMERA_ROT.reshape(3, 3).tolist())


def scene(shift=(0., 0.), occluded=False, tray_yaw=0., source_contacts=False, world_path=None,
          tray_pos_x=None):

    if not np.isfinite(tray_yaw) or abs(tray_yaw)>np.pi:
        raise ValueError('finite declared initial tray yaw required')
    spec = mujoco.MjSpec.from_file(str(WORLD if world_path is None else world_path))
    spec.body('n_base_link').add_camera(name='vehicle_load_cam',
        pos=CAMERA['position_in_base'], quat=CAMERA_QUAT, fovy=CAMERA['fovy_deg'])
    for name, cls in (('optical_red_0', 'red'), ('optical_red_1', 'red'), ('optical_blue', 'blue')):
        body = spec.worldbody.add_body(name=name)
        body.add_freejoint(name=name+'_free')
        body.add_geom(name=name+'_geom', type=(mujoco.mjtGeom.mjGEOM_BOX if cls=='red'
            else mujoco.mjtGeom.mjGEOM_CYLINDER),
            size=SIZE[cls].tolist() if cls=='red' else [.015, .025, 0.],
            mass=.03, rgba=pv.RED_RGBA if cls=='red' else pv.BLUE_RGBA,
            **({'friction':[.6,.005,.0001]} if source_contacts else {}))
    if occluded:
        # Independent initial fault scene, NOT runtime teleport of an obstacle.
        spec.body('n_base_link').add_geom(name='optical_occluder', type=mujoco.mjtGeom.mjGEOM_BOX,
            pos=[2.19, 0., 1.15], size=[.20, .35, .02], rgba=[.4, .4, .4, 1.],
            contype=0, conaffinity=0)  # visual fault only; cargo collision unchanged
    model = spec.compile()
    model.vis.global_.offwidth = CAMERA['width']
    model.vis.global_.offheight = CAMERA['height']
    # Declared camera sampling candidate: no offscreen multisampling. This
    # changes rendered pixels only; the physical geometry/solver is untouched.
    model.vis.quality.offsamples = 0
    data = mujoco.MjData(model)
    mujoco.mj_resetDataKeyframe(model, data, 0); mujoco.mj_forward(model, data)
    base_id = model.body('n_base_link').id
    base_pos = data.xpos[base_id].copy()
    base_rot = data.xmat[base_id].reshape(3, 3).copy()
    # The tray floor underside is body z-.04; declare its initial support on
    # the main deck crown. This is initialization, not a settled-contact claim.
    if tray_pos_x is None:
        crown_id = model.geom('c_deck_roller_2').id
    else:
        # G7-b: tray starts on the SOURCE BAND, not the deck. Support height is
        # declared on a band-roller crown (same +.04 floor-underside rule).
        crown_id = model.geom('c_fixed_roller_1_3').id
    crown = data.geom_xpos[crown_id].copy(); crown[2] += model.geom_size[crown_id][0]
    tray_pos = base_pos + base_rot @ np.array([2.19+shift[0], shift[1], 0.])
    if tray_pos_x is not None:
        tray_pos[0] = float(tray_pos_x)
    tray_pos[2] = crown[2]+.04
    adr = pv.free_addr(model, 'c_payload')
    data.qpos[adr:adr+3] = tray_pos;data.qpos[adr+3:adr+7]=[1.,0.,0.,0.]
    mujoco.mj_forward(model, data)
    template = derive_cells(model, data)  # catalog preparation BEFORE physics
    floor = model.geom('c_tray_floor').id; rim = model.geom('c_tray_wall_xp').id
    template['rim_height_m'] = float(model.geom_pos[rim][2]+model.geom_size[rim][2]
        -model.geom_pos[floor][2]-model.geom_size[floor][2])
    # Prepare the local catalog in its reference orientation, THEN rotate the
    # separately initialized physical scene. Derive-cells is not a rotated
    # world perception algorithm and cannot be used as one.
    cosine,sine=np.cos(tray_yaw),np.sin(tray_yaw)
    rotation=np.array([[cosine,-sine,0.],[sine,cosine,0.],[0.,0.,1.]])
    for index, name in enumerate(('optical_red_0', 'optical_red_1', 'optical_blue')):
        xyz = tray_pos+rotation @ (cell_centre_world(template,index)-tray_pos);xyz[2]+=.025
        adr = pv.free_addr(model, name)
        data.qpos[adr:adr+3] = xyz; data.qpos[adr+3:adr+7] = [1., 0., 0., 0.]
    adr=pv.free_addr(model,'c_payload')
    data.qpos[adr+3:adr+7]=[np.cos(tray_yaw/2),0.,0.,np.sin(tray_yaw/2)]
    mujoco.mj_forward(model, data)
    floor_z = float((np.array(template['deck']['centre'])-base_pos) @ base_rot[:, 2]
                    +template['deck']['size'][2])
    truth = ((data.xpos[model.body('c_payload').id]-base_pos) @ base_rot)[:2].tolist()
    return model, data, template, floor_z, truth


def main():
    parser = argparse.ArgumentParser(description=__doc__); parser.add_argument('--run-id', required=True)
    args = parser.parse_args(); out = ROOT/'reports'/args.run_id; out.mkdir(exist_ok=False)
    pv.W, pv.H = CAMERA['width'], CAMERA['height']
    contract = json.loads((ROOT/'config/joint_world_v5_loaded.profile.json').read_text())['observation_contract']
    report = dict(scope='INITIALIZED_VEHICLE_RGBD_ZERO_PHYSICS', physics_steps=0,
        runtime_qpos_writes=0, loaded_navigation='NOT_RUN', full_order='NOT_RUN', v1_complete=False,
        world_sha256=hashlib.sha256(WORLD.read_bytes()).hexdigest(), camera=CAMERA, cases={},
        assumptions=['virtual rigid overhead vehicle camera; no hardware commissioning',
                     'declared planar support height and chromatic catalog',
                     'separate initialized scenes, not runtime retention or fault recovery'])
    for name, shift, occluded in [('reference', (0., 0.), False),
                                  ('shifted', (.035, .03), False),
                                  ('occluded', (0., 0.), True)]:
        model, data, template, floor_z, truth = scene(shift, occluded)
        with mujoco.Renderer(model, height=pv.H, width=pv.W) as renderer:
            renderer.update_scene(data, camera='vehicle_load_cam'); rgb = renderer.render().copy()
            renderer.enable_depth_rendering(); renderer.update_scene(data, camera='vehicle_load_cam')
            depth = renderer.render().copy()
        Image.fromarray(rgb).save(out/(name+'.png')); np.save(out/(name+'-depth.npy'), depth)
        result = observe(rgb, depth, CAMERA, template, SIZE, roi=(1.7, 2.7, -.4, .4),
                         floor_z=floor_z, observed_sim_s=float(data.time), observed_wall_s=time.monotonic())
        error = (float(np.linalg.norm(np.array(result['tray_in_base_xy'])-truth))
                 if result['status']=='RESOLVED' else None)
        passed = ((result['status']=='UNKNOWN') if occluded else
                  result['status']=='RESOLVED' and result['counts']==dict(red=2, blue=1) and error<=.02)
        # Authorization is a distinct explicit count check + declared IDEAL contact
        # evidence. NO actual contact claim is made by this zero-step probe.
        gated = dict(result, count_verified=result.get('counts')==dict(red=2, blue=1), support='deck')
        gate = observed_load_gate(gated, contract, now_sim_s=data.time, now_wall_s=time.monotonic())
        report['cases'][name] = dict(observation=result, truth_for_judge=truth,
            localization_error_m=error, result='PASS' if passed else 'FAIL',
            synthetic_support_gate=gate, sim_s=float(data.time))
        if name=='reference':
            missing = observe(rgb, np.zeros_like(depth), CAMERA, template, SIZE,
                roi=(1.7, 2.7, -.4, .4), floor_z=floor_z,
                observed_sim_s=data.time, observed_wall_s=time.monotonic())
            report['cases']['missing_depth'] = dict(observation=missing,
                result='PASS' if missing['status']=='UNKNOWN' and missing['counts'] is None else 'FAIL')
    report['result'] = 'PASS' if all(c['result']=='PASS' for c in report['cases'].values()) else 'FAIL'
    (out/'report.json').write_text(json.dumps(report, indent=2, default=lambda x:
        x.tolist() if isinstance(x, np.ndarray) else float(x))+'\n')
    print(json.dumps(dict(result=report['result'], cases={k:v['result'] for k,v in report['cases'].items()})))
    return 0 if report['result']=='PASS' else 1


if __name__=='__main__':
    raise SystemExit(main())
