"""Initialized load + continuously gated short motion, NOT Nav2/full order.

Only WorldOwner advances one physical world. Cargo truth is logged for the
judge, never supplied to vision or wheel control. Contact is ideal touch.
"""
import argparse
import hashlib
import json
import signal
import sys
import time
from pathlib import Path
import mujoco
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT/'experiments'), str(ROOT/'src')]
from probe_vehicle_rgbd import scene, CAMERA, SIZE, WORLD
from vehicle_rgbd_reader import observe as read_frame
from loaded_nav_envelope import observed_load_gate
from load_motion_gate import LoadMotionGate
from w4_plant import LogisticsPlant
from world_owner import WorldOwner
from humanoid_posture_permit import HumanoidPosturePermit
from probe_h3_w5 import H1_runtime
from humanoid007.runtime import OPEN
from make_joint_nav_profile import posture_check
import probe_p3_vision as pv


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run-id', required=True)
    parser.add_argument('--missing-depth', action='store_true')
    args = parser.parse_args()
    out = ROOT/'reports'/args.run_id; out.mkdir(exist_ok=False)
    started = time.monotonic(); owner = None; view = None; rows = []; frames = []
    report = dict(scope='INITIALIZED_LOAD_CONTINUOUS_RGBD_SHORT_MOTION',
        world_sha256=hashlib.sha256(WORLD.read_bytes()).hexdigest(),
        source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        loaded_navigation='NOT_RUN', full_order='NOT_RUN', v1_complete=False,
        runtime_qpos_writes=0, missing_depth=args.missing_depth,
        assumptions=['declared vehicle camera and support plane', 'ideal solver contact sensor',
                     'catalog chromatic parts initialized, NOT actual H supply/arm loading',
                     'original open-loop wheel-rate diagnostic, NOT Nav2/3D collision avoidance',
                     'camera renderer shadows disabled; physics/contact geometry unchanged'])
    def interrupted(signum, frame):
        raise KeyboardInterrupt('owned probe interrupted')
    signal.signal(signal.SIGTERM, interrupted)
    try:
        pv.W, pv.H = CAMERA['width'], CAMERA['height']
        model, data, template, floor_z, _ = scene()
        plant = LogisticsPlant(model=model, data=data, world=WORLD)
        robot = H1_runtime(WORLD, model=model, data=data)
        arm = robot.policy.default[robot.arm_ids].copy()
        contract = json.loads((ROOT/'config/joint_world_v5_loaded.profile.json').read_text())['observation_contract']
        gate = dict(allowed=False, reason_code='LOAD_ENVELOPE_UNKNOWN'); sequence = [0]
        movement_gate = LoadMotionGate(contract)
        latest = {}; motion_commands = [0]
        def permission():
            sequence[0] += 1
            return dict(owner='world', epoch=1, generation=1, sequence=sequence[0],
                observed_wall_s=time.monotonic(), cancel_requested=False,
                zone_clear=True, stop_chain_healthy=True)
        permit = HumanoidPosturePermit(permission, model=model, data=data,
            owner='world', epoch=1, generation=1, max_age_s=.5)
        owner = WorldOwner(model, data, permit); owner.attach(plant)
        view = mujoco.Renderer(model, height=pv.H, width=pv.W)
        next_frame = 0.; next_sample = 0.; signs = plant._wheel_signs()
        def control():
            posture = posture_check(model, data)
            if not posture['allowed']:
                raise RuntimeError(posture['reason_code'])
            rate = 1. if 2. <= data.time < 5. else 0.
            authorized = movement_gate.check(latest, now_sim_s=float(data.time),
                now_wall_s=time.monotonic(), motion_requested=bool(rate))
            if not authorized['motion_allowed']: rate = 0.
            if rate: motion_commands[0] += 1
            base = plant._control(lambda side: plant._wheel_rate_torque(side, rate*signs[side]))
            result = robot.control(np.zeros(3), arm, {s: OPEN for s in ('left', 'right')},
                                   stationary=False, stopping=True, base_control=base)
            robot.tick += 1
            return result
        while data.time < 10.:
            if time.monotonic()-started > 110.:
                raise TimeoutError('motion wall budget')
            # Wall freshness is independent of simulation progress. A slow
            # physics/control loop must refresh BEFORE its permit expires.
            if (data.time >= next_frame or (latest and time.monotonic()-
                    latest['observed_wall_s'] >= contract['max_wall_age_s']/2)):
                stamp = time.monotonic()
                view.disable_depth_rendering(); view.update_scene(data, camera='vehicle_load_cam')
                view.scene.flags[mujoco.mjtRndFlag.mjRND_SHADOW] = False
                view.scene.flags[mujoco.mjtRndFlag.mjRND_REFLECTION] = False
                rgb = view.render().copy()
                view.enable_depth_rendering(); view.update_scene(data, camera='vehicle_load_cam')
                view.scene.flags[mujoco.mjtRndFlag.mjRND_SHADOW] = False
                view.scene.flags[mujoco.mjtRndFlag.mjRND_REFLECTION] = False
                depth = view.render().copy()
                rendered_at = time.monotonic()
                if args.missing_depth: depth[:] = 0.
                measured = read_frame(rgb, depth, CAMERA, template, SIZE,
                    roi=(1.7, 2.7, -.4, .4), floor_z=floor_z,
                    observed_sim_s=float(data.time), observed_wall_s=stamp)
                measured['count_verified'] = measured.get('counts') == dict(red=2, blue=1)
                measured['support'] = 'deck' if plant.tray_support()==['deck'] else 'UNKNOWN'
                latest = measured
                gate = observed_load_gate(latest, contract, now_sim_s=float(data.time),
                                          now_wall_s=time.monotonic())
                frames.append(dict(sim_s=float(data.time), processing_wall_s=time.monotonic()-stamp,
                    rendering_wall_s=rendered_at-stamp, reading_wall_s=time.monotonic()-rendered_at,
                    gate=gate, observation=measured))
                next_frame = float(data.time)+.2
            if data.time >= next_sample:
                bid = model.body('n_base_link').id; rotation = data.xmat[bid].reshape(3, 3)
                relative = (data.xpos[model.body('c_payload').id]-data.xpos[bid]) @ rotation
                rows.append(dict(sim_s=float(data.time), vehicle_xyz=data.xpos[bid].tolist(),
                    tray_xyz=data.xpos[model.body('c_payload').id].tolist(),
                    tray_relative_judge_only=relative.tolist(), support=plant.tray_support(),
                    vehicle_speed=float(np.linalg.norm(data.qvel[[model.jnt_dofadr[model.joint(n).id]
                        for n in ('n_slide_x', 'n_slide_y', 'n_slide_z')]]))))
                next_sample += .1
            owner.step(control)
        moving = [r for r in rows if r['sim_s']>=2.]
        held = [r for r in rows if r['sim_s']>=6.]
        displacement = float(np.linalg.norm(np.array(moving[-1]['vehicle_xyz'])[:2]-
                                            np.array(moving[0]['vehicle_xyz'])[:2]))
        slip = max(float(np.linalg.norm(np.array(r['tray_relative_judge_only'])[:2]-
                    np.array(moving[0]['tray_relative_judge_only'])[:2])) for r in moving)
        speed = max(r['vehicle_speed'] for r in held)
        drift = max(float(np.linalg.norm(np.array(r['vehicle_xyz'])-held[0]['vehicle_xyz'])) for r in held)
        checks = dict(actual_motion=displacement>.1, retained_slip=slip<=.005,
            physical_stop=speed<=.01 and drift<=.005,
            live_rgbd_all_authorized=movement_gate.latched_reason is None,
            final_rgbd_resolved=latest.get('status')=='RESOLVED' and latest.get('counts')==dict(red=2,blue=1),
            actual_support=all(r['support']==['deck'] for r in moving),
            no_safety_freeze=owner.frozen is None, same_world=plant.data is robot.d and plant.model is robot.m)
        report.update(result='PASS' if all(checks.values()) else 'FAIL',
            checks={k:'PASS' if v else 'FAIL' for k,v in checks.items()},
            displacement_m=displacement, max_slip_m=slip, stop_speed_mps=speed, stop_drift_m=drift)
        report['motion_refusal']=movement_gate.latched_reason
        if args.missing_depth:
            report['negative_safety_result']='PASS' if (motion_commands[0]==0 and
                checks['physical_stop'] and movement_gate.latched_reason=='LOAD_ENVELOPE_UNKNOWN') else 'FAIL'
    except (Exception, KeyboardInterrupt) as exc:
        report.update(result='FAIL', error=type(exc).__name__+': '+str(exc))
        if args.missing_depth:
            report['negative_safety_result'] = ('PASS' if owner and owner.frozen
                and motion_commands[0]==0 and latest.get('status')=='UNKNOWN' else 'FAIL')
            report['physical_stop'] = 'NOT_RUN_SIM_FREEZE_ONLY'
    finally:
        if view is not None: view.close()
        report.update(truth_judge_only=rows, sensor_frames=frames,
            simulation_frozen=owner.frozen if owner else None, owner_steps=owner.steps if owner else 0,
            wall_s=time.monotonic()-started,
            nonzero_commands=motion_commands[0] if 'motion_commands' in locals() else 0)
        (out/'report.json').write_text(json.dumps(report, indent=2,
            default=lambda x: x.tolist() if isinstance(x, np.ndarray) else float(x))+'\n')
        print(json.dumps({k:v for k,v in report.items() if k not in ('truth_judge_only', 'sensor_frames')}, indent=2))
    return 0 if report.get('result')=='PASS' else 1


if __name__=='__main__':
    raise SystemExit(main())
