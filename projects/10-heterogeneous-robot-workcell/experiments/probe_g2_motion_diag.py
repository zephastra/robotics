"""G2 time-aligned motion diagnosis: static-loaded vs straight-drive-loaded.

Continuation doc section 5: separate the three root causes (2 ms contact gaps /
>5 mm tray slip / deck yaw beyond +/-0.01 rad) with per-tick ALIGNED evidence.
`c_deck_yaw` is a mechanical DOF of the deck on the chassis (stiffness 40,
damping 8, range +/-0.05 rad) -- NOT vehicle map yaw; turning the robot is not
a deck failure. The controlled pair differs ONLY in whether the wheels are
commanded. Sampling is POST-STEP on every physics tick; cargo truth is
judge-only; there are zero runtime cargo qpos writes; there is no Nav2 stack
here (direct wheel-rate servo through the plant's own law, declared).
"""
import argparse
import hashlib
import json
from pathlib import Path
import sys
import time

import mujoco
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'experiments'))
sys.path.insert(0, str(ROOT / 'src'))
from probe_vehicle_rgbd import scene
from probe_vehicle_nav2 import json_value, exception_chain
from build_hinged_retainer_candidate import TARGET as WORLD, PRELOAD_ANGLE
from contact_model_candidate import enable_multiccd
from w4_plant import LogisticsPlant
from world_owner import WorldOwner
from humanoid_posture_permit import HumanoidPosturePermit, posture_status
from probe_h3_w5 import H1_runtime
from humanoid007.runtime import OPEN
import probe_p4_belt as belt

NAMES = ('c_retainer_-1', 'c_retainer_1')
CLOSE_S, SETTLE_S, ARM_S, POST_S = 4.0, 3.0, 4.0, 1.0
WHEEL_R_M = 0.04
YAW_BOUND_RAD = 0.01
SLIP_BOUND_M = 0.005
ROW_CAP = 6000


def json_scalar(value):
    if isinstance(value, np.generic):
        return value.item()
    raise TypeError('unsupported evidence value: %r' % type(value))


def deck_support_force(model, data):
    """Total normal force over every deck-geom <-> payload-geom contact pair."""
    tray = model.body('c_payload').id
    adr = int(model.body_geomadr[tray])
    tray_geoms = set(range(adr, adr + int(model.body_geomnum[tray])))
    total = 0.0
    count = 0
    for i, c in enumerate(data.contact[:data.ncon]):
        g1, g2 = int(c.geom1), int(c.geom2)
        for deck_g, other in ((g1, g2), (g2, g1)):
            name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, deck_g) or ''
            if name.startswith('c_deck') and other in tray_geoms:
                f = np.empty(6)
                mujoco.mj_contactForce(model, data, i, f)
                total += max(0.0, float(f[0]))
                count += 1
                break
    return count, total


def wheel_joint_dof(model, side):
    return int(model.jnt_dofadr[model.joint('n_wheel_%s_joint' % side).id])


def joint_slots(model, name):
    j = model.joint(name).id
    return int(model.jnt_qposadr[j]), int(model.jnt_dofadr[j])


def shoe_measurement(model, data, name):
    tray = model.body('c_payload').id
    geom = model.geom(name + '_shoe').id
    joint = model.joint(name + '_joint').id
    normal = 0.0
    count = 0
    for i, c in enumerate(data.contact[:data.ncon]):
        pair = (int(c.geom1), int(c.geom2))
        if geom in pair and tray in [int(model.geom_bodyid[g]) for g in pair]:
            f = np.empty(6)
            mujoco.mj_contactForce(model, data, i, f)
            normal += max(0.0, float(f[0]))
            count += 1
    return dict(position_rad=float(data.qpos[model.jnt_qposadr[joint]]),
                speed_radps=float(data.qvel[model.jnt_dofadr[joint]]),
                tray_contacts=int(count), normal_force_n=normal)


def gaps_and_extremes(rows, phase):
    """Zero-support-force spans (ms) and extremes inside one phase window."""
    window = [r for r in rows if r['phase'] == phase]
    gaps = []
    run = 0
    dt = None
    for a, b in zip(window, window[1:]):
        dt = b['sim_s'] - a['sim_s']
        run = run + 1 if a['support_force_n'] <= 0.0 else 0
        if b['support_force_n'] > 0.0 and run:
            gaps.append(round(run * dt * 1000.0, 6))
            run = 0
    if window and window[-1]['support_force_n'] <= 0.0 and run:
        gaps.append(round((run + 1) * (dt or 0.0) * 1000.0, 6))
    return dict(
        samples=len(window),
        max_abs_yaw_rad=max((abs(r['yaw_rad']) for r in window), default=None),
        max_abs_slide_y_m=max((abs(r['slide_y_m']) for r in window), default=None),
        min_support_force_n=min((r['support_force_n'] for r in window), default=None),
        zero_force_gap_ms=gaps,
        max_slip_m=max((r['slip_from_ref_m'] for r in window
                        if r['slip_from_ref_m'] is not None), default=None),
        peak_chassis_speed_mps=max((abs(r['chassis_speed_mps']) for r in window),
                                   default=None),
    )


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run-id', required=True)
    parser.add_argument('--arm', choices=('static', 'drive'), required=True,
                        help='the single factor under test: are the wheels commanded')
    parser.add_argument('--drive-speed', type=float, default=0.08,
                        help='declared diagnostic speed (doc 6.2: 0.08 m/s is a '
                             'diagnostic condition, not a release default)')
    parser.add_argument('--leg-m', type=float, default=0.5)
    args = parser.parse_args()
    if Path(args.run_id).name != args.run_id:
        parser.error('bare run ID required')
    arm_phase = 'drive' if args.arm == 'drive' else 'static'
    out = ROOT / 'reports' / args.run_id
    out.mkdir(exist_ok=False)
    start = time.monotonic()
    rows = []
    report = dict(scope='G2_MOTION_DIAGNOSIS_JUDGE_ONLY', transport='NOT_RUN',
                  v1_complete=False, runtime_cargo_qpos_writes=0, arm=args.arm,
                  declared_drive_speed_mps=args.drive_speed,
                  declared_leg_m=args.leg_m, declared_windows_s=dict(
                      close=CLOSE_S, settle=SETTLE_S, arm=ARM_S, post=POST_S),
                  sample_phase='post_step', nav_commands='NONE (direct wheel servo)',
                  slip_reference='tray-in-deck frame at settle start (judge-only)',
                  world_sha256=hashlib.sha256(WORLD.read_bytes()).hexdigest(),
                  source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
                  assumptions=['ideal joint/tactile signals', 'initialized tray on deck',
                               'only first vehicle; no swept-volume qualification'])
    try:
        model, data, template, floor_z, _ = scene(shift=(-.15, 0.), occluded=False,
            source_contacts=True, world_path=WORLD)
        report['contact_candidate'] = enable_multiccd(model)
        plant = LogisticsPlant(model=model, data=data, world=WORLD)
        sequence = [0]

        def observation():
            sequence[0] += 1
            return dict(owner='world', epoch=1, generation=1, sequence=sequence[0],
                        observed_wall_s=time.monotonic(), cancel_requested=False,
                        zone_clear=True, stop_chain_healthy=True)
        permit = HumanoidPosturePermit(observation, model=model, data=data,
            owner='world', epoch=1, generation=1, max_age_s=.5)
        owner = WorldOwner(model, data, permit)
        owner.attach(plant)
        robot = H1_runtime(WORLD, model=model, data=data)
        arm = robot.policy.default[robot.arm_ids].copy()
        drives = [model.actuator(name + '_drive').id for name in NAMES]
        signs = plant._wheel_signs()
        state = dict(phase='close', wheel_rate={s: 0.0 for s in signs},
                     x_target=None, stop_command_sim=None, physical_stop_sim=None,
                     stop_command_tick=None, physical_stop_tick=None)
        yaw_q, yaw_d = joint_slots(model, 'c_deck_yaw')
        sly_q, sly_d = joint_slots(model, 'c_deck_slide_y')
        slx_q, _ = joint_slots(model, 'c_deck_slide')

        def control():
            base = plant.park_control()
            if state['phase'] == 'drive':
                error = state['x_target'] - plant.chassis_x()
                command = np.sign(error) * min(args.drive_speed, abs(error) / .4 + .01)
                for side, actuator in plant.wheel_actuators.items():
                    rate = (command / WHEEL_R_M) * signs[side]
                    state['wheel_rate'][side] = rate
                    base[actuator] = plant._wheel_rate_torque(side, rate)
            else:
                for side in state['wheel_rate']:
                    state['wheel_rate'][side] = 0.0
            result = robot.control(np.zeros(3), arm,
                {s: OPEN for s in ('left', 'right')},
                stationary=False, stopping=True, base_control=base)
            for drive in drives:
                result[drive] = PRELOAD_ANGLE
            return result

        reference = None
        prev_x = float(plant.chassis_x())
        phase_deadline = data.time + CLOSE_S
        while data.time < CLOSE_S + SETTLE_S + ARM_S + POST_S + 1e-9:
            if time.monotonic() - start > 150:
                raise TimeoutError('g2 wall budget')
            if state['phase'] == 'settle' and reference is None:
                reference = belt.tray_in_deck_frame(
                    model, data, model.body('c_payload').id, model.body('c_deck').id)
            if data.time >= phase_deadline:
                if state['phase'] == 'close':
                    state['phase'] = 'settle'
                    phase_deadline = data.time + SETTLE_S
                elif state['phase'] == 'settle':
                    state['phase'] = arm_phase
                    if args.arm == 'drive':
                        state['x_target'] = plant.chassis_x() + args.leg_m
                    phase_deadline = data.time + ARM_S
                elif state['phase'] == arm_phase:
                    if args.arm == 'drive':
                        state['stop_command_sim'] = float(data.time)
                        state['stop_command_tick'] = len(rows)
                        plant.stop()
                    state['phase'] = 'post'
                    phase_deadline = data.time + POST_S
                else:
                    break
            owner.step(control)
            now_x = float(plant.chassis_x())
            speed = (now_x - prev_x) / max(float(model.opt.timestep), 1e-9)
            prev_x = now_x
            contact_count, contact_force = deck_support_force(model, data)
            rel = belt.tray_in_deck_frame(model, data, model.body('c_payload').id,
                                          model.body('c_deck').id)
            row = dict(
                sim_s=float(data.time), phase=state['phase'],
                chassis_x_m=now_x, chassis_speed_mps=float(speed),
                yaw_rad=float(data.qpos[yaw_q]), yaw_rate_radps=float(data.qvel[yaw_d]),
                slide_y_m=float(data.qpos[sly_q]), slide_y_rate_mps=float(data.qvel[sly_d]),
                slide_x_m=float(data.qpos[slx_q]),
                support_contacts=int(contact_count), support_force_n=contact_force,
                rel_tray_in_deck_m=[float(rel[0]), float(rel[1]), float(rel[2])],
                slip_from_ref_m=(None if reference is None else float(np.linalg.norm(
                    np.array(rel[:2]) - np.array(reference[:2])))),
                posture=posture_status(float(data.qpos[2]),
                                       [float(v) for v in data.qpos[3:7]]),
                humanoid_base_z_m=float(data.qpos[2]),
                commanded_wheel_rate_radps=dict(state['wheel_rate']),
                actual_wheel_rate_radps={s: float(data.qvel[wheel_joint_dof(model, s)])
                                         for s in state['wheel_rate']},
            )
            for name in NAMES:
                row['shoe_' + name] = shoe_measurement(model, data, name)
            rows.append(row)
            if (state['physical_stop_tick'] is None
                    and state['stop_command_tick'] is not None and abs(speed) <= 0.01):
                state['physical_stop_tick'] = len(rows) - 1
                state['physical_stop_sim'] = float(data.time)
        plant.stop()

        report.update(
            phase_seen=sorted({r['phase'] for r in rows}),
            sim_end_s=float(data.time), ticks=len(rows),
            yaw_bound_rad=YAW_BOUND_RAD, slip_bound_m=SLIP_BOUND_M,
            windows=dict(close=gaps_and_extremes(rows, 'close'),
                         settle=gaps_and_extremes(rows, 'settle'),
                         arm=gaps_and_extremes(rows, arm_phase),
                         post=gaps_and_extremes(rows, 'post')),
            stop_command_sim_s=state['stop_command_sim'],
            physical_stop_sim_s=state['physical_stop_sim'],
            verdict_pending='classification happens in the reader, not here')
    except Exception as exc:
        report.update(status='ERROR', error=exception_chain(exc))
    else:
        report['status'] = 'COMPLETED'
    finally:
        report['wall_s'] = time.monotonic() - start
        report['rows_judge_only'] = rows[-ROW_CAP:]
        (out / 'report.json').write_text(
            json.dumps(report, indent=2, default=json_value) + '\n')
        print(json.dumps({k: v for k, v in report.items()
                          if k != 'rows_judge_only'}, indent=2))
    return 0 if report.get('status') == 'COMPLETED' else 1


if __name__ == '__main__':
    raise SystemExit(main())
