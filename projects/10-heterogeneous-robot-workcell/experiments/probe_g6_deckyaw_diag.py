"""G6 deck-yaw attribution: single straight leg vs segmented stop-go leg.

Gate event being attributed (run-05, p4-nav2-v7-05-receiver): c_deck_yaw
reached -0.011011 rad at sim 29.31, outside the declared +/-0.01 band, which
fail-closed the transport posture gate. G2 established that a 0.315 m straight
leg at 0.08 m/s does NOT excite yaw (1.13e-09 rad). This probe measures the
two candidate drivers at run-05 scale (4.63 m at 0.2 m/s):
  --segments 1 : one continuous leg   -> distance-driven hypothesis
  --segments N : N stop-go sub-legs   -> start/stop transient hypothesis
Classification happens in the reader, not here (same discipline as G2).
This is a controlled wheel-servo contrast, NOT a Nav2 replay; the Nav2
command stream is a different excitation and run-05 remains the only Nav2
evidence. Judge-only: no acceptance gates are evaluated or changed.
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
CLOSE_S, SETTLE_S, POST_S = 4.0, 3.0, 1.0
WHEEL_R_M = 0.04
YAW_BOUND_RAD = 0.01
SLIP_BOUND_M = 0.005
ROW_CAP = 8000
SEGMENT_TOL_M = 0.005
SEGMENT_STOP_SPEED_MPS = 0.01


def json_scalar(value):
    if isinstance(value, np.generic):
        return value.item()
    raise TypeError('unsupported evidence value: %r' % type(value))


def deck_support_force(model, data):
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


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run-id', required=True)
    parser.add_argument('--leg-m', type=float, default=4.63,
                        help='declared total leg, run-05 scale (birth x to station_b approach)')
    parser.add_argument('--drive-speed', type=float, default=0.2,
                        help='declared diagnostic speed (v7 base profile desired_linear_vel)')
    parser.add_argument('--segments', type=int, default=1,
                        help='1 = continuous leg (distance hypothesis); N>1 = stop-go sub-legs')
    parser.add_argument('--arm-window-s', type=float, default=40.,
                        help='drive phase sim budget; must cover leg at drive speed plus stop windows')
    parser.add_argument('--settle-stop-s', type=float, default=1.0,
                        help='held stop between segments (commands zeroed via control, no plant.stop hidden stepping)')
    args = parser.parse_args()
    if Path(args.run_id).name != args.run_id or args.segments < 1 or args.leg_m <= 0:
        parser.error('bare run ID, leg > 0, segments >= 1 required')
    out = ROOT / 'reports' / args.run_id
    out.mkdir(exist_ok=False)
    start = time.monotonic()
    rows = []
    report = dict(scope='G6_DECK_YAW_ATTRIBUTION_JUDGE_ONLY', transport='NOT_RUN',
                  v1_complete=False, runtime_cargo_qpos_writes=0,
                  declared_leg_m=args.leg_m, declared_drive_speed_mps=args.drive_speed,
                  declared_segments=args.segments, declared_stop_s=args.settle_stop_s,
                  declared_windows_s=dict(close=CLOSE_S, settle=SETTLE_S,
                                          arm=args.arm_window_s, post=POST_S),
                  sample_phase='post_step', nav_commands='NONE (direct wheel servo)',
                  gate_event_attributed='run-05 c_deck_yaw -0.011011 rad at sim 29.31',
                  classification='reader-side, not here',
                  world_sha256=hashlib.sha256(WORLD.read_bytes()).hexdigest(),
                  source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
                  assumptions=['ideal joint/tactile signals', 'initialized tray on deck',
                               'only first vehicle; direct wheel servo, NOT a Nav2 replay'])
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
                     targets=[], seg=0, seg_phase='move', stop_until=None,
                     stop_command_sim=None, physical_stop_sim=None,
                     stop_command_tick=None, physical_stop_tick=None,
                     yaw_at_seg_start=None, x_at_seg_start=None,
                     per_segment=[])
        yaw_q, yaw_d = joint_slots(model, 'c_deck_yaw')
        sly_q, sly_d = joint_slots(model, 'c_deck_slide_y')
        slx_q, _ = joint_slots(model, 'c_deck_slide')

        def control():
            base = plant.park_control()
            if state['phase'] == 'drive' and state['targets']:
                target = state['targets'][state['seg']]
                if state['seg_phase'] == 'stop':
                    for side in state['wheel_rate']:
                        state['wheel_rate'][side] = 0.0
                else:
                    error = target - plant.chassis_x()
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
        while data.time < CLOSE_S + SETTLE_S + args.arm_window_s + POST_S + 1e-9:
            if time.monotonic() - start > 900:
                raise TimeoutError('g6 yaw attribution wall budget')
            if state['phase'] == 'settle' and reference is None:
                reference = belt.tray_in_deck_frame(
                    model, data, model.body('c_payload').id, model.body('c_deck').id)
            if data.time >= phase_deadline:
                if state['phase'] == 'close':
                    state['phase'] = 'settle'
                    phase_deadline = data.time + SETTLE_S
                elif state['phase'] == 'settle':
                    state['phase'] = 'drive'
                    leg = args.leg_m / args.segments
                    state['targets'] = [plant.chassis_x() + leg * (k + 1)
                                        for k in range(args.segments)]
                    state['yaw_at_seg_start'] = float(data.qpos[yaw_q])
                    state['x_at_seg_start'] = float(plant.chassis_x())
                    phase_deadline = data.time + args.arm_window_s
                elif state['phase'] == 'drive':
                    state['stop_command_sim'] = float(data.time)
                    state['stop_command_tick'] = len(rows)
                    plant.stop()
                    state['phase'] = 'post'
                    phase_deadline = data.time + POST_S
                elif state['phase'] == 'post':
                    break
            owner.step(control)
            now_x = float(plant.chassis_x())
            speed = (now_x - prev_x) / max(float(model.opt.timestep), 1e-9)
            prev_x = now_x
            contact_count, contact_force = deck_support_force(model, data)
            rel = belt.tray_in_deck_frame(model, data, model.body('c_payload').id,
                                          model.body('c_deck').id)
            seg_note = None
            if state['phase'] == 'drive' and state['targets']:
                target = state['targets'][state['seg']]
                if state['seg_phase'] == 'move':
                    error = abs(target - now_x)
                    if (error <= SEGMENT_TOL_M and abs(speed) <= SEGMENT_STOP_SPEED_MPS
                            and state['yaw_at_seg_start'] is not None):
                        seg_note = dict(segment=state['seg'] + 1, end_x_m=now_x,
                                        end_yaw_rad=float(data.qpos[yaw_q]),
                                        seg_yaw_drift_rad=float(data.qpos[yaw_q]) - state['yaw_at_seg_start'],
                                        seg_len_m=now_x - state['x_at_seg_start'])
                        state['per_segment'].append(seg_note)
                        state['seg'] += 1
                        if state['seg'] >= len(state['targets']):
                            state['stop_command_sim'] = float(data.time)
                            state['stop_command_tick'] = len(rows)
                            plant.stop()
                            state['phase'] = 'post'
                            phase_deadline = data.time + POST_S
                        else:
                            state['seg_phase'] = 'stop'
                            state['stop_until'] = float(data.time) + args.settle_stop_s
                            state['yaw_at_seg_start'] = float(data.qpos[yaw_q])
                            state['x_at_seg_start'] = float(now_x)
                elif state['seg_phase'] == 'stop' and data.time >= (state['stop_until'] or 0.):
                    state['seg_phase'] = 'move'
            row = dict(
                sim_s=float(data.time), phase=state['phase'],
                seg_phase=state['seg_phase'], segment=state['seg'] + 1,
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

        arm_rows = [r for r in rows if r['phase'] == 'drive']
        yaw_series = [r['yaw_rad'] for r in arm_rows]
        report.update(
            phase_seen=sorted({r['phase'] for r in rows}),
            sim_end_s=float(data.time), ticks=len(rows),
            yaw_bound_rad=YAW_BOUND_RAD, slip_bound_m=SLIP_BOUND_M,
            yaw_summary=dict(start_rad=(yaw_series[0] if yaw_series else None),
                             end_rad=(yaw_series[-1] if yaw_series else None),
                             min_rad=min(yaw_series, default=None),
                             max_rad=max(yaw_series, default=None),
                             net_drift_rad=(yaw_series[-1] - yaw_series[0] if yaw_series else None),
                             max_abs_rad=max((abs(v) for v in yaw_series), default=None)),
            per_segment=state['per_segment'],
            final_slip_m=max((r['slip_from_ref_m'] for r in rows
                              if r['slip_from_ref_m'] is not None), default=None),
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
