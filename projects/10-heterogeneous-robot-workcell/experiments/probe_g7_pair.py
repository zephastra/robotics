# -*- coding: utf-8 -*-
"""G7 paired short experiments: separate HOLD capability from RECEIVE capability.

Per the 2026-10-07 external review (adopted): stop inferring the whole mechanism
from one full-chain failure. Two arms, each short, each with its own verdict rows:

  hold    (A): declared teleport init, drive cycle accel/cruise/brake/stop-hold/
               gentle-arc/stop -- NEVER enters the funnel window (x <= 8.0 guard).
               Judges the 5 mm hold budget per phase (restored original criterion).
  receive (B): declared teleport init at y=0, dock law VERBATIM to station_b, then
               the production unload (shoes open + deck/receiver rollers at
               UNLOAD_ROLLER_SPEED) -- answers whether a zero-lateral-error transit
               can actually dock AND unload on v7.

DECLARATIONS: diagnostic teleport initialization (vehicle x=6.5, tray at catch
offset on the deck) exactly like probe_g7c_transit_dial.py; no welds, no collision
changes, no acceptance thresholds invented (the 5 mm hold budget is the restored
original; support/speed criteria are the commissioning family).
"""
import argparse
import json
import sys
from pathlib import Path

ROOT = Path('/home/ziling/projects/010_heterogeneous_robot_workcell')
sys.path[:0] = [str(ROOT / 'src'), str(ROOT / 'experiments')]

import mujoco
import numpy as np

from probe_vehicle_rgbd import scene
from w4_plant import LogisticsPlant

V7_WORLD = ROOT / 'assets/world_p5_candidate_v7_hinged_retainer.xml'
START_X = 6.5
CATCH = 1.9412
CLOSE_TARGET = -0.025
OPEN_TARGET = 1.5708
SETTLE_S = 1.0
HOLD_GATE_M = 0.005          # restored original transport-qualification budget
FUNNEL_GUARD_X = 8.0         # hold arm must never reach the funnel window
UNLOAD_BUDGET_S = 30.0

ap = argparse.ArgumentParser()
ap.add_argument('--mode', required=True, choices=('hold', 'receive'))
ap.add_argument('--run-id', required=True)
args = ap.parse_args()

OUT = ROOT / 'reports' / args.run_id
OUT.mkdir(parents=True, exist_ok=False)

model, data, _t, _f, _ = scene(shift=(0., 0.), tray_yaw=0., source_contacts=True,
                               world_path=V7_WORLD, tray_pos_x=4.55)
plant = LogisticsPlant(model=model, data=data, world=V7_WORLD)
DOCK_X = float(plant.stations['station_b']['dock_x_m'])
signs = plant._wheel_signs()

j_slide_x = int(model.jnt_qposadr[model.joint('n_slide_x').id])
j_slide_y = int(model.jnt_qposadr[model.joint('n_slide_y').id])
j_yaw = int(model.jnt_qposadr[model.joint('n_yaw').id])
j_deck_y = int(model.jnt_qposadr[model.joint('c_deck_slide_y').id])
j_deck_yaw = int(model.jnt_qposadr[model.joint('c_deck_yaw').id])
j_ret = [int(model.jnt_qposadr[model.joint(n + '_joint').id]) for n in ('c_retainer_-1', 'c_retainer_1')]
tray_body = model.body('c_payload').id
tray_jid = int(model.body_jntadr[tray_body])
tj = int(model.jnt_qposadr[tray_jid])
tray_vadr = int(model.jnt_dofadr[tray_jid])
unload_ids = [i for i in range(model.nu)
              if (mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, i) or ''
                  ).startswith(('c_deck_roller', 'c_recv_roller'))]

# -- declared teleport init (identical to the dial probe) ---------------------
data.qpos[j_slide_x] += (START_X - plant.chassis_x())
mujoco.mj_forward(model, data)
chassis = plant.chassis_x()
roller0 = model.geom('c_deck_roller_0').id
ride_z = float(data.geom_xpos[roller0][2] + model.geom_size[roller0][0])
data.qpos[tj + 0] = chassis + CATCH
data.qpos[tj + 1] = 0.0
data.qpos[tj + 2] = ride_z + 0.05
data.qpos[tj + 3:tj + 7] = (0., 0., 0., 1.)
data.qvel[tray_vadr:tray_vadr + 6] = 0.
mujoco.mj_forward(model, data)


def base_ctrl():
    ctrl = plant.park_control()
    for side, act in plant.wheel_actuators.items():
        ctrl[act] = plant._wheel_rate_torque(side, 0.0)
    return ctrl


def drive_ctrl(v_cmd, yaw_rate=0.0):
    """v_cmd m/s -> per-side wheel rate servo (dock-law conversion verbatim).

    Gentle arc: +-10% left/right rate split (declared; no inverse model)."""
    ctrl = plant.park_control()
    f = 1.0 + (0.10 if yaw_rate > 0 else (-0.10 if yaw_rate < 0 else 0.0))
    for side, act in plant.wheel_actuators.items():
        s = f if side == 'left' else (2.0 - f if side == 'right' else 1.0)
        ctrl[act] = plant._wheel_rate_torque(side, (v_cmd * s / 0.04) * signs[side])
    return ctrl


def step_with(ctrl, shoe=CLOSE_TARGET, rollers=None):
    """Dial semantics: shoe close is re-asserted EVERY step unless overridden.

    (recv-01 instrument fault: a shoe=None default left the shoes open through
    settle; the drive-start torque step then threw the unrestrained tray
    rearward past the open shoes to the chassis face -- offset 1.9412->0.2536
    in 0.066 s of sim. The dial probes close by default; so does this probe.)"""
    if shoe is not None:
        for drv in ('c_retainer_-1_drive', 'c_retainer_1_drive'):
            ctrl[model.actuator(drv).id] = shoe
    if rollers is not None:
        for rid in unload_ids:
            ctrl[rid] = rollers
    data.ctrl[:] = ctrl
    mujoco.mj_step(model, data)


def support():
    return list(plant.tray_support())


def tray_offset():
    return float(data.qpos[tj + 0]) - plant.chassis_x()


def phase(name, t_end, ctrl_fn, log, shoe=CLOSE_TARGET, rollers=None):
    """Run one phase; return per-phase verdict dict (5 mm budget vs catch)."""
    t0 = float(data.time)
    max_drift, max_off = 0.0, tray_offset()
    while float(data.time) < t_end:
        step_with(ctrl_fn(float(data.time) - t0), shoe=shoe, rollers=rollers)
        off = tray_offset()
        max_drift = max(max_drift, abs(off - CATCH))
        max_off = off
        if args.mode == 'hold' and plant.chassis_x() >= FUNNEL_GUARD_X:
            return dict(phase=name, verdict='ABORT_FUNNEL_GUARD',
                        chassis_x=round(plant.chassis_x(), 4))
    row = dict(phase=name, sim_s=round(float(data.time), 3),
               chassis_x=round(plant.chassis_x(), 4),
               yaw=round(float(data.qpos[j_yaw]), 4),
               tray_offset=round(max_off, 5),
               max_drift_vs_catch_m=round(max_drift, 5),
               within_5mm=bool(max_drift <= HOLD_GATE_M),
               support=support(),
               ret=[round(float(data.qpos[a]), 4) for a in j_ret])
    log.write(json.dumps(row) + '\n')
    print(json.dumps(row))
    return row


log = open(OUT / 'pair_log.jsonl', 'w', buffering=1)
phases = []
report = dict(scope='G7_PAIR_HOLD_VS_RECEIVE', mode=args.mode,
              world=V7_WORLD.name,
              world_sha256=__import__('hashlib').sha256(V7_WORLD.read_bytes()).hexdigest(),
              declarations=['teleport init x=6.5 / tray at catch %.4f (diagnostic, '
                            'same as dial probes)' % CATCH,
                            'hold gate = 5 mm vs catch (restored original budget)',
                            'no welds, no collision edits, no new thresholds'],
              phases=phases)

if args.mode == 'hold':
    step_with(base_ctrl())
    while float(data.time) < SETTLE_S:
        step_with(base_ctrl())
    phases.append(phase('accel_0_to_0.08', float(data.time) + 4.0,
                        lambda t: drive_ctrl(min(0.08, 0.02 * (t + 0.05))), log))
    phases.append(phase('cruise_0.08', float(data.time) + 3.0,
                        lambda t: drive_ctrl(0.08), log))
    phases.append(phase('brake_to_0', float(data.time) + 1.6,
                        lambda t: drive_ctrl(max(0.0, 0.08 - 0.05 * t)), log))
    phases.append(phase('stop_hold', float(data.time) + 2.0,
                        lambda t: base_ctrl(), log))
    phases.append(phase('gentle_arc_left', float(data.time) + 1.5,
                        lambda t: drive_ctrl(0.06, yaw_rate=+1.0), log))
    phases.append(phase('straighten', float(data.time) + 1.0,
                        lambda t: drive_ctrl(0.06), log))
    phases.append(phase('final_stop', float(data.time) + 1.0,
                        lambda t: base_ctrl(), log))
    gate = dict(hold_budget_m=HOLD_GATE_M,
                all_phases_within_5mm=all(p.get('within_5mm') for p in phases),
                tray_still_on_deck=support() == ['deck'],
                shoes_stay_closed=all(q <= -0.02 for q in
                                      (float(data.qpos[a]) for a in j_ret)),
                end_chassis_speed=round(float(np.linalg.norm(
                    data.qvel[j_slide_x:j_slide_x + 2])), 4))
    gate['verdict'] = ('PASS' if (gate['all_phases_within_5mm']
                                  and gate['tray_still_on_deck']
                                  and gate['shoes_stay_closed']) else 'FAIL')
    report['hold_gate'] = gate

else:  # receive
    # approach: the dock law VERBATIM (dial skeleton), log every 0.25 s
    ride_lost_step = None
    prev_riding = True
    t_appr0 = float(data.time)
    while abs(plant.chassis_x() - DOCK_X) > 0.02:
        if float(data.time) - t_appr0 > 180.0:
            report['approach'] = dict(verdict='DRIVE_DEADLINE',
                                      chassis_x=round(plant.chassis_x(), 4))
            break
        err = DOCK_X - plant.chassis_x()
        if abs(err) > 0.5:
            cmd = np.sign(err) * min(0.15, abs(err) / .4 + .01)
        else:
            cmd = np.sign(err) * min(0.05, abs(err) / .4 + .005)
        step_with(drive_ctrl(cmd), shoe=CLOSE_TARGET)
        off = tray_offset()
        if abs(off - CATCH) > HOLD_GATE_M and prev_riding:
            prev_riding = False
            ride_lost_step = dict(sim_s=round(float(data.time), 3),
                                  chassis_x=round(plant.chassis_x(), 4),
                                  tray_offset=round(off, 5))
        if int(float(data.time) / 0.25) != int((float(data.time) - model.opt.timestep) / 0.25):
            log.write(json.dumps(dict(tag='approach', sim_s=round(float(data.time), 3),
                                      chassis_x=round(plant.chassis_x(), 4),
                                      tray_offset=round(off, 5),
                                      deck_y=round(float(data.qpos[j_deck_y]), 5),
                                      ret=[round(float(data.qpos[a]), 4) for a in j_ret])) + '\n')
    report['approach'] = dict(verdict='REACHED_DOCK' if abs(plant.chassis_x() - DOCK_X) <= 0.02
                              else 'INCOMPLETE',
                              dock_x=DOCK_X, end_chassis_x=round(plant.chassis_x(), 4),
                              first_5mm_exceed=ride_lost_step,
                              tray_offset_at_end=round(tray_offset(), 5),
                              support_at_end=support())
    print(json.dumps(report['approach']))
    # dock reached: tray must still be aboard for the unload to be meaningful
    if support() == ['deck'] and abs(plant.chassis_x() - DOCK_X) <= 0.02:
        # shoes open (convergence loop, G7-d v3 pattern)
        for _ in range(3000):
            ctrl = base_ctrl()
            step_with(ctrl, shoe=OPEN_TARGET)
            if all(float(data.qpos[a]) >= 1.5 for a in j_ret):
                break
        report['shoes_open_qpos'] = [round(float(data.qpos[a]), 4) for a in j_ret]
        # production unload: deck + receiver rollers at UNLOAD_ROLLER_SPEED
        from probe_c_full_chain import UNLOAD_ROLLER_SPEED
        t_u0 = float(data.time)
        while float(data.time) - t_u0 < UNLOAD_BUDGET_S:
            step_with(base_ctrl(), shoe=OPEN_TARGET, rollers=float(UNLOAD_ROLLER_SPEED))
            if support() == ['receiver_band']:
                break
        report['unload'] = dict(verdict=('TRAY_ON_RECEIVER' if support() == ['receiver_band']
                                         else 'UNLOAD_TIMEOUT'),
                                unload_sim_s=round(float(data.time) - t_u0, 3),
                                end_support=support(),
                                tray_x=round(float(data.qpos[tj]), 4),
                                tray_z=round(float(data.qpos[tj + 2]), 4))
        print(json.dumps(report['unload']))
    else:
        report['unload'] = dict(verdict='NOT_RUN_TRAY_NOT_ABOARD_OR_DOCK_INCOMPLETE',
                                end_support=support())
        print(json.dumps(report['unload']))
    gate = dict(transit_kept_aboard=report['approach']['verdict'] == 'REACHED_DOCK'
                and report['approach']['support_at_end'] == ['deck'],
                dock_reached=report['approach']['verdict'] == 'REACHED_DOCK',
                unload_onto_receiver=report['unload']['verdict'] == 'TRAY_ON_RECEIVER')
    gate['verdict'] = 'PASS' if all(gate.values()) else 'FAIL'
    report['receive_gate'] = gate

log.close()
report['final_support'] = support()
(OUT / 'report.json').write_text(json.dumps(report, indent=1))
print('PAIR_DONE', args.mode, report.get('hold_gate', report.get('receive_gate')))
