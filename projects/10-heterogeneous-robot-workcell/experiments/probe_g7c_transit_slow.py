# -*- coding: utf-8 -*-
"""Transit dial variant: slow (0.02 m/s) funnel transit, y offset +2.3 mm.

Answer: does slowing the dock approach keep the tray aboard despite the
2 mm-total lane clearance being eaten by lateral drift?
Dial every step inside the funnel window (chassis >= 8.4).
"""
import json
import sys
from pathlib import Path

ROOT = Path('/home/ziling/projects/010_heterogeneous_robot_workcell')
sys.path[:0] = [str(ROOT / 'src'), str(ROOT / 'experiments')]

import mujoco
import numpy as np

from probe_vehicle_rgbd import scene
from w4_plant import LogisticsPlant

OUT = ROOT / 'reports' / 'p4-g7c-transit-dial-03-slow'
OUT.mkdir(parents=True, exist_ok=True)

START_X = 6.5
START_Y = 0.0023
DOCK_TOL = 0.02
SETTLE_S = 1.0
DRIVE_DEADLINE_S = 300.0
SLOW_FROM_ERR = 1.15   # last 1.15 m = from before the funnel mouth
SLOW_SPEED = 0.02
DIAL_FROM_X = 8.4      # per-step telemetry inside the funnel window

V7 = ROOT / 'assets/world_p5_candidate_v7_hinged_retainer.xml'
model, data, _t, _f, _x = scene(shift=(0., 0.), tray_yaw=0., source_contacts=True,
                                world_path=V7, tray_pos_x=4.55)
plant = LogisticsPlant(model=model, data=data, world=V7)
signs_g7 = plant._wheel_signs()
CLOSE_TARGET = -0.025
DOCK_X = float(plant.stations['station_b']['dock_x_m'])

j_slide_x = int(model.jnt_qposadr[model.joint('n_slide_x').id])
j_slide_y = int(model.jnt_qposadr[model.joint('n_slide_y').id])
j_deck_x = int(model.jnt_qposadr[model.joint('c_deck_slide').id])
j_deck_y = int(model.jnt_qposadr[model.joint('c_deck_slide_y').id])
j_deck_yaw = int(model.jnt_qposadr[model.joint('c_deck_yaw').id])
j_ret = [int(model.jnt_qposadr[model.joint(n + '_joint').id]) for n in ('c_retainer_-1', 'c_retainer_1')]
tray_body = model.body('c_payload').id
tray_jid = int(model.body_jntadr[tray_body])
tj = int(model.jnt_qposadr[tray_jid])
tvd = int(model.jnt_dofadr[tray_jid])

data.qpos[j_slide_x] += (START_X - plant.chassis_x())
data.qpos[j_slide_y] += (START_Y - float(data.qpos[j_slide_y]))
mujoco.mj_forward(model, data)
chassis = plant.chassis_x()

CATCH = 1.9412
roller0 = model.geom('c_deck_roller_0').id
ride_z = float(data.geom_xpos[roller0][2] + model.geom_size[roller0][0])
data.qpos[tj + 0] = chassis + CATCH
data.qpos[tj + 1] = START_Y
data.qpos[tj + 2] = ride_z + 0.05
data.qpos[tj + 3:tj + 7] = (0., 0., 0., 1.)
data.qvel[tvd:tvd + 6] = 0.
mujoco.mj_forward(model, data)


def base_ctrl():
    return plant.park_control()


def step_with(ctrl):
    for drv in ('c_retainer_-1_drive', 'c_retainer_1_drive'):
        ctrl[model.actuator(drv).id] = CLOSE_TARGET
    data.ctrl[:] = ctrl
    mujoco.mj_step(model, data)


def tray_contacts():
    out = []
    for i in range(data.ncon):
        c = data.contact[i]
        g1, g2 = int(c.geom[0]), int(c.geom[1])
        b1, b2 = int(model.geom_bodyid[g1]), int(model.geom_bodyid[g2])
        if tray_body not in (b1, b2):
            continue
        n1 = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, g1) or ('g%d' % g1)
        n2 = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, g2) or ('g%d' % g2)
        force = np.zeros(6)
        mujoco.mj_contactForce(model, data, i, force)
        out.append('%s~%s d=%.5f f=%.3f' % (n1, n2, c.dist, float(np.linalg.norm(force[:3]))))
    return out


step_with(base_ctrl())
while float(data.time) < SETTLE_S:
    step_with(base_ctrl())

fh = open(OUT / 'per_step.jsonl', 'w', buffering=1)
row5 = [float(data.time), plant.chassis_x()]
verdict = 'RUN'
try:
    while abs(plant.chassis_x() - DOCK_X) > DOCK_TOL:
        if float(data.time) > DRIVE_DEADLINE_S:
            verdict = 'DRIVE_DEADLINE'
            break
        err = DOCK_X - plant.chassis_x()
        if abs(err) > SLOW_FROM_ERR:
            cmd = np.sign(err) * min(0.15, abs(err) / .4 + .01)
        else:
            cmd = np.sign(err) * min(SLOW_SPEED, abs(err) / .4 + .002)
        ctrl = base_ctrl()
        for side, act in plant.wheel_actuators.items():
            ctrl[act] = plant._wheel_rate_torque(side, (cmd / 0.04) * signs_g7[side])
        step_with(ctrl)
        if plant.chassis_x() >= DIAL_FROM_X:
            fh.write(json.dumps(dict(
                t=round(float(data.time), 3), cx=round(plant.chassis_x(), 4),
                cy=round(float(data.qpos[j_slide_y]), 5),
                deck_y=round(float(data.qpos[j_deck_y]), 5),
                deck_yaw=round(float(data.qpos[j_deck_yaw]), 5),
                tray=round(float(data.qpos[tj]), 4), trayz=round(float(data.qpos[tj + 2]), 4),
                off=round(float(data.qpos[tj]) - plant.chassis_x(), 4),
                ret=[round(float(data.qpos[a]), 4) for a in j_ret],
                ct=tray_contacts())) + '\n')
        if float(data.time) - row5[0] >= 5.0:
            row5 = [float(data.time), plant.chassis_x()]
finally:
    fh.close()

rows = [json.loads(l) for l in open(OUT / 'per_step.jsonl')]


def riding(r):
    return any('tray_floor' in c and 'deck_roller' in c for c in r['ct'])


first_off = None
for i, r in enumerate(rows):
    if not riding(r) and i > 0 and riding(rows[i - 1]):
        first_off = i
        break

report = dict(scope='G7C_TRANSIT_SLOW_VARIANT', status=verdict,
              start_y=START_Y, slow_speed=SLOW_SPEED, slow_from_err=SLOW_FROM_ERR,
              end_chassis_x=round(plant.chassis_x(), 4),
              end_tray_x=round(float(data.qpos[tj]), 4),
              end_tray_z=round(float(data.qpos[tj + 2]), 4),
              end_offset=round(float(data.qpos[tj]) - plant.chassis_x(), 4),
              per_step_rows=len(rows), first_step_without_ride=first_off,
              tray_on_ground=bool(float(data.qpos[tj + 2]) < 0.5),
              final_contacts=tray_contacts())
(OUT / 'report.json').write_text(json.dumps(report, indent=1))
print(json.dumps(report, indent=1, ensure_ascii=False))

if first_off is not None:
    lo = max(0, first_off - 40)
    print('--- %d steps before ride loss ---' % (first_off - lo))
    for r in rows[lo:first_off + 2]:
        big = [c for c in r['ct'] if float(c.rsplit('f=', 1)[1]) > 0.05]
        print('t=%.2f cx=%.4f cy=%.5f deck_y=%.5f tray=%.4f ret=%s %s'
              % (r['t'], r['cx'], r['cy'], r['deck_y'], r['tray'], r['ret'], big))
