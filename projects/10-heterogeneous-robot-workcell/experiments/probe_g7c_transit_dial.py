# -*- coding: utf-8 -*-
"""G7-c funnel-transit micro-dial: per-step telemetry of the dock approach.

SCOPE: JUDGE-ONLY diagnostic. No gate verdicts, no acceptance thresholds.
WHAT IT ANSWERS: srcl6 measured the tray leaving the deck rollers between
nudge rows 125-130 s while the deck frame tip entered the receiver funnel
(w2_taper/chamfer rails, 1 mm total press-fit vs the 0.600 m deck in a
0.602 m lane). Every paper mechanism was excluded against the compiled XML:
the vehicle is planar (no pitch DOF), the deck x-slide is a 20000 N/m
position servo (+-3000 N), the y/yaw springs only absorb lateral. This probe
re-runs ONLY the dock approach from a declared teleported start and records
EVERY tray/deck contact with forces at every 2 ms step, so the first step
that loses the tray-floor/roller contacts can be named together with the
contacts that fired just before it.

DECLARATIONS:
- the tray is teleported to the deck-riding pose (diagnostic initialization,
  NOT a runtime loading claim; the runtime evidence is srcl6 itself);
- the vehicle is teleported to x=6.5 (3.1 m before the dock) to keep the run
  short; the approach law is the probe's own law verbatim;
- shoe close is re-asserted every step (the srcl6 fix), settle 1 s before
  driving.
"""
import json
import sys
from pathlib import Path

ROOT = Path('/home/ziling/projects/010_heterogeneous_robot_workcell')
sys.path[:0] = [str(ROOT / 'src'), str(ROOT / 'experiments')]

import mujoco
import numpy as np

import probe_p3_vision as pv
from probe_vehicle_rgbd import scene
from w4_plant import LogisticsPlant

OUT = ROOT / 'reports' / 'p4-g7c-transit-dial-01'
OUT.mkdir(parents=True, exist_ok=True)

START_X = 6.5
DOCK_TOL = 0.02
SETTLE_S = 1.0
DRIVE_DEADLINE_S = 90.0
STALL_MOVE_M = 0.002
STALL_WINDOW_S = 5.0

V7_WORLD = ROOT / 'assets/world_p5_candidate_v7_hinged_retainer.xml'

model, data, _template, _floor_z, _ = scene(
    shift=(0., 0.), tray_yaw=0., source_contacts=True,
    world_path=V7_WORLD, tray_pos_x=4.55)

plant = LogisticsPlant(model=model, data=data, world=V7_WORLD)
DOCK_X = float(plant.stations['station_b']['dock_x_m'])
signs_g7 = plant._wheel_signs()
CLOSE_TARGET = -0.025
OPEN_TARGET = 1.5708

# -- addressing ---------------------------------------------------------------
j_slide_x = int(model.jnt_qposadr[model.joint('n_slide_x').id])
j_slide_y = int(model.jnt_qposadr[model.joint('n_slide_y').id])
j_yaw = int(model.jnt_qposadr[model.joint('n_yaw').id]) if model.joint('n_yaw').id >= 0 else None
if j_yaw is None:
    for cand in ('n_joint_yaw', 'n_base_yaw'):
        try:
            j_yaw = int(model.jnt_qposadr[model.joint(cand).id])
            break
        except KeyError:
            continue
j_deck_x = int(model.jnt_qposadr[model.joint('c_deck_slide').id])
j_deck_y = int(model.jnt_qposadr[model.joint('c_deck_slide_y').id])
j_deck_yaw = int(model.jnt_qposadr[model.joint('c_deck_yaw').id])
j_ret = [int(model.jnt_qposadr[model.joint(n + '_joint').id]) for n in ('c_retainer_-1', 'c_retainer_1')]
tray_body = model.body('c_payload').id
tray_jid = int(model.body_jntadr[tray_body])
tj = int(model.jnt_qposadr[tray_jid])
tray_vadr = int(model.jnt_dofadr[tray_jid])

# -- declared teleport: vehicle to START_X ------------------------------------
x_now = plant.chassis_x()
data.qpos[j_slide_x] += (START_X - x_now)
mujoco.mj_forward(model, data)
chassis = plant.chassis_x()

# -- declared teleport: tray onto the deck at the catch offset ----------------
CATCH = 1.9412  # live-model geometry: shoe -x face (+2.005) minus wall reach (+0.065)
roller0 = model.geom('c_deck_roller_0').id
ride_z = float(data.geom_xpos[roller0][2] + model.geom_size[roller0][0])
# tj/tray_vadr come from the addressing section (jnt_qposadr of c_payload_free).
# body origin rest height = floor bottom (ride_z) + 0.04 (floor geom offset);
# +0.01 drop -> place origin at ride_z + 0.05.
data.qpos[tj + 0] = chassis + CATCH
data.qpos[tj + 1] = 0.0
data.qpos[tj + 2] = ride_z + 0.05  # 1 cm drop measured at the FLOOR, not the origin
data.qpos[tj + 3:tj + 7] = (0., 0., 0., 1.)
data.qvel[tray_vadr:tray_vadr + 6] = 0.
mujoco.mj_forward(model, data)


def base_ctrl():
    ctrl = plant.park_control()
    for side, act in plant.wheel_actuators.items():
        ctrl[act] = plant._wheel_rate_torque(side, 0.0)
    return ctrl


def step_with(ctrl, close=True):
    if close:
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
        if tray_body not in (b1, b2) and model.body('c_deck').id not in (b1, b2):
            continue
        n1 = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, g1) or ('g%d' % g1)
        n2 = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, g2) or ('g%d' % g2)
        force = np.zeros(6)
        mujoco.mj_contactForce(model, data, i, force)
        out.append('%s~%s d=%.5f f=%.3f' % (n1, n2, c.dist, float(np.linalg.norm(force[:3]))))
    return out


def snap_row(tag):
    return dict(tag=tag, sim_s=round(float(data.time), 3),
                chassis_x=round(plant.chassis_x(), 4),
                chassis_y=round(float(data.qpos[j_slide_y]), 5),
                yaw=round(float(data.qpos[j_yaw]) if j_yaw is not None else 0., 6),
                deck_x=round(float(data.qpos[j_deck_x]), 5),
                deck_y=round(float(data.qpos[j_deck_y]), 5),
                deck_yaw=round(float(data.qpos[j_deck_yaw]), 5),
                ret=[round(float(data.qpos[a]), 4) for a in j_ret],
                tray=[round(float(v), 4) for v in data.qpos[tj:tj + 3]],
                tray_qvel=[round(float(v), 3) for v in data.qvel[tray_vadr:tray_vadr + 3]],
                contacts=tray_contacts())


# -- phase 1: settle, shoes closing -------------------------------------------
step_with(base_ctrl())
while float(data.time) < SETTLE_S:
    step_with(base_ctrl())

# -- phase 2: the dock approach, the probe's own law --------------------------
dial = open(OUT / 'transit_dial.jsonl', 'w', buffering=1)
nudge_last = [float(data.time), plant.chassis_x()]
stall_marks = []
verdict = 'RUN'
try:
    while abs(plant.chassis_x() - DOCK_X) > DOCK_TOL:
        if float(data.time) > DRIVE_DEADLINE_S:
            verdict = 'DRIVE_DEADLINE'
            break
        err = DOCK_X - plant.chassis_x()
        if abs(err) > 0.5:
            cmd = np.sign(err) * min(0.15, abs(err) / .4 + .01)
        else:
            cmd = np.sign(err) * min(0.05, abs(err) / .4 + .005)
        ctrl = plant.park_control()
        for side, act in plant.wheel_actuators.items():
            ctrl[act] = plant._wheel_rate_torque(side, (cmd / 0.04) * signs_g7[side])
        step_with(ctrl)
        if float(data.time) - nudge_last[0] >= STALL_WINDOW_S:
            moved = abs(plant.chassis_x() - nudge_last[1])
            row = snap_row('window')
            row['moved_m'] = round(moved, 4)
            dial.write(json.dumps(row) + '\n')
            if moved < STALL_MOVE_M:
                stall_marks.append((round(float(data.time), 2), round(moved, 4)))
                verdict = 'STALLED'
                break
            nudge_last = [float(data.time), plant.chassis_x()]
finally:
    dial.close()

# -- phase 3: analysis from the dial ------------------------------------------
rows = [json.loads(l) for l in open(OUT / 'transit_dial.jsonl')]


def riding(row):
    return any('tray_floor' in c and 'deck_roller' in c for c in row['contacts'])


first_off = None
for i, r in enumerate(rows):
    if not riding(r) and i > 0 and riding(rows[i - 1]):
        first_off = i
        break

report = dict(
    scope='G7C_FUNNEL_TRANSIT_MICRO_DIAL', status=verdict,
    declarations=['teleported start x=%.2f and tray at catch offset %.4f (diagnostic init)',
                  'approach law = probe_vehicle_nav2 dock law verbatim',
                  'shoe close re-asserted every step (srcl6 fix present)'],
    dock_x=DOCK_X, end_chassis_x=round(plant.chassis_x(), 4),
    windows=len(rows), stall_marks=stall_marks,
    tray_ride=({'first_step_without_roller_contact': first_off,
                'window_at': rows[first_off]['sim_s'] if first_off is not None else None}
               if rows else None),
    final=snap_row('final'))
(OUT / 'report.json').write_text(json.dumps(report, indent=1))

print(json.dumps(report, indent=1, ensure_ascii=False))

# the money shot: the 30 windows around the ride loss, full contacts
if first_off is not None:
    lo = max(0, first_off - 30)
    sel = open(OUT / 'ride_loss_context.jsonl', 'w')
    for r in rows[lo:first_off + 3]:
        sel.write(json.dumps(r) + '\n')
    sel.close()
    print('--- ride-loss context windows %d..%d ---' % (lo, first_off + 2))
    for r in rows[lo:first_off + 3]:
        print(r['tag'], 'sim=%.2f cx=%.4f tray=%.4f deck_y=%.5f deck_yaw=%.5f ret=%s contacts=%s'
              % (r['sim_s'], r['chassis_x'], r['tray'][0], r['deck_y'], r['deck_yaw'],
                 r['ret'], r['contacts']))
