"""Is the 0.10 m `no_self_contact` failure an ENTRY TRANSIENT, or my own row definition?

Two candidate explanations, and the second one is cheaper:

  H1 ENTRY JERK   the ramp's first moments (or the controller switch at t=0) produce a torque step
                  that throws the legs together before the descent settles.
  H2 ROW DEFECT   `min(self_mm for s in trace)` takes the WORST over the whole run, including
                  transients. The project already settled how to judge this at `D078` /
                  `probe_h_selfclear.py`: judge the SETTLED state, and require transients to
                  RESOLVE. A stricter rule here would report a defect for a transient that the
                  project's own criterion calls acceptable.

Measured: the deepest self-contact and its pair, sampled EVERY control step for the first 0.30 s
(not every 0.05 s), the settled value over the last 0.50 s, whether the contact resolves, the base's
vertical acceleration, and the torque STEP at t=0 -- the last tick of the settle against the first
tick of the crouch, per joint. A jerk has to show up as a step in the command; if there is no step,
H1 has nothing to stand on.
"""
import json
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'experiments'))
sys.path.insert(0, str(ROOT / 'src'))

import mujoco  # noqa: E402
import build_p3_world as bp3  # noqa: E402

bp3.install()
import probe_p4_balance as PB  # noqa: E402
import probe_p4_crouch2 as C2  # noqa: E402

DEPTH = 0.10
RAMP_S, HOLD_S = 2.0, 1.5

ref = json.loads((ROOT / 'reports' / 'p4-balance-04' / 'report.json').read_text(encoding='utf-8'))
KP, KD = ref['controller']['kp_bal'], ref['controller']['kd_bal']

rig = PB.Rig()
rig.capture_standing()
rig.reset()
rig.settle(1.0)

# --- the torque the settle leaves on each actuator, i.e. the state the crouch starts from --------
settle_ctrl = np.array(rig.d.ctrl, dtype=float)

standing_z = float(rig.d.xpos[rig.m.body(PB.BASE).id][2])
n_ramp = int(RAMP_S / rig.m.opt.timestep)
n_hold = int(HOLD_S / rig.m.opt.timestep)
fine = int(0.30 / rig.m.opt.timestep)
print(f'standing z {standing_z:.5f}; sampling every control step for the first 0.30 s')
print()

rows = []
corr = 0.0
q_prev = np.array(rig.d.qpos, dtype=float)
rig.prev_com, rig.prev_t = rig.humanoid_com(), float(rig.d.time)
first_ctrl = None
prev_vz = 0.0
for i in range(n_ramp + n_hold):
    frac = min(1.0, (i + 1) / n_ramp)
    commanded_z = float(rig.pelvis0[2]) - DEPTH * frac
    measured_z = float(rig.d.xpos[rig.m.body(PB.BASE).id][2])
    corr = float(np.clip(1.0 * (commanded_z - measured_z), -0.06, 0.06))
    eff = max(0.0, DEPTH * frac - corr)
    target = np.array([rig.pelvis0[0] + C2.dx_for(eff), rig.pelvis0[1],
                       float(rig.pelvis0[2]) - eff], dtype=float)
    seed = np.array(rig.d.qpos, dtype=float)
    q_try, resid, _m = rig.solve_posture(tuple(target), seed)
    if max(resid.values()) > PB.IK_ACCEPT_M:
        q_try = q_prev
    else:
        q_prev = q_try
    com_ref = float(rig.posture_com(q_try)[0])
    com_vx = rig.com_x_velocity()
    rig.prev_com, rig.prev_t = rig.humanoid_com(), float(rig.d.time)
    rig.tick(q_try, balance=True, kp_bal=KP, kd_bal=KD, com_ref_x=com_ref, com_vx=com_vx,
             gravity_ff=True)
    if first_ctrl is None:
        first_ctrl = np.array(rig.d.ctrl, dtype=float)

    deepest, pair = rig.deepest_self_mm()
    vz = float(rig.d.qvel[2])
    az = (vz - prev_vz) / rig.m.opt.timestep
    prev_vz = vz
    if i < fine or i % int(0.05 / rig.m.opt.timestep) == 0:
        rows.append({'t': float(rig.d.time), 'step': i, 'eff_depth': eff,
                     'drop_mm': (standing_z - measured_z) * 1000,
                     'self_mm': deepest, 'pair': pair,
                     'base_vz': vz, 'base_az': az,
                     'leg_vel': rig.leg_vel()})

print(f'{"t":>6} {"step":>5} {"eff_d":>7} {"drop_mm":>8} {"self_mm":>8} {"base_vz":>9} '
      f'{"base_az":>10} {"legvel":>7}  pair')
for r in rows[:40]:
    print(f'{r["t"]:6.3f} {r["step"]:5d} {r["eff_depth"]:7.4f} {r["drop_mm"]:8.2f} '
          f'{r["self_mm"]:8.3f} {r["base_vz"]:+9.5f} {r["base_az"]:+10.2f} '
          f'{r["leg_vel"]:7.4f}  {r["pair"]}')

# ---- the two hypotheses, decided separately ----------------------------------------------------
all_self = [r['self_mm'] for r in rows]
worst = min(all_self)
worst_at = min(rows, key=lambda r: r['self_mm'])
settled = [r['self_mm'] for r in rows if r['t'] > rig.d.time - HOLD_S - 0.5]
print()
print('--- H2: is the row stricter than the project rule? ---')
print(f'worst self-contact anywhere in the run : {worst:.3f} mm  '
      f'(at t {worst_at["t"]:.3f} s, {worst_at["pair"]})')
print(f'worst over the last 0.50 s (settled)   : {min(settled) if settled else float("nan"):.3f} mm')
print(f'project rule (`probe_h_selfclear.py`)  : judge the SETTLED state, and require any transient '
      f'to RESOLVE within the window')
resolves = all(abs(r['self_mm']) < 0.05 for r in rows if r['t'] > worst_at['t'] + 0.20)
print(f'did it resolve (all |self| < 0.05 mm later than worst+0.20 s)? {resolves}')

print()
print('--- H1: is there a torque STEP at t=0? settle\'s last tick vs crouch\'s first tick ---')
d = np.abs(first_ctrl - settle_ctrl)
scale = np.abs(settle_ctrl) + 1e-9
worst_i = int(np.argmax(d))
name = mujoco.mj_id2name(rig.m, mujoco.mjtObj.mjOBJ_ACTUATOR, worst_i) or '<unnamed>'
print(f'largest absolute ctrl change : {d[worst_i]:.4f} on {name} '
      f'(settle {settle_ctrl[worst_i]:+.4f} -> first {first_ctrl[worst_i]:+.4f})')
print(f'number of actuators whose ctrl changed by more than 1 % of their own level: '
      f'{int(np.sum(d > 0.01 * scale))}')
print(f'largest relative change      : {float(np.max(d / scale)) * 100:.2f} %')
print()
print('peak |base vertical accel| in the first 0.30 s: '
      f'{max(abs(r["base_az"]) for r in rows[:fine]):.2f} m/s2')
print('peak leg joint speed in the first 0.30 s     : '
      f'{max(r["leg_vel"] for r in rows[:fine]):.4f} rad/s')
