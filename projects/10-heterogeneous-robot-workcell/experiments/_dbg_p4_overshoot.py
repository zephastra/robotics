"""WHY does the pelvis end up 17 % deeper than commanded? Measure, do not guess.

The overshoot is nearly a constant FRACTION of the command (+16.8/+26.3/+33.7/+44.3 mm for 0.10/
0.15/0.20/0.25 m = 16.8/17.5/16.9/17.7 %), which is the signature of a SCALE error rather than a
load-dependent sag -- a sag would grow faster than linearly. Three candidate causes, all separable
by printing the right numbers:

  A  REFERENCE BIAS   `pelvis0` is captured after a 1.2 s settle, while `run()` takes its own
                      `standing_z` after a 1.0 s settle. If the humanoid is still sinking between
                      those two moments the command is biased -- but a bias is CONSTANT, not
                      proportional, so the measured growth already argues against it.
  B  TRACKING LAG     the joints do not reach the IK's targets, so the leg is over-flexed for the
                      commanded pelvis. Shows up as target-minus-measured per joint.
  C  PELVIS TILT      the IK pins the pelvis ORIENTATION to the standing one, but nothing controls
                      the free joint's rotation. If the pelvis tips, the body origin is no longer
                      where the IK solved for, and the feet are no longer flat.

Printed per step: the command, the measurement, the tilt, the six leg joint errors, and the foot's
own error against the pose the IK was told to hold.
"""
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
from _dbg_p4_legs import LEG_CHAIN, FOOT, rot_err  # noqa: E402
import json  # noqa: E402

DEPTH = float(sys.argv[1]) if len(sys.argv) > 1 else 0.25
DX = float(sys.argv[2]) if len(sys.argv) > 2 else -0.12
RAMP_S, HOLD_S = 2.0, 1.5

bp3.install()
rig = PB.Rig()
rig.capture_standing()
ref = json.loads((ROOT / 'reports' / 'p4-balance-04' / 'report.json').read_text(encoding='utf-8'))
KP, KD = ref['controller']['kp_bal'], ref['controller']['kd_bal']

rig.reset()
rig.settle(1.0)
standing_z = float(rig.d.xpos[rig.m.body(PB.BASE).id][2])
print(f'pelvis0 (captured after a 1.2 s settle) : {np.round(rig.pelvis0, 6)}')
print(f'standing_z (taken after a 1.0 s settle): {standing_z:.6f}')
print(f'>>> REFERENCE BIAS pelvis0[2] - standing_z = '
      f'{(rig.pelvis0[2] - standing_z) * 1000:+.3f} mm')
print(f'depth {DEPTH}, dx {DX}, gains kp {KP:.1f} kd {KD:.1f}')
print()

q_stand = np.array(rig.q_stand, dtype=float)
quat_stand = np.array(q_stand[3:7], dtype=float)


def tilt_deg():
    q = np.array(rig.d.qpos[3:7], dtype=float)
    R1 = np.zeros(9)
    R2 = np.zeros(9)
    mujoco.mju_quat2Mat(R1, quat_stand)
    mujoco.mju_quat2Mat(R2, q)
    return float(np.degrees(np.linalg.norm(rot_err(R1.reshape(3, 3), R2.reshape(3, 3)))))


rig.prev_com, rig.prev_t = rig.humanoid_com(), float(rig.d.time)
n_ramp = int(RAMP_S / rig.m.opt.timestep)
n_tot = n_ramp + int(HOLD_S / rig.m.opt.timestep)
every = max(1, int(0.25 / rig.m.opt.timestep))
worst_joint = {}
for i in range(n_tot):
    frac = min(1.0, (i + 1) / n_ramp)
    seed = np.array(rig.d.qpos, dtype=float)
    commanded = np.array([rig.pelvis0[0] + DX * frac, rig.pelvis0[1],
                          rig.pelvis0[2] + (-DEPTH) * frac], dtype=float)
    q_target, resid, marg = rig.solve_posture(commanded, seed)
    com_ref = float(rig.posture_com(q_target)[0])
    com_vx = rig.com_x_velocity()
    rig.prev_com, rig.prev_t = rig.humanoid_com(), float(rig.d.time)
    rig.tick(q_target, balance=True, kp_bal=KP, kd_bal=KD, com_ref_x=com_ref, com_vx=com_vx,
             gravity_ff=True)

    errs = {}
    for side in ('L', 'R'):
        for n in LEG_CHAIN[side]:
            j = rig.m.joint('h_' + n).id
            adr = int(rig.m.jnt_qposadr[j])
            errs[n] = float(q_target[adr]) - float(rig.d.qpos[adr])
    for n, v in errs.items():
        if abs(v) > abs(worst_joint.get(n, 0.0)):
            worst_joint[n] = v

    if i % every == 0 or i == n_tot - 1:
        meas = np.array(rig.d.xpos[rig.m.body(PB.BASE).id], dtype=float)
        fpe, fpe_pair = 0.0, ''
        for s in ('L', 'R'):
            fid = rig.m.body(FOOT[s]).id
            e = float(np.linalg.norm(rig.foot_pos[s] - np.asarray(rig.d.xpos[fid], dtype=float)))
            if e > fpe:
                fpe, fpe_pair = e, s
        fp = rig.foot_pitch_deg()
        print(f't={float(rig.d.time):5.2f} frac={frac:4.2f} '
              f'cmd_z={commanded[2]:.5f} meas_z={meas[2]:.5f} '
              f'err={(meas[2] - commanded[2]) * 1000:+7.2f}mm '
              f'tilt={tilt_deg():5.2f}deg footerr={fpe * 1000:6.3f}mm({fpe_pair}) '
              f'pitch={fp["L"]:5.2f} '
              f'dHIP={errs["J00_HIP_PITCH_L"]:+7.4f} dKNEE={errs["J03_KNEE_PITCH_L"]:+7.4f} '
              f'dANK={errs["J04_ANKLE_PITCH_L"]:+7.4f}')

print()
print('worst |target - measured| per leg joint over the whole run (rad | deg):')
for n, v in sorted(worst_joint.items(), key=lambda kv: -abs(kv[1]))[:8]:
    print(f'   {n:22s} {v:+.5f} rad  ({np.degrees(v):+7.2f} deg)')
