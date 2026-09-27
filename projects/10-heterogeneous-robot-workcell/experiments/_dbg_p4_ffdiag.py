"""Why did `mj_inverse` with qacc=0 return torques up to 137,892 N.m for +-160 N.m actuators?

The proposal was right in principle: `qfrc_bias` is the free-floating gravity torque, and a robot
standing on its feet needs `tau = qfrc_bias + (Jc^T fc)`. `mj_inverse` is the shortcut that is
supposed to give that in one call. It did not.

Test on the STANDING pose, where the correct answer is small and known: the joints hold a nearly
straight leg under 86 kg, so the true joint torques are tens of N.m, not tens of thousands.

Compare, per leg joint:
  qfrc_bias       free-floating gravity + Coriolis
  qfrc_inverse    what `mj_inverse` reports for qacc = 0
  qfrc_constraint the force the contacts are already applying
A correct `qfrc_inverse` should be close to `qfrc_bias + (contact contribution)`, and the contact
contribution is bounded by the ~850 N the floor carries. Anything far above that is not a torque.
"""
import sys
from pathlib import Path

import numpy as np

ROOT = Path('/home/ziling/projects/010_heterogeneous_robot_workcell')
sys.path.insert(0, str(ROOT / 'experiments'))
sys.path.insert(0, str(ROOT / 'src'))

import mujoco  # noqa: E402
import build_p3_world as bp3  # noqa: E402

bp3.install()
import probe_p4_balance as PB  # noqa: E402

rig = PB.Rig()
rig.capture_standing()
rig.reset()
rig.settle(1.0)

JOINTS = ('J00_HIP_PITCH_L', 'J03_KNEE_PITCH_L', 'J04_ANKLE_PITCH_L',
          'J06_HIP_PITCH_R', 'J09_KNEE_PITCH_R', 'J10_ANKLE_PITCH_R')

print(f'{"joint":22s} {"bias":>10} {"inverse":>12} {"constraint":>12} {"actuator span":>14}')
spans = {}
for n in JOINTS:
    a = mujoco.mj_name2id(rig.m, mujoco.mjtObj.mjOBJ_ACTUATOR, 'h_motor_' + n)
    spans[n] = abs(float(rig.m.actuator_ctrlrange[a][1])) if a >= 0 else float('nan')

bias = {n: float(rig.d.qfrc_bias[int(rig.m.jnt_dofadr[rig.m.joint('h_' + n).id])]) for n in JOINTS}

mujoco.mj_forward(rig.m, rig.d)
qvel_saved = np.array(rig.d.qvel, dtype=float)
rig.d.qvel[:] = 0.0
rig.d.qacc[:] = 0.0
mujoco.mj_inverse(rig.m, rig.d)
inv = {n: float(rig.d.qfrc_inverse[int(rig.m.jnt_dofadr[rig.m.joint('h_' + n).id])]) for n in JOINTS}
con = {n: float(rig.d.qfrc_constraint[int(rig.m.jnt_dofadr[rig.m.joint('h_' + n).id])])
       for n in JOINTS}
rig.d.qvel[:] = qvel_saved

for n in JOINTS:
    print(f'{n:22s} {bias[n]:10.3f} {inv[n]:12.3f} {con[n]:12.3f} {spans[n]:14.1f}')

print()
print('ratio |inverse| / actuator span, worst:', 
      f'{max(abs(inv[n]) / spans[n] for n in JOINTS):.1f}x')
print('ratio |bias|    / actuator span, worst:',
      f'{max(abs(bias[n]) / spans[n] for n in JOINTS):.1f}x')
print()
print('ncon =', rig.d.ncon, ' (the feet are on the floor, so the chain IS closed)')
print('nefc =', rig.d.nefc)
print()
print('--- the same call, WITHOUT forcing qacc to zero ---')
rig.d.qvel[:] = 0.0
mujoco.mj_forward(rig.m, rig.d)
mujoco.mj_inverse(rig.m, rig.d)          # qacc left as the forward solution
inv2 = {n: float(rig.d.qfrc_inverse[int(rig.m.jnt_dofadr[rig.m.joint('h_' + n).id])])
        for n in JOINTS}
for n in JOINTS:
    print(f'{n:22s} qacc={rig.d.qacc[0]:+.4f}  inverse {inv2[n]:12.3f}  '
          f'ratio {abs(inv2[n]) / spans[n]:6.2f}x')
rig.d.qvel[:] = qvel_saved
