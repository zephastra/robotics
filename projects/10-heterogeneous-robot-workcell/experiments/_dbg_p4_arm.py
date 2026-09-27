"""Where does the 264 N.m elbow-yaw demand come from? Print the breakdown, do not guess."""
import sys
from pathlib import Path

import numpy as np

ROOT = Path('/home/ziling/projects/010_heterogeneous_robot_workcell')
sys.path.insert(0, str(ROOT / 'experiments'))
sys.path.insert(0, str(ROOT / 'src'))
import probe_p4_reach as P  # noqa: E402

rig = P.Rig(P.load(ROOT / 'assets' / 'world_w2_logistic.xml'), 'h_payload')
print('kp per joint:', {n.split('_',1)[1]: round(rig.kp[n],1) for n in P.ARM_JOINTS})
print('kd per joint:', {n.split('_',1)[1]: round(rig.kd[n],2) for n in P.ARM_JOINTS})
rig.reset()
rig.settle()
rig.start_origin = np.array(rig.d.xpos[rig.tray], dtype=float)
grip = rig.grip_points()
worst, pose, per, q = P.solve_arms(rig, grip, np.array(rig.d.qpos))
print('grasp residual mm', round(worst * 1000, 4))
rig.d.qpos[:] = q
rig.d.qvel[:] = 0.0
import mujoco
mujoco.mj_forward(rig.m, rig.d)

print()
print('--- the solved posture, joint by joint ---')
for n in P.ARM_JOINTS:
    adr, dof = rig.qadr(n), rig.dofadr(n)
    print(f'  {n:24s} qpos {np.degrees(rig.d.qpos[adr]):8.2f} deg  '
          f'qvel {rig.d.qvel[dof]:9.4f}  bias {rig.d.qfrc_bias[dof]:10.3f} N.m')

print()
print('--- hold for 3 s with close = 0.50, sampling every 250 steps ---')
for step in range(1500):
    rig.step(arm_target=pose, hand_close=0.50, steps=1)
    if step % 250 == 0 or step == 1499:
        errs = {n: abs(float(rig.d.qpos[rig.qadr(n)]) - pose[n]) for n in P.ARM_JOINTS}
        vels = {n: abs(float(rig.d.qvel[rig.dofadr(n)])) for n in P.ARM_JOINTS}
        wname = max(errs, key=errs.get)
        vname = max(vels, key=vels.get)
        print(f'  step {step:5d}  worst |err| {errs[wname] * 1000:8.2f} mrad ({wname})  '
              f'worst |qvel| {vels[vname]:8.3f} rad/s ({vname})  '
              f'contacts {rig.d.ncon}')

print()
print('--- final breakdown per arm joint ---')
for n in P.ARM_JOINTS:
    adr, dof = rig.qadr(n), rig.dofadr(n)
    err = float(rig.d.qpos[adr]) - pose[n]
    vel = float(rig.d.qvel[dof])
    bias = float(rig.d.qfrc_bias[dof])
    pd = rig.kp[n] * (-err) - rig.kd[n] * vel
    print(f'  {n:24s} err {np.degrees(err):9.3f} deg  vel {vel:9.4f}  bias {bias:10.3f}  '
          f'PD {pd:10.3f}  demand {abs(pd + bias):10.3f}  span {rig.arm_span[n]:6.0f}')

print()
print('--- and with the fingers fully open (close = 0.0) ---')
rig.reset()
rig.settle()
rig.d.qpos[:] = q
rig.d.qvel[:] = 0.0
mujoco.mj_forward(rig.m, rig.d)
for step in range(1500):
    rig.step(arm_target=pose, hand_close=0.02, steps=1)
for n in P.ARM_JOINTS:
    adr, dof = rig.qadr(n), rig.dofadr(n)
    err = float(rig.d.qpos[adr]) - pose[n]
    vel = float(rig.d.qvel[dof])
    bias = float(rig.d.qfrc_bias[dof])
    pd = rig.kp[n] * (-err) - rig.kd[n] * vel
    print(f'  {n:24s} err {np.degrees(err):9.3f} deg  vel {vel:9.4f}  bias {bias:10.3f}  '
          f'demand {abs(pd + bias):10.3f}  span {rig.arm_span[n]:6.0f}')
print('DONE')
