"""One variable: how much left-shoulder abduction clears the palm from the hip?

The corrected arm leaves ONE real humanoid self-contact: `h_LINK_HIP_YAW_L x h_lh_palm` at 3.376 mm,
transient, at step 85. The declared stand pose has the left arm at
`[SHOULDER_PITCH 0, SHOULDER_ROLL +0.05, SHOULDER_YAW 0, ELBOW_PITCH -0.1, ELBOW_YAW 0]`, i.e. the
palm hangs essentially against the hip.

This sweeps ONLY `J14_SHOULDER_ROLL_L` and reports the worst self-contact for each value, so the
choice is a measurement rather than a guess about which way the arm should move. Everything else --
the world, the law, the seed state, the window -- is held fixed.
"""
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'experiments'))
sys.path.insert(0, str(ROOT / 'src'))

import mujoco  # noqa: E402
import numpy as np  # noqa: E402

import build_p3_world as bp3  # noqa: E402
import merge_world as mw  # noqa: E402

STEPS = 500
JOINT = 'J14_SHOULDER_ROLL_L'
#: The right arm is the mirror of the left: `stand.yaml` declares `[0, +0.05, 0, -0.1, 0]` for the
#: left arm and `[0, -0.05, 0, -0.1, 0]` for the right. The first sweep found that abducting only
#: the LEFT arm simply moved the worst contact to the RIGHT palm, so the two are swept together --
#: a one-sided sweep on a symmetric pose measures the pose's symmetry, not the fix.
MIRROR = 'J19_SHOULDER_ROLL_R'
MIRROR_SIGN = -1.0
#: The declared value plus a spread either side of it, so the sweep can show the contact getting
#: WORSE in one direction -- a sweep that only improves would not tell a reader which direction
#: is the safe one.
OFFSETS = (0.00, 0.05, 0.10, 0.20, 0.30, 0.40)


def bname(model, i):
    return mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, i) or f'<body{i}>'


def worst_self(model, data):
    worst = (0.0, None, None)
    for i in range(data.ncon):
        g1, g2 = int(data.contact.geom[i][0]), int(data.contact.geom[i][1])
        n1 = bname(model, int(model.geom_bodyid[g1]))
        n2 = bname(model, int(model.geom_bodyid[g2]))
        if not (n1.startswith('h_') and n2.startswith('h_')):
            continue
        d = float(data.contact.dist[i])
        if d < worst[0]:
            worst = (d, n1, n2)
    return worst


def arm(offset):
    bp3.install()
    _text, layout, model, _s, _k, _n, _a = mw.build_text()
    names, target, kp, kd = mw.stand_law()
    target = np.array(target, dtype=float)
    index = names.index(JOINT)
    declared = float(target[index])
    mirror_index = names.index(MIRROR)
    mirror_declared = float(target[mirror_index])
    target[index] = declared + offset
    target[mirror_index] = mirror_declared + MIRROR_SIGN * offset
    data = mujoco.MjData(model)
    qpos, ctrl, _a2, _notes = mw.merged_home(model, layout)
    data.qpos[:] = qpos
    data.ctrl[:] = ctrl
    worst = (0.0, None, None)
    worst_step = None
    for step in range(STEPS):
        ctrl_now = np.zeros(model.nu, dtype=float)
        affine = int(mujoco.mjtBias.mjBIAS_AFFINE)
        joint_trn = int(mujoco.mjtTrn.mjTRN_JOINT)
        for a in range(model.nu):
            if int(model.actuator_biastype[a]) != affine or float(model.actuator_biasprm[a][1]) == 0.0:
                continue
            if int(model.actuator_trntype[a]) != joint_trn:
                continue
            jid = int(model.actuator_trnid[a, 0])
            if int(model.jnt_type[jid]) not in (2, 3):
                continue
            adr = int(model.jnt_qposadr[jid])
            value = float(qpos[adr])
            if int(model.actuator_ctrllimited[a]):
                lo, hi = (float(v) for v in model.actuator_ctrlrange[a])
                value = min(max(value, lo), hi)
            ctrl_now[a] = value
        index_map = mw._stand_actuator_index(model)
        for k, name in enumerate(names):
            jid = model.joint('h_' + name).id
            if jid < 0:
                continue
            adr, dof = int(model.jnt_qposadr[jid]), int(model.jnt_dofadr[jid])
            torque = ((float(target[k]) - float(data.qpos[adr])) * float(kp[k])
                      - float(data.qvel[dof]) * float(kd[k]))
            for a in index_map.get(int(jid), ()):
                ctrl_now[a] = torque
        data.ctrl[:] = ctrl_now
        mujoco.mj_step(model, data)
        w = worst_self(model, data)
        if w[0] < worst[0]:
            worst = w
            worst_step = step
    body = model.body('h_LINK_BASE').id
    return {'offset_rad': offset, 'offset_deg': round(np.degrees(offset), 3),
            'target_rad': declared + offset,
            'mirror_rad': mirror_declared + MIRROR_SIGN * offset,
            'worst_mm': round(worst[0] * 1000, 3), 'pair': (worst[1], worst[2]),
            'at_step': worst_step, 'base_z': round(float(data.xpos[body][2]), 4)}


def main():
    names, target, _kp, _kd = mw.stand_law()
    declared = float(np.array(target, dtype=float)[names.index(JOINT)])
    print(f'{JOINT}: declared target {declared:+.4f} rad ({np.degrees(declared):+.3f} deg)')
    print(f'window: {STEPS} steps at the model timestep, standing law re-evaluated every step')
    print()
    print(f"{'offset rad':>11} {'(deg)':>8} {'target':>9} {'worst self':>12} {'at step':>8}  pair")
    rows = []
    for offset in OFFSETS:
        r = arm(offset)
        rows.append(r)
        print(f"{r['offset_rad']:+11.3f} {r['offset_deg']:+8.3f} {r['target_rad']:+9.4f} "
              f"{r['worst_mm']:10.3f} mm {str(r['at_step']):>8}  {r['pair'][0]} x {r['pair'][1]}")
    print()
    best = min(rows, key=lambda r: r['worst_mm'])
    print(f"best of the sweep: offset {best['offset_rad']:+.3f} rad "
          f"({best['offset_deg']:+.3f} deg) -> {best['worst_mm']:.3f} mm"
          f"  ({best['pair'][0]} x {best['pair'][1]})")
    cleared = [r for r in rows if r['worst_mm'] > -1.0]
    if cleared:
        pick = min(cleared, key=lambda r: abs(r['offset_rad']))
        print(f"smallest offset that clears 1 mm: {pick['offset_rad']:+.3f} rad "
              f"({pick['offset_deg']:+.3f} deg) -> {pick['worst_mm']:.3f} mm")
    else:
        print('no offset in this sweep clears 1 mm; the contact is not a shoulder-roll problem')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
