"""Three questions the new proposal depends on, each answerable by a number.

Q1  IS `sole_flat` ALREADY A RIGID KINEMATIC CONSTRAINT? `solve_posture` drives a SIX-dimensional
    task -- foot position AND foot orientation -- on a 6-joint chain, so the sole target is pinned
    by construction. If the TARGET is flat while the PHYSICAL foot is pitched 3.87 deg at 0.25 m,
    then the roll is not a kinematics problem at all: something in the ANKLE'S OWN control channel is
    pulling the foot off the flat target. That distinction decides whether the proposal's first half
    is work to do or work already done.

Q2  WHO ROLLS THE FOOT? Decompose the ankle command into its three contributions -- the PD term, the
    free-floating gravity term `qfrc_bias`, and the balance term `kp*e + kd*vx` -- at the depth where
    `sole_flat` fails. If the balance term dominates the ankle's demand, it is the roll.

Q3  IS THE STANCE TORQUE SYMMETRIC? The proposal wants `tau_gravity_stance(q_des)` from the nominal
    geometry instead of from a contact projection, precisely because D094's projection came out
    asymmetric (L -29.99 vs R +15.96 N.m at the ankle). So: is the inverse-dynamics torque at the
    COMMANDED pose symmetric between the legs, joint by joint? If it is, the proposal's mechanism is
    available; if it is not, the asymmetry lives in the geometry rather than in the projection.
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
import merge_world as mw  # noqa: E402
import probe_p4_balance as PB  # noqa: E402
import probe_p4_crouch2 as C2  # noqa: E402

HIPS = {'L': ('J00_HIP_PITCH_L', 'J01_HIP_ROLL_L', 'J02_HIP_YAW_L'),
        'R': ('J06_HIP_PITCH_R', 'J07_HIP_ROLL_R', 'J08_HIP_YAW_R')}
KNEES = {'L': ('J03_KNEE_PITCH_L',), 'R': ('J09_KNEE_PITCH_R',)}


def ik_target_error(rig, q_target):
    """The foot's position and orientation error the IK target itself leaves. Q1."""
    m, ik = rig.m, rig.ik
    ik.qpos[:] = q_target
    mujoco.mj_forward(m, ik)
    out = {}
    for side in ('L', 'R'):
        fid = m.body(PB.FOOT[side]).id
        ep = rig.foot_pos[side] - np.asarray(ik.xpos[fid], float)
        er = np.asarray(ik.xmat[fid], float).reshape(3, 3) @ rig.foot_mat[side].T
        # rotation angle of the residual rotation matrix
        ang = float(np.arccos(np.clip((np.trace(er) - 1.0) / 2.0, -1.0, 1.0)))
        out[side] = {'pos_err_mm': float(np.linalg.norm(ep)) * 1000, 'rot_err_deg': np.degrees(ang)}
    return out


def static_stance_tau(rig, q_target, *, ctrl=None):
    """`qfrc_inverse` at the COMMANDED configuration with qacc = 0. Q3: is it symmetric?"""
    m = rig.m
    sc = mujoco.MjData(m)
    sc.qpos[:] = q_target
    sc.qvel[:] = 0.0
    sc.ctrl[:] = (np.zeros(m.nu) if ctrl is None else ctrl)
    mujoco.mj_forward(m, sc)
    sc.qacc[:] = 0.0
    mujoco.mj_inverse(m, sc)
    return np.array(sc.qfrc_inverse, float), np.array(sc.qfrc_bias, float)


def main():
    ref = __import__('json').loads(
        (ROOT / 'reports' / 'p4-balance-04' / 'report.json').read_text(encoding='utf-8'))
    kp_bal, kd_bal = ref['controller']['kp_bal'], ref['controller']['kd_bal']
    print(f'balance gains from p4-balance-04: kp_bal {kp_bal:.1f}, kd_bal {kd_bal:.1f}')

    for depth in (0.20, 0.25, 0.30):
        rig = PB.Rig()
        rig.capture_standing()
        r = C2.run(rig, depth, C2.dx_for(depth), correct=True, kp_bal=kp_bal, kd_bal=kd_bal,
                   ramp_s=2.0, hold_s=1.5)
        end = r['trace'][-1]
        print(f'\n================ depth {depth:.2f} m ================')
        def ankle_ctrl(rig):
            out = {}
            for side, n in PB.ANKLE_PITCH.items():
                a = mujoco.mj_name2id(rig.m, mujoco.mjtObj.mjOBJ_ACTUATOR, f'h_motor_{n}')
                out[side] = float(rig.d.ctrl[a])
            return out

        print(f"  physical: drop {end['drop_m'] * 1000:.1f} mm, foot pitch "
              f"{ {k: round(v, 2) for k, v in end['foot_pitch_deg'].items()} } deg, "
              f"ankle ctrl { {k: round(v, 3) for k, v in ankle_ctrl(rig).items()} } N.m, "
              f"self {end['self_mm']:.3f} mm")

        # ---- Q1: is the IK TARGET flat? ---------------------------------------------------
        pelvis = (rig.pelvis0[0] + C2.dx_for(depth), rig.pelvis0[1], rig.pelvis0[2] - depth)
        q_t, resid, marg = rig.solve_posture(pelvis, np.array(rig.d.qpos, float))
        tgt = ik_target_error(rig, q_t)
        print(f'  Q1 IK target foot error: '
              + '; '.join(f'{s} pos {v["pos_err_mm"]:.4f} mm rot {v["rot_err_deg"]:.4f} deg'
                          for s, v in tgt.items()))
        print(f'     IK residual { {k: round(v, 6) for k, v in resid.items()} } m, '
              f'tightest joint margin {min(marg.values()):.4f} rad')

        # the commanded sole angle for each foot, i.e. what the IK ASKED for
        m, ik = rig.m, rig.ik
        ik.qpos[:] = q_t
        mujoco.mj_forward(m, ik)
        for side in ('L', 'R'):
            fid = m.body(PB.FOOT[side]).id
            R = np.asarray(ik.xmat[fid], float).reshape(3, 3)
            up = R @ np.array([0.0, 0.0, 1.0])
            print(f'     commanded sole {side}: up-vector {np.round(up, 5)} -> '
                  f'{np.degrees(np.arccos(np.clip(up[2], -1, 1))):.3f} deg off vertical')

        # ---- Q2: who rolls the foot? -------------------------------------------------------
        e = float(rig.humanoid_com()[0]) - float(rig.posture_com(q_t)[0])
        vx = rig.com_x_velocity()
        raw = kp_bal * e + kd_bal * vx
        print(f'  Q2 balance term: com_err {e * 1000:+.3f} mm, vx {vx:+.5f} m/s -> raw '
              f'{raw:+.3f} N.m added to EACH ankle')
        bias, applied, pd_term = {}, {}, {}
        for side in ('L', 'R'):
            n = PB.ANKLE_PITCH[side]
            dof = int(rig.m.jnt_dofadr[rig.m.joint('h_' + n).id])
            bias[side] = float(rig.d.qfrc_bias[dof])
            a = mujoco.mj_name2id(rig.m, mujoco.mjtObj.mjOBJ_ACTUATOR, f'h_motor_{n}')
            applied[side] = float(rig.d.ctrl[a])
            pd_term[side] = applied[side] - bias[side] - raw
        print(f'     ankle decomposition per side:')
        for side in ('L', 'R'):
            print(f'       {side}: total {applied[side]:+8.3f} = PD {pd_term[side]:+8.3f} '
                  f'+ qfrc_bias {bias[side]:+.4f} + balance {raw:+.3f}')

        # ---- Q3: is the stance torque symmetric? -------------------------------------------
        inv, bias_t = static_stance_tau(rig, q_t)
        des = static_stance_tau(rig, q_t, ctrl=np.zeros(rig.m.nu))[0]
        print(f'  Q3 static stance torque at the COMMANDED pose (qfrc_inverse, qacc=0):')
        print(f'     {"joint":>22} {"bias":>10} {"inv(ctrl=0)":>12} {"inv(cur ctrl)":>13} '
              f'{"L-R (ctrl=0)":>13}')
        for i, n in enumerate(PB.LEG_CHAIN['L']):
            nR = PB.LEG_CHAIN['R'][i]
            dL = int(rig.m.jnt_dofadr[rig.m.joint('h_' + n).id])
            dR = int(rig.m.jnt_dofadr[rig.m.joint('h_' + nR).id])
            print(f'     {n:>22} {bias_t[dL]:10.3f} {des[dL]:12.3f} {inv[dL]:13.3f} '
                  f'{des[dL] - des[dR]:13.3f}')
        hipknee = [d for side in ('L', 'R') for n in HIPS[side] + KNEES[side]
                   for d in (int(rig.m.jnt_dofadr[rig.m.joint('h_' + n).id]),)]
        ank = [int(rig.m.jnt_dofadr[rig.m.joint('h_' + PB.ANKLE_PITCH[s]).id]) for s in ('L', 'R')]
        print(f'     |hip+knee| max {max(abs(des[d]) for d in hipknee):.3f} N.m   '
              f'|ankle| {[round(float(des[d]), 3) for d in ank]} N.m')
        gap = [abs(des[int(rig.m.jnt_dofadr[rig.m.joint("h_" + PB.LEG_CHAIN['L'][i]).id])]
                   - des[int(rig.m.jnt_dofadr[rig.m.joint("h_" + PB.LEG_CHAIN['R'][i]).id])])
               for i in range(6)]
        print(f'     symmetry gap L-R per joint (deg): {[round(float(v), 4) for v in gap]}')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
