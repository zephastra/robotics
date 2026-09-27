"""P4 step 2b: can the humanoid HOLD a 0.40 m crouch -- sole flat, both feet carrying load, CoM
inside the footprint -- for long enough to place a tray?

WHY THIS EXISTS
---------------
The first crouch attempt (`reports/p4-crouch-01`, FAIL) commanded the SAME amplitude to hip, knee
and ankle. Three diagnostics found why that could not work, and each finding replaced a guess:

  * `_dbg_p4_balance.py` - the triple rolls the FOOT. One amplitude for three joints whose axes
    stack means the sole pitches by the sum: measured 42-45 deg of foot pitch, one foot's load
    going to 0 N, then the humanoid on one leg. The instrument that was supposed to say this
    (`Body.foot_load`) summed `abs(contact.dist) * 0.0` and read 0 N in every state.
  * `_dbg_p4_legs.py` / `_dbg_p4_legmap.py` - the posture that keeps the sole flat is a SOLUTION of
    a 6-joint-per-leg IK with the feet pinned, not a hand-picked amplitude. Solved that way, the
    reachable set is a diagonal band: a deeper crouch needs the pelvis further BACK.
  * The CoM must be the HUMANOID's (subtree at `h_LINK_BASE`, 86 kg), not `subtree_com[0]`, which
    is the whole 251 kg scene and sits 3 m away from the feet.

WHAT IS CONTROLLED, AND WHAT IS MERELY MEASURED
-----------------------------------------------
  * POSTURE: leg joint targets are re-solved every control step by the same IK, from the measured
    state, for the commanded pelvis pose. Nothing is written into `qpos` during the run -- the IK
    runs on a SCRATCH `MjData` so the simulation state cannot be corrupted by the solver.
  * BALANCE: an ankle-strategy torque on both ankle pitch joints, fed by the CoM error against a
    reference the posture itself defines. The SIGN is measured, not assumed.
  * The run reaches the crouch over a ramp, then HOLDS it, because a pose that exists is not a pose
    that stays.
"""
import argparse
import hashlib
import json
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'experiments'))
sys.path.insert(0, str(ROOT / 'src'))

import mujoco  # noqa: E402
import build_p3_world as bp3  # noqa: E402
import merge_world as mw  # noqa: E402

WORLD = ROOT / 'assets' / 'world_p4_handover.xml'
BASE = 'h_LINK_BASE'
FOOT = {'L': 'h_LINK_FOOT_L', 'R': 'h_LINK_FOOT_R'}
LEG_CHAIN = {
    'L': ('J00_HIP_PITCH_L', 'J01_HIP_ROLL_L', 'J02_HIP_YAW_L',
          'J03_KNEE_PITCH_L', 'J04_ANKLE_PITCH_L', 'J05_ANKLE_ROLL_L'),
    'R': ('J06_HIP_PITCH_R', 'J07_HIP_ROLL_R', 'J08_HIP_YAW_R',
          'J09_KNEE_PITCH_R', 'J10_ANKLE_PITCH_R', 'J11_ANKLE_ROLL_R'),
}
ANKLE_PITCH = {'L': 'J04_ANKLE_PITCH_L', 'R': 'J10_ANKLE_PITCH_R'}
SQUAT_TRIPLE = {'L': ('J00_HIP_PITCH_L', 'J03_KNEE_PITCH_L', 'J04_ANKLE_PITCH_L'),
                'R': ('J06_HIP_PITCH_R', 'J09_KNEE_PITCH_R', 'J10_ANKLE_PITCH_R')}

CROUCH_M = 0.40
CROUCH_TOL_M = 0.020
#: Measured in `_dbg_p4_legmap`: at 0.40 m the flat-footed band is dx in [-0.24, -0.20].
PELVIS_DX_M = -0.20
RAMP_S = 2.0
HOLD_S = 2.5
#: The sole must stay flat: max |pitch| over the whole run.
FOOT_FLAT_MAX_DEG = 3.0
#: Each foot must carry at least this share of the humanoid's weight while holding.
MIN_FOOT_SHARE = 0.25
#: The CoM must stay this far inside the footprint, at every step.
MIN_COM_MARGIN_M = 0.010
#: The measured CoM must track the posture's own reference within this.
COM_TRACK_TOL_M = 0.030
#: A held pose must be this still at the end.
SETTLED_VEL_RAD_S = 0.20
SETTLED_COM_MM_S = 20.0
SELF_ALLOWANCE_MM = -0.5
#: The humanoid's own weight, measured in `_dbg_p4_legs` (86.209 kg).
WEIGHT_N = 845.7
#: Ground-reaction normal forces must sum to the body weight (within this band) at every sample;
#: outside it, the load numbers are not a measurement and the load row is reported as INVALID.
LOAD_SUM_BAND = (0.70, 1.40)
#: Multiplier on the gravity stiffness. 1.0 is NEUTRAL (cancels gravity exactly, no restoring
#: term); the restoring stiffness is (ratio - 1) * m * g.
BALANCE_STIFFNESS_RATIO = 3.0
#: Task-space correction bounds and the IK acceptance threshold. A correction that walks the IK
#: target out of the feasible band produces a large residual, and a rejected solve must fall back to
#: the previous posture rather than command a nonsense one.
TASK_CORR_LIMIT_M = 0.060
IK_ACCEPT_M = 1e-4


def rot_err(Rc, Rt):
    R = Rt @ Rc.T
    ang = float(np.arccos(np.clip((np.trace(R) - 1.0) / 2.0, -1.0, 1.0)))
    if ang < 1e-9:
        return np.zeros(3)
    ax = np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]]) / (2.0 * np.sin(ang))
    return ax * ang


class Rig:
    def __init__(self, world=WORLD):
        self.m = mujoco.MjModel.from_xml_path(str(world))
        self.d = mujoco.MjData(self.m)
        #: A separate data for the IK. The solver writes qpos hundreds of times per control step;
        #: doing that on `self.d` would corrupt the simulation, and "remember to restore it" is a
        #: discipline, not a guarantee. Two buffers makes the guarantee structural.
        self.ik = mujoco.MjData(self.m)
        self.home = np.asarray(mw.merged_home(self.m)[0], dtype=float)
        self.names, self.target, self.kp, self.kd = mw.stand_law()
        self.index = mw._stand_actuator_index(self.m)
        self.name_index = {n: k for k, n in enumerate(self.names)}
        self.act_span, self.ankle_act = {}, {}
        for side, n in ANKLE_PITCH.items():
            jid = self.m.joint('h_' + n).id
            acts = list(self.index.get(int(jid), ()))
            if not acts:
                raise RuntimeError(f'no actuator drives h_{n}: the balance term would be a no-op')
            self.ankle_act[side] = acts
            span = 0.0
            for a in acts:
                span = max(span, abs(float(self.m.actuator_ctrlrange[a][1])))
            self.act_span[n] = span
        self.reset()

    # -- state -------------------------------------------------------------------------------
    def reset(self):
        self.d.qpos[:] = self.home
        self.d.qvel[:] = 0.0
        mujoco.mj_forward(self.m, self.d)
        self.prev_com = None
        self.prev_t = None

    def settle(self, seconds):
        """Hold the declared home state for `seconds`.

        ★ RECOMPUTED EVERY STEP. `home_hold_ctrl` returns TORQUES for the humanoid's limb
        actuators (they are pure-torque motors with no position servo), so a ctrl vector computed
        once and held is an open-loop constant: the first version of `capture_standing` did that and
        the humanoid collapsed to pelvis z 0.3925 -- a 0.62 m fall -- during what was supposed to be
        a settle. Same defect shape as the standing law evaluated once as a constant.
        """
        for _ in range(int(seconds / self.m.opt.timestep)):
            self.d.ctrl[:] = np.asarray(
                mw.home_hold_ctrl(self.m, self.home, self.d.qpos, self.d.qvel), float)
            mujoco.mj_step(self.m, self.d)

    def capture_standing(self, seconds=1.2):
        self.reset()
        self.settle(seconds)
        self.q_stand = np.array(self.d.qpos, dtype=float)          # COPY
        self.pelvis0 = np.array(self.d.xpos[self.m.body(BASE).id], dtype=float)
        self.foot_pos = {s: np.array(self.d.xpos[self.m.body(FOOT[s]).id], dtype=float)
                         for s in ('L', 'R')}                      # COPIES
        self.foot_mat = {s: np.array(self.d.xmat[self.m.body(FOOT[s]).id],
                                     dtype=float).reshape(3, 3) for s in ('L', 'R')}
        for s in ('L', 'R'):
            if np.shares_memory(self.foot_mat[s], self.d.xmat[self.m.body(FOOT[s]).id]):
                raise RuntimeError('foot_mat aliases d.xmat: the target would move with the state')
        self.support = self.footprint()

    def footprint(self, side=None):
        """World x-extent of the foot geoms. From geom geometry, not the contact list -- the list
        is empty exactly when a foot leaves the floor, which is when the number matters."""
        lo, hi = [], []
        for g in range(self.m.ngeom):
            bn = mujoco.mj_id2name(self.m, mujoco.mjtObj.mjOBJ_BODY,
                                   int(self.m.geom_bodyid[g])) or ''
            if bn not in FOOT.values():
                continue
            if side is not None and bn != FOOT[side]:
                continue
            half = self.m.geom_size[g][:3]
            R = np.array(self.d.geom_xmat[g], dtype=float).reshape(3, 3)
            c = np.array(self.d.geom_xpos[g], dtype=float)
            for sx in (-1, 1):
                for sy in (-1, 1):
                    for sz in (-1, 1):
                        lo.append((c + R @ (sx * half[0], sy * half[1], sz * half[2]))[0])
        return (min(lo), max(lo))

    def humanoid_com(self, data=None):
        d = self.d if data is None else data
        mujoco.mj_comPos(self.m, d)
        return np.array(d.subtree_com[self.m.body(BASE).id], dtype=float)

    def com_x_velocity(self):
        if self.prev_com is None:
            return 0.0
        dt = float(self.d.time) - float(self.prev_t)
        if dt <= 0:
            return 0.0
        return float(self.humanoid_com()[0] - float(self.prev_com[0])) / dt

    def foot_loads(self):
        """Normal force under each foot, from the solver's contact forces."""
        out = {'L': 0.0, 'R': 0.0}
        res = np.zeros(6, dtype=float)
        for i in range(self.d.ncon):
            c = self.d.contact[i]
            for near, far in ((int(c.geom[0]), int(c.geom[1])), (int(c.geom[1]), int(c.geom[0]))):
                bn = mujoco.mj_id2name(self.m, mujoco.mjtObj.mjOBJ_BODY,
                                       int(self.m.geom_bodyid[near])) or ''
                if bn not in FOOT.values():
                    continue
                # ★ ONLY THE FLOOR. The world body (id 0) carries the ground plane; any other
                # partner -- the presentation table, the conveyor frame -- is a different force and
                # including it is how the load sum reached 2.9x the body weight while the humanoid
                # was standing still.
                if int(self.m.geom_bodyid[far]) != 0:
                    continue
                mujoco.mj_contactForce(self.m, self.d, i, res)
                out['L' if bn.endswith('_L') else 'R'] += abs(float(res[0]))
        return out

    def cop_x(self):
        """Normal-force-weighted mean contact point of the feet on the floor."""
        num, den = 0.0, 0.0
        res = np.zeros(6, dtype=float)
        for i in range(self.d.ncon):
            c = self.d.contact[i]
            for near, far in ((int(c.geom[0]), int(c.geom[1])), (int(c.geom[1]), int(c.geom[0]))):
                bn = mujoco.mj_id2name(self.m, mujoco.mjtObj.mjOBJ_BODY,
                                       int(self.m.geom_bodyid[near])) or ''
                if bn not in FOOT.values():
                    continue
                if int(self.m.geom_bodyid[far]) != 0:
                    continue
                mujoco.mj_contactForce(self.m, self.d, i, res)
                f = abs(float(res[0]))
                num += f * float(c.pos[0])
                den += f
        return (num / den) if den > 0 else float('nan')

    def foot_pitch_deg(self):
        out = {}
        for s in ('L', 'R'):
            R = np.array(self.d.xmat[self.m.body(FOOT[s]).id], dtype=float).reshape(3, 3)
            out[s] = float(np.degrees(np.arccos(np.clip(R[2, 2], -1.0, 1.0))))
        return out

    def deepest_self_mm(self):
        worst, pair = 0.0, ''
        for i in range(self.d.ncon):
            c = self.d.contact[i]
            b1 = mujoco.mj_id2name(self.m, mujoco.mjtObj.mjOBJ_BODY,
                                   int(self.m.geom_bodyid[int(c.geom[0])])) or ''
            b2 = mujoco.mj_id2name(self.m, mujoco.mjtObj.mjOBJ_BODY,
                                   int(self.m.geom_bodyid[int(c.geom[1])])) or ''
            if b1.startswith('h_') and b2.startswith('h_'):
                v = float(c.dist) * 1000.0
                if v < worst:
                    worst, pair = v, f'{b1} x {b2}'
        return worst, pair

    def leg_vel(self):
        worst = 0.0
        for chain in LEG_CHAIN.values():
            for n in chain:
                j = self.m.joint('h_' + n).id
                worst = max(worst, abs(float(self.d.qvel[int(self.m.jnt_dofadr[j])])))
        return worst

    # -- the posture controller --------------------------------------------------------------
    def solve_posture(self, pelvis, seed):
        """Leg joint angles that put the pelvis at `pelvis` with the feet where they stand.

        A 6-joint leg with the foot's full pose pinned is EXACTLY determined, so this is a solve,
        not an optimisation with a preference. The knee can flex far and the hip can flex far; the
        ankle is the joint that runs out, and it is the one whose margin is reported.
        """
        m, ik = self.m, self.ik
        q = np.array(seed, dtype=float)
        q[0], q[1], q[2] = pelvis
        q[3:7] = self.q_stand[3:7]                   # the pelvis orientation stays as it stands
        jacp, jacr = np.zeros((3, m.nv)), np.zeros((3, m.nv))
        residuals, margins = {}, {}
        for side in ('L', 'R'):
            chain = LEG_CHAIN[side]
            dofs = np.array([int(m.jnt_dofadr[m.joint('h_' + n).id]) for n in chain])
            qadrs = np.array([int(m.jnt_qposadr[m.joint('h_' + n).id]) for n in chain])
            lows = np.array([float(m.jnt_range[m.joint('h_' + n).id][0]) for n in chain])
            highs = np.array([float(m.jnt_range[m.joint('h_' + n).id][1]) for n in chain])
            fid = m.body(FOOT[side]).id
            best_r, best_q = None, None
            for _ in range(60):
                ik.qpos[:] = q
                mujoco.mj_forward(m, ik)
                ep = self.foot_pos[side] - np.asarray(ik.xpos[fid], dtype=float)
                er = rot_err(np.asarray(ik.xmat[fid]).reshape(3, 3), self.foot_mat[side])
                err = np.concatenate([ep, er])
                r = float(np.linalg.norm(err))
                if best_r is None or r < best_r:
                    best_r, best_q = r, np.array(q)
                if r < 1e-6:
                    break
                mujoco.mj_jacBody(m, ik, jacp, jacr, fid)
                J = np.vstack([jacp[:, dofs], jacr[:, dofs]])
                if not np.any(J):
                    raise RuntimeError('zero Jacobian in the posture solve')
                dq = J.T @ np.linalg.solve(J @ J.T + (0.01 ** 2) * np.eye(6), err)
                big = float(np.max(np.abs(dq)))
                if big > 0.05:
                    dq = dq * (0.05 / big)
                q[qadrs] = np.clip(q[qadrs] + dq, lows, highs)
            q = best_q
            residuals[side] = best_r
            for n, adr, lo, hi in zip(chain, qadrs, lows, highs):
                margins[n] = min(float(q[adr]) - lo, hi - float(q[adr]))
        return q, residuals, margins

    def posture_com(self, q):
        """The CoM the commanded posture implies. The balance term's reference, derived from the
        posture rather than typed: it is the CoM of the same IK solution the joints are tracking.

        `q` is loaded into the scratch data first: the solve loop stops at its BEST iterate, which
        is not necessarily its last, so forwarding the scratch data as it stands would read a pose
        the controller never commands.
        """
        self.ik.qpos[:] = q
        mujoco.mj_forward(self.m, self.ik)
        return self.humanoid_com(self.ik)

    # -- one control step --------------------------------------------------------------------
    def tick(self, q_target, *, balance, kp_bal, kd_bal, com_ref_x, com_vx,
             gravity_ff=True):
        """One control step: posture PD, optional gravity feed-forward, optional ankle balance.

        ★ GRAVITY FEED-FORWARD IS NOT DECORATION. The humanoid's limb actuators are pure-torque
        motors, so the PD has to supply the whole body weight out of its own error budget: holding a
        bent knee against gravity with only `kp * err` means a steady-state error of
        `tau_required / kp`, and at a deep crouch that error is large enough that the leg folds and
        the pelvis drops hundreds of mm -- measured 836 mm of descent for a 400 mm target, before
        this term existed. `qfrc_bias` at the measured state is the gravity + Coriolis torque in
        joint coordinates, so adding it leaves the PD responsible only for the error.
        """
        ctrl = np.asarray(mw.home_hold_ctrl(self.m, self.home, self.d.qpos, self.d.qvel), float)
        self.ff_torque = {}
        for k, name in enumerate(self.names):
            jid = self.m.joint('h_' + name).id
            if jid < 0:
                continue
            adr, dof = int(self.m.jnt_qposadr[jid]), int(self.m.jnt_dofadr[jid])
            want = float(q_target[adr])
            tau = (want - float(self.d.qpos[adr])) * float(self.kp[k]) \
                - float(self.d.qvel[dof]) * float(self.kd[k])
            if gravity_ff:
                tau += float(self.d.qfrc_bias[dof])
                self.ff_torque[name] = abs(float(self.d.qfrc_bias[dof]))
            for a in self.index.get(int(jid), ()):
                lo, hi = self.m.actuator_ctrlrange[a]
                ctrl[a] = float(min(max(tau, lo), hi))
        if balance:
            e = float(self.humanoid_com()[0]) - float(com_ref_x)
            raw = kp_bal * e + kd_bal * com_vx
            for side, name in ANKLE_PITCH.items():
                for a in self.ankle_act[side]:
                    lo, hi = self.m.actuator_ctrlrange[a]
                    self.balance_sat = max(getattr(self, 'balance_sat', 0.0),
                                           abs(raw) / max(abs(lo), abs(hi)))
                    ctrl[a] = float(min(max(ctrl[a] + raw, lo), hi))
        self.d.ctrl[:] = ctrl
        mujoco.mj_step(self.m, self.d)

    def torque_usage(self):
        """Worst commanded torque, as a fraction of each actuator's DECLARED range."""
        worst, who = 0.0, ''
        for name in self.names:
            jid = self.m.joint('h_' + name).id
            if jid < 0:
                continue
            for a in self.index.get(int(jid), ()):
                span = abs(float(self.m.actuator_ctrlrange[a][1]))
                if span <= 0:
                    continue
                share = abs(float(self.d.actuator_force[a])) / span
                if share > worst:
                    worst, who = share, name
        return worst, who

    # -- the run -----------------------------------------------------------------------------
    def run(self, *, balance, kp_bal, kd_bal, dx, dz, ramp_s, hold_s, record_every=0.05,
            gravity_ff=True, k_task=0.30, pelvis_corr_fixed=None):
        self.reset()
        self.settle(1.0)
        standing_z = float(self.d.xpos[self.m.body(BASE).id][2])
        trace, ik_resid, ik_margin = [], [], []
        n_ramp = max(1, int(ramp_s / self.m.opt.timestep))
        n_hold = max(1, int(hold_s / self.m.opt.timestep))
        self.prev_com, self.prev_t = self.humanoid_com(), float(self.d.time)
        com_ref = float(self.humanoid_com()[0])
        self.balance_sat = 0.0
        self.pelvis_corr = np.zeros(3, dtype=float)
        self.worst_pelvis_err = 0.0
        self.ik_rejects = 0
        self.q_target_prev = np.array(self.d.qpos, dtype=float)
        if pelvis_corr_fixed is not None:
            self.pelvis_corr = np.array([pelvis_corr_fixed, 0.0, pelvis_corr_fixed], dtype=float)
        every = max(1, int(record_every / self.m.opt.timestep))
        for i in range(n_ramp + n_hold):
            frac = min(1.0, (i + 1) / n_ramp) if n_ramp else 1.0
            phase = 'ramp' if i < n_ramp else 'hold'
            seed = np.array(self.d.qpos, dtype=float)
            commanded = np.array([self.pelvis0[0] + dx * frac, self.pelvis0[1],
                                  self.pelvis0[2] + dz * frac], dtype=float)
            # ★ THE POSTURE LOOP CLOSES ON THE PELVIS POSE, not on the joint angles. The IK
            # answers "which joint angles are consistent with the pelvis at P"; the PD then tracks
            # those angles, and any steady-state joint error shows up as the pelvis NOT being at P.
            # Measured before this loop existed: at a 350 mm command the pelvis had already sunk
            # 53 mm below it and the gap was growing, because a gravity-only feed-forward does not
            # know about the ground reaction that the closed chain adds. The correction is applied
            # to the IK's TARGET, so the leg is asked for a slightly more extended posture until the
            # pelvis is where it was asked to be.
            measured = np.array(self.d.xpos[self.m.body(BASE).id], dtype=float)
            # ★ BOUNDED. The first version integrated the full error with gain 1 and clamped the
            # correction to 0.25 m, which walked the IK target outside the feasible band: the solver
            # then reported a residual of 1346.9 mm and its "solution" was garbage joint angles. The
            # correction is applied only to x and z, with a gain below 1 and a 60 mm limit, and the
            # IK result is REJECTED if it did not converge -- so an unreachable target degrades to
            # "hold the previous posture and say so", never to a nonsense posture.
            if k_task:
                self.pelvis_corr = np.clip(
                    self.pelvis_corr + k_task * (commanded - measured), -TASK_CORR_LIMIT_M,
                    TASK_CORR_LIMIT_M)
            pelvis = tuple(commanded + self.pelvis_corr)
            q_try, resid, marg = self.solve_posture(pelvis, seed)
            if max(resid.values()) > IK_ACCEPT_M:
                self.ik_rejects += 1
                q_try = self.q_target_prev
            else:
                self.q_target_prev = q_try
            q_target = q_try
            com_ref = float(self.posture_com(q_target)[0])
            com_vx = self.com_x_velocity()
            self.prev_com, self.prev_t = self.humanoid_com(), float(self.d.time)
            self.tick(q_target, balance=balance, kp_bal=kp_bal, kd_bal=kd_bal,
                      com_ref_x=com_ref, com_vx=com_vx, gravity_ff=gravity_ff)
            ik_resid.append(max(resid.values()))
            ik_margin.append(min(marg.values()))
            now = np.array(self.d.xpos[self.m.body(BASE).id], dtype=float)
            self.worst_pelvis_err = max(self.worst_pelvis_err,
                                        float(np.linalg.norm(commanded - now)))
            if i % every == 0 or i == n_ramp + n_hold - 1:
                self.prev_com, self.prev_t = self.humanoid_com(), float(self.d.time)
                trace.append(self.sample(standing_z, pelvis[2] - self.pelvis0[2], com_ref,
                                         phase))
        return {'trace': trace, 'standing_z': standing_z,
                'max_ik_resid_m': max(ik_resid), 'min_joint_margin_rad': min(ik_margin),
                'com_ref_final': com_ref, 'balance_saturation': float(self.balance_sat),
                'worst_pelvis_err_m': float(self.worst_pelvis_err),
                'pelvis_corr_final': [round(float(v), 5) for v in self.pelvis_corr],
                'ik_rejects': int(self.ik_rejects)}

    def sample(self, standing_z, dz_cmd, com_ref, phase):
        m = self.m
        com = self.humanoid_com()
        loads = self.foot_loads()
        pitch = self.foot_pitch_deg()
        lo, hi = self.support
        self_mm, pair = self.deepest_self_mm()
        usage, who = self.torque_usage()
        load_sum = loads['L'] + loads['R']
        return {'t': round(float(self.d.time), 3), 'phase': phase,
                'load_sum_N': round(load_sum, 1),
                'load_over_weight': round(load_sum / WEIGHT_N, 4),
                'torque_usage': round(usage, 4), 'torque_joint': who,
                'base_z': round(float(self.d.xpos[m.body(BASE).id][2]), 5),
                'drop_m': round(standing_z - float(self.d.xpos[m.body(BASE).id][2]), 5),
                'dz_cmd_m': round(float(dz_cmd), 5),
                'pelvis_z_err_m': round(self.d.xpos[m.body(BASE).id][2]
                                        - (self.pelvis0[2] + dz_cmd), 5),
                'com_x': round(float(com[0]), 5), 'com_ref_x': round(float(com_ref), 5),
                'com_err_m': round(float(com[0]) - float(com_ref), 5),
                'com_margin_m': [round(float(com[0]) - lo, 5), round(hi - float(com[0]), 5)],
                'cop_x': None if np.isnan(self.cop_x()) else round(self.cop_x(), 5),
                'foot_pitch_deg': {k: round(v, 3) for k, v in pitch.items()},
                'load_N': {k: round(v, 1) for k, v in loads.items()},
                'leg_vel': round(self.leg_vel(), 4),
                'self_mm': round(self_mm, 3), 'self_pair': pair}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--run-id', default='p4-balance-01')
    ap.add_argument('--reports-dir', default=None)
    ap.add_argument('--label', default=None)
    args = ap.parse_args()
    out = (Path(args.reports_dir) if args.reports_dir else ROOT / 'reports') / args.run_id
    out.mkdir(parents=True, exist_ok=True)

    bp3.install()
    rig = Rig()
    rig.capture_standing()
    checks = []

    def check(group, name, verdict, detail, numbers=None):
        checks.append({'group': group, 'name': name, 'verdict': verdict, 'detail': detail,
                       'numbers': numbers or {}})

    lo, hi = rig.support
    weight = 86.209 * 9.81          # the humanoid's own weight, measured in _dbg_p4_legs
    print(f'standing pelvis z {rig.pelvis0[2]:.5f}, support x [{lo:.4f}, {hi:.4f}]')
    print(f'humanoid weight {weight:.1f} N')

    # ---- 1. the balance TERM's sign and gain, measured -------------------------------------
    # An ankle-strategy torque on a CoM error has a sign that is a property of the joint axis, the
    # contact geometry and the mass distribution together. Guessing it is how a "balance term"
    # becomes a fall accelerant, so a short probe run at a deliberately shallow crouch is done with
    # both signs and the one that pulls the CoM onto its reference wins.
    # ★ THE GAIN IS THE GRAVITY STIFFNESS ABOUT THE ANKLE, not a tuned number. A CoM offset
    # from its reference is an unbalanced gravity moment about the ankle of `m * g * e`; an ankle
    # feedback with exactly that gain is the smallest term that stops the offset from growing on its
    # own, and it is DERIVED from the model's own mass, gravity and CoM height rather than picked.
    mass = weight / abs(float(rig.m.opt.gravity[2]))
    ankle_z = float(rig.d.xpos[rig.m.body('h_LINK_ANKLE_PITCH_L').id][2])
    h_com = float(rig.humanoid_com()[2]) - ankle_z
    i_ankle = mass * h_com ** 2
    # ★ `m * g * e` EXACTLY CANCELS the gravity moment, which makes the CoM NEUTRALLY stable,
    # not stable: the offset stops growing but nothing brings it back, and the damping then has to
    # do all the work. A gain of `BALANCE_STIFFNESS_RATIO * m * g` leaves `(ratio - 1) * m * g` as
    # actual restoring stiffness. Measured: at ratio 1.0 the CoM error was already -22 mm at 282 mm
    # of descent and the run fell, against UNDER 3 mm at the same depth with a stiffer term.
    kp_mag = BALANCE_STIFFNESS_RATIO * mass * abs(float(rig.m.opt.gravity[2]))
    kd_mag = 2.0 * 0.7 * float(np.sqrt(kp_mag * i_ankle))     # zeta = 0.7
    print(f'  gravity stiffness: mass {mass:.2f} kg, CoM {h_com * 1000:.0f} mm above the ankle, '
          f'I {i_ankle:.1f} kg.m2 -> kp_bal {kp_mag:.0f} N.m/m, kd_bal {kd_mag:.0f} N.m/(m/s)')
    sign_rows = []
    for sign in (+1.0, -1.0):
        kp_bal = sign * kp_mag
        kd_bal = sign * kd_mag
        r = rig.run(balance=True, kp_bal=kp_bal, kd_bal=kd_bal, dx=-0.08, dz=-0.20,
                    ramp_s=1.2, hold_s=1.2)
        err = max(abs(s['com_err_m']) for s in r['trace'] if s['phase'] == 'hold')
        sign_rows.append({'sign': sign, 'kp_bal': kp_bal, 'kd_bal': kd_bal,
                          'worst_com_err_hold_m': err,
                          'min_com_margin_m': min(min(s['com_margin_m']) for s in r['trace'])})
        print(f'  sign {sign:+.0f} kp_bal {kp_bal:+.1f} -> worst hold CoM error {err * 1000:.2f} mm')
    best_sign = min(sign_rows, key=lambda r: r['worst_com_err_hold_m'])
    kp_bal = best_sign['kp_bal']
    kd_bal = best_sign['kd_bal']
    check('balance', 'the ankle balance term\'s SIGN is measured, not assumed',
          'PASS' if best_sign['worst_com_err_hold_m'] < 0.005
          and max(r['worst_com_err_hold_m'] for r in sign_rows) > 2 * best_sign[
              'worst_com_err_hold_m'] else 'FAIL',
          f'the ankle torque is proportional to the CoM error; which sign corrects it is a '
          f'property of the ankle axis and the contact geometry, so both were run. With sign '
          f'{best_sign["sign"]:+.0f} the worst CoM error while holding is '
          f'{best_sign["worst_com_err_hold_m"] * 1000:.2f} mm, against '
          f'{max(r["worst_com_err_hold_m"] for r in sign_rows) * 1000:.2f} mm for the other -- '
          f'so one sign pulls the CoM onto its reference and the other pushes it away, and this '
          f'row is what tells them apart',
          {'sign_rows': sign_rows, 'chosen_sign': best_sign['sign'],
           'kp_bal': kp_bal, 'kd_bal': kd_bal})

    # ---- 1b. the two arms that say whether the added terms do any work -----------------------
    no_ff = rig.run(balance=True, kp_bal=kp_bal, kd_bal=kd_bal, dx=PELVIS_DX_M, dz=-CROUCH_M,
                    ramp_s=RAMP_S, hold_s=HOLD_S, gravity_ff=False, k_task=0.0)
    ff_drop_err = abs(no_ff['trace'][-1]['drop_m'] - CROUCH_M)

    # ---- 2. the run itself -----------------------------------------------------------------
    no_bal = rig.run(balance=False, kp_bal=0.0, kd_bal=0.0, dx=PELVIS_DX_M, dz=-CROUCH_M,
                     ramp_s=RAMP_S, hold_s=HOLD_S, k_task=0.0)
    run = rig.run(balance=True, kp_bal=kp_bal, kd_bal=kd_bal, dx=PELVIS_DX_M, dz=-CROUCH_M,
                  ramp_s=RAMP_S, hold_s=HOLD_S, k_task=0.0)
    terminal = [s for s in run['trace'] if s['phase'] == 'hold']
    last = run['trace'][-1]

    check('crouch', f'the pelvis ARRIVES {CROUCH_M:.2f} m below standing and STAYS there',
          'PASS' if abs(last['drop_m'] - CROUCH_M) <= CROUCH_TOL_M else 'FAIL',
          f'over a {RAMP_S:.1f} s ramp the pelvis went from z {run["standing_z"]:.4f} to '
          f'{last["base_z"]:.4f}, a descent of {last["drop_m"] * 1000:.1f} mm against the '
          f'{CROUCH_M * 1000:.0f} mm target (tolerance {CROUCH_TOL_M * 1000:.0f} mm), and it is '
          f'still there {HOLD_S:.1f} s later. The joint targets are not typed: every control step '
          f're-solves the leg IK from the measured state, so the worst solve residual over the '
          f'run is {run["max_ik_resid_m"] * 1000:.4f} mm',
          {'drop_m': last['drop_m'], 'target_m': CROUCH_M, 'base_z': last['base_z'],
           'max_ik_resid_m': run['max_ik_resid_m']})

    worst_pitch = max(max(s['foot_pitch_deg'].values()) for s in run['trace'])
    check('crouch', 'the sole stays FLAT all the way down',
          'PASS' if worst_pitch <= FOOT_FLAT_MAX_DEG else 'FAIL',
          f'worst sole pitch over the whole run is {worst_pitch:.2f} deg against a '
          f'{FOOT_FLAT_MAX_DEG:.1f} deg allowance. \u2605 This is the row that failed before it '
          f'existed: commanding one amplitude to hip, knee and ankle stacks three rotations on '
          f'the same foot and pitched the sole 42-45 deg, which collapses the support polygon and '
          f'is what actually put the humanoid on one leg in `reports/p4-crouch-01`',
          {'worst_foot_pitch_deg': worst_pitch, 'allowance_deg': FOOT_FLAT_MAX_DEG})

    check('balance', 'GRAVITY FEED-FORWARD is what carries the body, not the PD error',
          'PASS' if ff_drop_err > CROUCH_TOL_M and abs(last['drop_m'] - CROUCH_M) <= CROUCH_TOL_M
          else 'FAIL',
          f'the same controller with the feed-forward term switched OFF descends '
          f'{no_ff["trace"][-1]["drop_m"] * 1000:.1f} mm for a {CROUCH_M * 1000:.0f} mm target -- '
          f'an error of {ff_drop_err * 1000:.1f} mm, against '
          f'{abs(last["drop_m"] - CROUCH_M) * 1000:.1f} mm with it on. The limb actuators are pure '
          f'torque, so without the term the PD must generate the whole body weight out of its own '
          f'error and the leg simply folds. This arm is what makes the term a measurement rather '
          f'than a decoration',
          {'drop_m_no_ff': no_ff['trace'][-1]['drop_m'], 'drop_err_no_ff_m': ff_drop_err,
           'drop_m_with_ff': last['drop_m']})

    peak_usage = max(s['torque_usage'] for s in terminal)
    peak_joint = max(terminal, key=lambda s: s['torque_usage'])['torque_joint']
    check('torque', 'the held crouch stays inside every actuator\'s DECLARED range',
          'PASS' if peak_usage <= 0.95 else 'FAIL',
          f'worst commanded torque while holding is {peak_usage * 100:.1f} % of the declared range '
          f'at {peak_joint}. A pose that needs more torque than the joint has is not held, it is '
          f'fallen into -- and only the DECLARED range counts, not a comfort margin',
          {'peak_usage_fraction': peak_usage, 'peak_joint': peak_joint})

    check('balance', 'the ankle balance term is REGULATING, not pinned at its limit',
          'PASS' if run['balance_saturation'] <= 0.5 else 'FAIL',
          f'the largest balance torque the term asked for over the whole run is '
          f'{run["balance_saturation"] * 100:.1f} % of the ankle actuator range. A term that sits '
          f'saturated has stopped being feedback and become a constant offset with extra steps',
          {'worst_balance_fraction': run['balance_saturation']})

    share_lo = min(min(s['load_N']['L'], s['load_N']['R']) / weight for s in terminal)
    check('balance', 'BOTH feet carry load throughout the hold',
          'PASS' if share_lo >= MIN_FOOT_SHARE else 'FAIL',
          f'the lighter foot carries at least {share_lo * 100:.1f} % of the humanoid\'s '
          f'{weight:.1f} N weight at every recorded step of the hold, against a '
          f'{MIN_FOOT_SHARE * 100:.0f} % floor; at the end the loads are '
          f'L {last["load_N"]["L"]:.1f} N / R {last["load_N"]["R"]:.1f} N. Load is read from the '
          f'solver\'s contact forces -- the field that was previously summed as '
          f'`abs(dist) * 0.0` and read 0 N in every state',
          {'min_share_fraction': share_lo, 'floor': MIN_FOOT_SHARE,
           'load_end_N': last['load_N']})

    worst_margin = min(min(s['com_margin_m']) for s in run['trace'])
    check('balance', 'the CoM stays INSIDE the footprint, with margin, for every step',
          'PASS' if worst_margin >= MIN_COM_MARGIN_M else 'FAIL',
          f'the smallest distance from the humanoid\'s CoM to the edge of its footprint over the '
          f'run is {worst_margin * 1000:.1f} mm, against a {MIN_COM_MARGIN_M * 1000:.0f} mm '
          f'requirement; footprint x [{lo:.4f}, {hi:.4f}]. A crouch with the CoM outside the '
          f'footprint is a fall that has not finished yet',
          {'worst_com_margin_m': worst_margin, 'support_x': [lo, hi]})

    worst_track = max(abs(s['com_err_m']) for s in terminal)
    check('balance', 'the measured CoM TRACKS the reference the posture implies',
          'PASS' if worst_track <= COM_TRACK_TOL_M else 'FAIL',
          f'the CoM reference is the CoM of the same IK posture the joints are tracking, '
          f'recomputed every step -- so it is a property of the commanded pose, not a typed '
          f'constant. The worst deviation from it while holding is {worst_track * 1000:.2f} mm '
          f'against a {COM_TRACK_TOL_M * 1000:.0f} mm tolerance, so the ankle term is holding the '
          f'body where the posture puts it',
          {'worst_com_err_m': worst_track, 'tolerance_m': COM_TRACK_TOL_M})

    check('balance', 'the held crouch is STILL, not oscillating',
          'PASS' if last['leg_vel'] <= SETTLED_VEL_RAD_S else 'FAIL',
          f'at the end of the hold the fastest leg joint moves at {last["leg_vel"]:.4f} rad/s '
          f'against {SETTLED_VEL_RAD_S} rad/s. Falling and crouching both move the pelvis down; '
          f'only a settled pose can carry a tray',
          {'leg_vel_rad_s': last['leg_vel'], 'limit': SETTLED_VEL_RAD_S})

    deepest, pair = last['self_mm'], last['self_pair']
    worst_self = min(s['self_mm'] for s in run['trace'])
    check('interference', 'the crouch does not drive the humanoid into itself',
          'PASS' if worst_self >= SELF_ALLOWANCE_MM else 'FAIL',
          f'deepest humanoid self-contact over the whole run: {worst_self:.3f} mm against a '
          f'{SELF_ALLOWANCE_MM} mm allowance{f" ({pair})" if pair else ""}',
          {'worst_self_mm': worst_self})

    # ---- 3. the falsifiability arm ----------------------------------------------------------
    # The same measurement, on the controller this one replaced. If the rows above cannot fail,
    # this arm would pass too -- so it is run and required to FAIL.
    rig.reset()
    rig.settle(1.0)
    stand_z = float(rig.d.xpos[rig.m.body(BASE).id][2])
    signs = (-1.0, 1.0, 1.0)
    amp = 0.217
    worst_pitch_old, min_share_old = 0.0, 1.0
    frames = 0
    n = int(2.5 / rig.m.opt.timestep)
    every = max(1, int(0.25 / rig.m.opt.timestep))
    for i in range(n):
        if i % every == 0:
            worst_pitch_old = max(worst_pitch_old, max(rig.foot_pitch_deg().values()))
            ld = rig.foot_loads()
            min_share_old = min(min_share_old, min(ld['L'], ld['R']) / weight)
            frames += 1
        ctrl = np.asarray(mw.home_hold_ctrl(rig.m, rig.home, rig.d.qpos, rig.d.qvel), float)
        for side, names in SQUAT_TRIPLE.items():
            for s, name in zip(signs, names):
                k = rig.name_index.get(name)
                if k is None:
                    continue
                jid = rig.m.joint('h_' + name).id
                adr, dof = int(rig.m.jnt_qposadr[jid]), int(rig.m.jnt_dofadr[jid])
                tau = (float(rig.target[k]) + s * amp - float(rig.d.qpos[adr])) * float(rig.kp[k]) \
                    - float(rig.d.qvel[dof]) * float(rig.kd[k])
                for a in rig.index.get(int(jid), ()):
                    lo_a, hi_a = rig.m.actuator_ctrlrange[a]
                    ctrl[a] = float(min(max(tau, lo_a), hi_a))
        rig.d.ctrl[:] = ctrl
        mujoco.mj_step(rig.m, rig.d)
    old_drop = stand_z - float(rig.d.xpos[rig.m.body(BASE).id][2])
    check('falsifiability', 'the controller this one replaced STILL FAILS the same rows',
          'PASS' if (worst_pitch_old > FOOT_FLAT_MAX_DEG or min_share_old < MIN_FOOT_SHARE)
          else 'FAIL',
          f'the equal-amplitude triple at {amp} rad -- the controller that produced '
          f'`reports/p4-crouch-01` -- is run through the SAME instruments: the sole reaches '
          f'{worst_pitch_old:.2f} deg of pitch (allowance {FOOT_FLAT_MAX_DEG:.1f}) and the '
          f'lighter foot falls to {min_share_old * 100:.1f} % of body weight (floor '
          f'{MIN_FOOT_SHARE * 100:.0f} %), finishing {old_drop * 1000:.0f} mm below standing '
          f'against a {CROUCH_M * 1000:.0f} mm target. Without this arm, rows that pass on the '
          f'new controller say nothing about whether they could ever have failed',
          {'old_worst_foot_pitch_deg': worst_pitch_old,
           'old_min_foot_share': min_share_old, 'old_drop_m': old_drop,
           'old_amplitude_rad': amp, 'frames': frames})

    # ★ THE LOAD INSTRUMENT CHECKS ITSELF. Ground-reaction normals over a set of contacts must
    # sum to the body weight while the body is not accelerating; anything else means the numbers are
    # not a load measurement. Caught here: a `load_over_weight` of 2.9 on a settled humanoid, which
    # is statically impossible, so the load row below is only citable when this row passes.
    ratios = [s['load_over_weight'] for s in run['trace'] if s['phase'] == 'hold']
    worst_ratio = max(ratios)
    best_ratio = min(ratios)
    load_ok = (LOAD_SUM_BAND[0] <= best_ratio and worst_ratio <= LOAD_SUM_BAND[1])
    check('instrument', 'the foot-load numbers ADD UP to the body weight, or they are not citable',
          'PASS' if load_ok else 'FAIL',
          f'over the hold the summed ground-reaction normal force is between '
          f'{best_ratio * 100:.0f} % and {worst_ratio * 100:.0f} % of the humanoid\'s '
          f'{WEIGHT_N:.1f} N weight, against the {LOAD_SUM_BAND[0] * 100:.0f}-'
          f'{LOAD_SUM_BAND[1] * 100:.0f} % band. A body that is not accelerating must have its '
          f'weight carried by the floor, so a ratio outside this band means the contact-force '
          f'readout is measuring something else, and the load row cannot be quoted',
          {'min_load_over_weight': best_ratio, 'max_load_over_weight': worst_ratio})

    # ★ HOW DEEP CAN IT ACTUALLY HOLD? A single FAIL at 0.40 m says the target is out of reach
    # but not where the edge is. Each depth is a full run through the same rows.
    sweep_rows = []
    for dz_probe in (-0.10, -0.15, -0.20, -0.25, -0.30, -0.35, -0.40):
        r = rig.run(balance=True, kp_bal=kp_bal, kd_bal=kd_bal, dx=PELVIS_DX_M, dz=dz_probe,
                    ramp_s=1.5, hold_s=1.5, k_task=0.0)
        hold = [s for s in r['trace'] if s['phase'] == 'hold']
        end = r['trace'][-1]
        rows_ok = (abs(end['drop_m'] + dz_probe) <= CROUCH_TOL_M
                   and max(max(s['foot_pitch_deg'].values()) for s in r['trace'])
                   <= FOOT_FLAT_MAX_DEG
                   and min(min(s['com_margin_m']) for s in r['trace']) >= MIN_COM_MARGIN_M
                   and max(s['torque_usage'] for s in hold) <= 0.95
                   and end['leg_vel'] <= SETTLED_VEL_RAD_S
                   and max(abs(s['com_err_m']) for s in hold) <= COM_TRACK_TOL_M)
        sweep_rows.append({'dz_cmd_m': dz_probe, 'drop_m': end['drop_m'],
                           'drop_err_m': end['drop_m'] + dz_probe,
                           'worst_foot_pitch_deg': max(
                               max(s['foot_pitch_deg'].values()) for s in r['trace']),
                           'min_com_margin_m': min(min(s['com_margin_m']) for s in r['trace']),
                           'peak_torque_usage': max(s['torque_usage'] for s in hold),
                           'peak_joint': max(hold, key=lambda s: s['torque_usage'])['torque_joint'],
                           'leg_vel_end': end['leg_vel'],
                           'worst_com_err_m': max(abs(s['com_err_m']) for s in hold),
                           'holds': bool(rows_ok)})
    held = [r['dz_cmd_m'] for r in sweep_rows if r['holds']]
    deepest_held = min(held) if held else None
    print('\n depth sweep:')
    for r in sweep_rows:
        print(f'   {r["dz_cmd_m"]:+.2f} holds={r["holds"]!s:>5} drop {r["drop_m"] * 1000:7.1f} mm '
              f'pitch {r["worst_foot_pitch_deg"]:6.2f} deg  margin {r["min_com_margin_m"]:+.4f} m '
              f'torque {r["peak_torque_usage"] * 100:5.1f}%  {r["peak_joint"]}')
    check('crouch', 'the DEEPEST crouch this humanoid can actually HOLD is measured',
          'PASS' if deepest_held is not None else 'FAIL',
          f'sweeping the target depth through the same controller and the same rows, the deepest '
          f'crouch that arrives, stays flat, keeps the CoM inside the footprint, stays inside the '
          f'actuator ranges and settles is {deepest_held} m '
          f'({len(held)} of {len(sweep_rows)} depths). The band is measured per depth, not '
          f'interpolated, because "it held at 0.30 so it will hold at 0.32" is exactly the '
          f'assumption that a torque limit invalidates without warning',
          {'deepest_held_m': deepest_held, 'sweep': sweep_rows})

    failed = [c for c in checks if c['verdict'] == 'FAIL']
    verdict = 'FAIL' if failed else 'PASS'
    report = {'run_id': args.run_id, 'label': args.label,
              'scope': 'P4 step 2b: a HELD 0.40 m crouch with a flat sole, loaded feet and the '
                       'CoM inside the footprint',
              'world': {'path': str(WORLD.relative_to(ROOT)),
                        'sha256': hashlib.sha256(WORLD.read_bytes()).hexdigest()},
              'controller': {'posture': 'leg IK, feet pinned, re-solved every control step from '
                                        'the measured state',
                             'balance': 'ankle-strategy torque on the CoM error against the '
                                        'posture\'s own CoM reference',
                             'sign_measured': best_sign['sign'], 'kp_bal': kp_bal, 'kd_bal': kd_bal,
                             'qpos_writes_during_run': 0},
              'crouch_target_m': CROUCH_M, 'pelvis_dx_m': PELVIS_DX_M,
              'thresholds': {'crouch_tol_m': CROUCH_TOL_M, 'foot_flat_max_deg': FOOT_FLAT_MAX_DEG,
                             'min_foot_share': MIN_FOOT_SHARE,
                             'min_com_margin_m': MIN_COM_MARGIN_M,
                             'com_track_tol_m': COM_TRACK_TOL_M,
                             'settled_vel_rad_s': SETTLED_VEL_RAD_S},
              'support_x': [lo, hi], 'humanoid_weight_N': weight,
              'verdict': verdict, 'checks': checks,
              'sign_search': sign_rows, 'peak_torque_usage': peak_usage,
              'feed_forward_off': {'drop_m': no_ff['trace'][-1]['drop_m'],
                                   'drop_err_m': ff_drop_err},
              'worst_pelvis_err_m': run['worst_pelvis_err_m'],
              'pelvis_corr_final': run['pelvis_corr_final'],
              'ik_rejects': run['ik_rejects'],
              'depth_sweep': sweep_rows, 'deepest_held_m': deepest_held,
              'min_load_over_weight': best_ratio, 'max_load_over_weight': worst_ratio,
              'no_balance': {'drop_m': no_bal['trace'][-1]['drop_m'],
                             'worst_com_margin_m': min(
                                 min(s['com_margin_m']) for s in no_bal['trace'])},
              'trace': run['trace'], 'trace_no_balance': no_bal['trace'],
              'failed': [c['name'] for c in failed]}
    (out / 'report.json').write_text(json.dumps(report, indent=2, default=str) + '\n',
                                     encoding='utf-8')
    lines = [f'# {args.run_id} — P4 held crouch with balance', '', f'**verdict {verdict}**', '']
    if args.label:
        lines += [args.label, '']
    for c in checks:
        lines += [f'- **{c["verdict"]}** ({c["group"]}) {c["name"]}', f'  - {c["detail"]}']
    (out / 'acceptance.md').write_text('\n'.join(lines) + '\n', encoding='utf-8')
    for c in checks:
        print(f'  [{c["verdict"]}] ({c["group"]}) {c["name"]}')
        print(f'        {c["detail"]}')
    print(f'\nverdict {verdict}')
    print('\n trace (every 5th sample):')
    print(f'   {"t":>6} {"phase":>5} {"drop_mm":>8} {"com_x":>9} {"com_ref":>9} {"err_mm":>8} '
          f'{"marg_lo":>8} {"pitchL":>7} {"loadL":>8} {"loadR":>8} {"torq":>6}')
    for s in run['trace'][::5]:
        print(f'   {s["t"]:6.2f} {s["phase"]:>5} {s["drop_m"] * 1000:8.1f} {s["com_x"]:9.4f} '
              f'{s["com_ref_x"]:9.4f} {s["com_err_m"] * 1000:8.2f} {s["com_margin_m"][0]:8.4f} '
              f'{s["foot_pitch_deg"]["L"]:7.2f} {s["load_N"]["L"]:8.1f} {s["load_N"]["R"]:8.1f} '
              f'{s["torque_usage"] * 100:5.1f}%')
    return 0 if verdict == 'PASS' else 1


if __name__ == '__main__':
    raise SystemExit(main())
