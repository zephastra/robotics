"""P4 step 2c: contact-consistent feed-forward -- done so that it is NOT an echo of the output.

WHAT THE REVIEW PROPOSED, AND WHAT IS MEASURED ABOUT IT
------------------------------------------------------
The proposal is `tau_ff = tau_bias - tau_contact`, with `tau_contact = sum_k J_k^T f_k` read from
`mj_contactForce`. The formula and the sign are RIGHT, and measured to be right three ways:

  * four (frame x sign) combinations were tried against `qfrc_constraint`; `f_world = frame.T @
    f_local` with `+f` on the robot body wins, which is also the convention the contact frame
    document claims;
  * an INDEPENDENT check that never touches `qfrc_constraint`: the six floating-base rows of
    `sum J^T f` must equal the total contact wrench. Measured vertical 844.184 N against a humanoid
    weight of 845.706 N (99.82 %) -- right by Newton, not by fitting;
  * the deficit is real and large. At a held 0.20 m crouch the free-floating `qfrc_bias` supplies
    **+12.387 N.m at the knee** while static equilibrium needs **-92.0 N.m** there. The missing
    closed-chain term is ~104 N.m of joint torque, and the term present has the WRONG SIGN.

But one step of the proposal cannot work as written, and it is measurable:
`tau_bias - cons` evaluated at the measured state IS the torque already applied. Over the
humanoid's 25 actuated dofs the two agree to a mean of 0.55 N.m (worst 2.83, at a shoulder), and at
the ankle and knee to 0.001 / 0.07 N.m. That is Newton's law for a body in equilibrium, not a
feed-forward: injecting the projection of the MEASURED force adds the current output to itself.
The `measured` arm below is kept precisely as the negative control that shows this.

THE VERSION THAT IS NOT CIRCULAR
--------------------------------
The ground has to carry the body, so the force is known BEFORE looking at the actuators: half the
humanoid's weight per foot, downward through the sole. That number depends only on mass and
gravity, so projecting it is a genuine feed-forward. It is evaluated EVERY control step (a law
evaluated once as a constant is the D078 defect), and clamped to each joint's own actuator range
with the clamp activations counted rather than assumed.
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

bp3.install()
import merge_world as mw  # noqa: E402
import probe_p4_balance as PB  # noqa: E402
import probe_p4_crouch2 as C2  # noqa: E402

WORLD = ROOT / 'assets' / 'world_p4_handover.xml'
FEET = {'L': 'h_LINK_FOOT_L', 'R': 'h_LINK_FOOT_R'}
ANKLE_ACT = {'L': 'h_motor_J04_ANKLE_PITCH_L', 'R': 'h_motor_J10_ANKLE_PITCH_R'}
#: The arm joint block, from the reach study's own declaration -- not re-typed here.
ARM_JOINTS = None          # filled from `probe_p4_reach` at run time
#: A feed-forward this large, relative to the joint's own declared range, is reported as suspicious
#: even when it is inside the range.
FF_SUSPICIOUS_FRACTION = 0.60


def bname(m, bid):
    return mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, int(bid)) or ''


class FFRig(PB.Rig):
    """The crouch rig with a contact feed-forward, an arm lock, and the instruments to judge them.

    Subclassed rather than edited: `probe_p4_balance.py` is a frozen judge whose evidence
    (`reports/p4-balance-04/`) is cited, so its behaviour must not move under it.
    """

    def __init__(self, world=WORLD):
        super().__init__(world)
        self.ff_mode = 'off'
        self.balance_on = True
        self.carry_pose = None
        self.ff_used, self.ankle_Nm = {}, {}
        #: dof -> the actuators that drive it, so the feed-forward can be clamped to each joint's
        #: OWN declared range instead of a global one.
        self.dof_actuators = {}
        for jid, acts in self.index.items():
            dof = int(self.m.jnt_dofadr[jid])
            self.dof_actuators.setdefault(dof, []).extend(int(a) for a in acts)
        self.pose_locked, self.pose_home = None, None
        self.ff_clamp_hits, self.ff_peak_fraction = 0, 0.0
        self.instrument = {}

    # -- the feed-forward -------------------------------------------------------------------
    def _foot_contact_rows(self):
        """(side, contact_index, body_on_robot_side, application point) for foot-vs-world rows.

        Only contacts whose partner is the WORLD body count: a foot touching the other foot, or a
        hand, is not a support force, and mixing them in is how a load readout once reached 2.9x
        body weight on a humanoid standing still.
        """
        out = []
        for i in range(self.d.ncon):
            c = self.d.contact[i]
            b1 = int(self.m.geom_bodyid[int(c.geom[0])])
            b2 = int(self.m.geom_bodyid[int(c.geom[1])])
            n1, n2 = bname(self.m, b1), bname(self.m, b2)
            side = next((s for s, f in FEET.items() if f in (n1, n2)), None)
            if side is None:
                continue
            body = b1 if b1 != 0 else b2
            other = b2 if b1 != 0 else b1
            if other != 0 or body == 0:
                continue
            out.append((side, i, body, np.array(c.pos, dtype=float)))
        return out

    def contact_ff(self):
        """Feed-forward joint torque from the support wrench, per joint DOF. Empty when off."""
        if self.ff_mode == 'off':
            return {}
        rows = self._foot_contact_rows()
        if not rows:
            return {}
        weight = float(self.m.body_subtreemass[self.m.body(PB.BASE).id]) \
            * abs(float(self.m.opt.gravity[2]))
        per_side = {}
        for side, _i, _b, _p in rows:
            per_side[side] = per_side.get(side, 0) + 1
        tau = np.zeros(self.m.nv)
        jacp = np.zeros((3, self.m.nv))
        jacr = np.zeros((3, self.m.nv))
        raw = np.zeros(6)
        for side, i, body, pos in rows:
            if self.ff_mode == 'measured':
                mujoco.mj_contactForce(self.m, self.d, i, raw)
                frame = np.array(self.d.contact.frame[i], dtype=float).reshape(3, 3)
                f_world = frame.T @ np.array(raw[:3], dtype=float)
            else:
                f_world = np.array([0.0, 0.0, weight / (len(per_side) * per_side[side])])
            jacp[:] = 0.0
            jacr[:] = 0.0
            mujoco.mj_jac(self.m, self.d, jacp, jacr, pos, body)
            tau += jacp.T @ f_world
        # equilibrium: applied = qfrc_bias - qfrc_constraint, and the support wrench IS the
        # constraint term, so the feed-forward carries it with a MINUS sign.
        out = {}
        for name in self.names:
            jid = self.m.joint('h_' + name).id
            if jid < 0:
                continue
            out[int(self.m.jnt_dofadr[jid])] = float(-tau[int(self.m.jnt_dofadr[jid])])
        return out

    def _clamp_ff(self, ff):
        """Clamp the feed-forward to each joint's own declared actuator range, counting the hits."""
        clipped, hits = {}, 0
        for dof, value in ff.items():
            span = 0.0
            for a in self.dof_actuators.get(dof, ()):
                lo, hi = self.m.actuator_ctrlrange[a]
                span = max(span, abs(float(hi)))
                if abs(value) > abs(float(hi)):
                    value = float(hi) if value > 0 else float(lo)
                    hits += 1
            clipped[dof] = value
            if span:
                self.ff_peak_fraction = max(self.ff_peak_fraction, abs(clipped[dof]) / span)
        return clipped, hits

    # -- the control step --------------------------------------------------------------------
    def tick(self, q_target, *, balance, kp_bal, kd_bal, com_ref_x, com_vx, gravity_ff=True):
        ff, hits = self._clamp_ff(self.contact_ff())
        self.ff_clamp_hits += hits
        self.ff_used = {d: v for d, v in ff.items() if abs(v) > 1e-9}
        ctrl = np.asarray(mw.home_hold_ctrl(self.m, self.home, self.d.qpos, self.d.qvel), float)
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
            tau += float(ff.get(dof, 0.0))
            for a in self.index.get(int(jid), ()):
                lo, hi = self.m.actuator_ctrlrange[a]
                ctrl[a] = float(min(max(tau, lo), hi))
        if balance and self.balance_on:
            e = float(self.humanoid_com()[0]) - float(com_ref_x)
            raw = kp_bal * e + kd_bal * com_vx
            for side, name in PB.ANKLE_PITCH.items():
                for a in self.ankle_act[side]:
                    lo, hi = self.m.actuator_ctrlrange[a]
                    self.balance_sat = max(getattr(self, 'balance_sat', 0.0),
                                           abs(raw) / max(abs(lo), abs(hi)))
                    ctrl[a] = float(min(max(ctrl[a] + raw, lo), hi))
        self.ankle_Nm = {}
        for side, aname in ANKLE_ACT.items():
            a = mujoco.mj_name2id(self.m, mujoco.mjtObj.mjOBJ_ACTUATOR, aname)
            if a >= 0:
                self.ankle_Nm[side] = float(ctrl[a])
        self.d.ctrl[:] = ctrl
        mujoco.mj_step(self.m, self.d)

    # -- the arm lock -------------------------------------------------------------------------
    def solve_posture(self, pelvis, seed):
        """Leg IK, then the ARM JOINTS ARE OVERWRITTEN with the locked carry pose.

        Without this the arm entries of the target are the measured arm angles, so the arm PD holds
        the arms wherever they already are -- a controller whose setpoint is its own measurement,
        which can never correct anything. Measured consequence: the right ring fingertip sits under
        a millimetre from the right hip at the home pose, so any leg motion closes the gap and the
        humanoid's own finger is the deepest self-contact for the whole run.
        """
        q, resid, marg = super().solve_posture(pelvis, seed)
        if self.carry_pose is not None:
            for name, angle in self.carry_pose.items():
                jid = self.m.joint('h_' + name).id
                if jid >= 0:
                    q[int(self.m.jnt_qposadr[jid])] = float(angle)
        return q, resid, marg

    def sample(self, *a, **k):
        s = super().sample(*a, **k)
        s['ankle_Nm'] = {side: round(v, 4) for side, v in self.ankle_Nm.items()}
        s['ff_abs_max_Nm'] = round(max((abs(v) for v in self.ff_used.values()), default=0.0), 4)
        return s

    # -- the instrument checks its own convention ---------------------------------------------
    def check_instrument(self):
        """Two checks on the projection itself, both able to fail.

        `free_base` -- the six floating-base rows of `sum J^T f` must equal the total contact
        wrench; measured with the MEASURED forces, so it validates the frame and the sign against
        Newton rather than against `qfrc_constraint`.
        `echo` -- `qfrc_bias - qfrc_constraint` must reproduce the torque already applied, over the
        humanoid's own actuated dofs. This is what makes the proposal's first step a no-op, so it is
        measured every run rather than asserted once.
        """
        m = self.m
        rows = self._foot_contact_rows()
        weight = float(m.body_subtreemass[m.body(PB.BASE).id]) * abs(float(m.opt.gravity[2]))
        jacp = np.zeros((3, m.nv))
        jacr = np.zeros((3, m.nv))
        raw = np.zeros(6)
        tau = np.zeros(m.nv)
        for _side, i, body, pos in rows:
            mujoco.mj_contactForce(m, self.d, i, raw)
            frame = np.array(self.d.contact.frame[i], dtype=float).reshape(3, 3)
            f_world = frame.T @ np.array(raw[:3], dtype=float)
            jacp[:] = 0.0
            jacr[:] = 0.0
            mujoco.mj_jac(m, self.d, jacp, jacr, pos, body)
            tau += jacp.T @ f_world
        # a SCRATCH forward, with qvel zeroed and the current ctrl: the echo test is about a
        # static equilibrium, and the live data carries whatever the last step left in it
        sc = mujoco.MjData(m)
        sc.qpos[:] = self.d.qpos
        sc.qvel[:] = 0.0
        sc.ctrl[:] = self.d.ctrl
        mujoco.mj_forward(m, sc)
        bias = np.array(sc.qfrc_bias, dtype=float)
        cons = np.array(sc.qfrc_constraint, dtype=float)
        self.d.ctrl[:] = sc.ctrl
        applied = np.zeros(m.nv)
        dofs = []
        for name in self.names:
            jid = m.joint('h_' + name).id
            if jid < 0:
                continue
            dof = int(m.jnt_dofadr[jid])
            dofs.append(dof)
            for a in self.index.get(int(jid), ()):
                applied[dof] += float(self.d.ctrl[a])
        resid = [abs((bias[d] - cons[d]) - applied[d]) for d in sorted(set(dofs))]
        self.instrument = {
            'free_base_vertical_N': float(tau[2]),
            'weight_N': float(weight),
            'free_base_vs_weight': float(tau[2] / weight),
            'echo_max_Nm': float(max(resid)),
            'echo_mean_Nm': float(np.mean(resid)),
            'foot_contacts': len(rows),
        }
        return self.instrument


def carry_pose_for(rig):
    """The nominal tray-carrying pose, from the reach study's own IK on the tray's grip interface.

    Derived, not typed: the two handle geoms define the interface, and the arm joint angles that put
    each hand on its handle are what the locked pose holds.
    """
    import probe_p4_reach as PR  # noqa: E402
    r = PR.Rig(rig.m, 'c_payload')
    r.reset()
    r.settle()
    grip = r.grip_points()
    _worst, pose, per, _q = PR.solve_arms(r, grip, np.array(r.d.qpos))
    return {k: float(v) for k, v in pose.items()}, per


def judge(run, depth, thresholds):
    tr = run['trace']
    hold = [s for s in tr if s['phase'] == 'hold']
    end = tr[-1]
    ratios = [s['load_over_weight'] for s in hold]
    self_all = [s['self_mm'] for s in tr]
    self_hold = [s['self_mm'] for s in hold]
    rows = {
        'arrives': abs(end['drop_m'] - depth) <= thresholds['crouch_tol_m'],
        'sole_flat': max(max(s['foot_pitch_deg'].values()) for s in tr)
        <= thresholds['foot_flat_max_deg'],
        'feet_loaded': min(min(s['load_N']['L'], s['load_N']['R']) / PB.WEIGHT_N
                           for s in hold) >= thresholds['min_foot_share'],
        'com_inside': min(min(s['com_margin_m']) for s in tr) >= thresholds['min_com_margin_m'],
        'com_tracks': max(abs(s['com_err_m']) for s in hold) <= thresholds['com_track_tol_m'],
        'settled': end['leg_vel'] <= thresholds['settled_vel_rad_s'],
        'torque_in_range': max(s['torque_usage'] for s in hold) <= 0.95,
        'load_adds_up': (min(ratios) >= thresholds['load_sum_band'][0]
                         and max(ratios) <= thresholds['load_sum_band'][1]),
        # D078: judge the SETTLED state, and require a transient to RESOLVE. A transient that
        # self-resolves is not a defect, and taking the worst value over the whole run (what this
        # row did before) is a stricter rule than the project's own.
        'no_self_contact': min(self_hold) >= PB.SELF_ALLOWANCE_MM,
        'ik_converged': run['rejects'] == 0,
    }
    worst_pitch = max(max(s['foot_pitch_deg'].values()) for s in tr)
    return {'dz_m': depth, 'ff_mode': run['ff_mode'], 'arms': run['arms'],
            'balance_on': run['balance_on'],
            'drop_m': end['drop_m'], 'drop_err_m': end['drop_m'] - depth,
            'cmd_err_m': end['cmd_err_m'], 'correction_m': end['correction_m'],
            'ik_rejects': run['rejects'],
            'worst_foot_pitch_deg': worst_pitch,
            'min_com_margin_m': min(min(s['com_margin_m']) for s in tr),
            'peak_torque_usage': max(s['torque_usage'] for s in hold),
            'leg_vel_end': end['leg_vel'],
            'worst_self_mm': min(self_all), 'worst_self_hold_mm': min(self_hold),
            'self_transient_resolves': min(self_hold) >= PB.SELF_ALLOWANCE_MM,
            'ankle_Nm_end': end['ankle_Nm'],
            'ankle_usage_end': max(abs(v) / 160.0 for v in end['ankle_Nm'].values()),
            'ff_abs_max_Nm': max(s['ff_abs_max_Nm'] for s in hold),
            'ff_clamp_hits': run['ff_clamp_hits'], 'ff_peak_fraction': run['ff_peak_fraction'],
            'instrument': run['instrument'],
            # ★ REAL BOOLEANS. Without the cast, `json.dumps(default=str)` turns numpy bools into
            # the STRINGS "True"/"False", and the string "False" is truthy -- so a reader that does
            # `if row['verdict']:` would read a FAIL as a PASS. The verdicts are cast here so the
            # evidence cannot be misread by a machine.
            'rows': {k: bool(v) for k, v in rows.items()}, 'holds': bool(all(rows.values()))}


def do_run(rig, depth, *, ff_mode, arms, kp_bal, kd_bal, ramp_s=2.0, hold_s=1.5,
           balance_on=True):
    rig.ff_mode = ff_mode
    rig.balance_on = balance_on
    rig.ff_clamp_hits, rig.ff_peak_fraction = 0, 0.0
    # `none` is the PRE-FIX behaviour and the only true control: with no locked pose the arm
    # entries of the IK target are the MEASURED angles, so the arm PD's setpoint is its own
    # measurement and it can never correct anything.
    rig.carry_pose = {'carry': rig.pose_locked, 'home': rig.pose_home, 'none': None}[arms]
    out = C2.run(rig, depth, C2.dx_for(depth), correct=True, kp_bal=kp_bal, kd_bal=kd_bal,
                 ramp_s=ramp_s, hold_s=hold_s)
    out['ff_mode'], out['arms'] = ff_mode, arms
    out['balance_on'] = balance_on
    out['ff_clamp_hits'], out['ff_peak_fraction'] = rig.ff_clamp_hits, rig.ff_peak_fraction
    out['instrument'] = dict(rig.instrument)
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--run-id', default='p4-ff2-01')
    ap.add_argument('--reports-dir', default=None)
    args = ap.parse_args()
    out = (Path(args.reports_dir) if args.reports_dir else ROOT / 'reports') / args.run_id
    out.mkdir(parents=True, exist_ok=True)

    ref = json.loads((ROOT / 'reports' / 'p4-balance-04' / 'report.json').read_text(encoding='utf-8'))
    kp_bal, kd_bal = ref['controller']['kp_bal'], ref['controller']['kd_bal']

    rig = FFRig()
    rig.capture_standing()
    pose, per_arm = carry_pose_for(rig)
    rig.pose_locked = pose
    import probe_p4_reach as PR  # noqa: E402
    rig.pose_home = {n: float(rig.target[k]) for k, n in enumerate(rig.names)
                     if n in PR.ARM_JOINTS}
    print(f'nominal tray-carry pose: {len(pose)} arm joints, worst IK residual '
          f'{max(v["residual_m"] for v in per_arm.values()) * 1000:.4f} mm')
    print(f'declared home pose: {len(rig.pose_home)} arm joints (the falsifiability arm for the '
          f'self-contact row)')

    rig.ff_mode = 'measured'
    inst = rig.check_instrument()
    print(f'instrument: {inst["foot_contacts"]} foot-vs-world contacts; measured-force projection '
          f'free-base vertical {inst["free_base_vertical_N"]:.3f} N vs weight '
          f'{inst["weight_N"]:.3f} N ({inst["free_base_vs_weight"] * 100:.2f} %); '
          f'|bias-cons - applied| mean {inst["echo_mean_Nm"]:.4f} worst {inst["echo_max_Nm"]:.4f} N.m')

    thresholds = {'crouch_tol_m': PB.CROUCH_TOL_M, 'foot_flat_max_deg': PB.FOOT_FLAT_MAX_DEG,
                  'min_foot_share': PB.MIN_FOOT_SHARE, 'min_com_margin_m': PB.MIN_COM_MARGIN_M,
                  'com_track_tol_m': PB.COM_TRACK_TOL_M, 'settled_vel_rad_s': PB.SETTLED_VEL_RAD_S,
                  'load_sum_band': list(PB.LOAD_SUM_BAND)}

    #: The A/B arms. `desired` ADDS the contact torque on top of the ankle CoM term; `replace`
    #: swaps the CoM term out for it, which is the only configuration in which the contact
    #: feed-forward is the mechanism rather than a second one fighting the first.
    ARMS = (('off', True), ('desired', True), ('desired', False), ('measured', True))
    ab, sweep = [], []
    for depth in (0.20, 0.40):
        for ff_mode, bal in ARMS:
            r = do_run(rig, depth, ff_mode=ff_mode, arms='carry', kp_bal=kp_bal, kd_bal=kd_bal,
                       balance_on=bal)
            e = judge(r, depth, thresholds)
            ab.append(e)
            print(f'  A/B {depth:.2f} ff={ff_mode:>8} bal={str(bal):>5} arms=carry  '
                  f'arr {e["drop_err_m"] * 1000:+7.1f}mm pitch {e["worst_foot_pitch_deg"]:6.2f}  '
                  f'ank {e["ankle_usage_end"] * 100:5.1f}% ({e["ankle_Nm_end"]})  '
                  f'ffmax {e["ff_abs_max_Nm"]:7.1f}  '
                  f'failed={[k for k, v in e["rows"].items() if not v] or "none"}')
    # SELECT BY A STATED RULE, not by taste: the arm that passes the most rows at the shallower
    # depth, ties broken by the smaller foot pitch. The sweep then uses exactly that arm.
    shallow = [e for e in ab if abs(e['dz_m'] - 0.20) < 1e-9 and e['arms'] == 'carry']
    best = max(shallow, key=lambda e: (sum(e['rows'].values()), -e['worst_foot_pitch_deg']))
    print(f'\n  selected arm: ff={best["ff_mode"]} bal={best["balance_on"]} '
          f'({sum(best["rows"].values())}/10 rows, pitch {best["worst_foot_pitch_deg"]:.2f} deg)')
    # ★ A DERIVED CHECK FOR DEFECT 1. Two arms that differ ONLY in `balance_on` must not produce
    # the same numbers; if they do, the switch is not wired.
    dead_switch = []
    for a in shallow:
        for b in shallow:
            if a is b or a['ff_mode'] != b['ff_mode'] or a['balance_on'] == b['balance_on']:
                continue
            if (a['drop_err_m'], a['worst_foot_pitch_deg'], a['peak_torque_usage']) == \
                    (b['drop_err_m'], b['worst_foot_pitch_deg'], b['peak_torque_usage']):
                dead_switch.append(f"{a['ff_mode']}: bal={a['balance_on']} and "
                                   f"bal={b['balance_on']} are bit-identical")
    if dead_switch:
        print('  ⚠ DEAD SWITCH SUSPECTED: ' + '; '.join(dead_switch))

    # THE CONTROL: the same feed-forward and balance setting, with the arms left to the pre-fix
    # zero-setpoint law. This is what says whether the arm lock is doing any work.
    for depth in (0.20, 0.40):
        r = do_run(rig, depth, ff_mode=best['ff_mode'], arms='none', kp_bal=kp_bal,
                   kd_bal=kd_bal, balance_on=best['balance_on'])
        e = judge(r, depth, thresholds)
        ab.append(e)
        print(f'  CONTROL {depth:.2f} ff={best["ff_mode"]} arms=none (pre-fix arm law)  '
              f'arr {e["drop_err_m"] * 1000:+7.1f}mm self worst {e["worst_self_mm"]:.3f} mm '
              f'(settled {e["worst_self_hold_mm"]:.3f}) -> '
              f'no_self_contact={e["rows"]["no_self_contact"]}')

    for depth in C2.DEPTHS:
        r = do_run(rig, depth, ff_mode=best['ff_mode'], arms='carry', kp_bal=kp_bal, kd_bal=kd_bal,
                   balance_on=best['balance_on'])
        e = judge(r, depth, thresholds)
        sweep.append(e)
        print(f'  {depth:.2f} holds={str(e["holds"]):>5} arr {e["drop_err_m"] * 1000:+7.1f}mm '
              f'corr {e["correction_m"] * 1000:+6.1f}mm pitch {e["worst_foot_pitch_deg"]:6.2f} '
              f'ank {e["ankle_Nm_end"]} torque {e["peak_torque_usage"] * 100:5.1f}% '
              f'rej {e["ik_rejects"]} failed={[k for k, v in e["rows"].items() if not v]}')
    held = [e['dz_m'] for e in sweep if e['holds']]
    print(f'\n*** D_MAX with the contact feed-forward = {max(held) if held else None} m '
          f'(was 0.20 m)')

    report = {'run_id': args.run_id,
              'scope': 'explicit contact feed-forward judged against the plan that proposed it',
              'instrument': inst,
              'carry_pose': {k: round(v, 6) for k, v in pose.items()},
              'carry_pose_residual_m': {k: v['residual_m'] for k, v in per_arm.items()},
              'controller': {'k_task': C2.K_TASK, 'task_limit_m': C2.TASK_LIMIT_M,
                             'kp_bal': kp_bal, 'kd_bal': kd_bal,
                             'ff_modes': ['off', 'desired', 'measured'],
                             'ff_clamp': 'per-joint, to its own actuator ctrlrange, hits counted'},
              'world': {'path': 'assets/world_p4_handover.xml',
                        'sha256': hashlib.sha256(
                            (ROOT / 'assets' / 'world_p4_handover.xml').read_bytes()).hexdigest()},
              'thresholds': thresholds, 'ab': ab, 'sweep': sweep,
              'dead_switch_suspects': dead_switch,
              'selected_arm': {'ff_mode': best['ff_mode'], 'balance_on': best['balance_on'],
                               'rule': 'most rows passed at 0.20 m, ties by smaller foot pitch'},
              'deepest_holding_depth_m': (max(held) if held else None),
              'previous_deepest_holding_depth_m': 0.20}
    (out / 'report.json').write_text(json.dumps(report, indent=2, default=str) + '\n',
                                     encoding='utf-8')
    lines = [f'# {args.run_id} — contact feed-forward, non-circular', '',
             f"**deepest holding depth {max(held) if held else None} m** (was 0.20 m)", '',
             '## the instrument validates its own convention', '',
             f"- feet-vs-world contacts used: **{inst['foot_contacts']}**",
             f"- measured-force projection, free-base vertical "
             f"**{inst['free_base_vertical_N']:.3f} N** against a humanoid weight of "
             f"**{inst['weight_N']:.3f} N** ({inst['free_base_vs_weight'] * 100:.2f} %)",
             f"- `|(qfrc_bias - qfrc_constraint) - applied|` over the humanoid's actuated dofs: "
             f"mean **{inst['echo_mean_Nm']:.4f}**, worst **{inst['echo_max_Nm']:.4f} N·m** "
             f"— i.e. the projection of the MEASURED force IS the current output", '',
             '## A/B', '',
             '| dz | ff | bal | arms | arrival err | foot pitch | ankle end (N·m) | ankle % | '
             'ff max (N·m) | failed |',
             '|---|---|---|---|---|---|---|---|---|---|']
    for e in ab:
        lines.append(f"| {e['dz_m']:.2f} | {e['ff_mode']} | {e['balance_on']} | {e['arms']} | "
                     f"{e['drop_err_m'] * 1000:+.1f} mm | {e['worst_foot_pitch_deg']:.2f}° | "
                     f"{max(abs(v) for v in e['ankle_Nm_end'].values()):.2f} | "
                     f"{e['ankle_usage_end'] * 100:.1f}% | {e['ff_abs_max_Nm']:.1f} | "
                     f"{', '.join(k for k, v in e['rows'].items() if not v) or '—'} |")
    lines += ['', '## depth sweep, ff=desired, arms locked in the carry pose', '',
              '| dz | holds | arrival err | correction | pitch | torque | IK rej | failed |',
              '|---|---|---|---|---|---|---|---|']
    for e in sweep:
        lines.append(f"| {e['dz_m']:.2f} | {e['holds']} | {e['drop_err_m'] * 1000:+.1f} mm | "
                     f"{e['correction_m'] * 1000:+.1f} mm | {e['worst_foot_pitch_deg']:.2f}° | "
                     f"{e['peak_torque_usage'] * 100:.0f}% | {e['ik_rejects']} | "
                     f"{', '.join(k for k, v in e['rows'].items() if not v) or '—'} |")
    (out / 'acceptance.md').write_text('\n'.join(lines) + '\n', encoding='utf-8')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
