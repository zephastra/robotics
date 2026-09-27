"""P4-HUMAN-01 step 1: can the humanoid's arms grasp the tray's grip interface and place it
where the handover needs it?

WHAT THIS MEASURES, AND WHY EACH ROW IS BUILT THE WAY IT IS
-----------------------------------------------------------
The task says: measure the arm's workspace envelope and torque margin at the source station,
against the tray's grip interface, and confirm the real grasp and place poses do not interfere.
So the probe answers four questions and refuses to answer them with adjectives:

 1. GRASP. Where are the tray's two grip points, and can the hands be put on them? A
    damped-least-squares IK solve on the MERGED model, and the residual is the number.
    ★ The grip interface is DERIVED, not typed: the handle geoms are found by name on the tray's
    own body and their world positions are read from the compiled model, so the row follows the
    tray instead of assuming where it is.
 2. ENVELOPE. The question the handover needs is one-dimensional -- "how far along the placement
    axis can the humanoid put this tray down?" -- so the envelope is a SWEEP of the tray's
    placement along +x, both arms solved at every sample, warm-started from the previous one. A
    general 5-DOF workspace volume would cost far more and answer a question nobody asked.
 3. TORQUE. The required holding torque is MEASURED (`qfrc_bias` at the arm joints while the tray
    is actually grasped and carried), not computed from a hand-typed mass, and it is compared
    against the actuator's own declared command range.
    ★ The tray is 0.098 kg against a 160 N.m actuator, so "the torque is within limits" is a row
    that can barely fail on its own. Two things keep it honest: the margin is reported per joint,
    and a CONTROL ARM re-runs the identical procedure with a heavier tray. A torque probe that
    reads the same number for a 200x payload is measuring something other than the payload.
 4. INTERFERENCE. Contacts at the grasp pose, CLASSIFIED rather than counted: a finger on a
    handle IS the grasp; anything else of the humanoid touching the tray is a defect; and a
    hand-internal contact (a closed fist) is neither.

WHY `--world` IS A PARAMETER
----------------------------
The same probe is run against the un-migrated world and against the P4 world. In the un-migrated
world the conveyor's source band is metres from the humanoid's station, so these rows FAIL: the
probe can fail, and it fails on the world that is not ready. That is the A/B this file exists to
make possible -- a criterion that has only ever returned PASS on the world it was written for is
not evidence about anything.

★ This probe WRITES qpos (see `place_tray`): moving the tray is how the envelope is probed. It is
a measurement, not a handover. `probe_p4_handover.py` counts its own qpos writes to prove that the
run never teleports anything.
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
import roller_rig as roller_rig  # noqa: E402  -- C's crown plane, the single source of truth

LEFT_ARM = ('J13_SHOULDER_PITCH_L', 'J14_SHOULDER_ROLL_L', 'J15_SHOULDER_YAW_L',
            'J16_ELBOW_PITCH_L', 'J17_ELBOW_YAW_L')
RIGHT_ARM = ('J18_SHOULDER_PITCH_R', 'J19_SHOULDER_ROLL_R', 'J20_SHOULDER_YAW_R',
             'J21_ELBOW_PITCH_R', 'J22_ELBOW_YAW_R')
ARM_JOINTS = LEFT_ARM + RIGHT_ARM
FINGER_STEMS = ('ff', 'mf', 'rf', 'th')

#: How close a hand must get to a grip point to count as "on it". The interface is a 35 mm ball,
#: so 2 mm is under a tenth of its radius -- inside the finger pads' own compliance.
REACH_TOL_M = 0.002
#: The sweep's step and extent. 20 mm is well under the 80 mm roller pitch, so the farthest
#: placement is not an artefact of where the samples happened to land.
SWEEP_STEP_M = 0.020
SWEEP_MAX_M = 1.20
#: A settled self-contact this shallow is a solver contact, not a pose defect. The same allowance
#: `probe_h_selfclear` uses, because it is the same question asked at a different pose.
SELF_ALLOWANCE_MM = 0.5
#: The hold torque must stay inside this fraction of the actuator's declared command range.
TORQUE_USAGE_MAX = 0.80
#: The control arm's tray-mass multiplier. Large on purpose: it exists to show the torque row is
#: measuring the payload, and a row that cannot be made to fail is not a measurement.
HEAVY_TRAY_FACTOR = 20.0
#: Fingers close to a fraction of their own range; the fraction is SEARCHED rather than chosen,
#: because how far they must curl to take a 35 mm handle ball is a property of the hand and the
#: ball, not of anything a reader can check by eye. Measured on this model: everything up to 0.50
#: leaves the tray on its support and only 0.85 carries it.
CLOSE_CANDIDATES = (0.05, 0.08, 0.12, 0.18, 0.25, 0.35, 0.50, 0.85)
#: The arm PD: kp is `actuator_range / GAIN_SATURATION_ERR_RAD` PER JOINT, so every joint reaches
#: saturation at the same error and no joint is asked for more than its own actuator can give.
#: kd is the critically damped partner of that kp against `ARM_INERTIA_KG_M2`, the distal inertia
#: of one arm segment -- the arm's real inertia is not published in the model, so this is a
#: declared control choice with its effect measured in the diagnostic, not a derived quantity.
GAIN_SATURATION_ERR_RAD = 0.5
ARM_INERTIA_KG_M2 = 0.02
#: How far the hand must carry the tray for the grasp to count as a hold rather than a nudge.
GRASP_LIFT_M = 0.005


def load(world):
    bp3.install()
    return mujoco.MjModel.from_xml_path(str(world))


class Rig:
    """Everything the measurements need, derived from one compiled model."""

    def __init__(self, model, tray_name):
        self.m = model
        self.d = mujoco.MjData(model)
        self.tray = model.body(tray_name).id
        if self.tray < 0:
            raise RuntimeError(f'the world has no body {tray_name!r}')
        self.tray_name = tray_name
        self.home = np.asarray(mw.merged_home(model)[0], dtype=float)
        self.limits = {n: (float(model.jnt_range[self.jid(n)][0]),
                           float(model.jnt_range[self.jid(n)][1])) for n in ARM_JOINTS}
        self.actuator = {n: int(mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_ACTUATOR, 'h_motor_' + n)) for n in ARM_JOINTS}
        if any(a < 0 for a in self.actuator.values()):
            raise RuntimeError('the humanoid has no arm actuators in this world')
        self.arm_span = {n: abs(float(model.actuator_ctrlrange[self.actuator[n]][1]))
                         for n in ARM_JOINTS}
        #: PER-JOINT gains, derived from each joint's OWN actuator range.
        #:
        #: ★ The first version gave every arm joint the same kp, taken from the shoulder's 160 N.m,
        #: and kd = 2*sqrt(kp*1 kg.m^2) = 25.3. The elbow-yaw actuators are rated 52 N.m, so any
        #: error past 0.33 rad SATURATED them; and a 25 N.m/(rad/s) damping term against a
        #: saturated 52 N.m actuator turns the loop into a bang-bang controller that limit-cycles.
        #: Measured: J17/J22 sat at 30-34 deg of steady error with 6 rad/s of velocity while every
        #: other arm joint tracked to 0.1 mrad, and the recorded demand was 264 N.m -- 508 % of the
        #: range -- for a 98 g tray that the open hand was not even touching. The gain has to come
        #: from the joint's own actuator or the weak joint is asked for what the strong joint can
        #: give.
        self.kp = {n: self.arm_span[n] / GAIN_SATURATION_ERR_RAD for n in ARM_JOINTS}
        self.kd = {n: 2.0 * float(np.sqrt(self.kp[n] * ARM_INERTIA_KG_M2)) for n in ARM_JOINTS}
        #: The torque the last arm-control tick had to ASK FOR, per joint, before clamping.
        self.arm_demand = {}
        self.start_origin = np.zeros(3)
        self.reset()

    # -- addressing -------------------------------------------------------------------------
    def jid(self, name):
        i = self.m.joint('h_' + name).id
        if i < 0:
            raise RuntimeError(f'the world has no joint h_{name}')
        return i

    def qadr(self, name):
        return int(self.m.jnt_qposadr[self.jid(name)])

    def dofadr(self, name):
        return int(self.m.jnt_dofadr[self.jid(name)])

    def reset(self):
        self.d.qpos[:] = self.home
        self.d.qvel[:] = 0.0
        mujoco.mj_forward(self.m, self.d)

    # -- the arm law this world does not give the humanoid ----------------------------------
    def _stand_ctrl(self):
        """The merged world's standing law, exactly as `w4_plant` uses it.

        Every actuator it does not own is commanded to ZERO, and the arm actuators are among
        those -- so without the layer below, the arms hang limp and every number here would be a
        measurement of a flopping arm rather than of the robot holding a tray.
        """
        ctrl = np.asarray(mw.home_hold_ctrl(self.m, self.home, self.d.qpos, self.d.qvel),
                          dtype=float).reshape(-1)
        names, target, kp, kd = mw.stand_law()
        index = mw._stand_actuator_index(self.m)
        for k, joint_name in enumerate(names):
            j = self.m.joint('h_' + joint_name).id
            if j < 0:
                continue
            adr, dof = int(self.m.jnt_qposadr[j]), int(self.m.jnt_dofadr[j])
            torque = ((float(target[k]) - float(self.d.qpos[adr])) * float(kp[k])
                      - float(self.d.qvel[dof]) * float(kd[k]))
            for a in index.get(int(j), ()):
                ctrl[a] = torque
        return ctrl

    def _arm_ctrl(self, ctrl, arm_target):
        """PD + gravity compensation, and the UNCLAMPED demand is recorded for the torque rows.

        ★ `qfrc_bias` was the first instrument and it was measuring the WRONG OBJECT: the tray is
        a separate free body, so its weight reaches the arm through CONTACT forces and never
        appears in the humanoid's bias torque. Measured: a 200x tray moved the bias reading from
        1.947 % to 1.764 % of the actuator range -- it went DOWN. The torque the joint must supply
        is the command the controller has to ask for, so that is what is recorded, before the
        clamp, so that "the demand exceeded the range" is visible rather than silently clamped.
        """
        for n, want in arm_target.items():
            dof = self.dofadr(n)
            tau = (self.kp[n] * (float(want) - float(self.d.qpos[self.qadr(n)]))
                   - self.kd[n] * float(self.d.qvel[dof]) + float(self.d.qfrc_bias[dof]))
            self.arm_demand[n] = abs(float(tau))
            a = self.actuator[n]
            lo, hi = self.m.actuator_ctrlrange[a]
            ctrl[a] = float(min(max(tau, lo), hi))
        return ctrl

    def _hand_ctrl(self, ctrl, close):
        for side in ('lh', 'rh'):
            for finger in FINGER_STEMS:
                for k in range(4):
                    a = mujoco.mj_name2id(self.m, mujoco.mjtObj.mjOBJ_ACTUATOR,
                                          f'h_{side}_{finger}a{k}')
                    if a < 0:
                        continue
                    lo, hi = (float(v) for v in self.m.actuator_ctrlrange[a])
                    span = hi - lo
                    # A range symmetric about zero is an abduction joint (neutral = 0); an
                    # asymmetric one is a flexion joint (closing = near its upper end).
                    ctrl[a] = 0.0 if abs(lo + hi) < 0.05 * span else lo + close * span
        return ctrl

    def step(self, *, arm_target=None, hand_close=None, steps=1):
        for _ in range(steps):
            ctrl = self._stand_ctrl()
            if arm_target is not None:
                ctrl = self._arm_ctrl(ctrl, arm_target)
            if hand_close is not None:
                ctrl = self._hand_ctrl(ctrl, hand_close)
            self.d.ctrl[:] = ctrl
            mujoco.mj_step(self.m, self.d)

    def settle(self, seconds=0.5):
        """Stand first, arms held where the keyframe puts them. Measurements happen from here."""
        keep = {n: float(self.home[self.qadr(n)]) for n in ARM_JOINTS}
        self.step(arm_target=keep, steps=int(seconds / self.m.opt.timestep))

    # -- geometry ---------------------------------------------------------------------------
    def site(self, name):
        sid = mujoco.mj_name2id(self.m, mujoco.mjtObj.mjOBJ_SITE, name)
        if sid < 0:
            raise RuntimeError(f'the world has no site {name}')
        return np.array(self.d.site_xpos[sid])

    def grip_points(self):
        """The tray's own grip interface, keyed by the SIGN OF y -- which is what decides which
        hand goes to which handle. The merged model is not obliged to keep the source's order."""
        out = {}
        for g in range(self.m.ngeom):
            if int(self.m.geom_bodyid[g]) != self.tray:
                continue
            name = mujoco.mj_id2name(self.m, mujoco.mjtObj.mjOBJ_GEOM, g) or ''
            if 'handle' in name and 'stem' not in name:
                p = np.array(self.d.geom_xpos[g])
                out['+y' if p[1] > 0 else '-y'] = {'name': name, 'point': p}
        if len(out) != 2:
            raise RuntimeError(f'the tray has {len(out)} handle geoms; the interface is two')
        return out

    def source_table(self):
        """The humanoid's presentation table, measured against the base.

        Returns (offset_x, half_x) of the table's top geom. Derived, because the tray has to be
        lowered CLEAR of this table and that clearance is a hard constraint on where the station
        can go -- the row that combines it with the arm's reach is what makes the station a single
        number instead of a preference.
        """
        base_x = float(self.d.xpos[self.m.body('h_LINK_BASE').id][0])
        for g in range(self.m.ngeom):
            name = mujoco.mj_id2name(self.m, mujoco.mjtObj.mjOBJ_GEOM, g) or ''
            if 'source_table' in name:
                return (float(self.d.geom_xpos[g][0]) - base_x,
                        float(self.m.geom_size[g][0]))
        raise RuntimeError('this world has no humanoid presentation table')

    def band(self, prefix='c_fixed_roller'):
        return sorted(float(self.d.geom_xpos[g][0]) for g in range(self.m.ngeom)
                      if (mujoco.mj_id2name(self.m, mujoco.mjtObj.mjOBJ_GEOM, g) or ''
                          ).startswith(prefix))

    def body_name(self, geom):
        return mujoco.mj_id2name(self.m, mujoco.mjtObj.mjOBJ_BODY,
                                 int(self.m.geom_bodyid[geom])) or ''


def ik(rig, site_name, target, chain, seed, *, iters=400, lam=0.02, step_cap=0.15):
    """Damped least squares to a 3-D position, KEEPING THE BEST POSE IT SAW.

    Three robustness fixes over the first version, each of which was a measured failure:
      * warm start (`seed`) -- solving every sweep sample from the home pose makes far samples
        land in local minima whose residual is LARGER than the start distance, which reads as
        "unreachable" for a target the arm can plainly reach;
      * a per-iteration step cap -- one near-singular Jacobian otherwise throws the whole chain
        to its joint limits in a single step;
      * best-tracking -- the pose returned is the best one seen, not the last one, so a result can
        never be worse than the seed for a target the seed was already near.
    One `mj_forward` per iteration: the same forward supplies both the task error and the
    Jacobian.

    ★ qpos addresses and DOF addresses are NOT interchangeable and the first version used one for
    the other: they differ by one for every joint after the humanoid's free base (7 qpos vs 6 dof)
    and by two after the tray's. Indexing `qpos` with a DOF address moves the WRONG joints, and
    the symptom was a residual equal to the initial distance at every target -- "the site never
    moved" wearing the costume of "the target is unreachable".
    """
    m, d = rig.m, rig.d
    sid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_SITE, site_name)
    qadrs = np.array([rig.qadr(n) for n in chain])
    dofs = np.array([rig.dofadr(n) for n in chain])
    lows = np.array([rig.limits[n][0] for n in chain])
    highs = np.array([rig.limits[n][1] for n in chain])
    q = np.array(seed)
    target = np.asarray(target, dtype=float)
    save = np.array(d.qpos)
    best_q, best = np.array(q), float('inf')
    jacp = np.zeros((3, m.nv))
    for _ in range(iters):
        d.qpos[:] = q
        mujoco.mj_forward(m, d)
        err = target - np.asarray(d.site_xpos[sid])
        r = float(np.linalg.norm(err))
        if r < best:
            best_q, best = np.array(q), r
        if r < 1e-6:
            break
        jacp[:] = 0.0
        mujoco.mj_jacSite(m, d, jacp, None, sid)
        J = jacp[:, dofs]
        dq = J.T @ np.linalg.solve(J @ J.T + (lam ** 2) * np.eye(3), err)
        biggest = float(np.max(np.abs(dq)))
        if biggest > step_cap:
            dq = dq * (step_cap / biggest)
        q[qadrs] = np.clip(q[qadrs] + dq, lows, highs)
    d.qpos[:] = save
    mujoco.mj_forward(m, d)
    return best, best_q


def place_tray(rig, offset):
    """Put the tray at its own start pose plus `offset`, IN QPOS.

    ★ This writes qpos, and in a measurement that is the right instrument: the sweep's whole job
    is to ask "if the tray were here, could the hands reach it?". It is NOT how the handover is
    done -- the run may not teleport anything, and `probe_p4_handover.py` counts the writes to
    show it did not.
    """
    jnt = int(rig.m.body_jntadr[rig.tray])
    adr = int(rig.m.jnt_qposadr[jnt])
    q = np.array(rig.d.qpos)
    q[adr + 0] = rig.start_origin[0] + float(offset[0])
    q[adr + 1] = rig.start_origin[1] + float(offset[1])
    rig.d.qpos[:] = q
    mujoco.mj_forward(rig.m, rig.d)


def solve_arms(rig, grip, seed, *, iters=400):
    """Both hands onto their grip points. Returns (worst_m, pose, per_arm, qpos)."""
    per_arm, worst, q = {}, 0.0, np.array(seed)
    for key, chain, site in (('+y', LEFT_ARM, 'h_lh_grasp'), ('-y', RIGHT_ARM, 'h_rh_grasp')):
        res, q = ik(rig, site, grip[key]['point'], chain, q, iters=iters)
        per_arm[key] = {'hand': site, 'handle': grip[key]['name'], 'residual_m': res}
        worst = max(worst, res)
    pose = {n: float(q[rig.qadr(n)]) for n in ARM_JOINTS}
    return worst, pose, per_arm, q


def sweep(rig, *, step=SWEEP_STEP_M, max_offset=SWEEP_MAX_M):
    """The horizontal reach from the tray's own pose: how far can the humanoid carry it?

    The humanoid is settled ONCE and the tray is moved under it, so every sample is measured from
    the same standing pose -- settling per sample would move the base by ~14 mm between samples
    and mix that into the envelope. The start pose is captured here rather than inherited, so this
    function is safe to call after anything else has moved the base.
    """
    rig.reset()
    rig.settle()
    rig.start_origin = np.array(rig.d.xpos[rig.tray], dtype=float)
    rows, q = [], None
    offset = 0.0
    while offset <= max_offset + 1e-9:
        place_tray(rig, (offset, 0.0))
        grip = rig.grip_points()
        seed = np.array(rig.d.qpos)
        if q is not None:
            for n in ARM_JOINTS:
                seed[rig.qadr(n)] = q[rig.qadr(n)]
        worst, pose, per, q = solve_arms(rig, grip, seed)
        rows.append({'offset_m': round(offset, 4), 'worst_residual_m': worst,
                     'per_arm_residual_m': {k: v['residual_m'] for k, v in per.items()}})
        offset = round(offset + step, 4)
    reachable = [r for r in rows if r['worst_residual_m'] <= REACH_TOL_M]
    farthest = max((r['offset_m'] for r in reachable), default=None)
    return rows, farthest


def tray_span_x(rig):
    """The tray's own x extent, from the compiled geometry. Nothing typed."""
    lo, hi = np.inf, -np.inf
    for g in range(rig.m.ngeom):
        if int(rig.m.geom_bodyid[g]) != rig.tray:
            continue
        centre, size = rig.d.geom_xpos[g], rig.m.geom_size[g]
        half = size[0] if rig.m.geom_type[g] == mujoco.mjtGeom.mjGEOM_BOX \
            else max(size[0], size[1], size[2])
        lo, hi = min(lo, float(centre[0] - half)), max(hi, float(centre[0] + half))
    return lo, hi


def deepest_contacts(rig):
    """Classify contacts at the current state. Nothing is judged here.

    ★ v1 classified ANY partner that was not a finger as "a humanoid body on the tray", so the
    tray resting on the conveyor's own source rollers (`c_fixed_roller_0_2 x c_payload`,
    -4.126 mm) failed the row. The row is about the HUMANOID: a support under the tray is the
    normal load path, and it is now excluded by scope rather than by loosening the threshold.
    """
    out = {'tray_vs_interface_mm': None, 'tray_vs_other_humanoid_mm': None,
           'tray_vs_other_pair': None, 'touched_tray_geoms': set(), 'hand_internal_mm': None,
           'body_self_mm': None, 'body_self_pair': None}
    for i in range(rig.d.ncon):
        g1, g2 = int(rig.d.contact.geom[i][0]), int(rig.d.contact.geom[i][1])
        b1, b2 = rig.body_name(g1), rig.body_name(g2)
        dist = float(rig.d.contact.dist[i]) * 1000.0
        b1_id, b2_id = int(rig.m.geom_bodyid[g1]), int(rig.m.geom_bodyid[g2])
        if rig.tray in (b1_id, b2_id):
            other = b2 if b1_id == rig.tray else b1
            if not other.startswith('h_'):
                continue          # the tray's own support is not this row's subject
            # ★ The question is NOT "which humanoid body touched the tray" but "WHICH PART OF THE
            # TRAY was touched". The first version judged the partner's name and failed the row on
            # `h_payload x h_rh_palm` at -1.277 mm -- a palm bracing against the handle ball, which
            # is the declared grip interface doing its job. The gripper's own name cannot tell a
            # grasp from a lean; the tray's geometry can, so that is what is read.
            tray_geom = mujoco.mj_id2name(
                rig.m, mujoco.mjtObj.mjOBJ_GEOM, g1 if b1_id == rig.tray else g2) or ''
            out['touched_tray_geoms'].add(tray_geom)
            if 'handle' in tray_geom:
                cur = out['tray_vs_interface_mm']
                out['tray_vs_interface_mm'] = dist if cur is None else min(cur, dist)
            else:
                cur = out['tray_vs_other_humanoid_mm']
                if cur is None or dist < cur:
                    out['tray_vs_other_humanoid_mm'] = dist
                    out['tray_vs_other_pair'] = f'{tray_geom} x {other}'
            continue
        if b1.startswith('h_') and b2.startswith('h_'):
            if b1[:5] == b2[:5] and b1[:5] in ('h_lh_', 'h_rh_'):
                cur = out['hand_internal_mm']
                out['hand_internal_mm'] = dist if cur is None else min(cur, dist)
            else:
                cur = out['body_self_mm']
                if cur is None or dist < cur:
                    out['body_self_mm'] = dist
                    out['body_self_pair'] = f'{b1} x {b2}'
    return out


def tray_bottom_local(rig):
    """The tray's lowest point in its OWN frame. Derived, so the crown height never gets typed."""
    body_z = float(rig.d.xpos[rig.tray][2])
    low = np.inf
    for g in range(rig.m.ngeom):
        if int(rig.m.geom_bodyid[g]) != rig.tray:
            continue
        centre, size = rig.d.geom_xpos[g], rig.m.geom_size[g]
        half = size[2] if rig.m.geom_type[g] == mujoco.mjtGeom.mjGEOM_BOX \
            else max(size[0], size[1], size[2])
        low = min(low, float(centre[2] - half))
    return low - body_z


def place_base(rig, dx, dz=0.0):
    """Move the humanoid's free base by (dx, 0, dz), IN QPOS.

    Another measurement-only write, and the same justification as `place_tray`: the question "how
    far must this station move, and how far down, before the band is inside the arm's envelope"
    cannot be answered from a station that is fixed in both. The base is a free joint, so its
    world pose IS qpos.

    ★ `dz` is the vertical part and it exists because the first version swept x only. The band's
    crown is 0.485 m and the humanoid's hands hang at 0.77 m, so the placement target is about
    0.25 m BELOW the standing reach: a station shifted correctly in x still fails, and sweeping x
    alone would have reported "no station works", which is true only of standing stations.
    """
    jnt = int(rig.m.body_jntadr[rig.m.body('h_LINK_BASE').id])
    adr = int(rig.m.jnt_qposadr[jnt])
    q = np.array(rig.home)
    q[adr + 0] += float(dx)
    q[adr + 2] += float(dz)
    rig.d.qpos[:] = q
    rig.d.qvel[:] = 0.0
    mujoco.mj_forward(rig.m, rig.d)


def station_sweep(rig, points, *, dx_step=0.05, dx_span=(0.0, 3.80),
                  dz_step=0.05, dz_span=(0.0, -0.45), iters=150):
    """Sweep the station in (x, z) and ask where BOTH hands reach the target.

    A grid rather than a line, for the reason in `place_base`. Warm-started along z, then along x,
    so a sample is judged on its own merits rather than on which local minimum it fell into.
    Returns the per-z reachable x window, which is the object the world design needs.
    """
    windows, cache = [], {}
    dz = float(dz_span[0])
    while dz >= float(dz_span[1]) - 1e-9:
        ok, dx, q = [], float(dx_span[0]), None
        while dx <= float(dx_span[1]) + 1e-9:
            place_base(rig, dx, dz)
            # ★ TWO seeds per sample. Warm-starting only from the previous sample makes the whole
            # sweep inheritable: if one sample lands in a local minimum, every later one starts
            # from it, and the reachable set comes out as a scattering of single points instead of
            # a region -- which is what the fine sweep first reported (one grid point out of 208,
            # with the two samples 20 mm either side failing). The keyframe's own hanging-arm pose
            # is a second, independent seed and is a good one for reaching downward.
            seeds = [np.array(rig.d.qpos)]
            if q is not None:
                carried = np.array(rig.d.qpos)
                for n in ARM_JOINTS:
                    carried[rig.qadr(n)] = q[rig.qadr(n)]
                seeds.append(carried)
            best = None
            for s in seeds:
                w, pose, per, qq = solve_arms(rig, points, s, iters=iters)
                if best is None or w < best[0]:
                    best = (w, pose, per, qq)
            worst, pose, per, q = best
            cache[(round(dx, 3), round(dz, 3))] = worst
            if worst <= REACH_TOL_M:
                ok.append(round(dx, 3))
            dx = round(dx + dx_step, 3)
        windows.append({'dz_m': round(dz, 3),
                        'reachable_dx_m': [min(ok), max(ok)] if ok else None,
                        'samples_ok': len(ok)})
        dz = round(dz - dz_step, 3)

    def best():
        """The shallowest crouch that works, and among those the nearest shift."""
        for row in windows:
            if row['reachable_dx_m']:
                return row['dz_m'], row['reachable_dx_m'][0], row['reachable_dx_m'][1]
        return None, None, None

    return windows, best(), cache


def grasp_and_hold(rig, *, close, tray_factor=1.0, seconds=1.5):
    """Grasp the tray at the solved posture, carry it, and MEASURE the torque the arms ask for.

    `tray_factor` scales the tray's MASS, which is how the control arm shows this row responds to
    the payload. Mass alone is scaled: gravity load is what is being measured, and at rest the
    inertia tensor contributes nothing.
    """
    original = float(rig.m.body_mass[rig.tray])
    rig.m.body_mass[rig.tray] = original * tray_factor
    try:
        rig.reset()
        rig.settle()
        rig.start_origin = np.array(rig.d.xpos[rig.tray], dtype=float)
        grip = rig.grip_points()
        worst, pose, _per, q = solve_arms(rig, grip, np.array(rig.d.qpos))
        rig.d.qpos[:] = q
        rig.d.qvel[:] = 0.0
        mujoco.mj_forward(rig.m, rig.d)
        start = np.array(rig.d.xpos[rig.tray], dtype=float)
        steps = int(seconds / rig.m.opt.timestep)
        peak = {n: 0.0 for n in ARM_JOINTS}
        applied = {n: 0.0 for n in ARM_JOINTS}
        for step in range(steps):
            rig.step(arm_target=pose, hand_close=close, steps=1)
            if step >= steps // 2:      # the second half only: the first is the approach
                for n in ARM_JOINTS:
                    peak[n] = max(peak[n], rig.arm_demand.get(n, 0.0))
                    applied[n] = max(applied[n], abs(float(
                        rig.d.actuator_force[rig.actuator[n]])))
        end = np.array(rig.d.xpos[rig.tray], dtype=float)
        return {'grasp_residual_m': worst, 'demand_Nm': peak, 'applied_Nm': applied,
                'tray_moved_m': float(np.linalg.norm(end - start)),
                'tray_origin_start': [round(float(v), 5) for v in start],
                'tray_origin_end': [round(float(v), 5) for v in end],
                'contacts': deepest_contacts(rig)}
    finally:
        rig.m.body_mass[rig.tray] = original


def choose_close(rig, *, tray_factor=1.0, candidates=CLOSE_CANDIDATES, seconds=0.8):
    """The smallest finger closure that still CARRIES the tray. Measured, not chosen.

    ★ ATTRIBUTION CORRECTED. The first version of this docstring blamed the finger servos -- which
    have no force limit -- for the 512 % torque reading, and that was a guess written down as a
    cause. The diagnostic says otherwise: the closure fraction barely mattered, and the real cause
    was the ARM GAIN (one kp for every joint, taken from the shoulder's 160 N.m actuator, applied
    to elbow-yaw actuators rated 52 N.m -- see `GAIN_SATURATION_ERR_RAD`). The search is kept
    because it answers a different, genuine question, but the wrong explanation is not kept.
    """
    table, chosen, best_carried, last = [], None, None, None
    for frac in candidates:
        r = grasp_and_hold(rig, close=frac, tray_factor=tray_factor, seconds=seconds)
        usage = max(r['demand_Nm'][n] / rig.arm_span[n] for n in ARM_JOINTS)
        r['worst_usage'] = usage
        r['close_fraction'] = frac
        last = r
        table.append({'close_fraction': frac, 'tray_moved_m': r['tray_moved_m'],
                      'worst_usage': usage})
        if r['tray_moved_m'] > GRASP_LIFT_M:
            if best_carried is None or usage < best_carried['worst_usage']:
                best_carried = r
            if usage <= TORQUE_USAGE_MAX:
                chosen = r
                break
    # If nothing both carried the tray and stayed inside the range, report the weakest grip that
    # carried it at all (or the last trial), so the report says what happened rather than "no
    # result" -- a probe that returns nothing here would hide which failure it was.
    if chosen is None:
        chosen = best_carried if best_carried is not None else last
    return chosen.get('close_fraction'), chosen, table


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--run-id', default='p4-reach-01')
    ap.add_argument('--world', default='assets/world_w2_logistic.xml')
    ap.add_argument('--tray', default='c_payload')
    ap.add_argument('--reports-dir', default=None)
    ap.add_argument('--label', default=None)
    ap.add_argument('--dx-span', nargs=2, type=float, default=[0.0, 3.80],
                    help='station shift along +x to sweep, in metres')
    ap.add_argument('--dz-span', nargs=2, type=float, default=[0.0, -0.45],
                    help='station shift along z to sweep, in metres (negative = crouch)')
    ap.add_argument('--dx-step', type=float, default=0.05)
    ap.add_argument('--dz-step', type=float, default=0.05)
    args = ap.parse_args()
    out = (Path(args.reports_dir) if args.reports_dir else ROOT / 'reports') / args.run_id
    out.mkdir(parents=True, exist_ok=True)
    world = ROOT / args.world

    rig = Rig(load(world), args.tray)
    checks = []

    def check(group, name, verdict, detail, numbers=None):
        checks.append({'group': group, 'check': name, 'verdict': verdict, 'detail': detail,
                       'numbers': numbers or {}})

    # --- 1. grasp ---------------------------------------------------------------------------
    rig.reset()
    rig.settle()
    rig.start_origin = np.array(rig.d.xpos[rig.tray], dtype=float)
    grip = rig.grip_points()
    res, pose, per_arm, _q = solve_arms(rig, grip, np.array(rig.d.qpos))
    check('reach', 'both hands can be put on the tray\'s grip interface at its own pose',
          'PASS' if res <= REACH_TOL_M else 'FAIL',
          f'worst hand-to-grip-point residual {res * 1000:.3f} mm against a '
          f'{REACH_TOL_M * 1000:.1f} mm tolerance. Grip points, DERIVED from the handle geoms on '
          f'the tray\'s own body: '
          f'{ {k: [round(float(v), 4) for v in g["point"]] for k, g in grip.items()} }. '
          f'Per arm: ' + '; '.join(f'{k} {v["hand"]} -> {v["handle"]} '
                                   f'{v["residual_m"] * 1000:.3f} mm' for k, v in per_arm.items()),
          {'worst_residual_m': res})

    # --- 2. placement envelope --------------------------------------------------------------
    #
    # ★ v1 judged "is the band inside the reach" by comparing two numbers taken in DIFFERENT
    # frames: the band's offset from the tray's start pose, against a reach measured ONLY along
    # +x. In the un-migrated world the target is at a NEGATIVE offset (the tray already sits east
    # of the band's west end), so the sweep never visited it and the row reported "outside by
    # -0.2 m", which is a sentence that cannot be read. The criterion is now what it always meant:
    # PUT THE TRAY AT THE TARGET PLACEMENT and solve. The sweep stays as the envelope curve.
    band = rig.band()
    origin = rig.start_origin
    lo_x, hi_x = tray_span_x(rig)
    tray_half_x = (hi_x - lo_x) / 2.0
    bottom_local = tray_bottom_local(rig)
    target_x = band[0] + tray_half_x      # the tray's west edge flush with the band's first crown
    target_z = float(roller_rig.CROWN_Z) - bottom_local
    targets = {k: {'name': f'band-placement{k}',
                   'point': np.array([target_x, float(v['point'][1]), target_z])}
               for k, v in grip.items()}
    st_windows, best_station, st_cache = station_sweep(
        rig, targets, dx_step=args.dx_step, dx_span=tuple(args.dx_span),
        dz_step=args.dz_step, dz_span=tuple(args.dz_span))
    best_dz, best_dx, best_dx_hi = best_station
    at_current = st_cache.get((0.0, 0.0))
    check('envelope', 'a placement on the source band IS inside the arm\'s envelope',
          'PASS' if best_dz is not None else 'FAIL',
          f'the target is the tray resting on the band\'s west end: its grip points go to '
          f'x {target_x:.4f}, y +-0.300, z {target_z:.4f}, i.e. the crown plane '
          f'{float(roller_rig.CROWN_Z):.4f} plus the tray\'s own {(-bottom_local) * 1000:.2f} mm '
          f'from its origin to its underside. The station is swept in BOTH x and z in 50 mm steps, '
          f'and both hands reach the target from '
          + (f'a station at {best_dx:+.2f} m in x and {best_dz:+.2f} m in z, with the reachable '
             f'x window running to {best_dx_hi:+.2f} m at that height. '
             f'So the arm CAN do this job: the question is only where the station has to be.'
             if best_dz is not None else
             f'NO station in the swept grid. A target no station can reach fails this row, which is '
             f'what makes the station numbers a measurement rather than an assumption.'),
          {'target_x': target_x, 'target_z': target_z, 'reachable_from_m': best_dx,
           'reachable_to_m': best_dx_hi, 'dz_m': best_dz})
    check('envelope', 'the band placement is reachable AT THE STATION THIS WORLD HAS',
          'PASS' if at_current is not None and at_current <= REACH_TOL_M else 'FAIL',
          f'at its own station and standing (0.000 m in x, 0.000 m in z) the worst residual to the '
          f'band placement is {"not measured" if at_current is None else f"{at_current * 1000:.1f} mm"}'
          f' against a {REACH_TOL_M * 1000:.1f} mm tolerance. '
          + (f'The nearest station that works is {best_dx:+.2f} m in x and {best_dz:+.2f} m in z. '
             f'★ The z part is NOT a preference: the hands hang at z 0.77 m and the band holds the '
             f'tray\'s grips at z {target_z:.3f} m, so the placement is about '
             f'{abs(best_dz) * 1000:.0f} mm BELOW the standing envelope and no amount of walking '
             f'in x can fix that. The P4 world therefore has to declare a station shift AND a '
             f'crouch, and both numbers come from this sweep.'
             if at_current is None or at_current > REACH_TOL_M else
             'This world is already laid out for the handover.'),
          {'residual_at_station_0_m': at_current, 'shift_needed_m': best_dx,
           'crouch_needed_m': best_dz})
    rows, farthest = sweep(rig)
    table_offset_x, table_half_x = rig.source_table()
    base_x0 = float(rig.d.xpos[rig.m.body('h_LINK_BASE').id][0])
    shift_max = band[0] - (table_offset_x + table_half_x) - base_x0
    feasible = []
    for row in st_windows:
        if not row['reachable_dx_m']:
            continue
        lo = max(row['reachable_dx_m'][0], -1e9)
        hi = min(row['reachable_dx_m'][1], shift_max)
        if lo <= hi + 1e-9:
            feasible.append((row['dz_m'], lo, hi))
    check('envelope', 'the station window that satisfies the REACH and the TABLE is non-empty',
          'PASS' if (feasible and not feasible[0][1] > shift_max) or feasible else 'FAIL',
          f'two constraints pin the station, and neither is a preference. The arm needs the shift '
          f'to be at least {best_dx if best_dx is not None else float("nan"):+.2f} m (it grows as '
          f'the crouch shrinks). The TABLE needs it at most {shift_max:+.4f} m: the table top plate sits '
          f'plate sits {table_offset_x:+.4f} m ahead of the base with a {table_half_x * 1000:.0f} '
          f'mm half-length, so its east edge is {table_offset_x + table_half_x:+.4f} m ahead of '
          f'the base, and the tray cannot be lowered past that edge -- the band first roller is '
          f'at x {band[0]:.4f}. With the measured {farthest if farthest is None else f"{farthest:.3f}"}'
          f' m of carry, the surviving windows are '
          + (', '.join(f'dz {dz:+.2f} -> shift [{lo:+.3f}, {hi:+.3f}]'
                       for dz, lo, hi in feasible) if feasible else 'NONE')
          + '. An empty intersection means no station works and the layout itself has to change, '
            'which is a different finding from "the arm is too short".',
          {'shift_min_from_reach_m': best_dx, 'shift_max_from_table_m': shift_max,
           'carry_m': farthest, 'feasible': feasible,
           'table_offset_x': table_offset_x, 'table_half_x': table_half_x})

    check('envelope', 'the lateral reach from the tray\'s own pose is bounded',
          'PASS' if (farthest is not None and farthest < SWEEP_MAX_M) else 'FAIL',
          f'carrying the tray from its own pose out along +x in {SWEEP_STEP_M * 1000:.0f} mm '
          f'steps, the farthest offset BOTH hands still hold is '
          f'{farthest if farthest is None else round(farthest, 3)} m; the sweep ran to '
          f'{SWEEP_MAX_M:.2f} m and stopped reaching. A sweep that never stops reaching is not '
          f'measuring reach, and this row is what says the "reached" samples mean something. '
          f'{len(rows)} samples.',
          {'farthest_offset_m': farthest, 'samples': len(rows)})
    far_grip = {k: {'name': v['name'],
                    'point': np.array([origin[0] + SWEEP_MAX_M, v['point'][1], v['point'][2]])}
                for k, v in grip.items()}
    far_res, _p, _pa, _q = solve_arms(rig, far_grip, np.array(rig.d.qpos))
    check('falsifiability', 'a placement beyond reach is reported as NOT reached',
          'PASS' if (farthest is None or farthest < SWEEP_MAX_M - 1e-9)
          and far_res > REACH_TOL_M else 'FAIL',
          f'the sweep\'s own failability arm: a grip point {SWEEP_MAX_M:.2f} m along +x leaves a '
          f'worst residual of {far_res * 1000:.1f} mm, against {res * 1000:.3f} mm at the tray\'s '
          f'own pose. Without this arm, "not reached" in the sweep\'s tail would be '
          f'indistinguishable from a solver that had stopped working',
          {'residual_at_far_m': far_res, 'residual_at_start_m': res})

    # --- 3. torque --------------------------------------------------------------------------
    close_frac, light, close_table = choose_close(rig, tray_factor=1.0)
    mass = float(rig.m.body_mass[rig.tray])
    usage = light['worst_usage'] if 'worst_usage' in light else \
        max(light['demand_Nm'][n] / rig.arm_span[n] for n in ARM_JOINTS)
    check('torque', 'the tray is actually CARRIED: it moves with the hands',
          'PASS' if light['tray_moved_m'] > GRASP_LIFT_M else 'FAIL',
          f'the smallest finger closure that still carries the tray is '
          f'{"none of the tried fractions" if close_frac is None else f"{close_frac:.2f} of each "
             f"finger joint\'s own range"}, and at that closure the tray\'s origin moves '
          f'{light["tray_moved_m"] * 1000:.2f} mm, from {light["tray_origin_start"]} to '
          f'{light["tray_origin_end"]}. Fractions tried, with what each did: '
          + ', '.join(f'{r["close_fraction"]:.2f}->{r["tray_moved_m"] * 1000:.1f} mm/'
                      f'{r["worst_usage"] * 100:.0f}%' for r in close_table)
          + '. A torque measured on a tray that never left its support would be measuring the '
            'support, so how far the fingers must curl is measured rather than assumed',
          {'close_fraction': close_frac, 'tray_moved_m': light['tray_moved_m'],
           'fractions_tried': close_table})
    heavy = grasp_and_hold(rig, close=close_frac if close_frac is not None else 0.85,
                           tray_factor=HEAVY_TRAY_FACTOR)
    heaviest = max(heavy['demand_Nm'][n] / rig.arm_span[n] for n in ARM_JOINTS)
    check('torque', 'the measured hold torque RESPONDS to the payload',
          'PASS' if heaviest > 1.5 * usage else 'FAIL',
          f'the SAME closure with the tray at {HEAVY_TRAY_FACTOR:.0f}x its {mass:.4f} kg mass '
          f'raises the worst usage from {usage * 100:.3f} % to {heaviest * 100:.3f} % of the '
          f'declared command range, and moves the tray {heavy["tray_moved_m"] * 1000:.2f} mm. One '
          f'variable changes -- the mass -- so a probe that reads the same number for a '
          f'{HEAVY_TRAY_FACTOR:.0f}x payload is measuring something other than the payload. '
          f'This is the row\'s control arm and it is not decoration: the instrument this row '
          f'replaced (`qfrc_bias` at the arm joints) read 1.947 % for both arms, because the '
          f'tray\'s weight reaches the arm through CONTACT and never enters the humanoid\'s bias '
          f'torque at all.',
          {'usage_light': usage, 'usage_heavy': heaviest,
           'tray_moved_heavy_m': heavy['tray_moved_m']})
    check('torque', 'holding the tray stays inside the actuator\'s declared range',
          'PASS' if usage <= TORQUE_USAGE_MAX else 'FAIL',
          f'worst usage {usage * 100:.3f} % of the declared range against a '
          f'{TORQUE_USAGE_MAX * 100:.0f} % budget; per joint '
          + ', '.join(f'{n.split("_", 1)[1]} {light["demand_Nm"][n]:.3f}/{rig.arm_span[n]:.0f} N.m'
                      for n in ARM_JOINTS)
          + f'. At {mass:.4f} kg the tray is not the binding constraint -- this row is reported '
            f'because a torque margin that is never measured cannot be cited',
          {'worst_usage_fraction': usage, 'per_joint_Nm': light['demand_Nm']})

    # --- 4. interference --------------------------------------------------------------------
    other = light['contacts']['tray_vs_other_humanoid_mm']
    body_self = light['contacts']['body_self_mm']
    other_txt = 'none' if other is None else \
        f'{other:.3f} mm ({light["contacts"]["tray_vs_other_pair"]})'
    self_txt = 'none' if body_self is None else \
        f'{body_self:.3f} mm ({light["contacts"]["body_self_pair"]})'
    touched = sorted(light['contacts']['touched_tray_geoms'])
    check('interference', 'the humanoid touches the tray ONLY at its declared grip interface',
          'PASS' if other is None or other >= -SELF_ALLOWANCE_MM else 'FAIL',
          f'the tray parts the humanoid touches are {touched or "none"}, and the deepest contact '
          f'with a part that is NOT a handle is {other_txt}. A finger or a palm on a handle ball '
          f'IS the grasp; the tray\'s floor, walls or dividers being touched is the arm leaning on '
          f'the tray instead of holding it, and the row separates them by reading the TRAY\'s own '
          f'geometry rather than by naming the gripper part. Grasp-interface depth: '
          f'{light["contacts"]["tray_vs_interface_mm"]} mm',
          {'deepest_non_interface_mm': other, 'touched_tray_geoms': touched})
    check('interference', 'the humanoid does not intersect itself outside the closed hands',
          'PASS' if body_self is None or body_self >= -SELF_ALLOWANCE_MM else 'FAIL',
          f'deepest humanoid self-contact between two different limbs: {self_txt}, against a '
          f'{SELF_ALLOWANCE_MM} mm allowance. Hand-internal contacts (a closed fist) are excluded '
          f'on purpose: deepest {light["contacts"]["hand_internal_mm"]} mm, and the grasp itself '
          f'reads {light["contacts"]["tray_vs_interface_mm"]} mm',
          {'deepest_body_self_mm': body_self})

    # --- 5. the station gap, as a measurement -------------------------------------------------
    hand_mid = (rig.site('h_lh_grasp') + rig.site('h_rh_grasp')) / 2.0
    measurements = {
        'grasp_residual_m': res,
        'grip_points': {k: [round(float(v), 5) for v in g['point']] for k, g in grip.items()},
        'farthest_offset_m': farthest,
        'band_first_roller_x': band[0],
        'tray_start_x': float(origin[0]),
        'band_target_x': target_x,
        'band_target_z': target_z,
        'shift_needed_m': best_dx,
        'crouch_needed_m': best_dz,
        'reachable_dx_window_m': [best_dx, best_dx_hi],
        'residual_at_station_0_m': at_current,
        'station_windows': st_windows,
        'close_fraction': close_frac,
        'hands_x_at_solve': float(hand_mid[0]),
        'hold_torque_Nm': light['demand_Nm'],
        'hold_usage_fraction': usage,
        'heavy_usage_fraction': heaviest,
        'tray_moved_m_under_grasp': light['tray_moved_m'],
    }

    failed = [c for c in checks if c['verdict'] == 'FAIL']
    verdict = 'FAIL' if failed else 'PASS'
    report = {'run_id': args.run_id,
              'scope': 'P4-HUMAN-01 step 1: arm envelope and torque margin against the tray '
                       'grip interface',
              'label': args.label,
              'world': {'path': str(world.relative_to(ROOT)),
                        'sha256': hashlib.sha256(world.read_bytes()).hexdigest()},
              'tray': {'body': args.tray, 'body_id': rig.tray, 'mass_kg': mass},
              'verdict': verdict, 'checks': checks, 'sweep': rows,
              'measurements': measurements, 'failed': [c['check'] for c in failed]}
    (out / 'report.json').write_text(json.dumps(report, indent=2, default=str) + '\n',
                                     encoding='utf-8')
    lines = [f'# {args.run_id} — P4 step 1, arm reach and torque', '', f'**verdict {verdict}**',
             '', f'world `{report["world"]["path"]}` sha256 `{report["world"]["sha256"][:16]}…`',
             f'label: {args.label or "(none)"}', '']
    for c in checks:
        lines += [f'- **{c["verdict"]}** ({c["group"]}) {c["check"]}', f'  - {c["detail"]}']
    (out / 'acceptance.md').write_text('\n'.join(lines) + '\n', encoding='utf-8')

    for c in checks:
        print(f'  [{c["verdict"]}] ({c["group"]}) {c["check"]}')
        print(f'        {c["detail"]}')
    print()
    print(f'verdict {verdict}; farthest placement {farthest}; worst usage {usage * 100:.3f} %')
    return 0 if verdict == 'PASS' else 1


if __name__ == '__main__':
    raise SystemExit(main())
