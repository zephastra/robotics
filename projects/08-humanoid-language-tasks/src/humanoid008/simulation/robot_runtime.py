"""Robot actuation/proprioception; no task access to object ground-truth poses.

Adapted from the 007 baseline's ``runtime.py`` into 008's ``simulation``
package (section 6.1). Changes: the project-root anchor, the policy import, and
``verify_assets`` reads ``assets/baseline_manifest.json``.
"""

import hashlib
import json
import math
import os
from pathlib import Path

import mujoco
import numpy as np
import yaml

from . import ROOT
from .walking_policy import T800Policy

# Yaw-tracking compensation for the T800 policy (docs/C_STATION_DIAGNOSIS.md).
#
# Measured yaw response (clean pose, 3 s per point):
#   omega 0.05-0.30 -> 0.1-1.5 deg/s   (a weak 'creep' region)
#   omega 0.40      -> 2.4 deg/s
#   omega 0.50      -> 7.6 deg/s        (knee)
#   omega 0.60      -> 17.9 deg/s
# hold_command emits only ~0.11-0.17 rad/s for a 6-10 deg yaw error, which
# sits deep in the creep region -- that is why the heading never corrects.
# So the fix is NOT a proportional gain: the request must clear the policy's
# yaw deadband. When the yaw error is above a threshold, snap the omega
# request to YAW_ACTIVE_OMEGA (which lands past the knee); below it, keep the
# proportional term so fine alignment still damps smoothly.
#
# Set HUMANOID008_YAW_FIX=1 to enable. Defaults OFF (== original behaviour).
#
# WHY OFF: enabling this together with corridor offset +0.03 regressed the
# 30-case acceptance batch (scripts/batch.py --split) from 26/30 to 14/30.
# It looked helpful on a single unperturbed case and is harmful across the
# perturbed suite. Do not flip this default without a full batch run.
# See docs/C_STATION_ROOTCAUSE.md.
YAW_FIX = os.environ.get("HUMANOID008_YAW_FIX", "0") == "1"
# Yaw error above this (rad) uses the full active request; below, proportional.
# Measured: 0.5 deg converges to a 0.50 deg residual (best); larger bands stall
# at the band value, e.g. 2 deg -> 2.77 deg residual.
YAW_DEADBAND_RAD = math.radians(0.5)
# Request used past the deadband (rad/s); 0.5 measured at 7.6 deg/s.
YAW_ACTIVE_OMEGA = 0.5
# Clip for the compensated yaw request. The original code clipped yaw at 0.3
# rad/s, which is inside the policy's deadband, so the compensated value needs
# a wider bound (0.6 measured at 17.9 deg/s).
YAW_REQUEST_CLIP = 0.6

OPEN = np.array([0.0, 0.0, 0.0, 0.0] * 3 + [0.3, 0.0, 0.0, 0.0])
GRASP = np.array(
    [
        -0.2684, 0.7374, 1.0586, 1.0175, 0.0, 0.8488, 0.9083, 0.7839,
        0.2684, 0.7374, 1.0586, 1.0175, 1.1135, 0.0672, 1.544, 0.0487,
    ]
)


def withdrawal_waypoints(anchors):
    """Open hands first clear the handles vertically, then return toward the body.

    The bounded workcell faces +X; this is not a general obstacle-avoidance path.
    """
    return [
        {
            side: np.asarray(anchors[side]) + np.array([x, sign * y, z])
            for side, sign in [("left", 1), ("right", -1)]
        }
        for x, y, z in [(-0.04, 0.025, 0.10), (-0.08, 0.04, 0.14)]
    ]


def orientation(quat):
    w, x, y, z = quat
    return (
        math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)),
        math.acos(np.clip(1 - 2 * (x * x + y * y), -1, 1)),
    )


def forward_kinematics(model, data, cameras=False):
    """Scratch planning only: update poses/Jacobians without solving dynamics."""
    mujoco.mj_kinematics(model, data)
    mujoco.mj_comPos(model, data)
    if cameras:
        mujoco.mj_camlight(model, data)


class Runtime:
    def __init__(self):
        self.policy = T800Policy(ROOT)
        self.m = mujoco.MjModel.from_xml_path(str(ROOT / "assets" / "combined.xml"))
        self.d = mujoco.MjData(self.m)
        self.m.vis.global_.offwidth, self.m.vis.global_.offheight = 960, 720
        joints = np.array([self.m.joint(n).id for n in self.policy.joints])
        self.qa, self.va = self.m.jnt_qposadr[joints], self.m.jnt_dofadr[joints]
        self.body_act = np.array([self._actuator(j) for j in joints])
        self.arms = {"left": np.arange(13, 18), "right": np.arange(18, 23)}
        self.arm_ids = np.arange(13, 23)
        self.hand_act = {
            side: np.array(
                [
                    self.m.actuator(prefix + p + "a" + str(i)).id
                    for p in ("ff", "mf", "rf", "th")
                    for i in range(4)
                ]
            )
            for side, prefix in [("left", "lh_"), ("right", "rh_")]
        }
        self.d.qpos[self.qa] = self.policy.default
        self.arm_target = self.policy.default[self.arm_ids].copy()
        self.arm_weight = 0.0
        self.hand_target = {s: OPEN.copy() for s in self.arms}
        for side, act in self.hand_act.items():
            qadr = self.m.jnt_qposadr[self.m.actuator_trnid[act, 0]]
            self.d.qpos[qadr] = OPEN
        self.head_target = np.zeros(2)
        self.command = np.zeros(3)
        self.integral = np.zeros(2)
        self.tick = 0
        stand = yaml.safe_load((ROOT / "config" / "t800" / "stand.yaml").read_text())
        self.stand_kp = np.concatenate(stand["stiffness"])[:13]
        self.stand_kd = np.concatenate(stand["damping"])[:13]
        self.stand_target = None
        self.stand_weight = 0.0
        self.stand_ready = 0.0
        mujoco.mj_forward(self.m, self.d)

    def _actuator(self, joint):
        ids = np.flatnonzero(self.m.actuator_trnid[:, 0] == joint)
        if len(ids) != 1:
            raise ValueError("Ambiguous body actuator")
        return int(ids[0])

    def feet_loaded(self):
        forces = {"L": 0.0, "R": 0.0}
        for i in range(self.d.ncon):
            contact = self.d.contact[i]
            geoms = [int(contact.geom1), int(contact.geom2)]
            for g, other in (geoms, geoms[::-1]):
                name = self.m.body(int(self.m.geom_bodyid[g])).name
                if (
                    name in ("LINK_FOOT_L", "LINK_FOOT_R", "LINK_ANKLE_ROLL_L", "LINK_ANKLE_ROLL_R")
                    and self.m.geom_type[other] == mujoco.mjtGeom.mjGEOM_PLANE
                ):
                    force = np.zeros(6)
                    mujoco.mj_contactForce(self.m, self.d, i, force)
                    forces[name[-1]] += max(0.0, float(force[0]))
        return min(forces.values()) > 10.0

    def step(self, command, arms=None, hands=None, head=None, stationary=False):
        if self.tick % 5 == 0:
            if stationary and self.stand_target is None:
                self.stand_ready = self.stand_ready + 0.01 if self.feet_loaded() else 0.0
                if self.stand_ready >= 0.2:
                    self.stand_target = self.d.qpos[self.qa[:13]].copy()
            requested = bool(stationary and self.stand_target is not None)
            self.stand_weight += np.clip(float(requested) - self.stand_weight, -0.01, 0.01)
            if not stationary and self.stand_weight <= 0:
                self.stand_target = None
                self.stand_ready = 0.0
            self.command += np.clip(np.asarray(command) - self.command, -0.004, 0.004)
            self.policy.infer(
                self.d.qpos[self.qa],
                self.d.qvel[self.va],
                self.d.qpos[3:7],
                self.d.qvel[3:6],
                self.command,
                self.d.time,
            )
            self.arm_weight += np.clip(float(arms is not None) - self.arm_weight, -0.005, 0.005)
            if arms is not None:
                self.arm_target += np.clip(np.asarray(arms) - self.arm_target, -0.005, 0.005)
            self.policy.target[self.arm_ids] = (
                (1 - self.arm_weight) * self.policy.target[self.arm_ids]
                + self.arm_weight * self.arm_target
            )
            if head is not None:
                limits = self.m.jnt_range[self.m.actuator_trnid[self.body_act[23:25], 0]]
                desired = np.clip(head, limits[:, 0], limits[:, 1])
                self.head_target += np.clip(desired - self.head_target, -0.004, 0.004)
            self.policy.target[23:25] = self.head_target
            if hands is not None:
                for side in self.arms:
                    self.hand_target[side] += np.clip(
                        np.asarray(hands[side]) - self.hand_target[side], -0.012, 0.012
                    )
        torque = self.policy.torques(self.d.qpos[self.qa], self.d.qvel[self.va])
        if self.stand_target is not None:
            posture = (
                self.stand_kp * (self.stand_target - self.d.qpos[self.qa[:13]])
                - self.stand_kd * self.d.qvel[self.va[:13]]
                + self.d.qfrc_bias[self.va[:13]]
            )
            torque[:13] = (1 - self.stand_weight) * torque[:13] + self.stand_weight * posture
        torque[self.arm_ids] += self.arm_weight * (
            self.d.qfrc_bias[self.va[self.arm_ids]] - 3 * self.d.qvel[self.va[self.arm_ids]]
        )
        limits = self.m.jnt_actfrcrange[self.m.actuator_trnid[self.body_act, 0]]
        self.d.ctrl[self.body_act] = np.clip(torque, limits[:, 0], limits[:, 1])
        for side, act in self.hand_act.items():
            self.d.ctrl[act] = self.hand_target[side]
        mujoco.mj_step(self.m, self.d)
        self.tick += 1

    def hold_command(self, goal, precise=False, brake=False, alignment=False):
        yaw, _ = orientation(self.d.qpos[3:7])
        error = np.asarray(goal[:2]) - self.d.qpos[:2]
        c, s = math.cos(yaw), math.sin(yaw)
        local = np.array([c * error[0] + s * error[1], -s * error[0] + c * error[1]])
        if precise and 0.02 < np.linalg.norm(error) < 0.45:
            integral_limit = 0.35 if alignment else 0.12
            self.integral = np.clip(
                self.integral + 0.4 * local * self.m.opt.timestep, -integral_limit, integral_limit
            )
        else:
            self.integral[:] = 0
        angle = math.atan2(math.sin(goal[2] - yaw), math.cos(goal[2] - yaw))
        velocity = np.array(
            [c * self.d.qvel[0] + s * self.d.qvel[1], -s * self.d.qvel[0] + c * self.d.qvel[1]]
        )
        translation = local * 0.8 + self.integral
        if brake:
            translation -= 0.9 * velocity
        limit = 0.30 if brake else 0.50
        # Yaw-tracking compensation (see YAW_FIX above). The policy's yaw
        # response has a deadband: requests below ~0.4 rad/s yield <2 deg/s, so a
        # small proportional request never corrects a drifted heading. Above a
        # small error threshold, snap the request past the knee; below it, keep
        # the proportional term so final alignment still damps smoothly.
        # With YAW_FIX off this is the original expression exactly.
        yaw_request = angle
        if YAW_FIX and abs(angle) > YAW_DEADBAND_RAD:
            yaw_request = math.copysign(YAW_ACTIVE_OMEGA, angle)
        yaw_request = float(
            np.clip(yaw_request, -YAW_REQUEST_CLIP, YAW_REQUEST_CLIP)
        )
        return np.clip(
            np.r_[translation, yaw_request], [-limit, -limit, -0.3], [limit, limit, 0.3]
        )

    def arm_ik(self, targets, seed=None, *, safeguarded=False, posture_reference=None):
        scratch = getattr(self, "_ik_scratch", None)
        if scratch is None:
            scratch = mujoco.MjData(self.m)
            self._ik_scratch = scratch
        scratch.qpos[:] = self.d.qpos
        if seed is not None:
            scratch.qpos[self.qa[self.arm_ids]] = seed
        for side, prefix in [("left", "lh_"), ("right", "rh_")]:
            ids = self.arms[side]
            limits = self.m.jnt_range[self.m.actuator_trnid[self.body_act[ids], 0]].copy()
            if safeguarded and posture_reference is not None:
                reference = (
                    np.asarray(posture_reference)[0:5]
                    if side == "left"
                    else np.asarray(posture_reference)[5:10]
                )
                limits[3, 1] = min(limits[3, 1], -0.10)
                limits[4, 0] = max(limits[4, 0], reference[4] - 0.25)
                limits[4, 1] = min(limits[4, 1], reference[4] + 0.25)
                scratch.qpos[self.qa[ids]] = np.clip(
                    scratch.qpos[self.qa[ids]], limits[:, 0], limits[:, 1]
                )
            for _ in range(80 if safeguarded else 30):
                forward_kinematics(self.m, scratch)
                error = np.asarray(targets[side]) - scratch.site(prefix + "grasp").xpos
                if np.linalg.norm(error) < 0.001:
                    break
                jac = np.zeros((3, self.m.nv))
                mujoco.mj_jacSite(self.m, scratch, jac, None, self.m.site(prefix + "grasp").id)
                J = jac[:, self.va[ids]]
                dq = J.T @ np.linalg.solve(J @ J.T + np.eye(3) * 0.001, error)
                if not safeguarded:
                    scratch.qpos[self.qa[ids]] = np.clip(
                        scratch.qpos[self.qa[ids]] + np.clip(dq, -0.08, 0.08),
                        limits[:, 0],
                        limits[:, 1],
                    )
                    continue
                before = scratch.qpos[self.qa[ids]].copy()
                accepted = False
                gradient = J.T @ error / (np.linalg.norm(J, ord="fro") ** 2 + 0.001)
                for direction in (dq, gradient):
                    step = direction * min(1.0, 0.08 / max(np.max(np.abs(direction)), 1e-12))
                    for scale in (1.0, 0.5, 0.25, 0.125):
                        scratch.qpos[self.qa[ids]] = np.clip(
                            before + scale * step, limits[:, 0], limits[:, 1]
                        )
                        forward_kinematics(self.m, scratch)
                        residual = np.asarray(targets[side]) - scratch.site(prefix + "grasp").xpos
                        if np.linalg.norm(residual) < np.linalg.norm(error) - 1e-10:
                            accepted = True
                            break
                    if accepted:
                        break
                if not accepted:
                    scratch.qpos[self.qa[ids]] = before
                    break
        return scratch.qpos[self.qa[self.arm_ids]].copy()

    def arm_error(self, targets, joints):
        scratch = getattr(self, "_error_scratch", None)
        if scratch is None:
            scratch = mujoco.MjData(self.m)
            self._error_scratch = scratch
        scratch.qpos[:] = self.d.qpos
        scratch.qpos[self.qa[self.arm_ids]] = joints
        forward_kinematics(self.m, scratch)
        return max(
            float(np.linalg.norm(scratch.site(prefix + "grasp").xpos - targets[side]))
            for side, prefix in [("left", "lh_"), ("right", "rh_")]
        )

    def gaze(self, point):
        scratch = getattr(self, "_gaze_scratch", None)
        if scratch is None:
            scratch = mujoco.MjData(self.m)
            self._gaze_scratch = scratch
        scratch.qpos[:] = self.d.qpos
        qa = self.qa[23:25]
        limits = self.m.jnt_range[self.m.actuator_trnid[self.body_act[23:25], 0]]

        def residual():
            forward_kinematics(self.m, scratch, cameras=True)
            camera = scratch.camera("eyes")
            direction = np.asarray(point) - camera.xpos
            direction /= max(np.linalg.norm(direction), 1e-9)
            return -camera.xmat.reshape(3, 3)[:, 2] - direction

        for _ in range(12):
            error = residual()
            if np.linalg.norm(error) < 0.005:
                break
            J = np.zeros((3, 2))
            for j, address in enumerate(qa):
                before = scratch.qpos[address]
                scratch.qpos[address] += 1e-4
                J[:, j] = (residual() - error) / 1e-4
                scratch.qpos[address] = before
            delta = -np.linalg.solve(J.T @ J + np.eye(2) * 0.01, J.T @ error)
            scratch.qpos[qa] = np.clip(
                scratch.qpos[qa] + np.clip(delta, -0.1, 0.1), limits[:, 0], limits[:, 1]
            )
        return scratch.qpos[qa].copy()

    def snapshot(self):
        yaw, tilt = orientation(self.d.qpos[3:7])
        return dict(
            time=float(self.d.time),
            base=self.d.qpos[:3].tolist(),
            yaw=yaw,
            tilt_deg=math.degrees(tilt),
            stationary_weight=float(self.stand_weight),
            arm_joints=self.d.qpos[self.qa[self.arm_ids]].tolist(),
            arm_command=self.arm_target.tolist(),
            arm_tracking_error=float(
                np.max(np.abs(self.arm_target - self.d.qpos[self.qa[self.arm_ids]]))
            ),
            head=self.d.qpos[self.qa[23:25]].tolist(),
            hands={s: self.d.site(p + "grasp").xpos.tolist() for s, p in [("left", "lh_"), ("right", "rh_")]},
        )


def verify_assets():
    """Verify imported assets against assets/baseline_manifest.json (section 6.2)."""
    manifest = json.loads((ROOT / "assets" / "baseline_manifest.json").read_text())
    for name, entry in manifest.get("files", {}).items():
        path = ROOT / name
        if not path.is_file():
            raise ValueError("Imported asset missing: " + name)
        actual = hashlib.sha256(path.read_bytes()).hexdigest()
        if actual != entry.get("sha256"):
            raise ValueError("Asset mismatch: " + name)
    return manifest
