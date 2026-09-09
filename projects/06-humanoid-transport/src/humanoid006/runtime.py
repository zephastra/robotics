"""Explicit joint ownership for a floating-base humanoid and articulated hand."""
from pathlib import Path
import hashlib
import json
import math
import mujoco
import numpy as np
from .policy import T800Policy

ROOT = Path(__file__).resolve().parents[2]
OPEN = np.array([0., 0., 0., 0.] * 3 + [.3, 0., 0., 0.])
GRASP = np.array([-.2684, .7374, 1.0586, 1.0175, 0., .8488, .9083, .7839,
                  .2684, .7374, 1.0586, 1.0175, 1.1135, .0672, 1.544, .0487])
GRASP[12:] = json.loads((ROOT/'config/mission.json').read_text())['thumb_grasp']


def orientation(quat):
    w, x, y, z = quat
    yaw = math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z))
    tilt = math.acos(np.clip(1-2*(x*x+y*y), -1, 1))
    return yaw, tilt


class Runtime:
    def __init__(self, baseline=False):
        self.policy = T800Policy(ROOT)
        self.m = mujoco.MjModel.from_xml_path(str(ROOT/('assets/t800/scene.xml' if baseline else 'assets/combined.xml')))
        self.d = mujoco.MjData(self.m)
        self.m.vis.global_.offwidth, self.m.vis.global_.offheight = 960, 720
        self.body_act = np.array([self.m.actuator(name).id for name in self.actuator_names()])
        ids = np.array([self.m.joint(name).id for name in self.policy.joints])
        self.qa, self.va = self.m.jnt_qposadr[ids], self.m.jnt_dofadr[ids]
        self.arm_ids = np.arange(18, 23)
        self.hand_act = np.array([self.m.actuator('rh_'+p+'a'+str(i)).id for p in ('ff','mf','rf','th') for i in range(4)]) if not baseline else np.array([], dtype=int)
        self.hand_qa = self.m.jnt_qposadr[self.m.actuator_trnid[self.hand_act, 0]]
        self.d.qpos[self.qa] = self.policy.default
        self.d.qpos[self.hand_qa] = OPEN if not baseline else []
        self.hand_target = OPEN.copy()
        self.arm_target = self.policy.default[self.arm_ids].copy()
        self.arm_weight = 0.
        self.command = np.zeros(3)
        self.position_integral = np.zeros(2)
        self.tick = 0
        mujoco.mj_forward(self.m, self.d)

    def actuator_names(self):
        # Resolve actuator by joint, never assume inserted hand joints retain old offsets.
        result = []
        for name in self.policy.joints:
            jid = self.m.joint(name).id
            indices = np.flatnonzero(self.m.actuator_trnid[:, 0] == jid)
            if len(indices) != 1:
                raise RuntimeError('Ambiguous actuator mapping: '+name)
            result.append(self.m.actuator(int(indices[0])).name)
        return result

    def hold_command(self, goal, precise=False):
        yaw, _ = orientation(self.d.qpos[3:7])
        error = np.asarray(goal[:2])-self.d.qpos[:2]
        c, s = math.cos(yaw), math.sin(yaw)
        local = np.array([c*error[0]+s*error[1], -s*error[0]+c*error[1]])
        distance = np.linalg.norm(error)
        if precise and .01 < distance < .45:
            self.position_integral = np.clip(self.position_integral+.5*local*self.m.opt.timestep,-.15,.15)
        else:
            self.position_integral[:] = 0.
        angle = math.atan2(math.sin(goal[2]-yaw), math.cos(goal[2]-yaw))
        return np.clip(np.r_[local*.8+self.position_integral, angle*1.2], [-.3,-.2,-.4], [.3,.2,.4])

    def step(self, command, arm_target=None, hand_target=None):
        if self.tick % 5 == 0:
            self.command += np.clip(np.array(command)-self.command, -.004, .004)
            q, dq = self.d.qpos[self.qa], self.d.qvel[self.va]
            # Feed actual arm states to the policy; never hide manipulation from observations.
            self.policy.infer(q, dq, self.d.qpos[3:7], self.d.qvel[3:6], self.command, self.d.time)
            requested = arm_target is not None
            self.arm_weight += np.clip(float(requested)-self.arm_weight, -.005, .005)
            if requested:
                self.arm_target += np.clip(np.asarray(arm_target)-self.arm_target, -.006, .006)
            indices = self.arm_ids
            self.policy.target[indices] = ((1-self.arm_weight)*self.policy.target[indices]
                                           + self.arm_weight*self.arm_target)
            if hand_target is not None:
                self.hand_target += np.clip(np.asarray(hand_target)-self.hand_target, -.012, .012)
        q, dq = self.d.qpos[self.qa], self.d.qvel[self.va]
        torque = self.policy.torques(q, dq)
        torque[self.arm_ids] += self.arm_weight*self.d.qfrc_bias[self.va[self.arm_ids]]
        torque[self.arm_ids] -= self.arm_weight*3.*dq[self.arm_ids]
        limits = self.m.jnt_actfrcrange[self.m.actuator_trnid[self.body_act, 0]]
        self.d.ctrl[self.body_act] = np.clip(torque, limits[:, 0], limits[:, 1])
        self.d.ctrl[self.hand_act] = self.hand_target if len(self.hand_act) else []
        mujoco.mj_step(self.m, self.d)
        self.tick += 1

    def snapshot(self):
        yaw, tilt = orientation(self.d.qpos[3:7])
        return dict(time=float(self.d.time), x=float(self.d.qpos[0]), y=float(self.d.qpos[1]),
                    z=float(self.d.qpos[2]), yaw=float(yaw), tilt_deg=float(math.degrees(tilt)),
                    arm_weight=float(self.arm_weight),
                    grasp_center=self.d.site('grasp_center').xpos.tolist() if len(self.hand_act) else None)

    def arm_ik(self, position, seed=None):
        scratch = mujoco.MjData(self.m)
        scratch.qpos[:] = self.d.qpos
        if seed is not None:
            scratch.qpos[self.qa[self.arm_ids]] = seed
        joints = self.m.actuator_trnid[self.body_act[self.arm_ids], 0]
        limits = self.m.jnt_range[joints]
        for _ in range(30):
            mujoco.mj_forward(self.m, scratch)
            error = np.asarray(position)-scratch.site('grasp_center').xpos
            if np.linalg.norm(error) < .001:
                break
            jac = np.zeros((3, self.m.nv))
            mujoco.mj_jacSite(self.m, scratch, jac, None, self.m.site('grasp_center').id)
            J = jac[:, self.va[self.arm_ids]]
            dq = J.T@np.linalg.solve(J@J.T+np.eye(3)*.001, error)
            index = self.qa[self.arm_ids]
            scratch.qpos[index] = np.clip(scratch.qpos[index]+np.clip(dq, -.08, .08), limits[:, 0], limits[:, 1])
        return scratch.qpos[self.qa[self.arm_ids]].copy()

    def grip_feedback(self, target, payload, forces):
        result = np.asarray(target).copy()
        for group, prefix in enumerate(('ff','mf','rf','th')):
            # Preserve supporting finger posture; restore only unloaded thumb contact.
            if prefix != 'th' or forces[prefix] >= .15:
                continue
            site = self.m.site(prefix+'_tip').id
            toward = np.asarray(payload)-self.d.site_xpos[site]
            toward /= max(np.linalg.norm(toward),1e-6)
            jac = np.zeros((3,self.m.nv))
            mujoco.mj_jacSite(self.m,self.d,jac,None,site)
            actuators = self.hand_act[group*4:group*4+4]
            dofs = self.m.jnt_dofadr[self.m.actuator_trnid[actuators,0]]
            gradient = toward@jac[:,dofs]
            error = max(0.,.15-forces[prefix])
            delta = .003*error*gradient/(gradient@gradient+.0001)
            result[group*4:group*4+4] += np.clip(delta,-.01,.01)
        return np.clip(result,self.m.actuator_ctrlrange[self.hand_act,0],self.m.actuator_ctrlrange[self.hand_act,1])


def verify_assets():
    manifest = json.loads((ROOT/'assets/manifest.json').read_text())
    for name, expected in manifest['sha256'].items():
        if hashlib.sha256((ROOT/name).read_bytes()).hexdigest() != expected:
            raise RuntimeError('Asset hash mismatch: '+name)
    return manifest
