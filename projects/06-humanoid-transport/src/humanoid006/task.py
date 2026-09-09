"""Measured stop-grasp-lift-carry-stop-release phases, never object attachments."""
import mujoco
import numpy as np
import json
from .runtime import ROOT, OPEN, GRASP, orientation

CONFIG = json.loads((ROOT/'config/mission.json').read_text())
SOURCE = np.array(CONFIG['source'])
DESTINATION = np.array(CONFIG['destination'])
GRASP_OFFSET = np.array(CONFIG['grasp_offset'])
CLEARANCE = CONFIG['clearance']


def observe(r):
    forces = {p:0. for p in ('ff','mf','rf','th')}
    source = target = False
    oid = r.m.geom('payload_geom').id
    for i in range(r.d.ncon):
        c = r.d.contact[i]
        if oid not in (c.geom1, c.geom2):
            continue
        other = c.geom2 if c.geom1 == oid else c.geom1
        source |= other == r.m.geom('source_support').id
        target |= other == r.m.geom('receiving_pad').id
        body = r.m.body(r.m.geom_bodyid[other]).name
        for prefix in forces:
            if body.startswith('rh_'+prefix+'_'):
                force = np.zeros(6)
                mujoco.mj_contactForce(r.m, r.d, i, force)
                forces[prefix] += max(float(force[0]), 0.)
    velocity = np.zeros(6)
    mujoco.mj_objectVelocity(r.m,r.d,mujoco.mjtObj.mjOBJ_BODY,r.m.body('payload').id,velocity,0)
    payload = r.d.body('payload').xpos.copy()
    palm = r.d.body('rh_palm')
    relative = palm.xmat.reshape(3,3).T@(payload-palm.xpos)
    return dict(payload=payload, relative=relative, forces=forces, source=source, target=target,
                speed=float(np.linalg.norm(velocity[3:])), base_speed=float(np.linalg.norm(r.d.qvel[:2])))


def grasped(obs):
    return obs['forces']['th']>.02 and sum(obs['forces'][p]>.02 for p in ('ff','mf','rf'))>=2


def retained(obs, reference):
    # Contact load can redistribute during a lift. Measure real hand-relative slip.
    return (sum(force>.02 for force in obs['forces'].values())>=2
            and reference is not None and np.linalg.norm(obs['relative']-reference)<.025)


def placed(obs):
    return (np.linalg.norm(obs['payload'][:2]-DESTINATION[:2])<.06
            and abs(obs['payload'][2]-DESTINATION[2])<.012 and obs['target']
            and sum(obs['forces'].values())<.02 and obs['speed']<.025)


class Mission:
    def __init__(self, r, stop_after_lift=False):
        self.r = r
        self.phase, self.since, self.stable_since = 'STAND', 0., None
        self.events = []
        self.result = None
        self.reason = ''
        self.arm = None
        self.hand = OPEN.copy()
        self.goal = np.zeros(3)
        self.arm_point = SOURCE+GRASP_OFFSET if CONFIG['approach']=='direct' else SOURCE+np.array([-.20,-.12,CLEARANCE])
        self.stop_after_lift = stop_after_lift
        self.max_lift = 0.
        self.loss_since = None
        self.reference = None
        self.alignment_retries = 0

    def enter(self, phase):
        self.phase, self.since, self.stable_since = phase, float(self.r.d.time), None
        self.events.append(dict(time=self.since, phase=phase))

    def finish(self, result, reason):
        self.result, self.reason = result, reason
        self.enter(result)

    def stable(self, condition, duration):
        now = self.r.d.time
        if not condition:
            self.stable_since = None
        elif self.stable_since is None:
            self.stable_since = now
        return self.stable_since is not None and now-self.stable_since >= duration

    def update(self):
        r, now = self.r, float(self.r.d.time)
        if self.result:
            return
        if not np.isfinite(r.d.qpos).all() or not np.isfinite(r.d.qvel).all():
            self.finish('FAILED', 'NONFINITE_STATE'); return
        obs = observe(r)
        age = now-self.since
        self.max_lift = max(self.max_lift, float(obs['payload'][2]-SOURCE[2]))
        if r.d.qpos[2] < .65 or orientation(r.d.qpos[3:7])[1] > np.deg2rad(45):
            self.finish('FAILED','BODY_FALL'); return
        if obs['payload'][2] < SOURCE[2]-.20:
            self.finish('FAILED','PAYLOAD_DROPPED'); return
        if age > 30:
            self.finish('FAILED','PHASE_TIMEOUT'); return
        if CONFIG['grip_feedback'] and self.phase in ('VERIFY_GRASP','LIFT','HOLD','CARRY','STOP','PLACE'):
            self.hand = r.grip_feedback(self.hand,obs['payload'],obs['forces'])
        if self.phase in ('LIFT','HOLD','CARRY','STOP','PLACE'):
            if not retained(obs,self.reference):
                if self.loss_since is None: self.loss_since=now
                if now-self.loss_since > .5:
                    self.finish('FAILED','GRASP_LOST'); return
            else: self.loss_since=None
        if self.phase == 'STAND':
            if now>3 and self.stable(obs['base_speed']<.08,1.):
                self.arm = r.policy.default[18:23].copy()
                self.enter('REACH')
        elif self.phase == 'REACH':
            self.hand = OPEN.copy() if CONFIG['approach']=='direct' else GRASP.copy()
            self.arm = r.arm_ik(self.arm_point, self.arm)
            error = np.linalg.norm(r.d.site('grasp_center').xpos-self.arm_point)
            if self.stable(error<.008 and obs['base_speed']<.08, .6):
                self.enter('CLOSE' if CONFIG['approach']=='direct' else 'ABOVE')
            elif np.linalg.norm(obs['payload']-SOURCE) > .05:
                self.finish('FAILED','OBJECT_DISPLACED_DURING_REACH')
        elif self.phase == 'ABOVE':
            self.hand = GRASP.copy()
            self.arm_point = SOURCE+np.array([-.20,-.12,CLEARANCE])+min(age/4,1)*np.array([.20,.12,0.])
            self.arm = r.arm_ik(self.arm_point,self.arm)
            if age>4 and self.stable(np.linalg.norm(r.d.site('grasp_center').xpos-self.arm_point)<.008,.6):
                self.enter('OPEN_ABOVE')
        elif self.phase == 'OPEN_ABOVE':
            self.arm = r.arm_ik(self.arm_point,self.arm)
            self.hand = GRASP+min(age/2,1)*(OPEN-GRASP)
            if age>3:
                self.enter('DESCEND')
        elif self.phase == 'DESCEND':
            self.arm_point = SOURCE+np.array([0.,0.,CLEARANCE])+min(age/5,1)*(GRASP_OFFSET-np.array([0.,0.,CLEARANCE]))
            self.arm = r.arm_ik(self.arm_point,self.arm)
            error = np.linalg.norm(r.d.site('grasp_center').xpos-self.arm_point)
            if age>5 and self.stable(error<.008,.6):
                self.enter('CLOSE')
        elif self.phase == 'CLOSE':
            self.arm_point = SOURCE+np.clip(obs['payload']-SOURCE,-.035,.035)+GRASP_OFFSET
            self.arm = r.arm_ik(self.arm_point, self.arm)
            self.hand = OPEN+min(age/3,1)*(GRASP-OPEN)
            if age>=4:
                self.enter('VERIFY_GRASP')
        elif self.phase == 'VERIFY_GRASP':
            self.arm_point = SOURCE+np.clip(obs['payload']-SOURCE,-.035,.035)+GRASP_OFFSET
            self.arm = r.arm_ik(self.arm_point, self.arm)
            if self.stable(grasped(obs),.5):
                self.lift_origin = self.arm_point.copy()
                self.reference = obs['relative'].copy()
                self.enter('LIFT')
            elif age>4:
                self.finish('FAILED','NO_OPPOSED_CONTACT')
        elif self.phase == 'LIFT':
            self.arm_point = self.lift_origin+np.array([0,0,.12*min(age/3,1)])
            self.arm = r.arm_ik(self.arm_point,self.arm)
            if age>4:
                if obs['payload'][2]-SOURCE[2]>.08 and not obs['source']:
                    self.enter('HOLD')
                else: self.finish('FAILED','INSUFFICIENT_LIFT')
        elif self.phase == 'HOLD':
            self.arm = r.arm_ik(self.arm_point,self.arm)
            slip = np.linalg.norm(obs['relative']-self.reference)
            if slip>.025: self.finish('FAILED','PAYLOAD_SLIP')
            elif age>2:
                if self.stop_after_lift:
                    self.finish('COMPLETED','STANDING_GRASP_LIFT_ONLY')
                else:
                    self.arm = r.d.qpos[r.qa[r.arm_ids]].copy()
                    self.goal = np.r_[r.d.qpos[:2]+DESTINATION[:2]-obs['payload'][:2],0.]
                    self.enter('CARRY')
        elif self.phase == 'CARRY':
            alignment = DESTINATION[:2]+np.array(CONFIG['settling_compensation'])
            self.goal[:2] = r.d.qpos[:2]+alignment-obs['payload'][:2]
            if self.stable(np.linalg.norm(obs['payload'][:2]-alignment)<.05 and obs['base_speed']<.08,.5):
                self.goal[:2] = r.d.qpos[:2]
                self.enter('STOP')
        elif self.phase == 'STOP':
            if self.stable(obs['base_speed']<.08,1.):
                if np.linalg.norm(obs['payload'][:2]-DESTINATION[:2])>.055:
                    self.alignment_retries += 1
                    if self.alignment_retries > 3:
                        self.finish('FAILED','ALIGNMENT_RETRIES_EXHAUSTED')
                    else:
                        self.enter('CARRY')
                else:
                    self.place_start = r.d.site('grasp_center').xpos.copy()
                    self.lower_distance = max(0.,float(obs['payload'][2]-DESTINATION[2]-.005))
                    self.enter('PLACE')
        elif self.phase == 'PLACE':
            target = self.place_start-np.array([0,0,self.lower_distance])
            self.arm_point = self.place_start+min(age/4,1)*(target-self.place_start)
            self.arm = r.arm_ik(self.arm_point,self.arm)
            near_surface = (np.linalg.norm(obs['payload'][:2]-DESTINATION[:2])<.055
                            and -.005<obs['payload'][2]-DESTINATION[2]<.04
                            and obs['speed']<.03 and obs['base_speed']<.08)
            if age>5 and self.stable(near_surface,.5):
                self.release_start = self.arm_point.copy()
                self.release_hand = self.hand.copy()
                self.enter('RELEASE')
        elif self.phase == 'RELEASE':
            release = self.release_hand.copy()
            release[4], release[8] = -.45, .45
            release[12:] = OPEN[12:]
            self.hand = self.release_hand+min(age/3,1)*(release-self.release_hand)
            if age>3:
                self.hand = release+min((age-3)/5,1)*(OPEN-release)
            # Keep the arm still while the fingers open.
            if age>9:
                self.retract_start = r.d.site('grasp_center').xpos.copy()
                self.enter('RETRACT')
        elif self.phase == 'RETRACT':
            self.hand = OPEN.copy()
            self.arm = r.arm_ik(self.retract_start+np.array([0,0,.18*min(age/4,1)]),self.arm)
            if age>5: self.enter('VERIFY_PLACE')
        elif self.phase == 'VERIFY_PLACE':
            if self.stable(placed(obs),1.): self.finish('COMPLETED','STOP_GRASP_CARRY_PLACE_VERIFIED')
            elif age>6: self.finish('FAILED','PLACEMENT_NOT_STABLE')
