"""Robot actuation/proprioception; no task access to object ground-truth poses."""
from pathlib import Path
import hashlib
import json
import math
import yaml
import mujoco
import numpy as np
from .policy import T800Policy

ROOT=Path(__file__).resolve().parents[2]
OPEN=np.array([0.,0.,0.,0.]*3+[.3,0.,0.,0.])
GRASP=np.array([-.2684,.7374,1.0586,1.0175,0.,.8488,.9083,.7839,
                .2684,.7374,1.0586,1.0175,1.1135,.0672,1.544,.0487])


class Names:
    """Resolve the humanoid's model names for whichever world is loaded.

    `assets/combined.xml` (the carried-over snapshot) and the merged worlds name the SAME robot
    differently -- `LINK_BASE` / `lh_grasp` / `J13_SHOULDER_PITCH_L` there, `h_LINK_BASE` /
    `h_lh_grasp` / `h_J13_SHOULDER_PITCH_L` in `merge_world.py`'s output -- and the object body is
    `payload` there and `c_payload` there.

    THE DEFAULT IS THE IDENTITY, deliberately. Every existing call site, and the frozen
    `reports/p1-h-seq-10` fingerprint of this file, describe the un-prefixed world; a mapping whose
    default moved anything would silently reinterpret them. The merged world passes `prefix='h_'`.
    """

    def __init__(self, prefix='', object_body='payload'):
        self.prefix = prefix
        self.object_body = object_body

    def n(self, name):
        """A robot name in this world."""
        return self.prefix + name

    def obj(self):
        """The object body's name in this world."""
        return self.object_body


def hand_sites_at(model, data, qa, arm_ids, joints, names=None):
    """Where the grasp sites sit when the arm joints are `joints`; scratch, no state change."""
    names = names or Names()
    scratch = mujoco.MjData(model)
    scratch.qpos[:] = data.qpos
    scratch.qpos[qa[arm_ids]] = joints
    mujoco.mj_kinematics(model, scratch)
    return {side: scratch.site(names.n(prefix + 'grasp')).xpos.copy()
            for side, prefix in [('left', 'lh_'), ('right', 'rh_')]}


#: Seconds for each stage of `ExitPath`: the Cartesian line out, then the joint ramp home.
#: Sized from reports/p1-h-exitpath-*-04. Three seconds traces the line without the hands
#: ever reaching the tray, and 2.5 s returns a 0.28 rad residual at a rate far below the
#: actuator's own rate limit, so the move is a path rather than a step.
EXIT_LINE_SECONDS = 3.0
EXIT_RETURN_SECONDS = 2.5


class ExitPath:
    """Return to the default posture without disturbing what was just placed.

    The previous behaviour was to command the default posture as soon as the exit stage
    started. `Runtime.step` rate-limits that arm target, so the arm interpolated in *joint*
    space, the open hands swept through the tray, and the tray moved 12.0 mm against a 5 mm
    limit (reports/p1-h-seq-07).

    A Cartesian arrival is not enough on its own. Driving the hands along a straight line
    to the default posture's hand *pose* keeps the tray still (0.07 mm) but leaves the arm
    0.284 rad off the posture, because a hand pose does not determine the joints
    (reports/p1-h-exitpath-line-04). Both stages are needed:

      1. `cart`  walk the hands along a Cartesian line to the default hand pose, solved by
                 IK from the current configuration every tick, so the commanded joints move
                 gradually instead of jumping.
      2. `joint` ramp the remaining joint distance home explicitly, which is a small, slow
                 move now that the hands are out of the object's space.

    Measured on the V1 tray from the same start pose, judged by the same criteria:

      | path                        | tray travel | arm off posture | exit |
      |-----------------------------|-------------|-----------------|------|
      | joint jump to default       | 12.0 mm     | 0.002 rad       | FAIL |
      | Cartesian line only         | 0.07 mm     | 0.284 rad       | FAIL |
      | line then ramp (this)       | 0.07 mm     | 0.0001 rad      | PASS |
      | outward first, then the two | 0.07 mm     | 0.0008 rad      | PASS |

    The variants stay in one class so the comparison is reproducible; the default is the one
    that passes with the widest margin and the least machinery.
    """

    VARIANTS = ('straight', 'line', 'line_then_default', 'out_y_then_default')
    DEFAULT_VARIANT = 'line_then_default'

    @classmethod
    def resolve_variant(cls, value):
        """The variant a command line value names; the measured default when it is None.

        The CLI cannot declare `choices=VARIANTS` where its other options are declared,
        because the import lives inside the probe's guard, so the same definition is applied
        here instead.
        """
        if value is None:
            return cls.DEFAULT_VARIANT
        if value not in cls.VARIANTS:
            raise ValueError(f'unknown exit path variant {value!r}; '
                             f'expected one of {list(cls.VARIANTS)}')
        return value

    def __init__(self, runtime, anchors, default_arms, variant=DEFAULT_VARIANT):
        if variant not in self.VARIANTS:
            raise ValueError(f'unknown exit path variant {variant!r}; '
                             f'expected one of {list(self.VARIANTS)}')
        self.variant = variant
        self.anchors = {s: np.asarray(v, dtype=float) for s, v in anchors.items()}
        self.default_arms = np.asarray(default_arms, dtype=float)
        self.default_hands = hand_sites_at(runtime.m, runtime.d, runtime.qa,
                                           runtime.arm_ids, self.default_arms,
                                           names=getattr(runtime, 'names', None))
        self.start_hands = None          # measured on the first tick of the exit stage
        self.previous_cart_goal = None
        self.joints_at_ramp_start = None
        self.hands_target = None         # what this tick asked the hands to reach
        self.segment = 0

    def segments(self):
        """The segment list for this variant: magnitudes are a property of the variant."""
        sign = {'left': 1.0, 'right': -1.0}
        outward = {s: self.anchors[s] + np.array([0.0, sign[s] * 0.12, 0.0])
                   for s in self.anchors}
        goal = dict(self.default_hands)
        if self.variant == 'straight':
            # A zero-second ramp means "command the default posture now", which is exactly
            # the old behaviour, kept so the comparison can be re-run from this class.
            return [{'kind': 'joint', 'seconds': 0.0}]
        if self.variant == 'line':
            return [{'kind': 'cart', 'targets': goal, 'seconds': 4.0}]
        if self.variant == 'line_then_default':
            return [{'kind': 'cart', 'targets': goal, 'seconds': EXIT_LINE_SECONDS},
                    {'kind': 'joint', 'seconds': EXIT_RETURN_SECONDS}]
        return [{'kind': 'cart', 'targets': outward, 'seconds': 1.0},
                {'kind': 'cart', 'targets': goal, 'seconds': 1.5},
                {'kind': 'joint', 'seconds': EXIT_RETURN_SECONDS}]

    def command(self, runtime, elapsed):
        """Arm joint targets `elapsed` seconds into the exit stage."""
        if self.start_hands is None:
            self.start_hands = {s: np.asarray(v, dtype=float)
                                for s, v in runtime.snapshot()['hands'].items()}
        segments = self.segments()
        begin = 0.0
        for index, segment in enumerate(segments):
            end = begin + segment['seconds']
            last = index == len(segments) - 1
            if elapsed > end and not last:
                if segment['kind'] == 'cart':
                    self.previous_cart_goal = segment['targets']
                begin = end
                continue
            self.segment = index
            fraction = (1.0 if segment['seconds'] <= 0 else
                        float(np.clip((elapsed - begin) / segment['seconds'], 0.0, 1.0)))
            if segment['kind'] == 'cart':
                goal = segment['targets']
                base = (self.previous_cart_goal if self.previous_cart_goal is not None
                        else self.start_hands)
                self.hands_target = {s: base[s] + fraction * (goal[s] - base[s])
                                     for s in goal}
                joints = runtime.arm_ik(self.hands_target)
                self.joints_at_ramp_start = joints
                return np.asarray(joints, dtype=float)
            if self.joints_at_ramp_start is None:
                self.joints_at_ramp_start = runtime.d.qpos[
                    runtime.qa[runtime.arm_ids]].copy()
            self.hands_target = None
            start = self.joints_at_ramp_start
            return np.asarray(start + fraction * (self.default_arms - start), dtype=float)
        return self.default_arms.copy()


def orientation(quat):
    w,x,y,z=quat
    return math.atan2(2*(w*z+x*y),1-2*(y*y+z*z)), math.acos(np.clip(1-2*(x*x+y*y),-1,1))


def forward_kinematics(model,data,cameras=False):
    """Scratch planning only: update poses/Jacobians without solving dynamics."""
    mujoco.mj_kinematics(model,data)
    mujoco.mj_comPos(model,data)
    if cameras:mujoco.mj_camlight(model,data)


class Runtime:
    def __init__(self,world=None,prefix='',object_body='payload',model=None,data=None):
        """`world` picks the assembled world file; None means the carried-over snapshot.

        The object body keeps its role name (`payload`) whichever model fills it, so the
        judges do not change. Which model that was is recorded per run, so a result can
        never be read as being about the V1 tray when it was measured on the 007 one.

        `model`/`data` let a caller hand in an ALREADY-LOADED world, so this runtime can share one
        `MjModel`/`MjData` with another controller. H3 needs that: the humanoid and the W5
        `LogisticsPlant` must act on the same world in the same continuous run, on the same tray,
        and two separately-loaded models would be two worlds that merely look alike.

        DEFAULTS UNCHANGED. With `model=None` this loads `world` itself and makes its own `MjData`,
        exactly as before, so H1/H2 and every other existing run is bit-for-bit unaffected. The
        runtime still writes its own arm/finger posture either way; it does NOT touch the free
        bodies, so a caller that set the world's home state keeps it -- which is what makes the
        station the WORLD's station rather than the probe's.
        """
        self.policy=T800Policy(ROOT)
        #: How this world spells the robot's names. Identity by default -- see `Names`.
        self.names=Names(prefix,object_body)
        self.shared_world = model is not None
        if self.shared_world:
            self.m=model
            self.d=mujoco.MjData(self.m) if data is None else data
        else:
            path=ROOT/'assets/combined.xml' if world is None else Path(world)
            self.m=mujoco.MjModel.from_xml_path(str(path))
            self.d=mujoco.MjData(self.m)
        self.m.vis.global_.offwidth,self.m.vis.global_.offheight=960,720
        joints=np.array([self.m.joint(self.names.n(n)).id for n in self.policy.joints])
        self.qa,self.va=self.m.jnt_qposadr[joints],self.m.jnt_dofadr[joints]
        self.body_act=np.array([self._actuator(j) for j in joints])
        self.arms={'left':np.arange(13,18),'right':np.arange(18,23)}
        self.arm_ids=np.arange(13,23)
        self.hand_act={side:np.array([self.m.actuator(self.names.n(prefix+p+'a'+str(i))).id
                      for p in ('ff','mf','rf','th') for i in range(4)])
                      for side,prefix in [('left','lh_'),('right','rh_')]}
        self.d.qpos[self.qa]=self.policy.default
        self.arm_target=self.policy.default[self.arm_ids].copy()
        self.arm_weight=0.
        self.hand_target={s:OPEN.copy() for s in self.arms}
        for side,act in self.hand_act.items():
            qadr=self.m.jnt_qposadr[self.m.actuator_trnid[act,0]]
            self.d.qpos[qadr]=OPEN
        self.head_target=np.zeros(2)
        self.command=np.zeros(3)
        self.integral=np.zeros(2)
        self.tick=0
        stand=yaml.safe_load((ROOT/'config/t800/stand.yaml').read_text())
        self.stand_kp=np.concatenate(stand['stiffness'])[:13]
        self.stand_kd=np.concatenate(stand['damping'])[:13]
        self.stand_target=None;self.stand_weight=0.;self.stand_ready=0.
        mujoco.mj_forward(self.m,self.d)

    def _actuator(self,joint):
        ids=np.flatnonzero(self.m.actuator_trnid[:,0]==joint)
        if len(ids)!=1: raise ValueError('Ambiguous body actuator')
        return int(ids[0])

    def feet_loaded(self):
        forces={'L':0.,'R':0.}
        for i in range(self.d.ncon):
            contact=self.d.contact[i]
            geoms=[int(contact.geom1),int(contact.geom2)]
            for g,other in (geoms,geoms[::-1]):
                name=self.m.body(int(self.m.geom_bodyid[g])).name
                if name in tuple(self.names.n(f) for f in ('LINK_FOOT_L','LINK_FOOT_R',
                                                           'LINK_ANKLE_ROLL_L','LINK_ANKLE_ROLL_R')) and self.m.geom_type[other]==mujoco.mjtGeom.mjGEOM_PLANE:
                    force=np.zeros(6);mujoco.mj_contactForce(self.m,self.d,i,force)
                    forces[name[-1]]+=max(0.,float(force[0]))
        return min(forces.values())>10.

    def control(self,command,arms=None,hands=None,head=None,stationary=False,stopping=False,
                base_control=None):
        """Produce a composed vector without writing ctrl or advancing physics.

        A candidate world owner may refuse before/after this computation. Internal
        policy state is not a physical command and cannot bypass the final writer.
        """
        if self.tick%5==0:
            if stationary and self.stand_target is None:
                self.stand_ready=self.stand_ready+.01 if self.feet_loaded() else 0.
                if self.stand_ready>=.2:self.stand_target=self.d.qpos[self.qa[:13]].copy()
            requested=bool(stationary and self.stand_target is not None)
            self.stand_weight+=np.clip(float(requested)-self.stand_weight,-.01,.01)
            if not stationary and self.stand_weight<=0:
                self.stand_target=None;self.stand_ready=0.
            slew=.02 if stopping else .004
            self.command+=np.clip(np.asarray(command)-self.command,-slew,slew)
            self.policy.infer(self.d.qpos[self.qa],self.d.qvel[self.va],self.d.qpos[3:7],
                              self.d.qvel[3:6],self.command,self.d.time)
            self.arm_weight+=np.clip(float(arms is not None)-self.arm_weight,-.005,.005)
            if arms is not None: self.arm_target+=np.clip(np.asarray(arms)-self.arm_target,-.005,.005)
            self.policy.target[self.arm_ids]=(1-self.arm_weight)*self.policy.target[self.arm_ids]+self.arm_weight*self.arm_target
            if head is not None:
                limits=self.m.jnt_range[self.m.actuator_trnid[self.body_act[23:25],0]]
                desired=np.clip(head,limits[:,0],limits[:,1])
                self.head_target+=np.clip(desired-self.head_target,-.004,.004)
            self.policy.target[23:25]=self.head_target
            if hands is not None:
                for side in self.arms:
                    self.hand_target[side]+=np.clip(np.asarray(hands[side])-self.hand_target[side],-.012,.012)
        torque=self.policy.torques(self.d.qpos[self.qa],self.d.qvel[self.va])
        if self.stand_target is not None:
            posture=(self.stand_kp*(self.stand_target-self.d.qpos[self.qa[:13]])
                     -self.stand_kd*self.d.qvel[self.va[:13]]+self.d.qfrc_bias[self.va[:13]])
            torque[:13]=(1-self.stand_weight)*torque[:13]+self.stand_weight*posture
        torque[self.arm_ids]+=self.arm_weight*(self.d.qfrc_bias[self.va[self.arm_ids]]-3*self.d.qvel[self.va[self.arm_ids]])
        limits=self.m.jnt_actfrcrange[self.m.actuator_trnid[self.body_act,0]]
        ctrl=np.array(self.d.ctrl if base_control is None else base_control,dtype=float,copy=True)
        ctrl[self.body_act]=np.clip(torque,limits[:,0],limits[:,1])
        for side,act in self.hand_act.items(): ctrl[act]=self.hand_target[side]
        return ctrl

    def step(self,command,arms=None,hands=None,head=None,stationary=False,stopping=False):
        owner=getattr(self,'step_owner',None)
        def produce():
            background=getattr(self,'background_control',None)
            base=None if background is None else background()
            return self.control(command,arms,hands,head,stationary,stopping,base_control=base)
        if owner is None:
            self.d.ctrl[:]=produce()
            mujoco.mj_step(self.m,self.d)
        else:
            if owner.model is not self.m or owner.data is not self.d:
                raise ValueError('humanoid must share final writer model and data')
            owner.step(produce)
        self.tick+=1

    def support_midpoint(self):
        return np.mean([self.d.body(self.names.n('LINK_FOOT_L')).xpos,
                        self.d.body(self.names.n('LINK_FOOT_R')).xpos],axis=0)

    def hold_command(self,goal,precise=False,brake=False,alignment=False,position=None):
        yaw,_=orientation(self.d.qpos[3:7])
        error=np.asarray(goal[:2])-(self.d.qpos[:2] if position is None else np.asarray(position)[:2])
        c,s=math.cos(yaw),math.sin(yaw)
        local=np.array([c*error[0]+s*error[1],-s*error[0]+c*error[1]])
        if precise and .02<np.linalg.norm(error)<.45:
            integral_limit=.35 if alignment else .12
            self.integral=np.clip(self.integral+.4*local*self.m.opt.timestep,-integral_limit,integral_limit)
        else: self.integral[:]=0
        angle=math.atan2(math.sin(goal[2]-yaw),math.cos(goal[2]-yaw))
        velocity=np.array([c*self.d.qvel[0]+s*self.d.qvel[1],-s*self.d.qvel[0]+c*self.d.qvel[1]])
        translation=local*.8+self.integral
        if brake:translation-=.9*velocity
        limit=.50 if alignment else (.30 if brake else .50)
        return np.clip(np.r_[translation,angle],[-limit,-limit,-.3],[limit,limit,.3])

    def arm_ik(self,targets,seed=None,*,safeguarded=False,posture_reference=None):
        scratch=getattr(self,'_ik_scratch',None)
        if scratch is None:
            scratch=mujoco.MjData(self.m);self._ik_scratch=scratch
        scratch.qpos[:]=self.d.qpos
        if seed is not None: scratch.qpos[self.qa[self.arm_ids]]=seed
        for side,prefix in [('left','lh_'),('right','rh_')]:
            ids=self.arms[side]
            limits=self.m.jnt_range[self.m.actuator_trnid[self.body_act[ids],0]].copy()
            if safeguarded and posture_reference is not None:
                reference=np.asarray(posture_reference)[0:5] if side=='left' else np.asarray(posture_reference)[5:10]
                # This task carries with flexed elbows. Preserve that branch and
                # bound elbow twist around the observed grasp, not a world pose.
                limits[3,1]=min(limits[3,1],-.10)
                limits[4,0]=max(limits[4,0],reference[4]-.25)
                limits[4,1]=min(limits[4,1],reference[4]+.25)
                scratch.qpos[self.qa[ids]]=np.clip(scratch.qpos[self.qa[ids]],limits[:,0],limits[:,1])
            for _ in range(80 if safeguarded else 30):
                forward_kinematics(self.m,scratch)
                error=np.asarray(targets[side])-scratch.site(self.names.n(prefix+'grasp')).xpos
                if np.linalg.norm(error)<.001: break
                jac=np.zeros((3,self.m.nv))
                mujoco.mj_jacSite(self.m,scratch,jac,None,
                                  self.m.site(self.names.n(prefix+'grasp')).id)
                J=jac[:,self.va[ids]]
                dq=J.T@np.linalg.solve(J@J.T+np.eye(3)*.001,error)
                if not safeguarded:
                    scratch.qpos[self.qa[ids]]=np.clip(scratch.qpos[self.qa[ids]]+np.clip(dq,-.08,.08),limits[:,0],limits[:,1])
                    continue
                before=scratch.qpos[self.qa[ids]].copy()
                # Joint-limit clipping can turn a descent direction into an
                # uphill step. Never accumulate those steps at a singularity.
                accepted=False
                # Uniform scaling preserves the descent direction. If a bound
                # blocks the damped Newton step, projected gradient provides a
                # feasible descent direction instead of stopping prematurely.
                gradient=J.T@error/(np.linalg.norm(J,ord='fro')**2+.001)
                for direction in (dq,gradient):
                    step=direction*min(1.,.08/max(np.max(np.abs(direction)),1e-12))
                    for scale in (1.,.5,.25,.125):
                        scratch.qpos[self.qa[ids]]=np.clip(before+scale*step,limits[:,0],limits[:,1])
                        forward_kinematics(self.m,scratch)
                        residual=np.asarray(targets[side])-scratch.site(self.names.n(prefix+'grasp')).xpos
                        if np.linalg.norm(residual)<np.linalg.norm(error)-1e-10:
                            accepted=True;break
                    if accepted:break
                if not accepted:
                    scratch.qpos[self.qa[ids]]=before
                    break
        return scratch.qpos[self.qa[self.arm_ids]].copy()

    def arm_error(self,targets,joints):
        scratch=getattr(self,'_error_scratch',None)
        if scratch is None:
            scratch=mujoco.MjData(self.m);self._error_scratch=scratch
        scratch.qpos[:]=self.d.qpos
        scratch.qpos[self.qa[self.arm_ids]]=joints
        forward_kinematics(self.m,scratch)
        return max(float(np.linalg.norm(scratch.site(self.names.n(prefix+'grasp')).xpos-targets[side]))
                   for side,prefix in [('left','lh_'),('right','rh_')])

    def gaze(self,point):
        # Solve only head joints using the camera optical axis; respect actual limits.
        scratch=getattr(self,'_gaze_scratch',None)
        if scratch is None:
            scratch=mujoco.MjData(self.m);self._gaze_scratch=scratch
        scratch.qpos[:]=self.d.qpos
        qa=self.qa[23:25]
        limits=self.m.jnt_range[self.m.actuator_trnid[self.body_act[23:25],0]]
        def residual():
            forward_kinematics(self.m,scratch,cameras=True)
            camera=scratch.camera(self.names.n('eyes'))
            direction=np.asarray(point)-camera.xpos
            direction/=max(np.linalg.norm(direction),1e-9)
            return -camera.xmat.reshape(3,3)[:,2]-direction
        for _ in range(12):
            error=residual()
            if np.linalg.norm(error)<.005: break
            J=np.zeros((3,2))
            for j,address in enumerate(qa):
                before=scratch.qpos[address]
                scratch.qpos[address]+=1e-4
                J[:,j]=(residual()-error)/1e-4
                scratch.qpos[address]=before
            delta=-np.linalg.solve(J.T@J+np.eye(2)*.01,J.T@error)
            scratch.qpos[qa]=np.clip(scratch.qpos[qa]+np.clip(delta,-.1,.1),limits[:,0],limits[:,1])
        return scratch.qpos[qa].copy()

    def snapshot(self):
        yaw,tilt=orientation(self.d.qpos[3:7])
        return dict(time=float(self.d.time),base=self.d.qpos[:3].tolist(),yaw=yaw,tilt_deg=math.degrees(tilt),
                    feet_midpoint=self.support_midpoint().tolist(),
                    stationary_weight=float(self.stand_weight),
                    arm_joints=self.d.qpos[self.qa[self.arm_ids]].tolist(),
                    arm_command=self.arm_target.tolist(),
                    arm_tracking_error=float(np.max(np.abs(self.arm_target-self.d.qpos[self.qa[self.arm_ids]]))),
                    head=self.d.qpos[self.qa[23:25]].tolist(),
                    hands={s:self.d.site(self.names.n(p+'grasp')).xpos.tolist()
                           for s,p in [('left','lh_'),('right','rh_')]})


# Manifest entries 010 deliberately did NOT carry over from project 007, with the
# reason. They are checked to be ABSENT: if one comes back, the tree has drifted toward
# 007's layout and every assumption about which config is live becomes suspect.
NOT_CARRIED_OVER={
 'config/scene.json':'007 scene config; 010 owns its own world and contracts '
                     '(docs/CONTRACTS.md)',
 'config/task.json':'007 task config; 010 orders live in core/orders (P2), not config/',
}

# Files under assets/ or policies/ that are 010-authored integration artifacts rather
# than copies, so the source manifest legitimately does not list them.
INTEGRATION_ARTIFACTS={'assets/manifest.json','assets/manifest-t800.json'}


def verify_assets(root=None):
    """Check the copied assets against the recorded source hashes.

    Refuses the run on: a listed file that is missing, one whose hash moved, or one that
    is present although 010 declared it not carried over. Files the manifest does not
    account for are returned as `unrecorded` instead of failing the run, because 010
    assembles its own integration files (see INTEGRATION_ARTIFACTS) and a hard failure
    there would be wrong. `unrecorded` is reported so it stays visible.
    """
    root=ROOT if root is None else Path(root)
    manifest=json.loads((root/'assets/manifest.json').read_text(encoding='utf-8'))
    problems=[]
    for name,expected in manifest['sha256'].items():
        path=root/name
        if name in NOT_CARRIED_OVER:
            if path.exists():
                problems.append(f'{name}: present, but 010 declared it not carried over '
                                f'({NOT_CARRIED_OVER[name]})')
            continue
        if not path.is_file():
            problems.append(f'{name}: missing')
            continue
        if hashlib.sha256(path.read_bytes()).hexdigest()!=expected:
            problems.append(f'{name}: hash differs from the recorded source')
    if problems:
        raise ValueError('asset check refused the run:\n  '+ '\n  '.join(problems))
    recorded=set(manifest['sha256'])|set(NOT_CARRIED_OVER)
    unrecorded=[]
    for base in ('assets','policies'):
        folder=root/base
        if not folder.is_dir():
            continue
        for path in sorted(folder.rglob('*')):
            if not path.is_file():
                continue
            rel=str(path.relative_to(root))
            if rel in recorded or rel in INTEGRATION_ARTIFACTS:
                continue
            unrecorded.append(rel)
    manifest=dict(manifest)
    manifest['not_carried_over']=dict(NOT_CARRIED_OVER)
    manifest['unrecorded']=unrecorded
    return manifest
