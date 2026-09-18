"""Visual task logic: scene coordinates and payload truth are deliberately absent."""
import json
import math
import numpy as np
from .runtime import ROOT,OPEN,GRASP,orientation,withdrawal_waypoints
from .vision import detect
from .stance import StancePlanner

CFG=json.loads((ROOT/'config/task.json').read_text())


def corridor_recovery_ready(origin,goal,position):
    """Intermediate waypoint only: reached its cross-section inside a bounded corridor."""
    segment=np.asarray(goal[:2])-np.asarray(origin[:2])
    length=np.linalg.norm(segment)
    if length<.2:return False
    direction=segment/length
    remaining=np.asarray(goal[:2])-np.asarray(position[:2])
    along=float(remaining@direction)
    cross=abs(float(remaining[0]*direction[1]-remaining[1]*direction[0]))
    return -.05<=along<=.025 and cross<=.14


class Mission:
    def __init__(self,r,mode='transport',stance='auto'):
        self.r,self.mode=r,mode
        self.phase='STAND'; self.since=0.; self.stable_since=None
        self.result=None;self.reason='';self.events=[]
        self.arm=None;self.hands={s:OPEN.copy() for s in ('left','right')}
        self.head=np.array([.1,0.]);self.goal=np.zeros(3)
        self.box=self.destination=None
        self.destination_marker=None
        self.next_frame=0.;self.last_box_time=-1.;self.loss_since=None
        self.box_center=None;self.axis=np.array([0.,1.,0.])
        self.targets=None;self.retries=0
        self.carry_origin=None;self.waypoint_recoveries=0
        self.stance_mode=stance;self.stance_plans=[];self.approach_attempts=0
        self.hold_anchor=None
        self.alignment_support_offset=None

    def choose_stance(self,center,operation,hand_offsets=None):
        planner=StancePlanner(self.r,self.destination.center)
        seed=self.r.policy.default[self.r.arm_ids] if self.arm is None else self.arm
        plan=planner.select(center,self.axis,seed,operation,hand_offsets=hand_offsets)
        plan['time']=float(self.r.d.time);self.stance_plans.append(plan)
        return plan['selected']

    def enter(self,phase):
        self.phase,self.since,self.stable_since=phase,float(self.r.d.time),None
        if phase=='CARRY':self.carry_origin=self.r.d.qpos[:2].copy()
        if phase=='CARRY' and self.retries>0 and self.stance_mode=='auto':
            self.alignment_support_offset=self.r.d.qpos[:2]-self.r.support_midpoint()[:2]
        if phase in ('CARRY','APPROACH'):self.hold_anchor=None
        self.events.append({'time':self.since,'phase':phase})

    def finish(self,result,reason):
        self.result,self.reason=result,reason
        self.enter(result)

    def stable(self,condition,duration):
        now=self.r.d.time
        if not condition: self.stable_since=None
        elif self.stable_since is None: self.stable_since=now
        return self.stable_since is not None and now-self.stable_since>=duration

    def perceive(self,frame):
        box=detect(frame,'box')
        target=detect(frame,'destination')
        if box is not None:
            self.box=box
            self.last_box_time=frame.time
        if target is not None:
            self.destination_marker=target.center.copy()
            target.center=target.center-CFG['destination_marker_offset_y']*target.long_axis
            self.destination=target

    def update(self,touch):
        if self.result: return
        r=self.r;now=float(r.d.time);age=now-self.since
        if not np.isfinite(r.d.qpos).all() or not np.isfinite(r.d.qvel).all():
            self.finish('FAILED','NONFINITE_STATE');return
        if r.d.qpos[2]<.65 or orientation(r.d.qpos[3:7])[1]>math.radians(45):
            self.finish('FAILED','BODY_FALL');return
        if age>(45. if self.phase=='CARRY' else CFG['phase_timeout']):
            self.finish('FAILED','PHASE_TIMEOUT_'+self.phase);return
        fresh=self.box is not None and now-self.box.time<CFG['max_detection_age']
        speed=np.linalg.norm(r.d.qvel[:2])
        if self.hold_anchor is not None and np.linalg.norm(r.d.qpos[:2]-self.hold_anchor[:2])>.12:
            self.finish('FAILED','STATIONARY_BASE_DRIFT');return
        if self.phase in ('LIFT','HOLD','CARRY','STOP','INSPECT_DESTINATION','INSPECT_BOX','LOWER'):
            if min(touch['left'],touch['right'])<.04:
                if self.loss_since is None: self.loss_since=now
                if now-self.loss_since>.6:
                    self.finish('FAILED','BILATERAL_CONTACT_LOST');return
            else: self.loss_since=None
        if self.phase=='STAND':
            if now>3 and self.stable(speed<.08,.8): self.enter('SEARCH')
        elif self.phase=='SEARCH':
            self.head=np.array([.45 if int(age/2)%2 else 0., .25*math.sin(age*.6)])
            if fresh and self.destination is not None:
                self.enter('OBSERVE')
        elif self.phase=='OBSERVE':
            if self.box is not None: self.head=r.gaze(self.box.center)
            if fresh and age>1 and self.stable(speed<.08,.5):
                self.box_center=self.box.center-self.box.normal*CFG['marker_height_above_center']
                self.axis=self.box.long_axis.copy()
                if abs(self.axis[2])>.1:
                    self.finish('FAILED','BOX_TOO_TILTED');return
                if self.mode=='vision':
                    self.finish('COMPLETED','VISUAL_LOCALIZATION_ONLY');return
                # Standing region derived from the measured object, not scene truth.
                delta=self.box_center[:2]-r.d.qpos[:2]-np.array([.23,0.])
                if self.stance_mode=='auto':
                    selected=self.choose_stance(self.box_center,'grasp')
                    if selected is None:
                        self.finish('FAILED','NO_FEASIBLE_GRASP_STANCE');return
                    delta=np.asarray(selected['xy'])-r.d.qpos[:2]
                if np.linalg.norm(delta)>(1e-6 if self.stance_mode=='auto' else .07):
                    self.approach_attempts+=1
                    if self.approach_attempts>2:
                        self.finish('FAILED','GRASP_STANCE_ALIGNMENT_LIMIT');return
                    self.goal[:2]=r.d.qpos[:2]+delta
                    self.enter('APPROACH')
                elif self.mode=='vision': self.finish('COMPLETED','VISUAL_LOCALIZATION_ONLY')
                else:
                    self.targets={s:self.box_center+sign*self.axis*CFG['handle_y']
                                  for s,sign in [('left',1),('right',-1)]}
                    self.arm=r.policy.default[r.arm_ids].copy()
                    self.enter('REACH')
        elif self.phase=='APPROACH':
            if self.box is not None: self.head=r.gaze(self.box.center)
            if self.stable(np.linalg.norm(r.d.qpos[:2]-self.goal[:2])<.06 and speed<.08,.8):
                self.goal[:2]=r.d.qpos[:2]
                self.enter('OBSERVE')
        elif self.phase=='REACH':
            self.head=r.gaze(self.box.center)
            if not fresh:
                self.finish('FAILED','TARGET_LOST_BEFORE_GRASP');return
            self.arm=r.arm_ik(self.targets,self.arm)
            errors=[np.linalg.norm(r.d.site(p+'grasp').xpos-self.targets[s])
                    for s,p in [('left','lh_'),('right','rh_')]]
            if self.stable(max(errors)<.012 and speed<.08,.6): self.enter('CLOSE')
        elif self.phase=='CLOSE':
            self.arm=r.arm_ik(self.targets,self.arm)
            self.hands={s:OPEN+min(age/3,1)*(GRASP-OPEN) for s in self.hands}
            if age>4 and self.stable(min(touch['left'],touch['right'])>.15,.3):
                self.lift_start={s:p.copy() for s,p in self.targets.items()}
                self.initial_box_z=float(self.box_center[2])
                self.enter('LIFT')
            elif age>7: self.finish('FAILED','NO_BILATERAL_GRASP')
        elif self.phase=='LIFT':
            self.targets={s:p+np.array([0,0,CFG['lift_height']*min(age/4,1)]) for s,p in self.lift_start.items()}
            self.arm=r.arm_ik(self.targets,self.arm)
            if fresh: self.head=r.gaze(self.box.center)
            if age>5:
                if fresh and self.box.center[2]-CFG['marker_height_above_center']>self.initial_box_z+.08 and not touch['source']:
                    self.enter('HOLD')
                else: self.finish('FAILED','LIFT_NOT_VISUALLY_CONFIRMED')
        elif self.phase=='HOLD':
            self.arm=r.arm_ik(self.targets,self.arm)
            if age>1:
                if self.mode=='lift': self.finish('COMPLETED','VISUAL_BIMANUAL_LIFT')
                else:
                    self.arm=r.d.qpos[r.qa[r.arm_ids]].copy()
                    self.carry_offset=np.mean([r.d.site(p+'grasp').xpos for p in ('lh_','rh_')],axis=0)-r.d.qpos[:3]
                    final_xy=self.destination.center[:2]-self.carry_offset[:2]
                    if self.stance_mode=='auto':
                        selected=self.choose_stance(self.destination.center+self.destination.normal*CFG['known_box_halfheight'],'place')
                        if selected is None:
                            self.finish('FAILED','NO_FEASIBLE_PLACE_STANCE');return
                        final_xy=np.asarray(selected['xy'])
                        # Loaded locomotion stops in a region, not at an exact IK
                        # pose. Approach from the clear side, then validate the
                        # measured stopping pose before allowing manipulation.
                        final_xy=final_xy-np.array([.08,0.])
                        self.stance_plans[-1]['approach_xy']=final_xy.tolist()
                    # Leave the source workstation sideways before moving forward.
                    # A straight path would walk the robot into the source table.
                    side_y=min(float(r.d.qpos[1]),float(final_xy[1]))-.65
                    self.route=[np.array([r.d.qpos[0],side_y]),np.array([final_xy[0],side_y]),final_xy]
                    self.goal[:2]=self.route.pop(0)
                    self.enter('CARRY')
        elif self.phase=='CARRY':
            self.head=r.gaze(self.destination.center)
            # Waypoints define a corridor, not manipulation stops. Final STOP
            # separately verifies low base speed before any placement motion.
            tolerance=.03 if self.stance_mode=='auto' and self.retries>0 and not self.route else .08
            measured=r.d.qpos[:2]
            if self.stance_mode=='auto' and self.retries>0 and self.alignment_support_offset is not None:
                measured=r.support_midpoint()[:2]+self.alignment_support_offset
            reached=np.linalg.norm(measured-self.goal[:2])<tolerance
            if self.stance_mode=='auto' and not self.route:reached=reached and speed<.08
            recovery=(not reached and bool(self.route) and self.carry_origin is not None
                      and age>8 and speed<.03
                      and corridor_recovery_ready(self.carry_origin,self.goal,r.d.qpos))
            if self.stable(reached or recovery,.5 if recovery else .1):
                if recovery:
                    self.waypoint_recoveries+=1
                    if self.waypoint_recoveries>2:
                        self.finish('FAILED','WAYPOINT_RECOVERY_LIMIT');return
                    self.events.append({'time':now,'phase':'CARRY','recovery':'CORRIDOR_CROSS_SECTION',
                                        'attempt':self.waypoint_recoveries})
                if self.route:
                    self.goal[:2]=self.route.pop(0)
                    self.enter('CARRY')
                else:
                    if self.stance_mode=='legacy':self.goal[:2]=r.d.qpos[:2]
                    self.enter('STOP')
        elif self.phase=='STOP':
            self.head=r.gaze(self.destination_marker)
            if age>1 and self.stable(speed<.08,.6):
                if self.stance_mode=='auto':
                    # Only anchor after measured stopping, never at waypoint entry.
                    # The planned goal stays intact; arrival geometry is checked next.
                    self.hold_anchor=np.r_[r.d.qpos[:2],orientation(r.d.qpos[3:7])[0]]
                self.enter('INSPECT_DESTINATION')
        elif self.phase=='INSPECT_DESTINATION':
            self.head=r.gaze(self.destination_marker)
            if self.stable(now-self.destination.time<.5 and age>1.,.5):
                self.placement_target=self.destination.center+self.destination.normal*CFG['known_box_halfheight']
                self.enter('INSPECT_BOX')
        elif self.phase=='INSPECT_BOX':
            # Observe sequentially: the box can occlude the placement surface.
            # Static station memory is explicit; no hidden pose refresh.
            center=np.mean([r.d.site(p+'grasp').xpos for p in ('lh_','rh_')],axis=0)
            self.head=r.gaze(center)
            if fresh and age>1 and self.stable(speed<.08,.5):
                target=self.placement_target
                box=self.box.center-self.box.normal*CFG['marker_height_above_center']
                start={s:r.d.site(p+'grasp').xpos.copy() for s,p in [('left','lh_'),('right','rh_')]}
                delta=target-box+np.array([0,0,.004])
                final_targets={s:p+delta for s,p in start.items()}
                solution=r.arm_ik(final_targets,self.arm)
                clearance_ok=True
                if self.stance_mode=='auto':
                    planner=StancePlanner(r,self.destination.center)
                    check=planner.evaluate(r.d.qpos[:2],final_targets,self.arm,'place')
                    self.stance_plans.append(dict(operation='arrival_check',time=now,**check))
                    clearance_ok=check['feasible']
                # Check the actual joint-limited bilateral workspace, not just base
                # error. Retain a bounded horizontal reach and physical contact checks.
                if not clearance_ok or np.linalg.norm(box[:2]-target[:2])>.18 or r.arm_error(final_targets,solution)>.01:
                    self.retries+=1
                    if self.retries>2:self.finish('FAILED','PLACEMENT_ALIGNMENT_FAILED');return
                    self.goal[:2]=r.d.qpos[:2]+target[:2]-box[:2]
                    if self.stance_mode=='auto':
                        offsets={s:p-box+np.array([0,0,.004]) for s,p in start.items()}
                        selected=self.choose_stance(target,'place',hand_offsets=offsets)
                        if selected is None:
                            self.finish('FAILED','NO_FEASIBLE_PLACE_STANCE');return
                        self.goal[:2]=selected['xy']
                    self.route=[]
                    self.enter('CARRY');return
                self.lower_start=start
                self.lower_delta=delta
                self.manipulation_reference=self.arm.copy()
                self.enter('LOWER')
        elif self.phase=='LOWER':
            self.arm=r.arm_ik({s:p+min(age/4,1)*self.lower_delta for s,p in self.lower_start.items()},self.arm,safeguarded=self.stance_mode=='auto',posture_reference=self.manipulation_reference)
            if fresh:self.head=r.gaze(self.box.center)
            if age>5 and touch['destination']:
                self.release_start={s:h.copy() for s,h in self.hands.items()}
                self.release_arm_start={s:r.d.site(p+'grasp').xpos.copy() for s,p in [('left','lh_'),('right','rh_')]}
                self.enter('RELEASE')
        elif self.phase=='RELEASE':
            self.hands={s:h+min(age/4,1)*(OPEN-h) for s,h in self.release_start.items()}
            if self.stance_mode=='auto':
                self.arm=r.arm_ik({s:p+np.array([0,sign*.025,.015])*min(age/4,1)
                                  for (s,p),sign in zip(self.release_arm_start.items(),(1,-1))},self.arm,safeguarded=True,posture_reference=self.manipulation_reference)
            if age>5:
                self.retract_start={s:r.d.site(p+'grasp').xpos.copy() for s,p in [('left','lh_'),('right','rh_')]}
                if self.stance_mode=='auto':
                    self.retract_clear,self.retract_target=withdrawal_waypoints(self.release_arm_start)
                else:
                    self.retract_target={s:p+np.array([0,sign*.08,.14])
                                         for (s,p),sign in zip(self.retract_start.items(),(1,-1))}
                self.enter('RETRACT')
        elif self.phase=='RETRACT':
            start,end,fraction=self.retract_start,self.retract_target,min(age/4,1)
            if self.stance_mode=='auto':
                start,end,fraction=(self.retract_start,self.retract_clear,min(age/2,1)) if age<2 else (self.retract_clear,self.retract_target,min((age-2)/2,1))
            self.arm=r.arm_ik({s:p+(end[s]-p)*fraction for s,p in start.items()},self.arm,safeguarded=self.stance_mode=='auto',posture_reference=self.manipulation_reference)
            if age>5:self.enter('VERIFY')
        elif self.phase=='VERIFY':
            if self.box is not None:self.head=r.gaze(self.box.center)
            good=(fresh and touch['destination'] and touch['left']<.02 and touch['right']<.02
                  and np.linalg.norm(self.box.center[:2]-self.placement_target[:2])<.06)
            if self.stable(good,1.):self.finish('COMPLETED','VISUAL_BIMANUAL_TRANSPORT')
