"""Bounded, forward-facing bilateral stance search in a known fixture workspace.

The copied collision model uses the explicit source map and visual destination.
Payload simulator pose is never queried. Sampled geometric checks are not a
continuous collision proof or a dynamic balance guarantee.
"""
import copy
import json
import mujoco
import numpy as np
from .runtime import ROOT,OPEN,GRASP,forward_kinematics,withdrawal_waypoints


def candidate_positions(center,current,operation='grasp'):
    result=[]
    if .20<=center[0]-current[0]<=.40 and abs(center[1]-current[1])<=.08:
        result.append(np.asarray(current[:2]).copy())
    distances=(.24,.28,.32,.36,.40,.44) if operation=='place' else (.24,.28,.32,.36)
    sides=[0.,-.04,.04]
    if operation=='place' and .04<abs(current[1]-center[1])<=.15:
        # Also check a forward-only correction from the measured lateral pose.
        # The same IK/clearance checks decide feasibility; no tolerance expands.
        sides.append(float(current[1]-center[1]))
    result.extend(np.array([center[0]-distance,center[1]+side])
                  for distance in distances for side in sides)
    return result


class StancePlanner:
    def __init__(self,r,destination):
        self.r=copy.copy(r)
        self.r._ik_scratch=self.r._error_scratch=self.r._gaze_scratch=None
        self.r.m=copy.copy(r.m)
        self.r.d=mujoco.MjData(self.r.m)
        self.r.d.qpos[:]=r.d.qpos
        known=json.loads((ROOT/'config/workcell.json').read_text())
        source=np.asarray(known['source_table_center'])
        target=np.asarray(destination)-np.array([0.,0.,.007])
        # Destination observation is the table top, inferred from its marker.
        for label,center in [('source',source),('destination',target)]:
            self.r.m.geom(label+'_table').pos[:]=center
            self.r.m.geom(label+'_leg').pos[:2]=center[:2]+[.08,0.]
        for name in ('destination_marker_plate','destination_marker_post'):
            self.r.m.geom(name).pos[:2]=target[:2]+[0.,.75]
        self.stations=[self.r.m.geom(n).id for n in ('source_table','source_leg','destination_table','destination_leg','destination_marker_plate','destination_marker_post')]
        self.robot=[g for g in range(r.m.ngeom) if r.m.geom_bodyid[g]!=0
                    and r.m.body(int(r.m.geom_bodyid[g])).name!='payload'
                    and (r.m.geom_contype[g] or r.m.geom_conaffinity[g])]
        self.original=r.d.qpos.copy()

    def separation_bound(self,a,b):
        """World AABB separation is a conservative lower bound, including for meshes."""
        r=self.r;boxes=[]
        for g in (a,b):
            rotation=r.d.geom_xmat[g].reshape(3,3)
            local=r.m.geom_aabb[g]
            boxes.append((r.d.geom_xpos[g]+rotation@local[:3],np.abs(rotation)@local[3:]))
        gap=np.maximum(np.abs(boxes[0][0]-boxes[1][0])-boxes[0][1]-boxes[1][1],0.)
        return float(np.linalg.norm(gap))

    def clearance(self):
        r=self.r
        forward_kinematics(r.m,r.d)
        body=hand=.2
        self.closest={}
        for g in self.robot:
            is_hand=r.m.body(int(r.m.geom_bodyid[g])).name.startswith(('lh_','rh_'))
            for station in self.stations:
                bound=self.separation_bound(g,station)
                distance=bound if bound>.03 else max(bound,mujoco.mj_geomDistance(r.m,r.d,g,station,.2,None))
                key='hand' if is_hand else 'body'
                if distance<(hand if is_hand else body):
                    self.closest[key]=[g,r.m.body(int(r.m.geom_bodyid[g])).name,r.m.geom(station).name,float(distance)]
                if is_hand:hand=min(hand,float(distance))
                else:body=min(body,float(distance))
        return body,hand

    def evaluate(self,xy,targets,seed,operation):
        r=self.r;r.d.qpos[:]=self.original;r.d.qpos[:2]=xy
        # Translation candidates retain the measured body orientation. Resetting
        # it to identity silently evaluated a different bilateral workspace.
        joints=np.asarray(seed).copy();minimum=.2;error=0.
        solve_options=dict(safeguarded=True,posture_reference=np.asarray(seed).copy()) if operation=='place' else {}
        stages=[targets,{s:p+[0,0,.12] for s,p in targets.items()}] if operation=='grasp' else [targets,*withdrawal_waypoints(targets)]
        for stage_index,stage in enumerate(stages):
            solution=r.arm_ik(stage,joints,**solve_options)
            error=max(error,r.arm_error(stage,solution))
            if error>.012:return dict(feasible=False,reason='BILATERAL_REACH',error=error,stage_index=stage_index)
            for fraction in np.linspace(0,1,6):
                r.d.qpos[r.qa[r.arm_ids]]=joints+fraction*(solution-joints)
                body,hand=self.clearance();minimum=min(minimum,body)
                if body<.025 or hand<.003:
                    return dict(feasible=False,reason='FIXTURE_CLEARANCE',body_clearance=body,hand_clearance=hand,closest=self.closest,fraction=float(fraction))
            joints=solution
            # Check closing/releasing fingers before the next arm segment.
            if stage_index==0:
                desired=GRASP if operation=='grasp' else OPEN
                addresses={s:r.m.jnt_qposadr[r.m.actuator_trnid[act,0]] for s,act in r.hand_act.items()}
                starts={s:r.d.qpos[q].copy() for s,q in addresses.items()}
                for fraction in np.linspace(0,1,6):
                    if operation=='place':
                        release_targets={s:p+np.array([0,sign*.025,.015])*fraction
                                         for (s,p),sign in zip(stage.items(),(1,-1))}
                        joints=r.arm_ik(release_targets,joints,**solve_options)
                        error=max(error,r.arm_error(release_targets,joints))
                        if error>.012:return dict(feasible=False,reason='RELEASE_REACH',error=error)
                        r.d.qpos[r.qa[r.arm_ids]]=joints
                    for side,q in addresses.items():r.d.qpos[q]=starts[side]+fraction*(desired-starts[side])
                    body,hand=self.clearance();minimum=min(minimum,body)
                    if body<.025 or hand<.003:
                        return dict(feasible=False,reason='FINGER_SWEEP_CLEARANCE',body_clearance=body,hand_clearance=hand)
        return dict(feasible=True,error=error,body_clearance=minimum)

    def select(self,center,axis,seed,operation,hand_offsets=None):
        targets={s:np.asarray(center)+(np.asarray(hand_offsets[s]) if hand_offsets is not None
                 else sign*np.asarray(axis)*.30) for s,sign in [('left',1),('right',-1)]}
        candidates=[]
        for xy in candidate_positions(center,self.original[:2],operation):
            result=self.evaluate(xy,targets,seed,operation)
            result['xy']=xy.tolist()
            if result['feasible']:
                weight=.02 if operation=='place' else .002
                # A low-residual arm solution is preferable to a barely reachable
                # far stance. Clearance remains a hard constraint above.
                reach_cost=20.*result['error'] if operation=='place' else 0.
                result['score']=float(np.linalg.norm(xy-self.original[:2])+weight/(result['body_clearance']+.01)+reach_cost)
            candidates.append(result)
        feasible=[c for c in candidates if c['feasible']]
        if operation=='place' and hand_offsets is not None:
            for candidate in feasible:
                x,y=candidate['xy']
                neighbors=sum(any(abs(other['xy'][0]-(x+dx))<1e-6
                                  and abs(other['xy'][1]-y)<1e-6 for other in feasible)
                              for dx in (-.04,.04))
                # A re-alignment must not aim at the far reach boundary again.
                # Prefer sampled interior positions that tolerate longitudinal
                # settling in either direction. Hard feasibility is unchanged.
                candidate['longitudinal_neighbors']=neighbors
                candidate['score']+=.2*(2-neighbors)
        chosen=min(feasible,key=lambda c:c['score']) if feasible else None
        return dict(operation=operation,candidates=candidates,selected=chosen,
                    geometry_source='known source map + visually located destination fixture',
                    scope='measured orientation, sampled arm paths; no global path or balance proof')
