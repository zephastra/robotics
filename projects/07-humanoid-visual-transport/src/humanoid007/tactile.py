"""Ideal simulated contact sensing; no object pose is exposed to the controller."""
import mujoco
import numpy as np


def sense(r):
    result={'left':0.,'right':0.,'source':False,'destination':False}
    payload=r.m.body('payload').id
    for i in range(r.d.ncon):
        contact=r.d.contact[i]
        body1,body2=r.m.geom_bodyid[[contact.geom1,contact.geom2]]
        if payload not in (body1,body2): continue
        other=contact.geom2 if body1==payload else contact.geom1
        name=r.m.body(r.m.geom_bodyid[other]).name
        force=np.zeros(6)
        mujoco.mj_contactForce(r.m,r.d,i,force)
        if name.startswith('lh_'): result['left']+=max(float(force[0]),0.)
        if name.startswith('rh_'): result['right']+=max(float(force[0]),0.)
        for station in ('source','destination'):
            if r.m.geom(other).name==station+'_table' and force[0]>.01: result[station]=True
    return result
