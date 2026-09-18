"""Per-physics-step workstation contact diagnostics; never used as object pose input."""
import mujoco
import numpy as np


class ContactMonitor:
    def __init__(self):
        self.pairs={}

    def sample(self,r,phase):
        active={}
        for i in range(r.d.ncon):
            c=r.d.contact[i]
            ids=[int(c.geom1),int(c.geom2)]
            names=[r.m.geom(g).name for g in ids]
            bodies=[r.m.body(int(r.m.geom_bodyid[g])).name for g in ids]
            station=next((n for n in names if n.startswith(('source_','destination_'))),None)
            if station is None or 'payload' in bodies:continue
            index=names.index(station);other=1-index
            if bodies[other]=='world':continue
            force=np.zeros(6);mujoco.mj_contactForce(r.m,r.d,i,force)
            key=station+' / '+bodies[other]
            active[key]=active.get(key,0.)+max(0.,float(force[0]))
        for key,force in active.items():
            if force<=.01:continue
            item=self.pairs.setdefault(key,dict(peak_normal_force_n=0.,contact_seconds=0.,
                normal_impulse_ns=0.,first_time=float(r.d.time),phases=[]))
            dt=float(r.m.opt.timestep)
            item['peak_normal_force_n']=max(item['peak_normal_force_n'],force)
            item['contact_seconds']+=dt
            item['normal_impulse_ns']+=force*dt
            if phase not in item['phases']:item['phases'].append(phase)

    def summary(self):
        return dict(sample='every physics step',force_threshold_n=.01,
                    contact_free=not self.pairs,pairs=self.pairs)
