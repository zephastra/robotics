"""Diagnostic-only self-contact logging; does not change control or collision settings."""
from pathlib import Path
from datetime import datetime,timezone
import sys,json
import numpy as np
import mujoco

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src'))
from humanoid007 import app

original=app.Runtime.step
pairs={}


def step(r,*args,**kwargs):
    original(r,*args,**kwargs)
    for i in range(r.d.ncon):
        c=r.d.contact[i]
        bodies=[r.m.body(int(r.m.geom_bodyid[g])).name for g in (c.geom1,c.geom2)]
        if any(b in ('world','payload') for b in bodies) or bodies[0]==bodies[1]:continue
        force=np.zeros(6);mujoco.mj_contactForce(r.m,r.d,i,force)
        if force[0]<=1.:continue
        key=' / '.join(sorted(bodies))
        record=pairs.setdefault(key,dict(peak_n=0.,peak_time=0.,seconds=0.))
        if force[0]>record['peak_n']:
            record['peak_n']=float(force[0]);record['peak_time']=float(r.d.time)
        record['seconds']+=float(r.m.opt.timestep)


if __name__=='__main__':
    app.Runtime.step=step
    result=app.main()
    output=ROOT/'reports'/('self-contacts-'+datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S%fZ')+'.json')
    output.write_text(json.dumps(pairs,indent=2)+'\n')
    print('Self-contact diagnostic:',output)
    for key,value in sorted(pairs.items(),key=lambda p:p[1]['peak_n'],reverse=True)[:20]:print(key,value)
    raise SystemExit(result)
