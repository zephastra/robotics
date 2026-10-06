"""Online PCL floor/bench estimator; calibrated points only, no body identity."""
import hashlib
import json
import subprocess
import time
from pathlib import Path
import numpy as np

ROOT=Path(__file__).resolve().parents[1]


def estimate(world,valid,roi,prior_z,*,rgb=None,cyan_only=False,max_age_s=.5):
    started=time.monotonic()
    binary=ROOT/'runtime/pcl_receiver_plane'
    if not binary.is_file():return dict(status='UNKNOWN',reason='PCL_EXECUTABLE_UNAVAILABLE')
    if world.shape!=(*valid.shape,3) or valid.dtype!=np.bool_ or not np.isfinite(prior_z):
        return dict(status='UNKNOWN',reason='INVALID_CLOUD_SHAPE')
    x0,x1,y0,y1=roi
    mask=valid & np.all(np.isfinite(world),axis=-1)
    mask&=(world[...,0]>=x0)&(world[...,0]<=x1)&(world[...,1]>=y0)&(world[...,1]<=y1)
    mask&=abs(world[...,2]-prior_z)<=.006
    if cyan_only:
        if np.shape(rgb)!=np.shape(world):return dict(status='UNKNOWN',reason='INVALID_RGB_SHAPE')
        color=np.asarray(rgb,float)
        mask&=np.all(np.isfinite(color),axis=-1)&(color[...,1]>.45*color[...,2])&(
            color[...,1]<.9*color[...,2])&(color[...,0]<.5*color[...,2])
    points=world[mask]
    rows=''.join('%.12g %.12g %.12g\n'%tuple(p) for p in points)
    try:
        result=subprocess.run([str(binary)],input=rows,text=True,capture_output=True,
                              check=True,timeout=max_age_s)
        answer=json.loads(result.stdout)
        if not isinstance(answer,dict) or answer.get('status') not in ('RESOLVED','UNKNOWN'):
            return dict(status='UNKNOWN',reason='INVALID_PCL_RESULT')
        age=time.monotonic()-started
        if age>max_age_s:return dict(status='UNKNOWN',reason='STALE_PCL_PLANE',age_s=age)
        z=answer.get('floor_z_m')
        if answer.get('status')=='RESOLVED' and (
                not isinstance(z,(int,float)) or not np.isfinite(z) or abs(z-prior_z)>.006):
            return dict(status='UNKNOWN',reason='PCL_RESULT_OUTSIDE_CALIBRATION')
        answer['backend']='native PCL VoxelGrid + SACSegmentation'
        answer['age_s']=age
        answer['binary_sha256']=hashlib.sha256(binary.read_bytes()).hexdigest()
        return answer
    except (subprocess.SubprocessError,ValueError,OSError):
        return dict(status='UNKNOWN',reason='PCL_EXECUTION_OR_FORMAT_FAILURE')
