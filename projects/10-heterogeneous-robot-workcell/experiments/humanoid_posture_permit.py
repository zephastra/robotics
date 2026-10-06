"""Continuous H1 BODY_FALL protection; self-state only, simulation freeze only."""
import math
import numpy as np
from workcell.runtime_permit import RuntimePermit


def posture_status(base_z,quaternion):
    q=np.asarray(quaternion,float)
    if q.shape!=(4,) or not math.isfinite(base_z) or not np.all(np.isfinite(q)) or not np.isclose(np.linalg.norm(q),1.,atol=1e-5):
        return dict(allowed=False,reason_code='POSTURE_UNKNOWN')
    # Same orientation definition and limits as probe_h_w5 BODY_FALL.
    _,x,y,_=q
    tilt=math.degrees(math.acos(float(np.clip(1-2*(x*x+y*y),-1,1))))
    return dict(allowed=base_z>=.65 and tilt<=35.,reason_code='BODY_FALL' if base_z<.65 or tilt>35 else None,
                base_z_m=float(base_z),tilt_deg=tilt)


class HumanoidPosturePermit(RuntimePermit):
    def __init__(self,observe,*,model,data,**kwargs):
        super().__init__(observe,**kwargs)
        self.address=int(model.jnt_qposadr[model.joint('h_base_free').id])
        self.data=data
        self.last_posture=None

    def check(self):
        permission=super().check()
        if not permission['allowed']:return permission
        adr=self.address
        self.last_posture=posture_status(float(self.data.qpos[adr+2]),self.data.qpos[adr+3:adr+7])
        if not self.last_posture['allowed']:
            return self.stop(self.last_posture['reason_code'],str(self.last_posture))
        return permission
