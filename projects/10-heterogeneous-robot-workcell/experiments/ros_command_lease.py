"""Controller-message lease, not a lease on repeated bridge transmissions.

Both wall and simulation age must be fresh. A republished old stamped command
cannot renew authorization. No ROS imports: these contracts are tested offline.
"""
import math
import time


class ControllerLease:
    def __init__(self, *, ttl_s=.5, v_max=.6, w_max=1.2, clock=time.monotonic):
        if not all(math.isfinite(x) and x>0 for x in (ttl_s,v_max,w_max)) or ttl_s>.5:
            raise ValueError('finite positive limits; maximum lease 0.5s')
        self.ttl,self.v_max,self.w_max,self.clock=ttl_s,v_max,w_max,clock
        self.accepted=None
        self.latest_stamp=None
        self.last_sim=None

    def offer(self,v,w,stamp_s,sim_s):
        if not all(math.isfinite(float(x)) for x in (v,w,stamp_s,sim_s)):
            return False,'NONFINITE_COMMAND'
        age=sim_s-stamp_s
        if age<0 or age>self.ttl:
            return False,'COMMAND_TIMESTAMP_NOT_FRESH'
        if self.latest_stamp is not None and stamp_s<=self.latest_stamp:
            return False,'COMMAND_TIMESTAMP_REPLAYED'
        if abs(v)>self.v_max or abs(w)>self.w_max:
            return False,'COMMAND_LIMIT_EXCEEDED'
        self.latest_stamp=stamp_s
        self.accepted=(float(v),float(w),float(stamp_s),self.clock())
        return True,'ACCEPTED'

    def value(self,sim_s):
        if not math.isfinite(sim_s):raise ValueError('nonfinite sim time')
        if self.last_sim is not None and sim_s<self.last_sim:
            self.accepted=None
            raise RuntimeError('SIM_TIME_REVERSED_REAUTHORIZATION_REQUIRED')
        self.last_sim=sim_s
        if self.accepted is None:return None
        v,w,stamp,wall=self.accepted
        wall_age=self.clock()-wall
        if not 0<=wall_age<=self.ttl or not 0<=sim_s-stamp<=self.ttl:return None
        return v,w,stamp
