"""ROS IPC on an existing shared world: no reset, extra physics, or pose on wire.

This is an integration building block, NOT an independently running Nav2 system.
The caller owns WorldOwner, the shared time and stationary controllers. State is
wheel rates and actual scene rays only; truth remains in independent reports.
"""
import math
import socket
import time
import numpy as np
import mujoco
from workcell_ipc import encode, state
from workcell_ipc.gate import CommandGate


def bounded_ranges(distances, geom_ids, minimum, maximum):
    """MuJoCo uses -1 for no hit; never turn no hit into a near obstacle."""
    distances=np.asarray(distances,float)
    geom_ids=np.asarray(geom_ids)
    if distances.shape!=geom_ids.shape:raise ValueError('ray result shape mismatch')
    if not 0<minimum<maximum or not math.isfinite(maximum):
        raise ValueError('invalid scan range')
    hit=(geom_ids>=0)&np.isfinite(distances)&(distances>=0)
    return np.clip(np.where(hit,distances,maximum),minimum,maximum)


class WallGuardedCommand:
    def __init__(self, config, clock=time.monotonic):
        self.clock=clock
        self.wall_limit=float(config['silence_s'])
        if not math.isfinite(self.wall_limit) or not 0<self.wall_limit<=.5:
            raise ValueError('wall silence limit must be finite and at most 0.5s')
        self.gate=CommandGate(ttl_s=config['ttl_s'],silence_s=config['silence_s'],
            v_max=config['v_max_mps'],w_max=config['w_max_radps'])
        self.last_wall=None
        self.last_sim=None

    def offer(self, raw, sim_s):
        accepted,reason=self.gate.offer(raw,sim_s)
        if accepted:self.last_wall=self.clock()
        return accepted,reason

    def step(self, sim_s):
        if self.last_sim is not None and sim_s<self.last_sim:
            raise RuntimeError('SIM_TIME_REVERSED_REAUTHORIZATION_REQUIRED')
        self.last_sim=sim_s
        now=self.clock()
        if self.last_wall is None or now-self.last_wall>self.wall_limit or now<self.last_wall:
            return 0.,0.,'SILENT','WALL_HEARTBEAT_EXPIRED'
        return self.gate.step(sim_s)


class JointWorldNavIO:
    def __init__(self,plant,config,*,prefix='n_',clock=time.monotonic):
        if prefix!='n_':
            raise ValueError('AMR2 bridge not yet implemented; do not imply general robot IDs')
        if config['ipc']['host']!='127.0.0.1' or config['ros']['domain_id']==42:
            raise ValueError('010 bridge requires loopback and a non-009 ROS domain')
        if plant.step_owner is None:
            raise ValueError('shared physics owner must be attached before navigation')
        if getattr(plant, 'heading_feedback_installed', False):
            raise ValueError('park heading feedback must be removed before Nav2 controls yaw')
        self.plant,self.model,self.data=plant,plant.model,plant.data
        self.config=config
        self.gate=WallGuardedCommand(config['command'],clock)
        self.base=self.model.body(prefix+'base_link').id
        self.lidar=self.model.site(prefix+'lidar_site').id
        self.dofs={side:int(self.model.jnt_dofadr[self.model.joint(
            prefix+'wheel_'+side+'_joint').id]) for side in ('left','right')}
        self.signs=plant._wheel_signs()
        self.radius=float(config['odom']['wheel_radius_m']);self.track=float(config['odom']['track_m'])
        if not all(math.isfinite(x) and x>0 for x in (self.radius,self.track)):
            raise ValueError('positive finite wheel geometry required')
        scan=config['scan'];nrays=int(scan['rays'])
        if not 2<=nrays<=1000:raise ValueError('bounded ray count required')
        angles=np.radians(np.linspace(scan['angle_min_deg'],scan['angle_max_deg'],nrays))
        self.local=np.stack([np.cos(angles),np.sin(angles),np.zeros(nrays)],axis=1)
        self.nrays=nrays;self.seq=0;self.sent=0;self.received=0;self.closed=False
        ipc=config['ipc']
        self.command_sock=socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
        self.state_sock=socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
        try:
            self.command_sock.bind((ipc['host'],int(ipc['command_port'])))
            self.command_sock.setblocking(False)
        except Exception:
            self.close();raise

    def poll(self):
        ipc=self.config['ipc']
        for _ in range(min(32,int(ipc['max_datagrams_per_tick']))):
            try:raw,peer=self.command_sock.recvfrom(min(65507,int(ipc['max_datagram_bytes'])))
            except BlockingIOError:break
            if peer[0]!='127.0.0.1':continue
            self.received+=1
            self.gate.offer(raw,float(self.data.time))

    def control(self, motion_permit=None):
        """Produce a vector only; WorldOwner alone may commit it and step."""
        if self.closed:raise RuntimeError('navigation IPC closed')
        self.poll()
        v,w,_,_=self.gate.step(float(self.data.time))
        # An optional load supervisor can revoke motion, NEVER disable the
        # original zero-rate brake. No additional ROS velocity producer.
        if motion_permit is not None and not motion_permit(bool(abs(v)+abs(w)>1e-8)):
            v,w=0.,0.
        rates={'left':(v-w*self.track/2)/self.radius,
               'right':(v+w*self.track/2)/self.radius}
        return self.plant._control(lambda side:self.plant._wheel_rate_torque(
            side,rates[side]*self.signs[side]))

    def publish(self):
        """Actual scene scan/wheel rates, with no body pose field."""
        if self.closed:raise RuntimeError('navigation IPC closed')
        scan=self.config['scan']
        vectors=self.local@self.data.xmat[self.base].reshape(3,3).T
        geomid=np.zeros(self.nrays,np.int32);dist=np.full(self.nrays,np.inf)
        maximum=float(scan['range_max_m'])
        mujoco.mj_multiRay(self.model,self.data,self.data.site_xpos[self.lidar].copy(),
            vectors.ravel(),None,True,self.base,geomid,dist,self.nrays,maximum+1.)
        ranges=bounded_ranges(dist,geomid,float(scan['range_min_m']),maximum)
        summary=self.gate.gate.summary(float(self.data.time))
        _,_,mode,_=self.gate.step(float(self.data.time))
        payload=state(seq=self.seq,sim_time=round(float(self.data.time),6),
            wheel_rate=[round(self.signs[side]*float(self.data.qvel[self.dofs[side]]),6)
                        for side in ('left','right')],
            ranges=[round(float(x),4) for x in ranges],range_min=float(scan['range_min_m']),
            range_max=maximum,gate={key:summary[key] for key in ('age_s','accepted','refused')})
        payload['gate']['mode']=mode
        raw=encode('state',payload)
        self.state_sock.sendto(raw,(self.config['ipc']['host'],int(self.config['ipc']['state_port'])))
        self.seq+=1;self.sent+=1
        return payload

    def close(self):
        for name in ('command_sock','state_sock'):
            sock=getattr(self,name,None)
            if sock is not None:sock.close()
        self.closed=True
