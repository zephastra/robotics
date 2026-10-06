"""Bounded RGB-D feedback positioning on the existing source rollers.

Only observed tray position feeds commands. Known station calibration defines
the target; physical stop is measured by the unchanged plant window.
"""
import numpy as np


def stage_to_loading(plant,owner,observe,target_x):
    start=float(plant.data.time);trace=[]
    spin=[i for i in range(plant.model.nu)
          if (plant.model.actuator(i).name or '').startswith('c_fixed_roller')]
    radii=[float(plant.model.geom_size[g][0]) for g in range(plant.model.ngeom)
           if (plant.model.geom(g).name or '').startswith('c_fixed_roller_')]
    radius=max(radii)
    while plant.data.time-start<45.:
        measured=observe()
        trace.append({k:v for k,v in measured.items() if k!='cells'})
        if measured['status']!='RESOLVED':
            owner.permit.stop('VISION_UNKNOWN','source tray unobservable')
            owner.step(plant.park_control)
            return dict(status='UNKNOWN',reason='SOURCE_VISION_UNKNOWN',trace=trace)
        error=float(target_x-measured['centre_xy_m'][0])
        if abs(error)<=.003:
            stop=plant._stop_transfer()
            final=observe()
            if not stop['stopped_confirmed'] or final['status']!='RESOLVED':
                return dict(status='UNKNOWN',reason='SOURCE_STOP_UNCONFIRMED',stop=stop,trace=trace)
            if abs(final['centre_xy_m'][0]-target_x)>.005:
                return dict(status='UNKNOWN',reason='SOURCE_LOADING_POSE_MISSED',stop=stop,trace=trace)
            return dict(final,stop=stop,trace=trace)
        speed=float(np.clip(1.2*error,-.05,.05))/radius
        # Reobserve every 0.10 sim seconds. The final owner checks runtime
        # permission every physical step, including during this bounded chunk.
        until=float(plant.data.time)+.10
        def command():
            ctrl=plant.park_control()
            for actuator in spin:ctrl[actuator]=speed
            return ctrl
        while plant.data.time<until:
            owner.step(command)
    owner.permit.stop('POSITIONING_TIMEOUT','bounded source approach exhausted')
    owner.step(plant.park_control)
    return dict(status='UNKNOWN',reason='SOURCE_POSITIONING_TIMEOUT',trace=trace)
