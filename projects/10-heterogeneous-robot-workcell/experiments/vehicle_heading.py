"""Optional restricted ideal-self-state yaw feedback, never a Nav2 replacement."""
import math
import numpy as np


def requested_rates(v, yaw, yaw_rate, signs, *, radius=.04, track=.30):
    if not all(math.isfinite(float(x)) for x in (v,yaw,yaw_rate,radius,track)) or radius<=0 or track<=0:
        raise ValueError('invalid vehicle feedback')
    if set(signs)!={'left','right'} or any(signs[s] not in (-1.,1.) for s in signs):
        raise ValueError('wheel signs must be measured unit signs')
    error=math.atan2(math.sin(-yaw),math.cos(-yaw))
    turn=float(np.clip(1.5*error-.25*yaw_rate,-.30,.30))
    return {'left':signs['left']*(v-turn*track/2)/radius,
            'right':signs['right']*(v+turn*track/2)/radius}


def measured_heading(plant):
    model,data=plant.model,plant.data
    base=model.body('n_base_link').id
    rotation=data.xmat[base].reshape(3,3)
    yaw=math.atan2(rotation[1,0],rotation[0,0])
    dof=int(model.jnt_dofadr[model.joint('n_yaw').id])
    return yaw,float(data.qvel[dof])


def control(plant,base,v=0.):
    yaw,yaw_rate=measured_heading(plant)
    rates=requested_rates(v,yaw,yaw_rate,plant._wheel_signs())
    result=np.array(base,float,copy=True)
    for side,act in plant.wheel_actuators.items():
        result[act]=plant._wheel_rate_torque(side,rates[side])
    return result


def install(plant):
    """Opt-in per-instance wheel-rate correction plus continuous motor braking.

    No model/state writes and no extra physical writer. Belt-only vectors used to
    leave raw wheel motors at zero torque; zero torque is NOT a parking brake.
    The original bounded rate servo still owns all wheel motor computations.
    """
    if getattr(plant,'heading_feedback_installed',False):
        raise RuntimeError('heading feedback already installed')
    signs=plant._wheel_signs()
    previous_servo=plant._wheel_rate_torque
    previous_hold=plant._hold
    def servo(side,want_rate):
        yaw,rate=measured_heading(plant)
        correction=requested_rates(0.,yaw,rate,signs)
        return previous_servo(side,float(want_rate)+correction[side])
    def hold():
        ctrl=np.array(previous_hold(),float,copy=True)
        for side,act in plant.wheel_actuators.items():ctrl[act]=servo(side,0.)
        return ctrl
    plant._wheel_rate_torque=servo
    plant._hold=hold
    # Keep exact callable identities for explicit navigation authority transfer.
    # Defaults are unchanged; no automatic disable/re-enable on an exception.
    plant._heading_feedback_base_servo=previous_servo
    plant._heading_feedback_parking_servo=servo
    plant.heading_feedback_installed=True
