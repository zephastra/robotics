"""Explicit parking -> Nav2 -> confirmed stop -> parking wheel authority.

No auto-recovery on exceptions. A failed navigation retains NAV2 ownership;
missing stop evidence cannot quietly re-enable a different wheel controller.
"""
import math


def grant_navigation(plant):
    if getattr(plant,'navigation_authority',None) is not None:
        raise RuntimeError('WHEEL_AUTHORITY_ALREADY_GRANTED')
    if (not getattr(plant,'heading_feedback_installed',False)
            or not hasattr(plant,'_heading_feedback_base_servo')
            or plant._wheel_rate_torque is not plant._heading_feedback_parking_servo):
        raise RuntimeError('PARKING_OWNER_UNCONFIRMED')
    token=object()
    plant.navigation_authority=token
    plant._wheel_rate_torque=plant._heading_feedback_base_servo
    plant.heading_feedback_installed=False
    return token


def return_to_parking(plant,token,stop,*,nav_process_stopped,ipc_closed):
    if getattr(plant,'navigation_authority',None) is not token or token is None:
        raise RuntimeError('WHEEL_AUTHORITY_TOKEN_INVALID')
    if nav_process_stopped is not True or ipc_closed is not True:
        raise RuntimeError('NAVIGATION_PRODUCER_STILL_AUTHORIZED')
    try:
        speed=float(stop['held_speed_mps']);drift=float(stop['drift_m'])
        confirmed=stop.get('stopped_confirmed') is True and all(math.isfinite(v) and v>=0 for v in (speed,drift)) and speed<=.01 and drift<=.005
    except (KeyError,TypeError,ValueError):confirmed=False
    if not confirmed:raise RuntimeError('PHYSICAL_STOP_UNCONFIRMED')
    if plant._wheel_rate_torque is not plant._heading_feedback_base_servo or plant.heading_feedback_installed:
        raise RuntimeError('WHEEL_AUTHORITY_CONFLICT')
    plant._wheel_rate_torque=plant._heading_feedback_parking_servo
    plant.heading_feedback_installed=True
    plant.navigation_authority=None
