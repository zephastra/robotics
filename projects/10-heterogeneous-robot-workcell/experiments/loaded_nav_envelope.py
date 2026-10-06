"""Declared catalog envelope and fresh observation gate, NOT loaded acceptance.

Planning uses catalog geometry and a declared support region, never runtime cargo
truth. RGB-D integration must supply measured tray position in the base frame;
this module does not obtain it from MuJoCo. Uncertainty shrinks the allowed region.
"""
import math
import numpy as np


def combined_envelope(self_points, centres_low, centres_high, cargo_radius, mechanism_allowance):
    points = np.asarray(self_points,float); low=np.asarray(centres_low,float); high=np.asarray(centres_high,float)
    if (points.shape!=(4,2) or low.shape!=(2,) or high.shape!=(2,)
            or not np.all(np.isfinite(points)) or not np.all(np.isfinite([low,high]))
            or np.any(high<low) or not math.isfinite(cargo_radius) or cargo_radius<=0
            or not math.isfinite(mechanism_allowance) or mechanism_allowance<0):
        raise ValueError('finite bounded declared geometry required')
    # A sphere radius remains conservative for ANY cargo orientation. This is
    # intentionally wider than fitting the observed upright tray once.
    expansion=cargo_radius+mechanism_allowance
    bottom=np.minimum(points.min(axis=0),low-expansion)
    top=np.maximum(points.max(axis=0),high+expansion)
    return [[float(bottom[0]),float(bottom[1])],[float(top[0]),float(bottom[1])],
            [float(top[0]),float(top[1])],[float(bottom[0]),float(top[1])]]


def observed_load_gate(observation, contract, *, now_sim_s, now_wall_s):
    """Sensor evidence, not order counts or simulated cargo body position.

    UNKNOWN includes stale/missing support or quantity evidence. An out-of-region
    position is REFUSED, not repaired by increasing the navigation footprint.
    """
    unknown=dict(allowed=False,reason_code='LOAD_ENVELOPE_UNKNOWN')
    if (observation.get('source')!='RGBD' or observation.get('status')!='RESOLVED'
            or observation.get('support')!='deck' or observation.get('count_verified') is not True):
        return unknown
    try:
        if not all(math.isfinite(v) for v in (now_sim_s,now_wall_s)): return unknown
        sim_age=now_sim_s-float(observation['observed_sim_s'])
        wall_age=now_wall_s-float(observation['observed_wall_s'])
        if not 0<=sim_age<=contract['max_sim_age_s'] or not 0<=wall_age<=contract['max_wall_age_s']: return unknown
        centre=np.asarray(observation['tray_in_base_xy'],float)
        uncertainty=float(observation['position_uncertainty_m'])
        low=np.asarray(contract['centre_low'],float); high=np.asarray(contract['centre_high'],float)
        if centre.shape!=(2,) or low.shape!=(2,) or high.shape!=(2,) or not np.all(np.isfinite([centre,low,high])):
            return unknown
        if not math.isfinite(uncertainty) or uncertainty<0: return unknown
        if np.any(centre-uncertainty<low) or np.any(centre+uncertainty>high):
            return dict(allowed=False,reason_code='LOAD_OUTSIDE_DECLARED_ENVELOPE')
    except (KeyError,TypeError,ValueError):return unknown
    return dict(allowed=True,reason_code=None)
