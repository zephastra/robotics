"""Source crown junction residual from CURRENT robot-side measurement.

The surveyed source station is a declared calibration; robot/crown state is
ideal simulated proprioception, not a cargo-truth input or visual estimate.
"""
import math
import numpy as np


def source_residual(source_last_xy, deck_first_xy, pitch, deck_rotation):
    source=np.asarray(source_last_xy,float);deck=np.asarray(deck_first_xy,float)
    rotation=np.asarray(deck_rotation,float)
    if source.shape!=(2,) or deck.shape!=(2,) or rotation.shape!=(3,3):
        raise ValueError('source/deck measured geometry dimensions invalid')
    if not np.all(np.isfinite(source)) or not np.all(np.isfinite(deck)) or not np.all(np.isfinite(rotation)):
        raise ValueError('source/deck measured geometry nonfinite')
    if not math.isfinite(pitch) or pitch<=0:raise ValueError('positive pitch required')
    if not np.allclose(rotation.T@rotation,np.eye(3),atol=1e-6) or np.linalg.det(rotation)<0:
        raise ValueError('invalid deck rotation')
    return dict(lateral_m=float(deck[1]-source[1]),
        longitudinal_m=float(source[0]+pitch-deck[0]),
        yaw_rad=math.atan2(rotation[1,0],rotation[0,0]))
