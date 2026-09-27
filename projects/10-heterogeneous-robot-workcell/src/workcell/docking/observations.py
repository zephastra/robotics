"""Observations: the only channel through which the docking controller may learn where it is.

Two rules decide the whole shape of this module, and both come from the guidance:

  * **a missing, stale, unnamed-frame or wrong-station observation is UNKNOWN, never zero.** A zero
    here reads as "perfectly aligned", which is the most dangerous possible default for a docking
    controller;
  * **the truth channel never enters.** Nothing here imports MuJoCo or reads a world pose, so a
    probe that knows the answer cannot pass it in by accident. The evaluator may compare against
    truth later; the controller may not.
"""
from pathlib import Path
import json

OBSERVATION_REFUSALS = frozenset({
    'REFUSED_NOT_A_MAPPING', 'REFUSED_MISSING_FIELD', 'REFUSED_UNKNOWN_FIELD',
    'REFUSED_BAD_TYPE', 'REFUSED_UNKNOWN_FRAME', 'REFUSED_WRONG_STATION',
    'REFUSED_INVALID', 'REFUSED_NO_MEASUREMENT', 'REFUSED_UNKNOWN_SENSOR',
    'STALE_OBSERVATION', 'UNKNOWN_MEASUREMENT',
})

OBSERVATION_KEYS = frozenset({'sensor_id', 'station_id', 'frame_id', 'sequence', 'sim_stamp',
                              'wall_receive_time', 'relative_pose', 'uncertainty',
                              'validity', 'rejection_reason', 'source_method',
                              'calibration_version'})
RELATIVE_POSE_KEYS = frozenset({'longitudinal_m', 'lateral_m', 'height_m', 'yaw_rad',
                                'measurement_dimensions'})


class ObservationRefused(ValueError):
    def __init__(self, reason, detail='', *, field=None):
        self.reason = reason
        self.detail = detail
        self.field = field
        super().__init__(f'{reason}: {detail}' if detail else reason)


def validate_observation(raw):
    """A docking observation, normalised. The dimension list is required because a relative pose
    that measures two axes is not the same claim as one that measures three, and the guidance asks
    for the measured dimensions to be explicit."""
    if not isinstance(raw, dict):
        raise ObservationRefused('REFUSED_NOT_A_MAPPING', type(raw).__name__)
    missing = sorted(OBSERVATION_KEYS - set(raw))
    if missing:
        raise ObservationRefused('REFUSED_MISSING_FIELD', f'{missing}')
    unknown = sorted(set(raw) - OBSERVATION_KEYS)
    if unknown:
        raise ObservationRefused('REFUSED_UNKNOWN_FIELD', f'{unknown}')
    pose = raw['relative_pose']
    if pose is None:
        raise ObservationRefused('REFUSED_NO_MEASUREMENT',
                                 'relative_pose is null: this observation says nothing, and it must '
                                 'not be read as zero')
    if not isinstance(pose, dict):
        raise ObservationRefused('REFUSED_BAD_TYPE', type(pose).__name__, field='relative_pose')
    pose_missing = sorted(RELATIVE_POSE_KEYS - set(pose))
    if pose_missing:
        raise ObservationRefused('REFUSED_MISSING_FIELD', f'{pose_missing}', field='relative_pose')
    dims = pose['measurement_dimensions']
    if not isinstance(dims, list) or not dims:
        raise ObservationRefused('REFUSED_NO_MEASUREMENT',
                                 'measurement_dimensions is empty: which axes were measured must be '
                                 'stated, because two axes are not a plane pose',
                                 field='relative_pose.measurement_dimensions')
    for axis in dims:
        if axis not in ('longitudinal', 'lateral', 'height', 'yaw'):
            raise ObservationRefused('REFUSED_BAD_TYPE', repr(axis),
                                     field='relative_pose.measurement_dimensions')
    for name in ('sim_stamp', 'wall_receive_time'):
        if not isinstance(raw[name], (int, float)) or isinstance(raw[name], bool):
            raise ObservationRefused('REFUSED_BAD_TYPE', repr(raw[name]), field=name)
    for name in ('sensor_id', 'station_id', 'frame_id', 'source_method', 'calibration_version'):
        if not isinstance(raw[name], str) or not raw[name]:
            raise ObservationRefused('REFUSED_MISSING_FIELD', f'{name} is empty', field=name)
    return dict(raw)


def accept_observation(raw, *, now_wall_s, contract, expected_station_id):
    """FRESH / STALE / UNKNOWN, with the reason. Never a value when the answer is unknown.

    The frame and the station are checked against the contract, so an observation about another
    station, or in a frame the contract never declared, cannot be mistaken for one about this dock.
    """
    try:
        obs = validate_observation(raw)
    except ObservationRefused as exc:
        return {'status': 'UNKNOWN', 'reason': exc.reason, 'detail': exc.detail,
                'age_s': None, 'observation': None}
    if obs['station_id'] != expected_station_id:
        return {'status': 'UNKNOWN', 'reason': 'REFUSED_WRONG_STATION',
                'detail': f"the observation is about {obs['station_id']!r}, this dock is "
                          f"{expected_station_id!r}", 'age_s': None, 'observation': obs}
    declared_frames = set((contract['frames'] or {}).values()) | {'world', 'odom', 'base_link'}
    if obs['frame_id'] not in declared_frames:
        return {'status': 'UNKNOWN', 'reason': 'REFUSED_UNKNOWN_FRAME',
                'detail': f"{obs['frame_id']!r} is not one of the contract's frames "
                          f"{sorted(declared_frames)}", 'age_s': None, 'observation': obs}
    if obs['validity'] is not True:
        return {'status': 'UNKNOWN', 'reason': 'REFUSED_INVALID',
                'detail': f"the sensor marked this invalid: {obs['rejection_reason']!r}",
                'age_s': None, 'observation': obs}
    age = float(now_wall_s) - float(obs['wall_receive_time'])
    if age < 0:
        return {'status': 'UNKNOWN', 'reason': 'REFUSED_BAD_TYPE',
                'detail': f'the observation claims to arrive in the future by {-age:.3f} s',
                'age_s': age, 'observation': obs}
    limit = float(contract['validity']['max_observation_age_s'])
    if age > limit:
        return {'status': 'STALE', 'reason': 'STALE_OBSERVATION',
                'detail': f'age {age:.3f} s exceeds the declared {limit} s', 'age_s': age,
                'observation': obs}
    return {'status': 'FRESH', 'reason': None, 'age_s': age, 'observation': obs,
            'detail': f'fresh by {limit - age:.3f} s, measured in '
                      f"{obs['relative_pose']['measurement_dimensions']} by "
                      f"{obs['source_method']} ({obs['calibration_version']})"}
