"""Vehicle-fixed optical calibration; no simulator, labels or cargo truth inputs.

Coordinates are in the vehicle base frame. Catalog support height is a bounded
fixture assumption, not measured contact or retention. Quantity authorization is
the caller's job: an observed count must not authorize its own order completion.
"""
import math
import numpy as np

import receiver_rgbd_reader as locator
import tray_rgbd_reader as counter


def base_cloud(depth, calibration):
    depth = np.asarray(depth, float)
    width, height = calibration['width'], calibration['height']
    rotation = np.asarray(calibration['rotation_camera_to_base'], float)
    position = np.asarray(calibration['position_in_base'], float)
    fovy = float(calibration['fovy_deg'])
    if (depth.shape != (height, width) or rotation.shape != (3, 3)
            or position.shape != (3,) or not np.all(np.isfinite(rotation))
            or not np.all(np.isfinite(position)) or not math.isfinite(fovy)
            or not 0 < fovy < 180 or min(width, height) <= 0
            or not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-8)
            or not np.isclose(np.linalg.det(rotation), 1., atol=1e-8)):
        raise ValueError('finite rigid camera calibration and matching depth required')
    focal = height / (2 * math.tan(math.radians(fovy) / 2))
    u, v = np.meshgrid(np.arange(width) + .5, np.arange(height) + .5)
    valid = np.isfinite(depth) & (depth > .01) & (depth < 10.)
    z = np.where(valid, depth, 0.)
    # MuJoCo rendered metric depth, camera -z forward, image rows downward.
    local = np.stack(((u-width/2)*z/focal, -(v-height/2)*z/focal, -z), axis=-1)
    points = local @ rotation.T + position
    points[~valid] = np.nan
    return points, valid


def observe(rgb, depth, calibration, template, half_size, *, roi, floor_z,
            observed_sim_s, observed_wall_s, position_uncertainty_m=.01):
    unknown = dict(source='RGBD', status='UNKNOWN', counts=None,
                   count_verified=False, observed_sim_s=observed_sim_s,
                   observed_wall_s=observed_wall_s)
    try:
        if not all(math.isfinite(v) for v in (observed_sim_s, observed_wall_s,
                                               position_uncertainty_m, floor_z)):
            raise ValueError('finite timestamps and support calibration required')
        if position_uncertainty_m < 0:
            raise ValueError('negative uncertainty')
        cloud, valid = base_cloud(depth, calibration)
        located = locator.locate(template, cloud, np.asarray(rgb), valid, roi, floor_z)
        if located['status'] != 'RESOLVED':
            return dict(unknown, reason=located['reason'], localization=located)
        counted = counter.read_cloud(rgb, depth, cloud, valid, located['cells'], half_size)
        if counted['status'] != 'RESOLVED':
            return dict(unknown, reason=counted['reason'], localization=located, counting=counted)
        return dict(source='RGBD', status='RESOLVED', reason='VEHICLE_RGBD_OBSERVED',
                    counts=counted['counts'], count_verified=False,
                    tray_in_base_xy=located['centre_xy_m'],
                    position_uncertainty_m=position_uncertainty_m,
                    observed_sim_s=observed_sim_s, observed_wall_s=observed_wall_s,
                    localization=located, counting=counted)
    except (KeyError, TypeError, ValueError, IndexError):
        return dict(unknown, reason='INVALID_SENSOR_INPUT')
