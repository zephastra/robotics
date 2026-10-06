import sys
from pathlib import Path
import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'experiments'))
from vehicle_rgbd_reader import base_cloud, observe
import vehicle_rgbd_reader


def calibration():
    return dict(width=2, height=2, fovy_deg=90.,
                rotation_camera_to_base=np.eye(3).tolist(), position_in_base=[2., 0., 1.])


def test_pixel_centres_and_camera_negative_z_are_explicit():
    cloud, valid = base_cloud(np.ones((2, 2)), calibration())
    assert valid.all()
    np.testing.assert_allclose(cloud[0, 0], [1.5, .5, 0.])
    np.testing.assert_allclose(cloud[1, 1], [2.5, -.5, 0.])


def test_invalid_depth_does_not_become_origin_measurement():
    cloud, valid = base_cloud(np.array([[0., np.nan], [np.inf, 1.]]), calibration())
    assert valid.sum() == 1
    assert np.isnan(cloud[~valid]).all()


@pytest.mark.parametrize('key,value', [('fovy_deg', 0.), ('fovy_deg', float('nan')),
    ('position_in_base', [float('nan'), 0., 1.]),
    ('rotation_camera_to_base', np.diag([1., 1., -1.]).tolist()),
    ('rotation_camera_to_base', (2*np.eye(3)).tolist())])
def test_bad_calibration_refused(key, value):
    cal = calibration(); cal[key] = value
    with pytest.raises(ValueError):
        base_cloud(np.ones((2, 2)), cal)


@pytest.mark.parametrize('depth', [np.zeros((2, 2)), np.ones((3, 2))])
def test_missing_or_wrong_depth_cannot_verify_quantity(depth):
    result = observe(np.zeros((2, 2, 3)), depth, calibration(), {}, {},
                     roi=(0., 4., -1., 1.), floor_z=0., observed_sim_s=0., observed_wall_s=1.)
    assert result['status'] == 'UNKNOWN' and result['counts'] is None
    assert result['count_verified'] is False


def test_successful_measurement_does_not_authorize_its_own_order(monkeypatch):
    monkeypatch.setattr(vehicle_rgbd_reader.locator, 'locate', lambda *args:
        dict(status='RESOLVED', cells={}, centre_xy_m=[2.2, .03]))
    monkeypatch.setattr(vehicle_rgbd_reader.counter, 'read_cloud', lambda *args:
        dict(status='RESOLVED', counts={'red':2, 'blue':1}))
    result = observe(np.zeros((2, 2, 3)), np.ones((2, 2)), calibration(), {}, {},
                     roi=(0., 4., -1., 1.), floor_z=0., observed_sim_s=0., observed_wall_s=1.)
    assert result['status']=='RESOLVED' and result['tray_in_base_xy']==[2.2, .03]
    assert result['counts']=={'red':2, 'blue':1}
    assert result['count_verified'] is False and 'support' not in result


@pytest.mark.parametrize('stamp', [float('nan'), float('inf')])
def test_invalid_timestamp_is_unknown(stamp):
    result = observe(np.zeros((2, 2, 3)), np.ones((2, 2)), calibration(), {}, {},
                     roi=(0., 4., -1., 1.), floor_z=0., observed_sim_s=stamp, observed_wall_s=1.)
    assert result['status']=='UNKNOWN' and not result['count_verified']
