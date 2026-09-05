import math
from mobile_manipulator.scan_filter import filter_self_returns


def test_masks_camera_mast():
    angle=math.atan2(-.22,-.48)
    assert math.isnan(filter_self_returns([math.hypot(.48,.22)],angle,0.)[0])


def test_keeps_real_nearby_obstacle():
    assert filter_self_returns([.36],0.,0.)==[.36]


def test_masks_low_palm_only_when_at_scan_height():
    assert math.isnan(filter_self_returns([.38],math.pi,0.,(-.24,0,.48))[0])
    assert filter_self_returns([.38],math.pi,0.,(-.24,0,.85))==[.38]
