from pathlib import Path
import sys
from types import SimpleNamespace
import numpy as np
ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'src'),str(ROOT/'experiments')]
from joint_world_nav_io import JointWorldNavIO


def fake_io(v=.2,w=0.):
    io=JointWorldNavIO.__new__(JointWorldNavIO)
    io.closed=False;io.poll=lambda:None
    io.data=SimpleNamespace(time=10.)
    io.gate=SimpleNamespace(step=lambda stamp:(v,w,'MOVING',None))
    io.radius=.1;io.track=.3;io.signs={'left':1.,'right':1.}
    # Return the final servo RATE REQUESTS, not synthetic physical stop evidence.
    io.plant=SimpleNamespace(_wheel_rate_torque=lambda side,rate:rate,
        _control=lambda callback:np.array([callback('left'),callback('right')]))
    return io


def test_load_refusal_replaces_nonzero_command_with_original_rate_brake():
    requests=[]
    result=fake_io().control(lambda moving:requests.append(moving) or False)
    np.testing.assert_array_equal(result,[0.,0.])
    assert requests==[True]


def test_authorized_load_uses_existing_single_servo():
    np.testing.assert_allclose(fake_io().control(lambda moving:True),[2.,2.])


def test_rotation_is_motion_and_requires_load_permission():
    requests=[]
    result=fake_io(v=0.,w=.5).control(lambda moving:requests.append(moving) or False)
    assert requests==[True]
    np.testing.assert_array_equal(result,[0.,0.])


def test_default_empty_navigation_command_path_is_unchanged():
    np.testing.assert_allclose(fake_io().control(),[2.,2.])


def test_sensor_arrays_and_scalars_persist_in_report_without_error():
    import json
    from probe_vehicle_nav2 import json_value
    data=json.loads(json.dumps(dict(cells={'centre':np.array([2.2,0.,.8])},
                                    sample=np.float64(.1)),default=json_value))
    assert data['cells']['centre']==[2.2,0.,.8] and data['sample']==.1
