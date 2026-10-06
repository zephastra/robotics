import sys
from pathlib import Path
import pytest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'experiments'))
from joint_nav_worker import speed_overrides


def test_speed_only_overrides_velocity_not_acceleration_or_safety():
    assert speed_overrides('controller_server',.08)=={'FollowPath.desired_linear_vel':.08}
    assert speed_overrides('velocity_smoother',.08)=={'max_velocity':[.08,0.,.15],'min_velocity':[-.08,0.,-.15]}
    assert speed_overrides('collision_monitor',.08)=={}
    assert speed_overrides('amcl',.08)=={}
    assert speed_overrides('controller_server',None)=={}


@pytest.mark.parametrize('speed',[float('nan'),float('inf'),0.,.049,.151])
def test_invalid_speed_refused(speed):
    with pytest.raises(ValueError):speed_overrides('controller_server',speed)
