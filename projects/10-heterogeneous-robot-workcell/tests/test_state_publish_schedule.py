from pathlib import Path
import sys
import pytest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'experiments'))
from state_publish_schedule import StatePublishSchedule


def test_normal_sim_cadence_is_preserved():
    schedule=StatePublishSchedule()
    assert schedule.due(0,0)
    assert not schedule.due(.01,.01)
    assert schedule.due(.05,.05)


def test_slow_camera_does_not_starve_new_physical_states():
    schedule=StatePublishSchedule()
    assert schedule.due(20.,10.)
    assert schedule.due(20.002,10.8)
    assert schedule.due(20.004,11.6)


def test_wall_fallback_never_fabricates_sim_time():
    schedule=StatePublishSchedule()
    assert schedule.due(20.,10.)
    assert not schedule.due(20.,20.)
    assert schedule.due(20.002,20.)


@pytest.mark.parametrize('sim,wall',[(0,1),(2,0)])
def test_reversed_clock_requires_explicit_failure(sim,wall):
    schedule=StatePublishSchedule();schedule.due(1,1)
    with pytest.raises(RuntimeError,match='CLOCK_REVERSED'):schedule.due(sim,wall)


@pytest.mark.parametrize('sim,wall',[(float('nan'),0),(0,float('inf'))])
def test_unknown_time_cannot_publish(sim,wall):
    with pytest.raises(ValueError):StatePublishSchedule().due(sim,wall)
