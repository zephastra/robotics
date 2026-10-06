import sys
from pathlib import Path

import mujoco
import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'experiments'))
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src'))
from world_owner import WorldOwner, WorldSafetyHold
from workcell.runtime_permit import RuntimePermit


def setup():
    model = mujoco.MjModel.from_xml_string('''<mujoco><worldbody>
      <body><joint name="j"/><geom size=".1"/></body></worldbody>
      <actuator><motor joint="j"/></actuator></mujoco>''')
    data = mujoco.MjData(model)
    state = dict(owner='test', epoch=1, generation=1, sequence=0,
                 observed_wall_s=0., cancel_requested=False, zone_clear=True,
                 stop_chain_healthy=True)
    clock = [0.]
    permit = RuntimePermit(lambda: state.copy(), owner='test', epoch=1,
                           generation=1, max_age_s=.5, clock=lambda: clock[0])
    return WorldOwner(model, data, permit), state, clock


def test_unique_writer_and_normal_step():
    owner, _, _ = setup()
    with pytest.raises(ValueError, match='already'):
        WorldOwner(owner.model, owner.data, owner.permit)
    assert owner.step(lambda: [0.]) > 0
    assert owner.steps == 1


@pytest.mark.parametrize('field,value', [('cancel_requested', True),
    ('zone_clear', False), ('stop_chain_healthy', None), ('observed_wall_s', -1.)])
def test_fault_freezes_whole_state_without_evaluating_controller(field, value):
    owner, state, _ = setup()
    state[field] = value
    before = owner.data.qpos.copy()
    with pytest.raises(WorldSafetyHold):
        owner.step(lambda: pytest.fail('must not evaluate stale command'))
    state[field] = False if field == 'cancel_requested' else True
    with pytest.raises(WorldSafetyHold):
        owner.step(lambda: [1.])
    assert owner.data.time == 0 and owner.steps == 0
    np.testing.assert_array_equal(owner.data.qpos, before)
    assert owner.frozen['stopped_confirmed'] is False


def test_permission_is_checked_again_after_slow_control_callback():
    owner, _, clock = setup()
    def slow():
        clock[0] = 1.
        return [1.]
    with pytest.raises(WorldSafetyHold, match='STALE_OBSERVATION'):
        owner.step(slow)
    assert owner.data.time == 0 and owner.data.ctrl[0] == 0


@pytest.mark.parametrize('control', [[float('nan')], [], [1., 2.]])
def test_invalid_control_never_reaches_final_writer(control):
    owner, _, _ = setup()
    with pytest.raises(WorldSafetyHold, match='CONTROL_UNAVAILABLE'):
        owner.step(lambda: control)
    assert owner.data.time == 0
