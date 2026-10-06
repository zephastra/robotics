import sys
from pathlib import Path

import mujoco
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src'))
from workcell.interlock import Interlock
from workcell.schema import SchemaRefused


def world(*, visual_only=False):
    attributes = 'contype="0" conaffinity="0"' if visual_only else ''
    m = mujoco.MjModel.from_xml_string('''<mujoco><worldbody>
      <body name="a_arm"><geom type="sphere" size=".1"/></body>
      <body name="h_robot" pos="2 0 0"><geom type="sphere" size=".1" %s/></body>
      </worldbody></mujoco>''' % attributes)
    d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)
    return m, d


def test_missing_collision_evidence_is_not_clearance():
    m, d = world(visual_only=True)
    lock = Interlock(model=m, data=d, margin_m=.2)
    allowed, reason = lock.request('arm')
    assert allowed is False
    assert reason == 'REFUSED_NO_EVIDENCE'
    assert lock.holder is None


def test_wrong_prefix_is_unknown_not_far_away():
    m, d = world()
    lock = Interlock(model=m, data=d, margin_m=.2,
                     prefix_by_actor={'arm': 'a_', 'humanoid': 'missing_'})
    assert lock.request('arm')[0] is False


def test_actual_pairs_beyond_sensor_budget_can_be_clear():
    m, d = world()
    lock = Interlock(model=m, data=d, margin_m=.2)
    assert lock.request('arm') == (True, 'GRANTED_BEYOND_BUDGET')


@pytest.mark.parametrize('margin,budget', [(float('nan'), 1), (.2, .1), (.2, float('inf'))])
def test_invalid_sensor_budget_cannot_prove_clearance(margin, budget):
    m, d = world()
    with pytest.raises(SchemaRefused):
        Interlock(model=m, data=d, margin_m=margin, budget_m=budget)
