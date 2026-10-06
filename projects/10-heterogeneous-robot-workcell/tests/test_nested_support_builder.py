"""Nested deck idlers must inherit the chassis frame, not remain at world origin."""
import xml.etree.ElementTree as ET
import sys
from pathlib import Path
import pytest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'experiments'))
from build_support_candidate import build


def test_nested_passive_support_preserves_original_row_and_parent():
    text='''<mujoco><worldbody><body name="car"><body name="deck">
    <body name="roller_0" pos="0 0 -.035"><joint name="r0"/>
      <geom name="g0" type="cylinder" size=".035 .3" mass="1"/></body>
    <body name="roller_1" pos=".08 0 -.035"><joint name="r1"/>
      <geom name="g1" type="cylinder" size=".035 .3" mass="1"/></body>
    </body></body></worldbody><actuator/><keyframe/></mujoco>'''
    result,pairs=build(text,split=True,roller_pattern=r'roller_\d+',
                       idler_prefix='support',parent_name='deck')
    before=ET.fromstring(text);after=ET.fromstring(result)
    deck=next(b for b in after.iter('body') if b.get('name')=='deck')
    assert len(pairs)==2
    assert not after.find('worldbody').findall("body[@name='support_0_side1']")
    for side in (-1,1):
        added=deck.find(f"body[@name='support_0_side{side}']")
        assert added is not None
        assert added.find('joint').get('damping')=='0'
        assert added.find('joint').get('frictionloss')=='0'
    for name in ('roller_0','roller_1'):
        a=next(b for b in before.iter('body') if b.get('name')==name)
        b=next(b for b in after.iter('body') if b.get('name')==name)
        assert ET.tostring(a)==ET.tostring(b)


def test_missing_parent_is_rejected():
    with pytest.raises(ValueError,match='parent missing'):
        build('<mujoco><worldbody/></mujoco>',parent_name='deck')
