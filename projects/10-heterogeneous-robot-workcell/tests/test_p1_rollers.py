"""Structural checks only, not physical acceptance."""
import importlib.util
from pathlib import Path
import xml.etree.ElementTree as ET

path=Path(__file__).resolve().parents[1]/'experiments/probe_rollers.py'
spec=importlib.util.spec_from_file_location('rollers',path)
module=importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


def test_no_overlapping_rollers():
    root=ET.fromstring(module.make_model())
    bodies=[b for b in root.findall('worldbody/body') if b.get('name').startswith('roller_')]
    assert len(bodies)==24
    for first,second in zip(bodies,bodies[1:]):
        gap=float(second.get('pos').split()[0])-float(first.get('pos').split()[0])
        radii=sum(float(b.find('geom').get('size').split()[0]) for b in (first,second))
        assert gap>radii


def test_payload_free_no_weld():
    root=ET.fromstring(module.make_model())
    assert root.find("worldbody/body[@name='tray']/freejoint") is not None
    assert root.find('equality') is None


def test_bounded_actuators():
    for motor in ET.fromstring(module.make_model()).findall('actuator/velocity'):
        assert motor.get('forcelimited')=='true'
        assert motor.get('ctrllimited')=='true'
