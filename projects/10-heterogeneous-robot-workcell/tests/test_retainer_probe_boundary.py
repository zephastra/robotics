import ast
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def test_retainer_uses_world_owner_and_no_runtime_cargo_write():
    source = (ROOT/'experiments/probe_tray_retainer.py').read_text()
    tree = ast.parse(source)
    assert 'owner.step(control)' in source
    for node in ast.walk(tree):
        if isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute):
            assert node.func.attr not in ('mj_step', 'mj_forward')
        if isinstance(node, ast.Assign):
            for target in node.targets:
                assert 'qpos' not in ast.unparse(target)
    assert 'transport_authorized=False' in source
    assert "report['close_refusal'] = 'INITIAL_VISUAL_UNKNOWN'" in source


def test_scene_default_world_is_preserved():
    source = (ROOT/'experiments/probe_vehicle_rgbd.py').read_text()
    assert 'WORLD if world_path is None else world_path' in source
    assert 'world_p5_candidate_v5.xml' in source
