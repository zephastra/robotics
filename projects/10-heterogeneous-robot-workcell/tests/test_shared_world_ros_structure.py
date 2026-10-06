"""Structural checks only; do not substitute for ROS/physical commissioning."""
import ast
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]


def test_shared_bridge_uses_controller_lease_and_original_timestamp():
    source=(ROOT/'experiments/joint_world_ros_bridge.py').read_text()
    tree=ast.parse(source)
    calls=[n for n in ast.walk(tree) if isinstance(n,ast.Call)]
    assert any(ast.unparse(n.func)=='controller_lease.offer' for n in calls)
    assert any(ast.unparse(n.func)=='controller_lease.value' for n in calls)
    command_call=next(n for n in calls if ast.unparse(n.func)=='command_payload')
    timestamp=next(k.value for k in command_call.keywords if k.arg=='issued_sim_time')
    assert 'issued_sim_time' in ast.unparse(timestamp)
    assert 'round(sim_time' not in ast.unparse(timestamp)


def test_ros_bridge_has_no_physics_writes_or_simulator_import():
    source=(ROOT/'experiments/joint_world_ros_bridge.py').read_text()
    tree=ast.parse(source)
    imports=[ast.unparse(n) for n in ast.walk(tree) if isinstance(n,(ast.Import,ast.ImportFrom))]
    assert not any('mujoco' in name or 'LogisticsPlant' in name for name in imports)


def test_nav_io_probe_one_owner_no_runtime_reset_in_loop():
    source=(ROOT/'experiments/probe_joint_world_nav_io.py').read_text()
    tree=ast.parse(source)
    calls=[ast.unparse(n.func) for n in ast.walk(tree) if isinstance(n,ast.Call)]
    assert calls.count('WorldOwner')==1
    for loop in (n for n in ast.walk(tree) if isinstance(n,ast.While)):
        assert 'mj_resetData' not in ast.unparse(loop)
        assert '.qpos[' not in ast.unparse(loop)
