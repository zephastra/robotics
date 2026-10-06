"""Commissioning topology tests without ROS, subprocesses, or physical stepping."""
import ast
import importlib.util
from pathlib import Path
import sys
import types

ROOT = Path(__file__).resolve().parents[1]


def test_minimal_launch_and_command_routing(monkeypatch):
    calls = []; descriptions = []
    def node(**kwargs): calls.append(kwargs); return kwargs
    class Service:
        def include_launch_description(self, description): descriptions.append(description)
        def run(self): return 0
    monkeypatch.setitem(sys.modules, 'launch', types.SimpleNamespace(LaunchDescription=list, LaunchService=Service))
    monkeypatch.setitem(sys.modules, 'launch_ros.actions', types.SimpleNamespace(Node=node))
    monkeypatch.setitem(sys.modules, 'ament_index_python.packages', types.SimpleNamespace(
        get_package_share_directory=lambda name: '/verified/share/'+name))
    path = ROOT/'experiments/joint_nav_worker.py'
    spec = importlib.util.spec_from_file_location('joint_nav_worker_test', path)
    worker = importlib.util.module_from_spec(spec); spec.loader.exec_module(worker)
    assert worker.launch_navigation()==0
    byname = {n['name']:n for n in calls}
    assert set(byname)==set(worker.NODES)|{'joint_world_lifecycle_manager'}
    assert not {'docking_server','following_server'} & set(byname)
    for name in ('controller_server','behavior_server','velocity_smoother'):
        assert byname[name]['remappings']==[('cmd_vel','cmd_vel_nav')]
    assert byname['collision_monitor']['remappings']==[]
    assert byname['joint_world_lifecycle_manager']['parameters'][0]['node_names']==list(worker.NODES)
    assert byname['map_server']['parameters'][1]['yaml_filename'].endswith('joint_world_v5.yaml')
    assert byname['bt_navigator']['parameters'][1]['bt_search_directories']==['/verified/share/nav2_bt_navigator/behavior_trees']


def test_ros_worker_has_no_simulator_import_or_truth_topic():
    source = (ROOT/'experiments/joint_nav_worker.py').read_text()
    tree = ast.parse(source)
    imports = [n.module for n in ast.walk(tree) if isinstance(n, ast.ImportFrom)]
    imports += [alias.name for n in ast.walk(tree) if isinstance(n,ast.Import) for alias in n.names]
    assert 'mujoco' not in imports and 'w4_plant' not in imports
    assert 'goal.pose.header.stamp = last_clock[0]' in source
    assert "report['cmd_vel_publishers'] != ['collision_monitor']" in source


def test_single_physics_owner_and_no_pose_feedback():
    source = (ROOT/'experiments/probe_joint_world_nav2.py').read_text()
    tree = ast.parse(source)
    calls = [ast.unparse(n.func) for n in ast.walk(tree) if isinstance(n, ast.Call)]
    assert calls.count('WorldOwner')==1
    assert calls.count('owner.step')==2  # navigation then sequential parking, same owner
    assert 'mujoco.mj_step' not in calls
    assert calls.count('mujoco.mj_resetDataKeyframe')==1  # before initialization only
    assert 'stationary=False, stopping=True' in source
    assert "posture_check(model, data)" in source
    assert "loaded_navigation='NOT_RUN'" in source
