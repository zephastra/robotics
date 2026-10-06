"""Declared dock localization prior overrides, without starting ROS."""
import sys
import types
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'experiments'))


def test_declared_station_prior_is_forwarded_to_amcl_only(monkeypatch):
    import joint_nav_worker as worker
    calls=[]
    class Service:
        def include_launch_description(self,description):pass
        def run(self):return 0
    monkeypatch.setitem(sys.modules,'launch',types.SimpleNamespace(LaunchDescription=list,LaunchService=Service))
    monkeypatch.setitem(sys.modules,'launch_ros.actions',types.SimpleNamespace(Node=lambda **kw:calls.append(kw)))
    monkeypatch.setitem(sys.modules,'ament_index_python.packages',types.SimpleNamespace(
        get_package_share_directory=lambda name:'/verified/share/'+name))
    assert worker.launch_navigation(loaded=True,centered=True,start=[4.42372,0.,0.])==0
    byname={n['name']:n for n in calls}
    assert byname['amcl']['parameters'][1]['initial_pose.x']==4.42372
    assert byname['amcl']['parameters'][1]['initial_pose.y']==0
    assert byname['amcl']['parameters'][1]['initial_pose.yaw']==0
    for name in worker.NODES:
        if name!='amcl':assert 'initial_pose.x' not in byname[name]['parameters'][1]
