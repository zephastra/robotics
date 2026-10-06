"""Guest integration contracts, no physical stepping or ROS launch."""
import ast
from pathlib import Path
import sys
from types import SimpleNamespace
import pytest

ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'experiments'),str(ROOT/'src')]


@pytest.mark.parametrize('goal',[[float('nan'),0,0],[0,float('inf'),0],[51,0,0],[0,0,4],[0,0]])
def test_declared_goal_rejects_invalid_domain(goal):
    from joint_nav_worker import declared_goal
    with pytest.raises(ValueError):declared_goal(goal,[1,2,0])


def test_declared_goal_default_and_explicit_goal_are_separate():
    from joint_nav_worker import declared_goal
    original=[1,2,0]
    assert declared_goal(None,original)==original
    assert declared_goal([5,0,0],original)==[5,0,0]
    assert original==[1,2,0]


def test_guest_has_no_initialization_or_extra_physical_owner():
    source=(ROOT/'experiments/continuous_nav_session.py').read_text()
    tree=ast.parse(source)
    calls=[ast.unparse(n.func) for n in ast.walk(tree) if isinstance(n,ast.Call)]
    assert not set(calls)&{'mujoco.mj_step','mujoco.MjModel','mujoco.MjData',
                           'mujoco.mj_resetDataKeyframe','WorldOwner','H1_runtime'}
    assert 'data.qpos[' not in source
    assert 'return io.control(allowed)' in source
    assert 'finally:pv.W,pv.H=old' in source


def test_source_leg_does_not_claim_loaded_navigation():
    from continuous_nav_session import ContinuousNavSession
    session=ContinuousNavSession.__new__(ContinuousNavSession)
    calls=[]
    session.plant=SimpleNamespace(drive_to=lambda x,**kw:calls.append((x,kw)) or {'stopped_confirmed':True})
    session.used=False
    result=session.move(dict(id='station_c',approach_x_m=4.4,approach_speed_mps=.3,approach_timeout_s=30))
    assert result=={'stopped_confirmed':True} and len(calls)==1 and session.used is False


@pytest.mark.parametrize('used,station',[(True,'station_b'),(False,'station_other')])
def test_guest_refuses_replay_or_uncommissioned_station(used,station):
    from continuous_nav_session import ContinuousNavSession
    session=ContinuousNavSession.__new__(ContinuousNavSession);session.used=used
    with pytest.raises(RuntimeError,match='NOT_COMMISSIONED'):session.move({'id':station})


def test_failed_navigation_cannot_be_promoted_by_small_arrival_error():
    from workcell.adapters.sim import SkillAdapter
    adapter=SkillAdapter(plant=SimpleNamespace(clock=lambda:0),boot_id='boot',
        stations={'station_b':{'approach_x_m':5}},envelope={'approach_m':.25},
        navigation_move=lambda station:dict(error_x_m=0,stopped_confirmed=True,end_s=1,
                                            navigation_succeeded=False))
    result=adapter.execute(dict(skill='MOVE_TO_STATION',arguments={'station_id':'station_b'},
        command_id='cmd',order_id='order',revision=1),now_s=0)
    assert result['status']=='FAILED' and result['reason_code']=='NAV_FAILED'


def test_docking_never_uses_navigation_callback():
    from workcell.adapters.sim import SkillAdapter
    called=[]
    plant=SimpleNamespace(clock=lambda:0,
        drive_to=lambda x,**kw:dict(end_s=1,stopped_confirmed=True),
        dock_residuals=lambda station:dict(lateral_m=0,longitudinal_m=0,yaw_rad=0))
    adapter=SkillAdapter(plant=plant,boot_id='boot',stations={'station_b':{'id':'station_b',
        'dock_x_m':5,'dock_timeout_s':30}},envelope={'dock_speed_mps':.06,
        'dock_lateral_m':.002,'dock_longitudinal_m':.02,'dock_yaw_rad':.0035},
        navigation_move=lambda station:called.append(station))
    result=adapter.execute(dict(skill='DOCK',arguments={'station_id':'station_b'},
        command_id='cmd',order_id='order',revision=1),now_s=0)
    assert result['status']=='SUCCEEDED' and not called


def test_load_navigation_is_mandatory_in_top_level_acceptance():
    from probe_candidate_multi_loading import aggregate_acceptance,REQUIRED
    report=dict(scope='TEST',nav2_required=True,transport_diagnostic_result='PASS')
    answer=aggregate_acceptance(report,{k:True for k in REQUIRED},True)
    assert answer['checks']['continuous_loaded_nav2']=='NOT_RUN'
    assert answer['diagnostic_result']=='FAIL'


def test_localization_prior_is_station_declaration_not_runtime_pose():
    source=(ROOT/'experiments/continuous_nav_session.py').read_text()
    assert "source_prior=[float(plant.stations['station_c']['dock_x_m']),0.,0.]" in source
    assert "' --start '+' '.join(map(str,source_prior))" in source
    assert 'data.xpos' not in source.split("' --start '")[1].split('owned.spawn')[0]


def test_exception_chain_preserves_controller_error_under_final_writer_hold():
    from probe_vehicle_nav2 import exception_chain
    try:
        try:raise ValueError('joint left declared transport posture')
        except ValueError:raise RuntimeError('CONTROL_UNAVAILABLE')
    except RuntimeError as error:
        assert exception_chain(error)==['RuntimeError: CONTROL_UNAVAILABLE',
            'ValueError: joint left declared transport posture']
