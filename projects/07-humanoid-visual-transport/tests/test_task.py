from types import SimpleNamespace
import numpy as np
from humanoid007.task import Mission
from humanoid007.task import corridor_recovery_ready


def test_stability_requires_continuous_condition():
    m=Mission(SimpleNamespace(d=SimpleNamespace(time=0.)))
    assert not m.stable(True,1.)
    m.r.d.time=.9
    assert not m.stable(False,1.)
    m.r.d.time=1.1
    assert not m.stable(True,1.)
    m.r.d.time=2.2
    assert m.stable(True,1.)


def test_failure_is_terminal():
    m=Mission(SimpleNamespace(d=SimpleNamespace(time=0.)))
    m.finish('FAILED','TEST')
    m.update({})
    assert m.reason=='TEST'


def test_missing_target_does_not_start_reach():
    r=SimpleNamespace(d=SimpleNamespace(time=6.,qpos=np.array([0,0,1.,1.,0,0,0]),qvel=np.zeros(6)))
    m=Mission(r);m.phase='SEARCH';m.since=4.
    m.update(dict(left=0,right=0,source=False,destination=False))
    assert m.phase=='SEARCH' and m.arm is None


def robot(time=2.,speed=.3):
    return SimpleNamespace(d=SimpleNamespace(time=time,qpos=np.array([0,0,1.,1.,0,0,0]),
                                            qvel=np.array([speed,0,0,0,0,0])),
                           gaze=lambda point:np.zeros(2))


def test_intermediate_waypoint_does_not_require_manipulation_stop():
    m=Mission(robot())
    m.phase='CARRY';m.stable_since=0.
    m.destination=SimpleNamespace(center=np.array([1.,0.,.8]))
    m.route=[np.array([1.,0.])]
    m.update(dict(left=1.,right=1.,source=False,destination=False))
    assert m.phase=='CARRY' and np.allclose(m.goal[:2],[1,0])
    assert not m.route


def test_final_stop_requires_low_base_speed():
    m=Mission(robot())
    m.phase='STOP';m.since=0.;m.stable_since=0.
    m.destination_marker=np.array([1.,.75,.8])
    m.update(dict(left=1.,right=1.,source=False,destination=False))
    assert m.phase=='STOP'


def test_stale_target_aborts_before_grasp():
    m=Mission(robot(speed=0.))
    m.phase='REACH';m.box=SimpleNamespace(center=np.zeros(3),time=0.)
    m.update(dict(left=0.,right=0.,source=False,destination=False))
    assert m.result=='FAILED' and m.reason=='TARGET_LOST_BEFORE_GRASP'


def test_one_hand_contact_loss_is_not_success():
    m=Mission(robot(speed=0.))
    m.phase='LIFT';m.loss_since=0.
    m.update(dict(left=0.,right=1.,source=False,destination=False))
    assert m.result=='FAILED' and m.reason=='BILATERAL_CONTACT_LOST'


def test_corridor_recovery_requires_reaching_cross_section():
    assert corridor_recovery_ready([0,0],[0,-.65],[.095,-.65])
    assert not corridor_recovery_ready([0,0],[0,-.65],[.095,-.55])
    assert not corridor_recovery_ready([0,0],[0,-.65],[.2,-.65])
    assert not corridor_recovery_ready([0,0],[0,-.65],[0,-.8])
    assert not corridor_recovery_ready([0,0],[0,-.1],[0,-.1])


def test_final_waypoint_cannot_use_corridor_recovery():
    m=Mission(robot(time=12.,speed=0.))
    m.phase='CARRY';m.since=0.;m.stable_since=0.
    m.carry_origin=np.array([0.,.65]);m.goal[:2]=[.095,0.]
    m.destination=SimpleNamespace(center=np.array([1.,0.,.8]));m.route=[]
    m.update(dict(left=1.,right=1.,source=False,destination=False))
    assert m.phase=='CARRY' and m.waypoint_recoveries==0


def test_intermediate_recovery_is_bounded_and_logged():
    m=Mission(robot(time=12.,speed=0.))
    m.phase='CARRY';m.since=0.;m.stable_since=0.
    m.carry_origin=np.array([0.,.65]);m.goal[:2]=[.095,0.]
    m.destination=SimpleNamespace(center=np.array([1.,0.,.8]));m.route=[np.array([1.,0.])]
    m.update(dict(left=1.,right=1.,source=False,destination=False))
    assert m.waypoint_recoveries==1 and np.allclose(m.goal[:2],[1,0])
    assert any(e.get('recovery')=='CORRIDOR_CROSS_SECTION' for e in m.events)


def test_waypoint_recovery_limit_cannot_loop_forever():
    m=Mission(robot(time=12.,speed=0.))
    m.phase='CARRY';m.since=0.;m.stable_since=0.;m.waypoint_recoveries=2
    m.carry_origin=np.array([0.,.65]);m.goal[:2]=[.095,0.]
    m.destination=SimpleNamespace(center=np.array([1.,0.,.8]));m.route=[np.array([1.,0.])]
    m.update(dict(left=1.,right=1.,source=False,destination=False))
    assert m.result=='FAILED' and m.reason=='WAYPOINT_RECOVERY_LIMIT'


def test_lowering_safeguard_is_scoped_to_automatic_manipulation():
    for mode in ('auto','legacy'):
        r=robot(speed=0.);calls=[]
        def solve(targets,seed,**kwargs):
            calls.append(kwargs);return np.zeros(10)
        r.arm_ik=solve
        m=Mission(r,stance=mode);m.phase='LOWER';m.since=1.
        m.lower_start={'left':np.zeros(3),'right':np.zeros(3)}
        m.lower_delta=np.array([0.,0.,-.1]);m.arm=np.zeros(10)
        m.manipulation_reference=np.zeros(10)
        m.update(dict(left=1.,right=1.,source=False,destination=False))
        assert len(calls)==1 and calls[0]['safeguarded']==(mode=='auto')
        assert np.array_equal(calls[0]['posture_reference'],m.manipulation_reference)


def test_retraction_does_not_add_release_displacement_twice():
    r=robot(time=8.,speed=0.);commands=[]
    r.arm_ik=lambda targets,seed,**kwargs: commands.append(targets) or np.zeros(10)
    positions={'lh_grasp':np.array([1.,.325,.865]),'rh_grasp':np.array([1.,-.325,.865])}
    r.d.site=lambda name:SimpleNamespace(xpos=positions[name])
    m=Mission(r);m.phase='RELEASE';m.since=2.;m.arm=np.zeros(10)
    m.manipulation_reference=np.zeros(10)
    m.release_start={s:np.zeros(16) for s in ('left','right')}
    m.release_arm_start={'left':np.array([1.,.3,.85]),'right':np.array([1.,-.3,.85])}
    touch=dict(left=0.,right=0.,source=False,destination=True)
    m.update(touch)
    assert m.phase=='RETRACT'
    assert np.allclose(m.retract_target['left'],[.92,.34,.99])
    assert np.allclose(m.retract_target['right'],[.92,-.34,.99])
    m.update(touch)
    assert np.allclose(commands[-1]['left'],positions['lh_grasp'])
    r.d.time=10.;m.update(touch)
    assert np.allclose(commands[-1]['left'],[.96,.325,.95])
    assert np.allclose(commands[-1]['right'],[.96,-.325,.95])
    r.d.time=12.;m.update(touch)
    assert np.allclose(commands[-1]['left'],m.retract_target['left'])


def test_withdrawal_targets_do_not_mutate_or_alias_release_anchor():
    from humanoid007.runtime import withdrawal_waypoints
    anchors={'left':np.array([1.,.3,.85]),'right':np.array([1.,-.3,.85])}
    saved={s:p.copy() for s,p in anchors.items()}
    clear,end=withdrawal_waypoints(anchors)
    for side in anchors:
        assert np.array_equal(anchors[side],saved[side])
        assert clear[side][0]<anchors[side][0]
        assert end[side][0]<clear[side][0]
        assert anchors[side][2]<clear[side][2]<end[side][2]
        assert not np.shares_memory(clear[side],anchors[side])
        assert not np.shares_memory(end[side],clear[side])
