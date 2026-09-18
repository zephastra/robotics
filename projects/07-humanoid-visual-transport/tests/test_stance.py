import numpy as np
from types import SimpleNamespace
import mujoco
from humanoid007.stance import candidate_positions,StancePlanner
from humanoid007.runtime import Runtime
from humanoid007.diagnostics import ContactMonitor


def test_candidates_follow_visual_target_without_fixed_global_goal():
    a=candidate_positions(np.array([.23,0,.85]),np.array([-1,0]))
    b=candidate_positions(np.array([1.23,.2,.85]),np.array([-1,0]))
    assert len(a)==len(b)==12
    assert np.allclose(np.asarray(b)-a,[1,.2])
    placement=candidate_positions(np.array([1.23,0,.85]),np.array([-1,0]),'place')
    assert len(placement)==18
    assert np.isclose(min(p[0] for p in placement),1.23-.44)


def test_place_search_includes_bounded_current_lateral_position():
    center=np.array([1.23,0.,.85])
    positions=candidate_positions(center,np.array([.85,.11]),'place')
    assert sum(np.isclose(p[1],.11) for p in positions)==6
    outside=candidate_positions(center,np.array([.85,.3]),'place')
    assert all(abs(p[1])<=.04 for p in outside)
    grasp=candidate_positions(center,np.array([.85,.11]),'grasp')
    assert all(abs(p[1])<=.04 for p in grasp)


def test_realign_prefers_sampled_interior_over_nearest_reach_edge(monkeypatch):
    import humanoid007.stance as module
    planner=StancePlanner.__new__(StancePlanner);planner.original=np.array([.90,.1,0,0,0,0,0])
    monkeypatch.setattr(module,'candidate_positions',lambda *args:[np.array([x,.1]) for x in (.9,.94,.98,1.02)])
    planner.evaluate=lambda xy,*args:dict(feasible=xy[0]<1.,body_clearance=.1,error=.001)
    plan=planner.select(np.array([1.23,0,.85]),np.array([0,1,0]),np.zeros(10),'place',
                        hand_offsets={'left':np.array([0,.3,0]),'right':np.array([0,-.3,0])})
    assert np.allclose(plan['selected']['xy'],[.94,.1])
    assert plan['selected']['longitudinal_neighbors']==2
    assert not plan['candidates'][-1]['feasible']


def test_planning_does_not_change_live_physics_and_ignores_payload_truth():
    r=Runtime();q=r.d.qpos.copy();pos=r.m.geom_pos.copy()
    p=StancePlanner(r,np.array([1.23,0,.81]))
    first=p.select(np.array([.23,0,.85]),np.array([0,1,0]),r.policy.default[r.arm_ids],'grasp')
    assert first['selected'] is not None
    assert np.array_equal(r.d.qpos,q) and np.array_equal(r.m.geom_pos,pos)
    address=r.m.joint('payload_free').qposadr[0]
    r.d.qpos[address:address+3]=[20,20,20]
    second=StancePlanner(r,np.array([1.23,0,.81])).select(np.array([.23,0,.85]),np.array([0,1,0]),r.policy.default[r.arm_ids],'grasp')
    assert first==second


def test_unreachable_object_rejected():
    r=Runtime()
    plan=StancePlanner(r,np.array([1.23,0,.81])).select(np.array([.23,0,3.]),np.array([0,1,0]),r.policy.default[r.arm_ids],'grasp')
    assert plan['selected'] is None
    assert all(not c['feasible'] for c in plan['candidates'])


def test_placement_candidates_use_observed_grasp_offsets():
    planner=StancePlanner.__new__(StancePlanner)
    planner.original=np.zeros(8)
    seen=[]
    def evaluate(xy,targets,seed,operation):
        seen.append(targets)
        return dict(feasible=True,body_clearance=.1,error=0.)
    planner.evaluate=evaluate
    center=np.array([1.,0.,.85])
    offsets={'left':np.array([.01,.29,.004]),'right':np.array([-.01,-.31,.006])}
    planner.select(center,np.array([0,1,0]),np.zeros(1),'place',hand_offsets=offsets)
    assert seen
    for targets in seen:
        for side in offsets:assert np.allclose(targets[side],center+offsets[side])


def test_candidate_translation_preserves_measured_body_orientation():
    r=Runtime()
    r.d.qpos[3:7]=[np.cos(.05),0,0,np.sin(.05)]
    planner=StancePlanner(r,np.array([1.23,0,.81]))
    seen=[]
    def unreachable(targets,seed,**kwargs):
        seen.append(planner.r.d.qpos[3:7].copy())
        # Placement starts with the actual lowering target, not an invented
        # extra raised pose that is never commanded by the task.
        assert all(np.array_equal(p,np.zeros(3)) for p in targets.values())
        return seed
    planner.r.arm_ik=unreachable
    planner.r.arm_error=lambda targets,seed:1.
    planner.evaluate([.5,.1],{'left':np.zeros(3),'right':np.zeros(3)},r.policy.default[r.arm_ids],'place')
    assert np.array_equal(seen[0],r.d.qpos[3:7])


def test_snapshot_reports_actual_arm_tracking_error():
    r=Runtime()
    r.arm_target=r.d.qpos[r.qa[r.arm_ids]].copy()
    r.arm_target[0]+=.2
    snapshot=r.snapshot()
    assert np.isclose(snapshot['arm_tracking_error'],.2)
    assert len(snapshot['arm_joints'])==len(snapshot['arm_command'])==10


def test_joint_limited_ik_never_worsens_seed_position_error():
    r=Runtime();seed=r.policy.default[r.arm_ids].copy()
    for center in ([.6,0,.7],[2.,0,.85],[.15,0,1.3]):
        targets={s:np.array(center)+[0,sign*.3,0] for s,sign in [('left',1),('right',-1)]}
        before=r.arm_error(targets,seed)
        solved=r.arm_ik(targets,seed,safeguarded=True)
        assert r.arm_error(targets,solved)<=before+1e-10


def test_manipulation_preserves_elbow_branch_without_changing_model_limits():
    r=Runtime();seed=r.policy.default[r.arm_ids].copy();limits=r.m.jnt_range.copy()
    targets={'left':np.array([.6,.3,.7]),'right':np.array([.6,-.3,.7])}
    result=r.arm_ik(targets,seed,safeguarded=True,posture_reference=seed)
    assert np.all(result[[3,8]]<=-.1+1e-12)
    assert np.all(np.abs(result[[4,9]]-seed[[4,9]])<=.25+1e-12)
    assert np.array_equal(r.m.jnt_range,limits)


def test_place_score_prefers_reach_margin_without_accepting_infeasible_pose(monkeypatch):
    import humanoid007.stance as module
    planner=StancePlanner.__new__(StancePlanner);planner.original=np.zeros(8)
    monkeypatch.setattr(module,'candidate_positions',lambda *args:[np.array([.8,0]),np.array([.84,0]),np.array([.9,0])])
    def evaluate(xy,*args):
        return dict(feasible=xy[0]!=.9,body_clearance=.1,error=.01 if xy[0]==.8 else 0.)
    planner.evaluate=evaluate
    result=planner.select(np.array([1.2,0,.85]),np.array([0,1,0]),np.zeros(10),'place')
    assert result['selected']['xy']==[.84,0.]


def test_posture_hold_requires_loaded_feet_and_releases_capture():
    r=Runtime();r.feet_loaded=lambda:False
    for _ in range(110):r.step(np.zeros(3),stationary=True)
    assert r.stand_target is None and r.stand_weight==0
    # Exercise the controller gate independently of a particular gait phase.
    r.feet_loaded=lambda:True
    for _ in range(110):r.step(np.zeros(3),stationary=True)
    assert r.stand_target is not None and 0<r.stand_weight<1
    for _ in range(110):r.step(np.zeros(3),stationary=False)
    assert r.stand_target is None and r.stand_weight==0


def test_contacts_have_force_duration_and_do_not_count_air():
    m=mujoco.MjModel.from_xml_string('<mujoco><worldbody><geom name="source_table" type="plane" size="1 1 .1"/><body name="robot" pos="0 0 .08"><freejoint/><geom type="sphere" size=".1" mass="1"/></body></worldbody></mujoco>')
    r=SimpleNamespace(m=m,d=mujoco.MjData(m));monitor=ContactMonitor()
    assert monitor.summary()['contact_free']
    for _ in range(20):
        mujoco.mj_step(m,r.d);monitor.sample(r,'TEST')
    result=monitor.summary();assert not result['contact_free']
    pair=result['pairs']['source_table / robot']
    assert pair['peak_normal_force_n']>0 and 0<pair['contact_seconds']<=r.d.time+1e-9
    assert pair['normal_impulse_ns']>0 and pair['phases']==['TEST']


def test_approach_braking_opposes_velocity_without_changing_goal():
    r=Runtime.__new__(Runtime)
    r.d=SimpleNamespace(qpos=np.array([0.,0.,1.,1.,0.,0.,0.]),qvel=np.array([.1,0,0,0,0,0]))
    r.m=SimpleNamespace(opt=SimpleNamespace(timestep=.002));r.integral=np.zeros(2)
    goal=np.array([.1,0.,0.]);saved=goal.copy()
    usual=r.hold_command(goal);braked=r.hold_command(goal,brake=True)
    assert braked[0]<usual[0] and np.max(np.abs(braked[:2]))<=.30
    assert np.array_equal(goal,saved)


def test_support_feedback_does_not_mistake_pelvis_lean_for_progress():
    r=Runtime.__new__(Runtime)
    r.d=SimpleNamespace(qpos=np.array([.9,0.,1.,1.,0,0,0]),qvel=np.zeros(6))
    r.m=SimpleNamespace(opt=SimpleNamespace(timestep=.002));r.integral=np.zeros(2)
    goal=np.array([.98,0.,0.]);support=np.array([.9,0.])
    first=r.hold_command(goal,position=support,alignment=True,brake=True)
    r.d.qpos[0]=.96
    leaned=r.hold_command(goal,position=support,alignment=True,brake=True)
    assert np.allclose(first,leaned)
    assert np.allclose(goal,[.98,0.,0.])


def test_realign_arrival_requires_support_translation_not_only_pelvis():
    from humanoid007.task import Mission
    r=SimpleNamespace(d=SimpleNamespace(time=12.,qpos=np.array([.96,0.,1.,1.,0,0,0]),qvel=np.zeros(6)),
                      gaze=lambda p:np.zeros(2),support_midpoint=lambda:np.array([.90,0.,0.]))
    m=Mission(r);m.phase='CARRY';m.route=[];m.since=0.;m.stable_since=0.;m.retries=1
    m.alignment_support_offset=np.zeros(2);m.goal[:2]=[.98,0.]
    m.destination=SimpleNamespace(center=np.array([1.23,0,.81]))
    touch=dict(left=1.,right=1.,source=False,destination=False)
    m.update(touch);assert m.phase=='CARRY'
    r.support_midpoint=lambda:np.array([.97,0.,0.])
    r.d.time=13.;m.update(touch);r.d.time=13.2;m.update(touch)
    assert m.phase=='STOP'


def test_explicit_stop_drains_command_without_changing_normal_slew():
    normal=Runtime();stopping=Runtime()
    normal.command[:]=stopping.command[:]=[.10,-.10,.10]
    normal.step(np.zeros(3))
    stopping.step(np.zeros(3),stopping=True)
    assert np.allclose(normal.command,[.096,-.096,.096])
    assert np.allclose(stopping.command,[.08,-.08,.08])
    # Commands converge toward zero without sign reversal. Physics is stepped,
    # never reset or held by modifying qpos.
    for _ in range(25):stopping.step(np.zeros(3),stopping=True)
    assert np.allclose(stopping.command,0)
    assert stopping.d.time>0


def test_alignment_can_keep_integral_authority_and_velocity_braking():
    r=Runtime.__new__(Runtime)
    r.d=SimpleNamespace(qpos=np.array([0.,0.,1.,1.,0.,0.,0.]),qvel=np.array([.1,0,0,0,0,0]))
    r.m=SimpleNamespace(opt=SimpleNamespace(timestep=.002));r.integral=np.array([.1,0.])
    goal=np.array([.08,0.,0.]);integral=r.integral.copy()
    usual=r.hold_command(goal,precise=True,alignment=True)
    r.integral=integral.copy()
    damped=r.hold_command(goal,precise=True,alignment=True,brake=True)
    assert np.isclose(usual[0]-damped[0],.09)
    assert r.integral[0]>.1 and np.max(np.abs(damped[:2]))<=.3


def test_automatic_final_stop_keeps_selected_goal():
    from humanoid007.task import Mission
    r=SimpleNamespace(d=SimpleNamespace(time=12.,qpos=np.array([0.,0.,1.,1.,0,0,0]),qvel=np.zeros(6)),gaze=lambda p:np.zeros(2))
    m=Mission(r);m.phase='CARRY';m.route=[];m.since=0.;m.stable_since=0.
    m.goal[:2]=[.04,0.];m.destination=SimpleNamespace(center=np.array([1.,0,.8]))
    m.update(dict(left=1.,right=1.,source=False,destination=False))
    assert m.phase=='STOP' and np.allclose(m.goal[:2],[.04,0])
    assert m.hold_anchor is None


def test_realign_waits_for_low_speed_before_stop():
    from humanoid007.task import Mission
    r=SimpleNamespace(d=SimpleNamespace(time=12.,qpos=np.array([0.,0.,1.,1.,0,0,0]),qvel=np.zeros(6)),
                      command=np.array([-.15,0.,0.]),gaze=lambda p:np.zeros(2))
    m=Mission(r);m.phase='CARRY';m.route=[];m.since=0.;m.stable_since=0.;m.retries=1
    m.goal[:2]=[.02,0.];m.destination=SimpleNamespace(center=np.array([1.,0,.8]))
    touch=dict(left=1.,right=1.,source=False,destination=False)
    r.d.qvel[0]=.12
    m.update(touch)
    assert m.phase=='CARRY'
    r.d.qvel[0]=0;r.d.time=13.;m.update(touch)
    r.d.time=13.2;m.update(touch)
    assert m.phase=='STOP' and m.hold_anchor is None


def test_hold_anchor_created_only_after_continuous_measured_stop():
    from humanoid007.task import Mission
    r=SimpleNamespace(d=SimpleNamespace(time=2.,qpos=np.array([.5,.1,1.,1.,0,0,0]),qvel=np.zeros(6)),gaze=lambda p:np.zeros(2))
    m=Mission(r);m.phase='STOP';m.since=0.;m.goal[:2]=[.45,0.]
    m.destination_marker=np.zeros(3)
    touch=dict(left=1.,right=1.,source=False,destination=False)
    m.update(touch);assert m.hold_anchor is None
    r.d.time=2.7;m.update(touch)
    assert m.phase=='INSPECT_DESTINATION' and np.allclose(m.hold_anchor,[.5,.1,0])
    assert np.allclose(m.goal[:2],[.45,0])


def test_stop_anchors_measured_pose_not_planned_goal():
    from humanoid007.task import Mission
    r=SimpleNamespace(d=SimpleNamespace(time=2.,qpos=np.array([.5,.1,1.,1.,0,0,0]),qvel=np.zeros(6)),gaze=lambda p:np.zeros(2))
    m=Mission(r);m.phase='STOP';m.since=0.;m.destination_marker=np.zeros(3)
    touch=dict(left=1.,right=1.,source=False,destination=False)
    m.goal=np.array([.4,.1,0.])
    m.update(touch)
    assert m.phase=='STOP' and m.hold_anchor is None
    r.d.qpos[0]=.41;r.d.time=4.;m.update(touch)
    assert m.phase=='INSPECT_DESTINATION' and m.hold_anchor is not None
    assert np.allclose(m.hold_anchor,[.41,.1,0.])
    assert np.allclose(m.goal,[.4,.1,0.])


def test_alignment_integral_is_bounded_and_requires_explicit_mode():
    r=Runtime.__new__(Runtime)
    r.d=SimpleNamespace(qpos=np.array([0.,0.,1.,1.,0.,0.,0.]),qvel=np.zeros(6))
    r.m=SimpleNamespace(opt=SimpleNamespace(timestep=.002));r.integral=np.zeros(2)
    for _ in range(10000):r.hold_command([.08,0,0],precise=True)
    assert np.isclose(r.integral[0],.12)
    for _ in range(10000):command=r.hold_command([.08,0,0],precise=True,alignment=True)
    assert np.isclose(r.integral[0],.35) and abs(command[0])<=.5


def test_scratch_kinematics_matches_full_forward_poses_and_jacobians():
    from humanoid007.runtime import forward_kinematics
    r=Runtime();rng=np.random.default_rng(7)
    for _ in range(4):
        full=mujoco.MjData(r.m);lite=mujoco.MjData(r.m)
        full.qpos[:]=r.d.qpos
        full.qpos[r.qa]+=rng.uniform(-.02,.02,len(r.qa))
        lite.qpos[:]=full.qpos
        mujoco.mj_forward(r.m,full);forward_kinematics(r.m,lite,cameras=True)
        for key in ('geom_xpos','geom_xmat','site_xpos','cam_xpos','cam_xmat'):
            assert np.allclose(getattr(full,key),getattr(lite,key),atol=1e-12,rtol=0)
        a=np.zeros((3,r.m.nv));b=np.zeros_like(a)
        mujoco.mj_jacSite(r.m,full,a,None,r.m.site('lh_grasp').id)
        mujoco.mj_jacSite(r.m,lite,b,None,r.m.site('lh_grasp').id)
        assert np.allclose(a,b,atol=1e-12,rtol=0)
