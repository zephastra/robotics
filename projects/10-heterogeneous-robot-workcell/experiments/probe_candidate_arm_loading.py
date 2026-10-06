"""Independent real-tray RGB-D pick/place probe, not H-to-order acceptance.

Only initialization may position the tray at its declared loading station.
Runtime commands use RGB-D part estimates and a declared stationary tray fixture.
Simulation truth is used only for the independent physical judge. No weld/reset.
"""
import argparse
import hashlib
import json
import sys
import time
from pathlib import Path

import mujoco
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT/'experiments'), str(ROOT/'src')]
import arm_bridge
import arm_rig as rig
import probe_p3_vision as pv
from probe_p4_vision import decide
from workcell.runtime_permit import RuntimePermit
from workcell.tray import derive_cells, cell_centre_world, cell_of
from w4_plant import LogisticsPlant
from world_owner import WorldOwner
import tray_rgbd_reader

WORLD = ROOT/'assets/world_p5_candidate_v3.xml'
CONVENTION = dict(forward=2, fwd_sign=-1., up_sign=-1., v_sign=1.)
STEPS = dict(approach=700, descend=400, close=500, lift=500,
             carry=900, lower=600, release=400, retreat=600, clear_view=700)
REQUIRED = ('rgbd_resolved_red','missing_depth_unknown','unknown_no_physics_step',
    'localized_within_original_20mm','actual_target_cell','actual_tray_support',
    'physical_lift_over_50mm','one_final_writer','no_runtime_teleport',
    'initial_observed_empty','final_visual_count_one_red','visual_count_correct_cell',
    'missing_count_unknown_not_zero')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run-id', required=True)
    parser.add_argument('--cell', type=int, choices=(0,1,2), default=1)
    args = parser.parse_args()
    out = ROOT/'reports'/args.run_id
    out.mkdir(parents=True, exist_ok=False)
    report = dict(scope='INITIALIZED_REAL_TRAY_ONE_RED_PART', phases=[],
        world_sha256=hashlib.sha256(WORLD.read_bytes()).hexdigest(),
        source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        runtime_qpos_writes=0, full_order='NOT_RUN', v1_complete=False,
        camera_convention='declared MuJoCo camera axes; no runtime oracle calibration',
        stationary_tray_pose='declared initialization, not runtime truth localization')
    checks, view = {}, None
    try:
        base = mujoco.MjModel.from_xml_path(str(WORLD))
        initial = mujoco.MjData(base)
        mujoco.mj_resetDataKeyframe(base, initial, 0)
        mujoco.mj_forward(base, initial)
        loading_x = float(initial.geom_xpos[base.geom('c_fixed_roller_1_2').id][0])
        bench_z, _ = pv.bench_geometry(base, initial)
        stock = initial.xpos[base.body('a_payload').id].copy()
        aim = [0.5*(loading_x+stock[0]), -.235, bench_z+.02]
        camera_pos = [aim[0], aim[1]-.55, bench_z+1.]
        spec = mujoco.MjSpec.from_file(str(WORLD))
        # Declared class coloration only, never cargo collision/material mechanics.
        spec.body('a_payload').geoms[0].rgba = pv.RED_RGBA
        camera = spec.worldbody.add_camera(name=pv.CAM, pos=camera_pos,
            quat=pv.look_at(camera_pos, aim), fovy=45.)
        model = spec.compile()
        data = mujoco.MjData(model)
        mujoco.mj_resetDataKeyframe(model, data, 0)
        adr = pv.free_addr(model, 'c_payload')
        data.qpos[adr] = loading_x  # sole declared independent tray initialization
        mujoco.mj_forward(model, data)
        report['initial_qpos_writes'] = 1
        plant = LogisticsPlant(model=model, data=data, world=WORLD)
        sequence = [0]
        def observe():
            sequence[0] += 1
            return dict(owner='shared_world', epoch=1, generation=1,
                sequence=sequence[0], observed_wall_s=time.monotonic(),
                cancel_requested=False, zone_clear=True, stop_chain_healthy=True)
        permit = RuntimePermit(observe, owner='shared_world', epoch=1,
                               generation=1, max_age_s=.5)
        owner = WorldOwner(model, data, permit)
        owner.attach(plant)
        arm_bridge.install('a_')
        ids = rig.ids(model)
        arm_ctrl = [model.actuator('a_actuator%d'%i).id for i in range(1,8)]
        grip_ctrl = model.actuator('a_actuator8').id
        arm_command = data.qpos[ids['arm_qadr']].copy()
        grip = [rig.GRIPPER_CTRL_OPEN]
        def control():
            ctrl = plant.park_control()
            ctrl[arm_ctrl] = arm_command
            ctrl[grip_ctrl] = grip[0]
            return ctrl
        cells = derive_cells(model, data)
        deck_top = float(cells['deck']['top_z'])
        # This is a fixture target derived ONCE at initialization; later tray drift
        # is independently judged and cannot be fed back as perfect perception.
        g = pv.part_size(model)
        target = np.array(cell_centre_world(cells, args.cell, z=deck_top+g[2]))
        tray_initial = data.xpos[model.body('c_payload').id].copy()
        # Fixture targets are derived before FIRST physics step, never from the
        # moving tray's true pose. Post-settle truth remains independent judging.
        for _ in range(250):
            owner.step(control)
        plan = mujoco.MjData(model)
        rotation = rig.home_hand_rot(model, plan)  # planning state only
        # Home hand can occlude the tray. Physically withdraw over the declared
        # fixed stock table before attempting EMPTY verification, never guess 0.
        _top, stock_roi = pv.bench_geometry(model,data)
        clear = np.array([(stock_roi[0]+stock_roi[1])/2,
                          (stock_roi[2]+stock_roi[3])/2,
                          bench_z+g[2]+rig.RETREAT_HEIGHT])
        plan.qpos[:] = data.qpos
        q0 = data.qpos[ids['arm_qadr']].copy()
        clear_q, _, _ = rig.ik(model,plan,clear,target_rot=rotation,q_seed=q0,iters=600)
        for k in range(1,701):
            arm_command[:] = q0+(clear_q-q0)*k/700
            owner.step(control)
        view = mujoco.Renderer(model, pv.H, pv.W)
        option = mujoco.MjvOption()
        option.geomgroup[5] = 0  # approved massless optical marker only
        def capture(depth=False):
            view.disable_depth_rendering()
            if depth:
                view.enable_depth_rendering()
            view.update_scene(data, camera=pv.CAM, scene_option=option)
            return view.render().copy()
        rgb, depth = capture(), capture(True)
        report['initial_visual_count'] = tray_rgbd_reader.read(model,data,rgb,depth,
            cells,g,CONVENTION)
        perception = pv.analyze(model, data, rgb, depth, bench_z, g, CONVENTION)
        decision, reasons = decide(perception, g)
        report['perception'] = dict(decision={k:None if v is None else v.tolist()
            for k,v in decision.items()}, reasons=reasons,
            components=[{k:v for k,v in c.items() if k not in ('pixels','rgb')}
                        for c in perception['comps']])
        checks['rgbd_resolved_red'] = decision['red'] is not None
        # Actual decision under missing depth must refuse, before any arm command.
        before = owner.steps
        missing = pv.analyze(model, data, rgb, np.zeros_like(depth), bench_z, g, CONVENTION)
        missing_decision, _ = decide(missing, g)
        checks['missing_depth_unknown'] = missing_decision['red'] is None
        checks['unknown_no_physics_step'] = owner.steps == before
        if decision['red'] is None:
            raise RuntimeError('RGBD_UNKNOWN: '+reasons['red'])
        pick = np.array([decision['red'][0], decision['red'][1], bench_z+g[2]])
        report['perceived_xy_error_m'] = float(np.linalg.norm(
            pick[:2]-data.xpos[model.body('a_payload').id][:2]))  # judge only
        checks['localized_within_original_20mm'] = report['perceived_xy_error_m'] < pv.LOC_TOL
        report['target_xyz'] = target.tolist()
        seq = [('approach',pick+[0,0,rig.PREGRASP_CLEARANCE],rig.GRIPPER_CTRL_OPEN),
               ('descend',pick,None), ('close',None,rig.GRIPPER_CTRL_CLOSED),
               ('lift',pick+[0,0,rig.LIFT_HEIGHT],None),
               ('carry',target+[0,0,rig.PREGRASP_CLEARANCE],None),
               ('lower',target,None), ('release',None,rig.GRIPPER_CTRL_OPEN),
               ('retreat',target+[0,0,rig.RETREAT_HEIGHT],None),
               ('clear_view',pick+[0,0,rig.RETREAT_HEIGHT],None)]
        for phase, point, grip_value in seq:
            if grip_value is not None:
                grip[0] = grip_value
            q0 = data.qpos[ids['arm_qadr']].copy()
            if point is not None:
                plan.qpos[:] = data.qpos
                q, pe, re = rig.ik(model, plan, np.asarray(point), target_rot=rotation,
                    q_seed=q0, iters=600)
                if pe > .001:
                    raise RuntimeError('IK residual exceeds unchanged 1mm criterion')
            else:
                q = q0
            for k in range(1,STEPS[phase]+1):
                arm_command[:] = q0+(q-q0)*k/STEPS[phase]
                owner.step(control)
            record = dict(phase=phase, sim_s=float(data.time),
                part_xyz=data.xpos[model.body('a_payload').id].tolist(),
                grasp_xyz=rig.grasp_point(model,data).tolist(),
                gripper_gap_m=float(rig.finger_gap(model,data)))
            report['phases'].append(record)
            print(json.dumps(record), flush=True)
        final = data.xpos[model.body('a_payload').id].copy()
        final_cells = derive_cells(model,data)
        touching = any({int(model.geom_bodyid[c.geom1]),int(model.geom_bodyid[c.geom2])}
            == {model.body('a_payload').id,model.body('c_payload').id}
            for c in data.contact[:data.ncon])
        report['final'] = dict(xyz=final.tolist(), cell=cell_of(final_cells,final),
            touching_real_tray=touching,
            tray_drift_m=float(np.linalg.norm(data.xpos[model.body('c_payload').id]-tray_initial)))
        checks['actual_target_cell'] = report['final']['cell']==args.cell
        checks['actual_tray_support'] = touching and abs(final[2]-(
            float(final_cells['deck']['top_z'])+g[2])) < .02
        lift = next(p for p in report['phases'] if p['phase']=='lift')
        checks['physical_lift_over_50mm'] = lift['part_xyz'][2] > bench_z+g[2]+.05
        checks['one_final_writer'] = owner.steps > 0 and plant.step_owner is owner
        checks['no_runtime_teleport'] = report['runtime_qpos_writes']==0
        rgb, depth = capture(), capture(True)
        # The reader gets the same declared initial fixture, NOT final true tray pose.
        observed = tray_rgbd_reader.read(model,data,rgb,depth,cells,g,CONVENTION)
        missing_observed = tray_rgbd_reader.read(model,data,rgb,np.zeros_like(depth),
            cells,g,CONVENTION)
        report['final_visual_count'] = observed
        report['missing_depth_visual_count'] = missing_observed
        checks['initial_observed_empty'] = report['initial_visual_count']['counts']=={'red':0,'blue':0}
        checks['final_visual_count_one_red'] = observed['status']=='RESOLVED' and observed[
            'counts']=={'red':1,'blue':0}
        checks['visual_count_correct_cell'] = bool(observed['parts']) and observed['parts'][0][
            'cell']==args.cell
        checks['missing_count_unknown_not_zero'] = missing_observed['status']=='UNKNOWN' and (
            missing_observed['counts'] is None)
    except Exception as exc:
        report['error'] = type(exc).__name__+': '+str(exc)
        checks['execution_completed'] = False
    finally:
        if view is not None:
            view.close()
    judged = {k:('PASS' if checks[k] else 'FAIL') if k in checks else 'NOT_RUN'
              for k in REQUIRED}
    if 'execution_completed' in checks:
        judged['execution_completed'] = 'FAIL'
    acceptance = dict(scope=report['scope'], checks=judged,
        diagnostic_result='PASS' if all(v=='PASS' for v in judged.values()) else 'FAIL',
        full_order='NOT_RUN', continuous_handover='NOT_RUN',
        visual_count='PASS' if checks.get('final_visual_count_one_red') else 'FAIL',
        v1_complete=False)
    (out/'report.json').write_text(json.dumps(report,indent=2)+'\n')
    (out/'acceptance.json').write_text(json.dumps(acceptance,indent=2)+'\n')
    print(json.dumps(acceptance,indent=2))
    return 0 if acceptance['diagnostic_result']=='PASS' else 1


if __name__=='__main__':
    raise SystemExit(main())
