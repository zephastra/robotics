"""Independent recorded-data judge for actual-loading Nav2 integration.

No executor PASS label is a substitute for the raw action/trajectory/sensor
evidence. No simulation or ROS is started. Full order and 3D safety stay NOT_RUN.
"""
import argparse
import hashlib
import json
import math
from pathlib import Path

ROOT=Path(__file__).resolve().parents[1]
NODES=('map_server','amcl','planner_server','controller_server','smoother_server',
       'behavior_server','bt_navigator','velocity_smoother','collision_monitor')
CARGO=('c_payload','a_payload','p5_red_second','p5_blue_cylinder')
REQUIRED=('execution_completed','actual_supply','actual_loading','action_succeeded',
          'all_nodes_active','single_command_producer','actual_commands','arrival',
          'actual_transport','vehicle_stopped','all_cargo_stopped','tray_retention',
          'load_evidence','transport_posture_authorized','authority_returned','no_runtime_pose_write')


def vector(v,n):
    return isinstance(v,list) and len(v)==n and all(isinstance(x,(int,float))
        and not isinstance(x,bool) and math.isfinite(x) for x in v)


def norm(v):return math.sqrt(sum(x*x for x in v))
def distance(a,b):return norm([x-y for x,y in zip(a,b)])


def loading_evidence(chain):
    """Three distinct stock entities actually rose, not a stage label alone."""
    phases=chain.get('phases',[])
    lifted_names=set()
    for item in range(3):
        rows=[r for r in phases if r.get('item')==item]
        before=next((r.get('truth_for_judge',{}) for r in rows if r.get('phase')=='approach'),{})
        after=next((r.get('truth_for_judge',{}) for r in rows if r.get('phase')=='lift'),{})
        risen=[]
        for name in CARGO[1:]:
            initial,lifted=before.get(name),after.get(name)
            if vector(initial,3) and vector(lifted,3) and lifted[2]-initial[2]>=.05:risen.append(name)
        if len(risen)!=1 or risen[0] in lifted_names:return False
        lifted_names.add(risen[0])
    return lifted_names==set(CARGO[1:])


def supply_evidence(chain):
    supplied=chain.get('humanoid_supply',{})
    trace=supplied.get('trace',[])
    if not trace or supplied.get('abnormal_contacts')!=[]:return False
    z0=trace[0].get('z_m')
    return (isinstance(z0,(int,float)) and math.isfinite(z0)
        and any(r.get('bilateral') is True and isinstance(r.get('z_m'),(int,float))
                and math.isfinite(r['z_m']) and r['z_m']-z0>=.05 for r in trace)
        and supplied.get('checks',{}).get('original_six_stages')=='PASS')


def judge(chain,nav):
    checks={key:False for key in REQUIRED};measurements={}
    checks.update(execution_completed=nav.get('status')=='COMPLETED' and not nav.get('error'),
        actual_supply=chain.get('handover_required') is True and supply_evidence(chain),
        actual_loading=loading_evidence(chain),
        authority_returned=nav.get('authority_returned') is True,
        transport_posture_authorized='posture_motion_refusal' in nav and nav['posture_motion_refusal'] is None,
        no_runtime_pose_write=chain.get('runtime_qpos_writes')==0 and nav.get('runtime_qpos_writes')==0)
    worker=nav.get('nav2_worker') or {}
    checks.update(action_succeeded=worker.get('status')=='SUCCEEDED' and worker.get('goal_accepted') is True
        and worker.get('action_status')==4 and worker.get('action_error_code')==0,
        all_nodes_active=all(worker.get('active',{}).get(n)==3 for n in NODES),
        single_command_producer=worker.get('cmd_vel_publishers')==['collision_monitor'],
        actual_commands=worker.get('nonzero_command_count',0)>0 and nav.get('commands_received',0)>0)
    rows=nav.get('truth_judge_only',[]);goal=nav.get('goal');stop_at=nav.get('stop_at_sim_s')
    valid=isinstance(rows,list) and len(rows)>=2 and all(
        vector(row.get('xyz'),3) and vector(row.get('velocity'),4)
        and isinstance(row.get('sim_s'),(int,float)) and math.isfinite(row['sim_s'])
        and set(row.get('cargo',{}))==set(CARGO)
        and all(vector(c.get('xyz'),3) and vector(c.get('relative_to_base'),3)
            and isinstance(c.get('point_speed_bound_mps'),(int,float))
            and math.isfinite(c['point_speed_bound_mps']) and c['point_speed_bound_mps']>=0
            for c in row['cargo'].values()) for row in rows)
    if valid:valid=all(0<b['sim_s']-a['sim_s']<=.12 for a,b in zip(rows,rows[1:]))
    if valid and vector(goal,3):
        err=distance(rows[-1]['xyz'][:2],goal[:2]);travel=distance(rows[-1]['xyz'][:2],rows[0]['xyz'][:2])
        checks['arrival']=err<=.25;checks['actual_transport']=travel>.3
        base_slip=max(distance(row['cargo']['c_payload']['relative_to_base'][:2],
                              rows[0]['cargo']['c_payload']['relative_to_base'][:2]) for row in rows)
        measurements.update(arrival_error_m=err,travel_m=travel,
                            peak_tray_relative_to_base_m=base_slip)
        # The compliant deck can move relative to the chassis. Original G-2
        # retention is tray-to-DECK, so base-frame drift cannot substitute.
        if all(vector(r['cargo']['c_payload'].get('relative_to_deck'),3) for r in rows):
            slip=max(distance(r['cargo']['c_payload']['relative_to_deck'][:2],
                              rows[0]['cargo']['c_payload']['relative_to_deck'][:2]) for r in rows)
            checks['tray_retention']=slip<=.005
            measurements['peak_tray_relative_slip_m']=slip
        else:measurements['tray_retention_evidence']='MISSING_DECK_FRAME_NOT_MEASURED'
        if isinstance(stop_at,(int,float)) and math.isfinite(stop_at):
            stopped=[r for r in rows if r['sim_s']>=stop_at+1]
            if len(stopped)>=2 and stopped[-1]['sim_s']-stopped[0]['sim_s']>=.5:
                speed=max(norm(r['velocity'][:3]) for r in stopped)
                drift=max(distance(r['xyz'],stopped[0]['xyz']) for r in stopped)
                cargo_speed=max(r['cargo'][n]['point_speed_bound_mps'] for r in stopped for n in CARGO)
                cargo_drift=max(distance(r['cargo'][n]['xyz'],stopped[0]['cargo'][n]['xyz']) for r in stopped for n in CARGO)
                checks['vehicle_stopped']=speed<=.01 and drift<=.005
                checks['all_cargo_stopped']=cargo_speed<=.01 and cargo_drift<=.005
                measurements.update(vehicle_speed_mps=speed,vehicle_drift_m=drift,
                    cargo_speed_mps=cargo_speed,cargo_drift_m=cargo_drift)
    frames=nav.get('sensor_frames',[])
    if frames:
        last=frames[-1].get('observation',{})
        checks['load_evidence']=nav.get('motion_refusal') is None and last.get('source')=='RGBD'
        checks['load_evidence'] &= (last.get('status')=='RESOLVED' and last.get('counts')=={'red':2,'blue':1}
            and last.get('count_verified') is True and last.get('support')=='deck')
    return dict(scope='INDEPENDENT_CONTINUOUS_LOADING_NAV2_NOT_FULL_ORDER',
        checks={k:'PASS' if v else 'FAIL' for k,v in checks.items()},measurements=measurements,
        integration_result='PASS' if all(checks.values()) else 'FAIL',
        full_order='NOT_RUN',three_dimensional_safety='NOT_RUN',v1_complete=False)


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--run-id',required=True);p.add_argument('--input-run',required=True);a=p.parse_args()
    if any(Path(v).name!=v for v in (a.run_id,a.input_run)):p.error('bare run IDs required')
    out=ROOT/'reports'/a.run_id;out.mkdir(exist_ok=False)
    paths=[ROOT/'reports'/a.input_run/n for n in ('report.json','continuous_nav.json')]
    result=judge(*(json.loads(path.read_text()) for path in paths))
    result['inputs']={str(path.relative_to(ROOT)):hashlib.sha256(path.read_bytes()).hexdigest() for path in paths}
    (out/'report.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))
    return 0 if result['integration_result']=='PASS' else 1


if __name__=='__main__':raise SystemExit(main())
