"""Independent empty shared-world Nav2 judge; original arrival/stop limits.

Reads recorded evidence only, never model state or an executor's PASS label.
Missing/invalid observations fail closed. This is NOT full-order acceptance.
"""
import argparse
import hashlib
import json
import math
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
NODES = ('map_server','amcl','planner_server','controller_server','smoother_server',
         'behavior_server','bt_navigator','velocity_smoother','collision_monitor')
REQUIRED = ('execution_completed','world_identity','shared_owner','no_runtime_pose_write',
            'nav2_action','all_lifecycle_active','single_final_command_writer',
            'fresh_commands','independent_arrival','actual_motion','physical_stop','no_freeze')


def finite_vector(value, length):
    return isinstance(value,list) and len(value)==length and all(
        isinstance(v,(int,float)) and not isinstance(v,bool) and math.isfinite(v) for v in value)


def distance(a,b): return math.hypot(a[0]-b[0],a[1]-b[1])


def judge(report,worker,profile):
    checks = {key:False for key in REQUIRED}; measurements = {}
    checks.update(execution_completed=report.get('status')=='COMPLETED' and not report.get('error'),
        world_identity=bool(profile.get('world_sha256')) and report.get('world_sha256')==profile.get('world_sha256'),
        shared_owner=report.get('same_model') is True and report.get('same_data') is True and report.get('owner_steps',0)>0,
        no_runtime_pose_write=report.get('runtime_qpos_writes')==0,
        nav2_action=worker.get('status')=='SUCCEEDED' and worker.get('goal_accepted') is True and worker.get('action_status')==4 and worker.get('action_error_code')==0,
        all_lifecycle_active=all(worker.get('active',{}).get(name)==3 for name in NODES),
        single_final_command_writer=worker.get('cmd_vel_publishers')==['collision_monitor'],
        fresh_commands=worker.get('nonzero_command_count',0)>0 and report.get('commands_received',0)>0,
        no_freeze='simulation_frozen' in report and report['simulation_frozen'] is None)
    trace=report.get('truth_judge_only'); stopping=report.get('stopping_at_sim_s')
    valid = isinstance(trace,list) and len(trace)>=2 and all(
        finite_vector(row.get('xyz'),3) and finite_vector(row.get('velocity'),4)
        and isinstance(row.get('sim_s'),(int,float)) and math.isfinite(row['sim_s']) for row in trace)
    if valid: valid=all(b['sim_s']>a['sim_s'] for a,b in zip(trace,trace[1:]))
    if valid and finite_vector(profile.get('goal'),3) and finite_vector(profile.get('start'),2):
        error=distance(trace[-1]['xyz'],profile['goal']); motion=distance(trace[-1]['xyz'],profile['start'])
        checks['independent_arrival']=error<=.25; checks['actual_motion']=motion>.1
        measurements.update(arrival_error_m=error,displacement_m=motion)
        if isinstance(stopping,(int,float)) and math.isfinite(stopping):
            rows=[r for r in trace if r['sim_s']>=stopping+1.]
            # At least the original 0.5-second hold window, not one quiet sample.
            if len(rows)>=2 and rows[-1]['sim_s']-rows[0]['sim_s']>=.5:
                speed=max(math.sqrt(sum(v*v for v in r['velocity'][:3])) for r in rows)
                drift=max(distance(r['xyz'],rows[0]['xyz']) for r in rows)
                checks['physical_stop']=speed<=.01 and drift<=.005
                measurements.update(stop_speed_mps=speed,stop_drift_m=drift,stop_window_s=rows[-1]['sim_s']-rows[0]['sim_s'])
    return dict(scope='INDEPENDENT_EMPTY_SHARED_WORLD_NAV2_NOT_LOADED_OR_ORDER',
        checks={k:'PASS' if checks[k] else 'FAIL' for k in REQUIRED},measurements=measurements,
        commissioning_result='PASS' if all(checks.values()) else 'FAIL',
        loaded_navigation='NOT_RUN',full_order='NOT_RUN',v1_complete=False)


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--run-id',required=True);p.add_argument('--input-run',required=True);a=p.parse_args()
    if any(Path(s).name!=s for s in (a.run_id,a.input_run)):p.error('bare run IDs required')
    out=ROOT/'reports'/a.run_id;out.mkdir(exist_ok=False)
    paths=[ROOT/'reports'/a.input_run/'report.json',ROOT/'reports'/a.input_run/'nav2_worker_report.json',ROOT/'config/joint_world_v5.profile.json']
    result=judge(*(json.loads(path.read_text()) for path in paths))
    result['inputs']={str(path.relative_to(ROOT)):hashlib.sha256(path.read_bytes()).hexdigest() for path in paths}
    (out/'report.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))
    return 0 if result['commissioning_result']=='PASS' else 1


if __name__=='__main__':raise SystemExit(main())
