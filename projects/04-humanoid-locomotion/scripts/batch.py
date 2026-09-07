"""Run independent processes; preserve every success, failure, and timeout."""
import argparse
from datetime import datetime, timezone
import json
from pathlib import Path
import subprocess
import sys
sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'src'))
from humanoid004.robots import manual_commands

ROOT=Path(__file__).resolve().parents[1]


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--robot',choices=['g1','t800'],default='g1')
    parser.add_argument('--suite',choices=['route','push','cancel','baseline'],default='route')
    parser.add_argument('--count',type=int,default=20)
    args=parser.parse_args()
    if args.count<1: parser.error('count must be positive')
    out=ROOT/'reports'/('batch-'+args.robot+'-'+args.suite+'-'+datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S%fZ'))
    out.mkdir(parents=True)
    if args.suite=='route':
        cases=[(f'route-seed-{i}',['--mode','route','--seed',str(i),'--vary-initial']) for i in range(args.count)]
    elif args.suite=='push':
        cases=[(f'push-{force}N',['--mode','push','--push-force',str(force)]) for force in [10,30,60,100,150,250,400,800]]
    elif args.suite=='cancel':
        cases=[(f'cancel-{t}s',['--mode','route','--cancel-at',str(t)]) for t in [0,8,30,45]]
    else:
        cases=[('zero',['--mode','baseline','--duration','30']),
               ('stand',['--mode','stand','--duration','60'])]
        controls=manual_commands(args.robot)
        for name,key in [('forward','W'),('backward','S'),('left','A'),('right','D'),('side_left','Q'),('side_right','E')]:
            cmd=controls[key]
            cases.append((name,['--mode','baseline','--duration','12','--command',*map(str,cmd)]))
    results=[]
    for name,options in cases:
        log=out/(name+'.log')
        try:
            proc=subprocess.run(['bash',str(ROOT/'run_demo.sh'),'--robot',args.robot,'--headless',*options],cwd=ROOT,
                                text=True,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,timeout=180)
            log.write_text(proc.stdout)
            report_path=next((Path(line) for line in reversed(proc.stdout.splitlines()) if line.endswith('/report.json')),None)
            report=json.loads(report_path.read_text()) if report_path and report_path.is_file() else {'status':'FAILED','reason':'MISSING_REPORT'}
            result={'name':name,'exit_code':proc.returncode,'report':str(report_path.relative_to(ROOT)) if report_path else None,
                    'status':report['status'],'reason':report['reason'],'events':report.get('events',[]),
                    'max_tilt_degrees':report.get('max_tilt_degrees'),'sim_seconds':report.get('sim_seconds'),
                    'final_pose':report.get('final_pose'),'stop_check':report.get('stop_check'),
                    'arguments':report.get('arguments'),'net_displacement_m':report.get('net_displacement_m'),
                    'max_displacement_from_start_m':report.get('max_displacement_from_start_m'),
                    'accumulated_yaw_degrees':report.get('accumulated_yaw_degrees')}
            if args.suite=='baseline' and name not in ('zero','stand'):
                # Survival alone must not be mistaken for a functioning direction key.
                pose=report.get('final_pose') or [0,0]
                pose=[v if isinstance(v,(int,float)) else 0 for v in pose]
                yaw=report.get('accumulated_yaw_degrees') or 0
                check={'forward':pose[0]>.3,'backward':pose[0]<-.3,
                       'left':yaw>30,'right':yaw<-30,
                       'side_left':pose[1]>.3,'side_right':pose[1]<-.3}[name]
                result['motion_response_passed']=bool(check and report['status']=='COMPLETED')
        except subprocess.TimeoutExpired as exc:
            log.write_text(str(exc.stdout))
            result={'name':name,'status':'FAILED','reason':'WALL_TIMEOUT','report':None,'exit_code':124}
        results.append(result)
        completed=sum(r['status']=='COMPLETED' for r in results)
        controlled_stops=sum(bool((r.get('stop_check') or {}).get('passed')) for r in results)
        summary={'robot':args.robot,'suite':args.suite,'planned':len(cases),'finished':len(results),'completed':completed,
                 'controlled_stops':controlled_stops,'cases':results,
                 'note':'Simulation-only, fixed model/flat ground. Seeds vary initial xy +/-0.04 m and yaw +/-0.08 rad for route suite.'}
        (out/'summary.json').write_text(json.dumps(summary,indent=2)+'\n')
        print(f'[{len(results)}/{len(cases)}] {name}: {result["status"]} {result["reason"]}',flush=True)
    print('Summary:',out/'summary.json',flush=True)
    if args.suite=='route':
        return 0 if completed>=math_ceil(.9*len(cases)) else 1
    if args.suite=='cancel': return 0 if controlled_stops==len(cases) else 1
    if args.suite=='baseline':
        return 0 if all(r['status']=='COMPLETED' and r.get('motion_response_passed',True) for r in results) else 1
    # Push suite is an experiment: a detected fall is a result, not a tool failure.
    return 0 if all(r.get('report') for r in results) else 1


def math_ceil(value):
    import math
    return math.ceil(value)


if __name__=='__main__': sys.exit(main())
