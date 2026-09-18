"""Reproducible bounded initial-condition suite; never discard failed cases."""
import argparse
from collections import Counter
from concurrent.futures import ThreadPoolExecutor, as_completed
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import random
import signal
import shutil
import subprocess
import time

ROOT = Path(__file__).resolve().parents[1]


def cases(seed=7):
    rng = random.Random(seed)
    result = []
    def add(group, box=(0., 0.), target=(0., 0.), yaw=0.):
        result.append(dict(id=f'{len(result)+1:02d}-{group}', group=group,
                           box=list(box), target=list(target), yaw=yaw))
    add('regression')
    add('regression', (.02, .01), (.05, .03))
    add('regression', (-.02, -.01), (-.05, -.03))
    for x in (-.02, 0., .02):
        for y in (-.01, 0., .01):
            add('box-grid', (x, y))
    for x in (-.05, 0., .05):
        for y in (-.03, 0., .03):
            add('target-grid', target=(x, y))
    for _ in range(9):
        add('combined-yaw', tuple(round(rng.uniform(-v, v), 5) for v in (.02, .01)),
            tuple(round(rng.uniform(-v, v), 5) for v in (.05, .03)),
            round(rng.uniform(-5., 5.), 3))
    return result


def fingerprint():
    paths = [ROOT / 'run_demo.sh', ROOT / 'requirements.txt', Path(__file__).resolve()]
    for directory in ('src', 'config', 'assets', 'policies'):
        paths.extend(p for p in (ROOT / directory).rglob('*') if p.is_file()
                     and '__pycache__' not in p.parts)
    return {p.relative_to(ROOT).as_posix(): hashlib.sha256(p.read_bytes()).hexdigest()
            for p in sorted(set(paths))}


def report_path(log):
    for line in log.splitlines():
        if line.startswith('Report: '):
            path = Path(line.removeprefix('Report: ').strip()).resolve()
            if path.parent != (ROOT / 'reports').resolve():
                raise ValueError('Report path outside the project reports directory')
            return path / 'report.json'
    return None


def classify(returncode, report, timed_out=False):
    if timed_out:
        return 'WALL_TIMEOUT'
    if report is None:
        return 'MISSING_REPORT'
    if returncode == 0 and report.get('status') == 'COMPLETED':
        return 'COMPLETED'
    if report.get('status') == 'COMPLETED':
        return 'INCONSISTENT_EXIT'
    return report.get('status', 'INVALID_REPORT') if returncode != 0 else 'INCONSISTENT_EXIT'


def acceptance(summary):
    """Task completion is necessary but not sufficient for bounded acceptance."""
    results=summary.get('results',[])
    expected=[c['id'] for c in summary.get('cases',[])]
    observed=[r.get('case',{}).get('id') for r in results]
    complete=bool(expected) and sorted(expected)==sorted(observed,key=lambda x:str(x)) and len(set(observed))==len(observed)
    tasks=complete and all(r.get('status')=='COMPLETED' for r in results)
    contacts=complete and all(
        isinstance(r.get('station_contact_diagnostics'),dict)
        and r['station_contact_diagnostics'].get('contact_free') is True
        and r['station_contact_diagnostics'].get('pairs')=={}
        for r in results)
    unchanged=summary.get('inputs_unchanged') is True
    return dict(passed=tasks and contacts and unchanged,all_cases_recorded=complete,
                all_tasks_completed=tasks,monitored_station_contact_free=contacts,
                inputs_unchanged=unchanged)


def execute(case, destination, timeout, duration, stance='auto', hold_controller='walking'):
    command = ['bash', 'run_demo.sh', '--headless', '--duration', str(duration), '--stance', stance,
               '--hold-controller',hold_controller,
               '--box-offset', *map(str, case['box']),
               '--target-offset', *map(str, case['target']), '--box-yaw', str(case['yaw'])]
    logfile = destination / (case['id'] + '.log')
    started = time.monotonic()
    timed_out = False
    with logfile.open('w') as output:
        process = subprocess.Popen(command, cwd=ROOT, stdout=output, stderr=subprocess.STDOUT,
                                   env={**os.environ, 'MUJOCO_GL': 'egl', 'PYTHONUNBUFFERED': '1'},
                                   start_new_session=True)
        try:
            process.wait(timeout=timeout)
        except subprocess.TimeoutExpired:
            timed_out = True
            # Kill only this case's own process group, never other user simulations.
            os.killpg(process.pid, signal.SIGTERM)
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait()
    record = dict(case=case, returncode=process.returncode,
                  wall_seconds=time.monotonic()-started, log=logfile.name)
    try:
        path = report_path(logfile.read_text(errors='replace'))
        report = json.loads(path.read_text()) if path and path.is_file() else None
        record.update(status=classify(process.returncode, report, timed_out),
                      report=str(path) if path else None)
        if report:
            events = report.get('events') or []
            working = [e for e in events if e.get('phase') not in ('COMPLETED', 'FAILED', 'ERROR', 'CANCELLED', 'TIMEOUT')]
            record.update(reason=report.get('reason'), evaluation=report.get('evaluation'),
                          last_event=(events or [None])[-1],
                          last_working_phase=working[-1]['phase'] if working else 'INITIALIZATION',
                          station_contact_samples=sum(bool(x.get('station_collisions')) for x in report.get('records', [])),
                          waypoint_recoveries=sum(bool(e.get('recovery')) for e in events))
            record['station_contact_diagnostics']=report.get('station_contact_diagnostics')
            (destination / (case['id'] + '.json')).write_text(json.dumps(report, indent=2)+'\n')
    except (ValueError, OSError) as exc:
        record.update(status='INVALID_REPORT', reason=str(exc))
    return record


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seed', type=int, default=7)
    parser.add_argument('--workers', type=int, choices=(1, 2, 4), default=1)
    parser.add_argument('--wall-timeout', type=float, default=300.)
    parser.add_argument('--duration', type=float, default=180.)
    parser.add_argument('--list', action='store_true', help='Show cases without launching physics')
    parser.add_argument('--case', nargs='+', help='Run selected case IDs in a NEW batch; never replace old failures')
    parser.add_argument('--stance', choices=('auto','legacy'), default='auto')
    parser.add_argument('--hold-controller',choices=('walking','posture'),default='walking')
    args = parser.parse_args()
    suite = cases(args.seed)
    if args.case:
        if set(args.case) - {c['id'] for c in suite}:
            parser.error('Unknown case ID; use --list to see all IDs')
        suite = [c for c in suite if c['id'] in args.case]
    if args.list:
        print(json.dumps(suite, indent=2)); return 0
    if not 0 < args.wall_timeout < float('inf') or not 0 < args.duration < float('inf'):
        parser.error('Timeouts must be finite and positive')
    destination = ROOT / 'reports' / ('batch-' + datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S%fZ'))
    destination.mkdir(parents=True, exist_ok=False)
    before = fingerprint()
    for relative in before:
        if relative.startswith(('src/','config/')) or relative in ('run_demo.sh','requirements.txt','scripts/batch.py'):
            saved=destination/'inputs'/relative
            saved.parent.mkdir(parents=True,exist_ok=True)
            shutil.copy2(ROOT/relative,saved)
    summary = dict(arguments=vars(args), cases=suite, fingerprint=before, results=[])
    def save():
        summary['counts'] = dict(Counter(r['status'] for r in summary['results']))
        summary['groups'] = {group: dict(Counter(r['status'] for r in summary['results']
                                               if r['case']['group'] == group))
                             for group in sorted({c['group'] for c in suite})}
        summary['failure_phases'] = dict(Counter(r.get('last_working_phase', 'UNKNOWN')
                                                for r in summary['results'] if r['status'] != 'COMPLETED'))
        summary['station_contact_cases'] = sum(r.get('station_contact_samples', 0)>0 for r in summary['results'])
        summary['force_contact_cases'] = sum(bool((r.get('station_contact_diagnostics') or {}).get('pairs')) for r in summary['results'])
        summary['acceptance']=acceptance(summary)
        temporary = destination / 'summary.tmp'
        temporary.write_text(json.dumps(summary, indent=2)+'\n')
        temporary.replace(destination / 'summary.json')
    save()
    print('Batch:', destination, flush=True)
    with ThreadPoolExecutor(max_workers=args.workers) as pool:
        futures = {pool.submit(execute, c, destination, args.wall_timeout, args.duration, args.stance, args.hold_controller): c for c in suite}
        for future in as_completed(futures):
            try:
                record = future.result()
            except Exception as exc:
                record = dict(case=futures[future], status='RUNNER_ERROR', reason=repr(exc))
            summary['results'].append(record)
            summary['results'].sort(key=lambda r: r['case']['id'])
            save()
            print(f"{len(summary['results'])}/{len(suite)} {record['case']['id']} {record['status']} {record.get('reason', '')}", flush=True)
    summary['inputs_unchanged'] = fingerprint() == before
    save()
    print(json.dumps(summary['counts']), flush=True)
    print('Acceptance:',json.dumps(summary['acceptance']),flush=True)
    return 0 if summary['acceptance']['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
