#!/usr/bin/env python3
"""Run an owned 010 probe with a wall deadline and durable, secret-free provenance."""
import argparse
import datetime
import hashlib
import json
import math
import os
from pathlib import Path
import re
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[1]


def fingerprint(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run-id', required=True)
    parser.add_argument('--timeout', type=float, required=True)
    parser.add_argument('script')
    parser.add_argument('arguments', nargs=argparse.REMAINDER)
    args = parser.parse_args(argv)
    if not re.fullmatch(r'[A-Za-z0-9][A-Za-z0-9_-]{0,95}', args.run_id):
        parser.error('invalid run ID')
    if not math.isfinite(args.timeout) or args.timeout <= 0:
        parser.error('timeout must be finite and positive')
    script = (ROOT / args.script).resolve()
    if not script.is_relative_to(ROOT / 'experiments') or script.suffix != '.py':
        parser.error('probe must be a Python file inside 010/experiments')
    if not script.is_file():
        parser.error('probe does not exist')
    if '--run-id' in args.arguments or any(a.startswith('--run-id=') for a in args.arguments):
        parser.error('run ID belongs to the runner')
    report_dir = ROOT / 'reports' / args.run_id
    runtime_dir = ROOT / 'runtime' / 'bounded_runs' / args.run_id
    if report_dir.exists() or runtime_dir.exists():
        parser.error('run ID already exists; evidence will not be overwritten')
    runtime_dir.mkdir(parents=True)
    command = [sys.executable, '-u', str(script), '--run-id', args.run_id, *args.arguments]
    hashes = {}
    for directory in ('src', 'experiments', 'scripts', 'config'):
        for path in sorted((ROOT / directory).rglob('*')):
            if path.is_file() and path.suffix in ('.py', '.json', '.yaml', '.cpp', '.h'):
                hashes[str(path.relative_to(ROOT))] = fingerprint(path)
    for path in sorted((ROOT / 'assets').glob('world*.xml')):
        hashes[str(path.relative_to(ROOT))] = fingerprint(path)
    manifest = {'schema_version': 1, 'run_id': args.run_id, 'command': command,
                'utc': datetime.datetime.now(datetime.timezone.utc).isoformat(),
                'deadline_wall_s': args.timeout, 'files_sha256': hashes,
                'wrapper_pid': os.getpid(), 'status': 'STARTING'}
    manifest_path = runtime_dir / 'manifest.json'

    def save():
        manifest_path.write_text(json.dumps(manifest, indent=2) + '\n', encoding='utf-8')

    save()
    environment = dict(os.environ)
    environment.update(OMP_NUM_THREADS='1', OPENBLAS_NUM_THREADS='1', MUJOCO_GL='egl')
    started = time.monotonic()
    with (runtime_dir / 'stdout.log').open('w', encoding='utf-8') as logfile:
        child = subprocess.Popen(command, cwd=ROOT, env=environment, stdout=logfile,
                                 stderr=subprocess.STDOUT)
        manifest.update(child_pid=child.pid, status='RUNNING')
        # The start tick disambiguates PID reuse. The child is directly owned by this wrapper.
        proc_stat = Path('/proc') / str(child.pid) / 'stat'
        if proc_stat.exists():
            manifest['child_start_ticks'] = proc_stat.read_text().rsplit(')', 1)[1].split()[19]
        save()
        print('RUN %s PID %s; log %s' % (args.run_id, child.pid, logfile.name), flush=True)
        try:
            code = child.wait(timeout=args.timeout)
            manifest['status'] = 'EXITED'
        except subprocess.TimeoutExpired:
            child.terminate()
            try:
                child.wait(timeout=5)
            except subprocess.TimeoutExpired:
                child.kill()
                child.wait()
            code = 124
            manifest['status'] = 'WALL_TIMEOUT'
        except KeyboardInterrupt:
            child.terminate()
            try:
                child.wait(timeout=5)
            except subprocess.TimeoutExpired:
                child.kill()
                child.wait()
            code = 130
            manifest['status'] = 'INTERRUPTED'
        manifest.update(exit_code=code, elapsed_wall_s=time.monotonic() - started)
        save()
    # Preserve failure manifests too, even when a probe never produced a report.
    report_dir.mkdir(parents=True, exist_ok=True)
    (report_dir / 'run_manifest.json').write_text(manifest_path.read_text(), encoding='utf-8')
    print('EXIT %s; evidence %s' % (code, report_dir), flush=True)
    return code


if __name__ == '__main__':
    raise SystemExit(main())
