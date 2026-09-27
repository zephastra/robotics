"""A UDP state-stream sniffer with NO ROS IN IT.

WHY THIS EXISTS
---------------
`p3-nav-03` left one question open: the datagrams stop reaching the bridge about 40 s into a run,
but the simulator keeps sending. Two very different things can look identical from the bridge:

  * the transport stops delivering (sender drops, socket buffer, kernel), or
  * the bridge's own loop stalls (a blocked publish, DDS discovery, `spin_once`) and the
    datagrams are simply not read.

A sniffer that only does `recvfrom` and `time.monotonic()` cannot be blamed on either ROS or
DDS, so whatever it sees decides between them. It records EVERY inter-arrival gap above a
threshold with the wall clock, plus the receive-buffer occupancy evidence a reader needs.

    ./.venv/bin/python experiments/sniff_n_state.py --seconds 60 --out /mnt/c/.../sniff.json

It is a diagnostic, not a judge: it writes timings and counts, no verdict.
"""
import argparse
import json
import pathlib
import socket
import sys
import time

ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src'))

#: Gaps at or above this are recorded individually. The stream runs at `state_rate_hz` of sim
#: time, so at realtime_factor 0.35 one datagram per ~0.14 s of wall time.
GAP_REPORT_S = 0.20


def modern_baseline(seconds):
    """Run for `seconds`, recording arrival times. No ROS, no publishing, no decoding."""
    import yaml
    cfg = yaml.safe_load((ROOT / 'config' / 'p3_nav.yaml').read_text(encoding='utf-8'))
    ipc = cfg['ipc']
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    rbuf = None
    try:
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 8 * 1024 * 1024)
        rbuf = int(sock.getsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF))
    except OSError as exc:
        rbuf = f'failed: {exc!r}'
    sock.bind((ipc['host'], int(ipc['state_port'])))
    sock.settimeout(float(ipc['socket_timeout_s']))

    started = time.monotonic()
    last_state_wall = started
    times, gaps, nbytes, n = [], [], 0, 0
    first = None
    while time.monotonic() - started < seconds:
        try:
            raw, _peer = sock.recvfrom(int(ipc['max_datagram_bytes']))
        except socket.timeout:
            continue
        now = time.monotonic()
        if first is None:
            first = now
        gap = now - last_state_wall
        if gap >= GAP_REPORT_S:
            gaps.append({'at_wall_s': round(now - started, 3), 'gap_s': round(gap, 3),
                         'datagram_index': n})
        last_state_wall = now
        times.append(now - started)
        nbytes += len(raw)
        n += 1

    elapsed = time.monotonic() - started
    intervals = [b - a for a, b in zip(times, times[1:])]
    return {
        'role': 'sniffer (no ROS, no DDS, no publishing)',
        'receive_buffer_bytes': rbuf,
        'socket_timeout_s': float(ipc['socket_timeout_s']),
        'state_port': int(ipc['state_port']),
        'wall_seconds': round(elapsed, 3),
        'datagrams': n,
        'bytes': nbytes,
        'first_datagram_at_wall_s': (round(first - started, 3) if first is not None else None),
        'datagrams_per_wall_s': round(n / elapsed, 3) if elapsed else None,
        'gap_threshold_s': GAP_REPORT_S,
        'gaps': gaps,
        'max_interval_s': round(max(intervals), 4) if intervals else None,
        'p99_interval_s': (round(sorted(intervals)[int(len(intervals) * 0.99)], 4)
                           if len(intervals) > 100 else None),
        'verdict_free': True,
    }


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--seconds', type=float, default=60.0)
    ap.add_argument('--out', default=None)
    args = ap.parse_args()
    out = modern_baseline(args.seconds)
    text = json.dumps(out, indent=2) + '\n'
    if args.out:
        pathlib.Path(args.out).write_text(text, encoding='utf-8')
        print(f'wrote {args.out}')
    print(text)
    return 0


if __name__ == '__main__':
    sys.exit(main())
