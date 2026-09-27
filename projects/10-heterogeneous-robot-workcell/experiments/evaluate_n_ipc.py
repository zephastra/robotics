"""Judge the P1-N probe from its three recorded files. Plain Python, no ROS, no sockets.

Inputs, all written by the run:
  * `reports/<run-id>/sim_report.json`      -- the simulator's counters and its truth trajectory
  * `reports/<run-id>/ros_report.json`      -- what the ROS side published and what it sent
  * `reports/<run-id>/checker_raw.json`     -- what an independent subscriber observed

Separating judgement from observation is deliberate. A checker that decides while it runs
cannot be tested without a live graph, and the decision logic is exactly the part most likely
to be wrong. Here it is a pure function of three files, so it can be tested with synthetic
ones -- including the cases that must FAIL.

Every check names the fact it measures and the number it found, and a check whose input is
missing reports NOT_RUN rather than passing by default.
"""
import argparse
import json
import math
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]


def wrapped(angle):
    """Angular difference on the circle.

    Angles live on S1: an odometry yaw of 5.45 rad and a truth yaw of -0.98 rad are 0.15 rad
    apart, not 6.4. The first version subtracted them as plain numbers and failed a run whose
    yaw error was inside the limit.
    """
    return abs((angle + math.pi) % (2.0 * math.pi) - math.pi)


def load(path):
    return json.loads(path.read_text(encoding='utf-8')) if path.is_file() else None


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--run-id', required=True)
    parser.add_argument('--tolerance', type=float, default=0.35,
                        help='fractional rate tolerance for the achieved sensor rates')
    parser.add_argument('--odom-error-limit-m', type=float, default=0.30)
    parser.add_argument('--odom-yaw-limit-rad', type=float, default=0.25)
    parser.add_argument('--reports-dir', default=None,
                        help='where run directories live; defaults to <root>/reports, and '
                             'exists so a test can point this judge at synthetic inputs')
    args = parser.parse_args()

    import yaml
    cfg = yaml.safe_load((ROOT / 'config' / 'n_probe.yaml').read_text(encoding='utf-8'))
    out = (Path(args.reports_dir) if args.reports_dir else ROOT / 'reports') / args.run_id
    sim = load(out / 'sim_report.json')
    ros = load(out / 'ros_report.json')
    checker = load(out / 'checker_raw.json')

    checks = []

    def check(name, verdict, detail):
        checks.append({'check': name, 'verdict': verdict, 'detail': detail})

    # ---- inputs present ---------------------------------------------------
    for label, data in (('sim_report', sim), ('ros_report', ros), ('checker_raw', checker)):
        check(f'{label} exists', 'PASS' if data else 'FAIL',
              'present' if data else f'missing: {out / (label + ".json")}')
    if not (sim and ros and checker):
        return _write(out, args.run_id, checks)

    # ---- one clock owner --------------------------------------------------
    rows = checker.get('rows') or []

    def publisher_counts(topic):
        counts = [(row.get('publishers') or {}).get(topic) for row in rows]
        return [count for count in counts if isinstance(count, int)]

    def check_one_publisher(topic, why):
        counts = publisher_counts(topic)
        if not counts:
            check(f'exactly one /{topic} publisher', 'NOT_RUN',
                  'no publisher observations recorded')
            return
        # The invariant is "never more than one, and it did appear". Early zeros are DDS
        # discovery (the observer starts before the bridge and sees an empty graph), while a
        # 2 means a second owner -- which for /clock is not a warning but a correctness
        # fault, because a second clock publisher makes tf2 discard its buffer.
        verdict = 'PASS' if max(counts) <= 1 and 1 in counts else 'FAIL'
        check(f'exactly one /{topic} publisher', verdict,
              f'counts seen: {sorted(set(counts))} over {len(counts)} samples at '
              f'1 Hz; a 0 is discovery, a 2 is a second owner. {why}')

    check_one_publisher('clock', 'The simulator owns time.')
    check_one_publisher('tf', 'One publisher per TF topic.')
    check_one_publisher('tf_static', 'One publisher per TF topic.')

    # ---- stamps are sim time, not wall time -------------------------------
    stamps = [row['clock_stamp'] for row in rows if row.get('clock_stamp') is not None]
    if len(stamps) >= 2:
        biggest = max(stamps)
        ordered = all(b >= a for a, b in zip(stamps, stamps[1:]))
        check('clock stamps are simulation time, not the wall clock',
              'PASS' if biggest < 1.0e5 and ordered else 'FAIL',
              f'largest stamp {biggest:.3f} s (wall-clock epoch would be ~1.7e9), monotonic={ordered}')
    else:
        check('clock stamps are simulation time, not the wall clock', 'NOT_RUN',
              f'only {len(stamps)} clock stamp(s) observed')
    check('clock advanced at all', 'PASS' if checker['counts']['clock'] > 0 else 'FAIL',
          f"{checker['counts']['clock']} clock messages observed")

    # ---- rates ------------------------------------------------------------
    # Measured per SIM second, from the stamps, not per wall second: the physics runs about
    # five times faster than the wall clock on this machine, so a wall-clock rate would read
    # as five times the configured rate and fail a correct run.
    seconds = checker.get('wall_seconds')
    expected_state_rate = float(cfg['sim']['state_rate_hz'])

    def rate_from_rows(key, topic):
        points = []
        for row in rows:
            count = (row.get('counts') or {}).get(topic)
            if row.get(key) is None or count is None:
                continue
            points.append((row[key], count))
        if len(points) < 2 or points[-1][0] <= points[0][0]:
            return None
        return (points[-1][1] - points[0][1]) / (points[-1][0] - points[0][0])

    for topic, key in (('scan', 'scan_stamp'), ('odom', 'odom_stamp'), ('clock', 'clock_stamp')):
        rate = rate_from_rows(key, topic)
        if rate is None:
            check(f'/{topic} rate is within {args.tolerance:.0%} of config', 'NOT_RUN',
                  f"not enough stamped observations (wall window {seconds} s)")
            continue
        ok = abs(rate - expected_state_rate) <= args.tolerance * expected_state_rate
        wall_rate = checker['counts'][topic] / seconds if seconds else None
        check(f'/{topic} rate is within {args.tolerance:.0%} of config',
              'PASS' if ok else 'FAIL',
              f'{rate:.2f} Hz per sim second vs {expected_state_rate:.2f} Hz configured'
              + (f' ({wall_rate:.1f} Hz per wall second)' if wall_rate else '')
              + f"; {checker['counts'][topic]} messages observed")

    # ---- scan geometry and content ----------------------------------------
    geometry = checker['observed'].get('scan_geometry')
    if geometry:
        nrays_cfg = int(cfg['scan']['rays'])
        expected_increment = (float(cfg['scan']['angle_max_deg'])
                              - float(cfg['scan']['angle_min_deg'])) / (nrays_cfg - 1)
        increment_deg = geometry['angle_increment'] * 180.0 / 3.141592653589793
        ok = (geometry['rays'] == nrays_cfg
              and abs(increment_deg - expected_increment) < 1e-6
              and geometry['frame_id'] == cfg['frames']['lidar'])
        check('scan geometry matches config', 'PASS' if ok else 'FAIL',
              f"{geometry['rays']} rays, increment {increment_deg:.6f} deg "
              f"(config {expected_increment:.6f}), frame {geometry['frame_id']!r}")
        finite_ok = (checker['observed']['scan_finite'] == geometry['rays'])
        check('every scan range is finite', 'PASS' if finite_ok else 'FAIL',
              f"{checker['observed']['scan_finite']}/{geometry['rays']} finite")
        inside = (geometry['range_min'] <= checker['observed']['scan_range_min']
                  and checker['observed']['scan_range_max'] <= geometry['range_max'])
        check('scan ranges are inside [range_min, range_max]',
              'PASS' if inside else 'FAIL',
              f"observed {checker['observed']['scan_range_min']:.3f} .. "
              f"{checker['observed']['scan_range_max']:.3f} m vs limits "
              f"[{geometry['range_min']}, {geometry['range_max']}]")
    else:
        check('scan geometry matches config', 'NOT_RUN', 'no scan received')

    # ---- one publisher per TF topic, and known children -------------------
    children = sorted(checker['observed']['tf_children'])
    expected_child = [cfg['frames']['base']]
    check('dynamic TF publishes exactly the odom->base edge',
          'PASS' if children == expected_child else 'FAIL',
          f'child frames seen: {children}, expected {expected_child}')
    static_children = sorted(checker['observed']['static_children'])
    expected_static = [cfg['frames']['lidar']]
    check('static TF publishes exactly the base->lidar edge',
          'PASS' if static_children == expected_static else 'FAIL',
          f'child frames seen: {static_children}, expected {expected_static}')

    # ---- odom moved, and the datagram carries no pose ----------------------
    first, last = checker['observed']['odom_first'], checker['observed']['odom_last']
    moved = None if not (first and last) else ((last[0] - first[0]) ** 2
                                               + (last[1] - first[1]) ** 2) ** 0.5
    check('odometry reports motion', 'PASS' if moved and moved > 0.05 else 'FAIL',
          f'odom travelled {moved:.4f} m between the first and last message'
          if moved is not None else 'no odom pose observed')
    no_pose = (sim['state_schema']['world_pose_in_state'] is False
               and sim['state_schema']['keys'])
    check('the state datagram carries no world pose', 'PASS' if no_pose else 'FAIL',
          f"state keys {sim['state_schema']['keys']}; the ROS side cannot copy a pose it is "
          f"never sent")

    # ---- the command gate actually held, and lapsed on silence -------------
    gate = sim.get('gate') or {}
    modes = gate.get('modes') or {}
    check('the gate accepted commands', 'PASS' if gate.get('accepted', 0) > 0 else 'FAIL',
          f"accepted {gate.get('accepted', 0)}, refused {gate.get('refused', 0)}, "
          f"reasons {gate.get('reasons')}")
    holding = modes.get('HOLDING', 0)
    silent = modes.get('SILENT', 0)
    check('the gate was actually in HOLDING while commands arrived',
          'PASS' if holding > 0 else 'FAIL', f'step modes: {modes}')
    if ros.get('command_stopped_at') is None:
        check('the gate falls back to zero when commands stop', 'NOT_RUN',
              'this run sent commands to the end; the fallback is measured by a run with '
              '--stop-commands-at')
    else:
        # The simulator keeps stepping after the commands stop, so if the lease did not
        # lapse the wheels would still be turning and SILENT would be zero. Requiring that
        # commands were accepted first stops the check from passing for the wrong reason: a
        # run where the ROS side never connected is silent from the first step, and that is
        # a broken link, not a working fallback.
        check('the gate falls back to zero when commands stop',
              'PASS' if (silent > 0 and gate.get('accepted', 0) > 0) else 'FAIL',
              f"SILENT steps {silent}, accepted {gate.get('accepted', 0)} before commands "
              f"stopped at sim {ros['command_stopped_at']} s; final gate mode "
              f"{gate.get('mode')!r}")

    # ---- odometry versus truth --------------------------------------------
    truth = sim.get('truth') or {}
    odom_final = ros.get('odom_final') or {}
    if truth.get('final') and odom_final:
        tx, ty, tyaw = truth['final']
        dx = odom_final['x'] - tx
        dy = odom_final['y'] - ty
        position_error = (dx * dx + dy * dy) ** 0.5
        yaw_error = wrapped(odom_final['yaw'] - tyaw)
        travelled = sim.get('truth_travel_m')
        check('odometry position error is within the diagnostic limit',
              'PASS' if position_error <= args.odom_error_limit_m else 'FAIL',
              f'{position_error:.4f} m (limit {args.odom_error_limit_m} m); odom '
              f"({odom_final['x']:.3f}, {odom_final['y']:.3f}) vs truth ({tx:.3f}, {ty:.3f})"
              + (f', travelled {travelled} m' if travelled else ''))
        check('odometry yaw error is within the diagnostic limit',
              'PASS' if yaw_error <= args.odom_yaw_limit_rad else 'FAIL',
              f'{yaw_error:.4f} rad (limit {args.odom_yaw_limit_rad} rad)')
    else:
        check('odometry position error is within the diagnostic limit', 'NOT_RUN',
              'no truth or no odom pose')

    # ---- the two halves ran to completion ---------------------------------
    check('the simulator completed', 'PASS' if sim.get('status') == 'COMPLETED' else 'FAIL',
          f"status {sim.get('status')}")
    check('the ROS side completed', 'PASS' if ros.get('status') == 'COMPLETED' else 'FAIL',
          f"status {ros.get('status')}")
    check('the ROS side refused no state datagrams',
          'PASS' if ros['counters']['decode_refusals'] == 0 else 'FAIL',
          f"{ros['counters']['decode_refusals']} refusals, reasons "
          f"{ros.get('decode_refusal_reasons')}")
    check('the ROS side decoded every state without a sequence gap',
          'PASS' if ros.get('seq_gaps') == 0 else 'FAIL',
          f"seq gaps {ros.get('seq_gaps')}, states received "
          f"{ros['counters']['states_received']}, simulator sent {sim['ipc']['states_sent']}")

    return _write(out, args.run_id, checks)


def _write(out, run_id, checks):
    failed = [c for c in checks if c['verdict'] == 'FAIL']
    not_run = [c for c in checks if c['verdict'] == 'NOT_RUN']
    verdict = 'FAIL' if failed else ('INCOMPLETE' if not_run else 'PASS')
    result = {'run_id': run_id, 'verdict': verdict, 'checks': checks,
              'failed': [c['check'] for c in failed],
              'not_run': [c['check'] for c in not_run]}
    (out / 'acceptance.json').write_text(json.dumps(result, indent=2) + '\n')
    print(json.dumps({'run_id': run_id, 'verdict': verdict,
                      'failed': result['failed'], 'not_run': result['not_run']}, indent=2))
    for item in checks:
        print(f"  {item['verdict']:<8} {item['check']}: {item['detail']}")
    return 0 if verdict == 'PASS' else 1


if __name__ == '__main__':
    # A judge that crashes must not look like a pass. The unit tests feed this script
    # synthetic files that must FAIL, and the first version crashed on one of them with a
    # KeyError while exiting non-zero for the wrong reason.
    try:
        raise SystemExit(main())
    except SystemExit:
        raise
    except Exception as exc:                                    # noqa: BLE001
        print(json.dumps({'verdict': 'FAIL',
                          'judge_error': f'{type(exc).__name__}: {exc}'}, indent=2))
        raise SystemExit(1)
