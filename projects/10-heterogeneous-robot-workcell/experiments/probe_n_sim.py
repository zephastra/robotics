"""P1-N probe, simulator side: owns the physics, the clock, and the command gate.

The scope this implements (docs/P1_FEASIBILITY.md, section N): "length-bounded local IPC:
sequence number, sim timestamp, command validity and a zero/stop fallback; no arbitrary
network commands. MuJoCo is the only publisher of physical state, the ROS process outputs
/clock, scan, wheel odometry and TF, and consumes velocity through the command gate. Wheel
odometry must not read the world pose; the scan is produced by scene rays."

Two consequences of that wording shape this file:

  * The state datagram has no pose field. The truth trajectory is recorded in this process's
    own report and is used only to evaluate the ROS side's estimate afterwards; it is never
    on the wire, so "odometry is not fed the truth" is a property of the schema rather than a
    promise in a comment.
  * The clock is the simulator's. `sim_time` is `MjData.time`, and the ROS side stamps every
    message from it, so there is exactly one origin for time in the system.

The plant is `assets/worlds/world_n_probe.xml`, a declared stand-in for the AMR (P1-ENV-01:
the AMR model is not chosen yet). See that file's header for the four measured versions it
took to get a chassis whose driven wheels carry the load.
"""
import argparse
import hashlib
import json
import math
import os
import signal
from pathlib import Path
import socket
import sys
import time

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src'))

from workcell_ipc import Refused, encode, state as state_payload   # noqa: E402
from workcell_ipc.gate import CommandGate                          # noqa: E402

WHEEL_JOINTS = ('wheel_left_joint', 'wheel_right_joint')


def _travelled(samples):
    """Path length from the recorded truth samples, for the odometry comparison."""
    total = 0.0
    for previous, current in zip(samples, samples[1:]):
        total += math.hypot(current[1] - previous[1], current[2] - previous[2])
    return total


def load_config(path=None):
    """`path` is optional so P3 can hand in its own world.

    The default is unchanged, which is what keeps P1-N's evidence reproducible: the P1
    command line still resolves to `config/n_probe.yaml` and nothing else moves.
    """
    import yaml
    path = Path(path) if path else ROOT / 'config' / 'n_probe.yaml'
    # Resolved against the project root when relative, so `--config config/p3_nav.yaml`
    # works from any cwd and `cfg_path.relative_to(ROOT)` below cannot fail. Measured:/n    # a relative path made the simulator exit with "ValueError: 'config/p3_nav.yaml'
    # is not in the subpath of <root>", and the only symptom was an ABSENT /clock --
    # which reads exactly like a broken bridge or a crashed plant.
    if not path.is_absolute():
        path = ROOT / path
    cfg = yaml.safe_load(path.read_text(encoding='utf-8'))
    required = ('schema_version', 'ros', 'ipc', 'sim', 'command', 'scan', 'odom', 'frames',
                'world')
    missing = [key for key in required if key not in cfg]
    if missing:
        raise SystemExit(f'config/n_probe.yaml is missing {missing}')
    if int(cfg['ros']['domain_id']) == 42:
        raise SystemExit('config refuses ROS domain 42: that is 009 (009/scripts/env.sh:28)')
    if int(cfg['schema_version']) != 1:
        raise SystemExit(f"unexpected schema_version {cfg['schema_version']!r}")
    return cfg, path


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--run-id', required=True)
    parser.add_argument('--world', default=None)
    parser.add_argument('--config', default=None,
                        help='config file to load instead of config/n_probe.yaml; the P3 nav arena needs its own world and is otherwise the same scenario')
    parser.add_argument('--duration', type=float, default=None)
    parser.add_argument('--idle', action='store_true',
                        help='do not open the command socket; the gate must fall back to zero')
    parser.add_argument('--allow-stop-signal', action='store_true',
                        help='let SIGUSR1 end the stepping loop early and still write the '
                             'report, so a run whose length is set by an event does not have '
                             'to wait out the whole duration (P1-N-07)')
    args = parser.parse_args()
    if Path(args.run_id).name != args.run_id:
        raise ValueError('run id must be a bare name')

    cfg, cfg_path = load_config(args.config)
    duration = float(args.duration if args.duration is not None else cfg['sim']['duration_s'])
    world = ROOT / (args.world or cfg['world'])
    out = ROOT / 'reports' / args.run_id
    # The run directory is shared with the ROS side and the observer, which may start first,
    # so the guard is "do not overwrite evidence", not "do not share a directory": with
    # exist_ok=False the simulator refused to start because the ROS side had already created
    # `reports/<run-id>/`, and it exited without writing anything at all.
    out.mkdir(parents=True, exist_ok=True)
    if (out / 'sim_report.json').is_file():
        raise SystemExit(f'{args.run_id} already has a sim report; choose a new run id')
    started = time.monotonic()

    report = dict(scope='PROBE_N_IPC', probe='N', side='sim', run_id=args.run_id,
                  command=sys.argv, status='ERROR', idle=bool(args.idle),
                  config_path=str(cfg_path.relative_to(ROOT)),
                  config_sha256=hashlib.sha256(cfg_path.read_bytes()).hexdigest(),
                  source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
                  world=str(world.relative_to(ROOT)),
                  ros_domain_id=int(cfg['ros']['domain_id']),
                  truth_samples=[])

    state_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    command_sock = None
    try:
        import mujoco

        report['world_sha256'] = hashlib.sha256(world.read_bytes()).hexdigest()
        m = mujoco.MjModel.from_xml_path(str(world))
        d = mujoco.MjData(m)
        report['mujoco'] = mujoco.__version__
        report['model'] = {'nq': int(m.nq), 'nv': int(m.nv), 'nu': int(m.nu),
                           'mass_kg': round(float(m.body_mass.sum()), 4)}
        report['equality_constraints'] = int(m.neq)
        base = m.body('base_link').id
        lidar = m.site('lidar_site').id
        dofs = {name: m.jnt_dofadr[m.joint(name).id] for name in WHEEL_JOINTS}
        actuators = [m.actuator(f'{name[:-6]}_motor').id for name in WHEEL_JOINTS]

        scan_cfg = cfg['scan']
        nrays = int(scan_cfg['rays'])
        angles = np.radians(np.linspace(scan_cfg['angle_min_deg'], scan_cfg['angle_max_deg'],
                                        nrays))
        local = np.stack([np.cos(angles), np.sin(angles), np.zeros(nrays)], axis=1)
        range_max = float(scan_cfg['range_max_m'])

        def scan():
            vec = local @ d.xmat[base].reshape(3, 3).T
            geomid = np.zeros(nrays, dtype=np.int32)
            dist = np.full(nrays, np.inf)
            mujoco.mj_multiRay(m, d, d.site_xpos[lidar].copy(), vec.ravel(), None, True,
                               base, geomid, dist, nrays, range_max + 1.0)
            # A ray that hits nothing reports the configured maximum, not inf: an infinite
            # range would be refused by the contract, and the LaserScan has no room for it.
            # CLAMP BOTH ENDS OF THE RANGE, and the clamp is the whole point.
            #
            # The ray is deliberately cast `range_max + 1.0` so a hit slightly past the
            # declared maximum is still DETECTED -- but a detected hit at 8.11 m is still
            # outside the contract this datagram is validated against
            # (`0.0 <= value <= range_max * 1.0001`), so passing it through gets the WHOLE
            # datagram refused as REFUSED_OUT_OF_RANGE.
            #
            # MEASURED, p3-tx-03 (unpatched): `scan observed_max_m 8.1135` against
            # `range_max_m 8.0`, 35 of 195 arrivals refused, ALL REFUSED_OUT_OF_RANGE.
            # The first version clamped only the no-hit case (`inf`) and left the
            # beyond-range case alone. P1's 8x8 m arena never had anything beyond 8.0 m,
            # so this never fired before; the P3 corridor is 11.039 m long.
            out = np.where(np.isfinite(dist), dist, range_max)
            return np.clip(out, float(cfg['scan']['range_min_m']), range_max)

        gate = CommandGate(ttl_s=cfg['command']['ttl_s'], silence_s=cfg['command']['silence_s'],
                           v_max=cfg['command']['v_max_mps'],
                           w_max=cfg['command']['w_max_radps'])
        ipc = cfg['ipc']
        host, state_port = ipc['host'], int(ipc['state_port'])
        if not args.idle:
            command_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            command_sock.bind((host, int(ipc['command_port'])))
            command_sock.setblocking(False)

        radius, track = float(cfg['odom']['wheel_radius_m']), float(cfg['odom']['track_m'])
        kp = float(cfg['sim']['wheel_kp'])
        torque_limit = float(cfg['sim']['wheel_torque_limit_nm'])
        timestep = float(cfg['sim']['timestep_s'])
        publish_every = 1.0 / float(cfg['sim']['state_rate_hz'])
        next_publish = 0.0
        wall_deadline = float(cfg['sim']['wall_deadline_s'])

        counters = dict(states_sent=0, state_encode_refusals=0, commands_received=0,
                        bytes_out=0)
        modes = {}
        next_state = None
        previous_publish_sim_time = None
        scan_stats = {'min': float('inf'), 'max': 0.0, 'finite': 0, 'rays': 0}

        for _ in range(600):
            d.ctrl[:] = 0.0
            mujoco.mj_step(m, d)
        report['settle_z'] = round(float(d.qpos[2]), 6)

        # Pace to the wall clock. Measured without this: the physics free-ran 24.002 sim
        # seconds in 0.534 s -- about 45x real time -- so the whole run finished while the
        # ROS process was still importing rclpy and every datagram was sent to nobody. A
        # clock that runs 45x faster than the wall clock is also a poor time source for
        # consumers whose timers, timeouts and leases are written in seconds.
        realtime = float(cfg['sim'].get('realtime_factor', 1.0))
        loop_wall_start = time.monotonic()
        loop_sim_start = float(d.time)
        report['realtime_factor'] = realtime

        # P1-N-07 sets the run length by when the goal is reached, and the simulator's
        # report is where the truth trajectory lives, so it is written on the way out. SIGUSR1
        # therefore ends the loop GRACEFULLY and still writes it, instead of the orchestrator
        # killing the process and losing the evidence. Opt-in: without the flag this behaves
        # exactly as P1-N-06 did.
        stop_requested = {'requested': False, 'signal': None}
        report['stop_signal_armed'] = bool(args.allow_stop_signal)
        if args.allow_stop_signal:
            def _on_stop(signum, _frame):
                stop_requested['requested'] = True
                stop_requested['signal'] = int(signum)

            signal.signal(signal.SIGUSR1, _on_stop)

        while d.time < duration and not stop_requested['requested']:
            if time.monotonic() - started > wall_deadline:
                report['status'] = 'WALL_TIMEOUT'
                break
            if command_sock is not None:
                for _ in range(int(ipc['max_datagrams_per_tick'])):
                    try:
                        raw, _peer = command_sock.recvfrom(int(ipc['max_datagram_bytes']))
                    except BlockingIOError:
                        break
                    counters['commands_received'] += 1
                    gate.offer(raw, d.time)

            v, w, mode, reason = gate.step(d.time)
            modes[mode] = modes.get(mode, 0) + 1
            targets = ((v - w * track / 2.0) / radius, (v + w * track / 2.0) / radius)
            for index, name in enumerate(WHEEL_JOINTS):
                error = targets[index] - d.qvel[dofs[name]]
                d.ctrl[actuators[index]] = float(np.clip(kp * error, -torque_limit,
                                                        torque_limit))
            mujoco.mj_step(m, d)
            if realtime > 0.0:
                lag = (loop_wall_start + (float(d.time) - loop_sim_start) / realtime
                       - time.monotonic())
                if lag > 0.0005:
                    time.sleep(lag)

            if d.time >= next_publish:
                # Next target is measured from now, not accumulated: the 1.2 s settle happens
                # before this loop, and an accumulating counter publishes every step until it
                # catches up.
                next_publish = float(d.time) + publish_every
                ranges = scan()
                scan_stats['min'] = min(scan_stats['min'], float(ranges.min()))
                scan_stats['max'] = max(scan_stats['max'], float(ranges.max()))
                scan_stats['finite'] += int(np.isfinite(ranges).sum())
                scan_stats['rays'] += int(ranges.size)
                summary = gate.summary(d.time)
                payload = state_payload(
                    seq=counters['states_sent'],
                    sim_time=round(float(d.time), 6),
                    wheel_rate=[round(float(d.qvel[dofs[n]]), 6) for n in WHEEL_JOINTS],
                    ranges=[round(float(value), 4) for value in ranges],
                    range_min=float(scan_cfg['range_min_m']),
                    range_max=range_max,
                    gate={'mode': summary['mode'], 'age_s': summary['age_s'],
                          'accepted': summary['accepted'], 'refused': summary['refused']})
                try:
                    raw = encode('state', payload)
                except Refused as exc:
                    counters['state_encode_refusals'] += 1
                    report.setdefault('state_encode_errors', []).append(exc.reason)
                else:
                    state_sock.sendto(raw, (host, state_port))
                    counters['states_sent'] += 1
                    counters['bytes_out'] += len(raw)
                # Truth is recorded HERE, in the simulator's own report, and compared against
                # the ROS side's estimate afterwards by an evaluator. It is never published:
                # the state schema above has no pose field at all.
                qw, qx, qy, qz = d.xquat[base]
                yaw = math.atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))
                next_state = payload
                previous_publish_sim_time = float(d.time)
                report['truth_samples'].append([round(float(d.time), 4),
                                                round(float(d.qpos[0]), 6),
                                                round(float(d.qpos[1]), 6),
                                                round(float(yaw), 6)])
        else:
            report['status'] = 'COMPLETED'
            report['stop_reason'] = ('SIGUSR1' if stop_requested['requested']
                                     else 'DURATION_ELAPSED')

        qw, qx, qy, qz = d.xquat[base]
        final_yaw = math.atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))
        report['sim_seconds'] = round(float(d.time), 6)
        loop_wall = time.monotonic() - loop_wall_start
        report['loop_wall_seconds'] = round(loop_wall, 3)
        report['achieved_realtime_ratio'] = (
            round((float(d.time) - loop_sim_start) / loop_wall, 3) if loop_wall > 0 else None)
        report['state_schema'] = {
            'keys': sorted(next_state.keys()) if next_state else [],
            'world_pose_in_state': bool(next_state and any(
                'pose' in key or key in ('x', 'y', 'yaw') for key in next_state)),
        }
        report['ipc'] = counters
        report['gate'] = gate.summary(d.time)
        report['gate']['modes'] = dict(sorted(modes.items()))
        report['scan'] = {'rays': int(scan_cfg['rays']),
                          'observed_min_m': None if scan_stats['min'] == float('inf')
                          else round(scan_stats['min'], 4),
                          'observed_max_m': round(scan_stats['max'], 4),
                          'finite_fraction': (round(scan_stats['finite'] / scan_stats['rays'], 6)
                                              if scan_stats['rays'] else 0.0)}
        report['truth'] = {'final': [round(float(d.qpos[0]), 6), round(float(d.qpos[1]), 6),
                                     round(final_yaw, 6)],
                           'samples': len(report['truth_samples']),
                           'travelled_m': round(_travelled(report['truth_samples']), 4)}
    except Exception as exc:
        report['error'] = f'{type(exc).__name__}: {exc}'
        if report.get('status') in (None, 'ERROR'):
            report['status'] = 'ERROR'
    finally:
        state_sock.close()
        if command_sock is not None:
            command_sock.close()
        report['wall_seconds'] = round(time.monotonic() - started, 3)
        (out / 'sim_report.json').write_text(json.dumps(report, indent=2) + '\n')
        head = {k: v for k, v in report.items() if k != 'truth_samples'}
        print(json.dumps(head, indent=2, ensure_ascii=False), flush=True)
    return 0 if report['status'] == 'COMPLETED' else 1


if __name__ == '__main__':
    raise SystemExit(main())
