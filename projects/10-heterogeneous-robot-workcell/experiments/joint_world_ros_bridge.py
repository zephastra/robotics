"""Candidate shared-world ROS bridge derived from 010's P1-N bridge.

Only fresh stamped controller messages may renew the command lease. The old
probe remains unchanged; this module does not claim commissioned Nav2 or SLAM.

Scope (docs/P1_FEASIBILITY.md section N): "MuJoCo is the only publisher of physical state, the
ROS process outputs /clock, scan, wheel odometry and TF, and consumes velocity through the
command gate. Wheel odometry must not read the world pose."

This process runs under the system interpreter with /opt/ros/lyrical sourced, because MuJoCo
is in the project .venv (Python 3.12) and rclpy is Python 3.14. The two cannot share an
interpreter, so the world state arrives over loopback UDP and this file turns it into ROS.

Four decisions worth stating, because each could be made the other way:

  * No rclpy timer drives publication. Publication is driven by the arrival of a state
    datagram, so the ROS graph is a mirror of the simulation rather than an independent clock.
  * This node does not set `use_sim_time`. It *publishes* /clock; a node that both publishes
    the clock and depends on it is a deadlock waiting for the first late datagram. Every
    outgoing stamp is taken from the datagram's `sim_time`.
  * Odometry is integrated from the wheel rates in the datagram using the wheel radius and
    track from config. There is no pose in the datagram to copy from -- which is the point.
  * The scan is republished from rays cast in the simulator through the actual scene. Nothing
    here knows where the obstacles are.
"""
import argparse
import hashlib
import json
import math
from pathlib import Path
import socket
import sys
import time

ROOT = Path(__file__).resolve().parents[1]

#: How long this side will go without a state datagram before deciding the stream has end.#:
#: IT USED TO BE 1.0 s, AND THAT IS A DEFECT. A UDP state stream can drop a BURST, and at
#: p3-nav-03's ~6.9 datagrams/s of wall time 1.0 s of silence is about fourteen consecutive
#: drops. On 2026-09-23 that is exactly what happened: the bridge gave up 40 s into a 144 s
#: run and reported STATE_STREAM_STOPPED_EARLY. Everything downstream then degraded
#: SILENTLY -- /clock stopped at 12.184 s, the orchestrator's observed clock stalled, Nav2
#: relayed 34 commands in the whole run, the controller blocked on a transform it could no
#: longer look up, and the round ended as GOAL_TIMEOUT, which reads like a navigation
#: failure and was not one.
#:
#: 5.0 s is DECLARED, not fitted: long enough to ride out a burst at any rate this project
#: runs at (state_rate_hz 20 of sim time, so >= 7/s of wall time at realtime_factor 0.35),
#: short enough that a genuinely dead simulator still ends a run promptly.
#:
#: The structural fix -- an explicit end-of-stream sentinel from the simulator, so 'the
#: stream ended' is a FACT rather than an inference from silence -- is recorded as the next
#: action instead of being done here, because it changes the datagram schema and P1-N's
#: recorded evidence is written against the current one.
STATE_GAP_TOLERANCE_S = 5.0
sys.path.insert(0, str(ROOT / 'src'))

from workcell_ipc import Refused, command as command_payload, decode, encode  # noqa: E402
from ros_command_lease import ControllerLease


def main():
    import rclpy
    from builtin_interfaces.msg import Time
    from geometry_msgs.msg import TransformStamped
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
    from rosgraph_msgs.msg import Clock
    from sensor_msgs.msg import LaserScan
    from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster

    parser = argparse.ArgumentParser()
    parser.add_argument('--config', default='config/n_probe.yaml')
    parser.add_argument('--run-id', required=True)
    parser.add_argument('--duration', type=float, default=None,
                        help='sim seconds to publish for; default from config')
    parser.add_argument('--stop-commands-at', type=float, default=None,
                        help='stop sending commands after this sim second, to test the '
                             'simulator-side silence fallback')
    parser.add_argument('--profile-start-sim-s', type=float, default=0.0,
                        help='hold the built-in profile until this sim second. P1-N-08 needs the '
                             'vehicle to stand still until AMCL is up: with set_initial_pose the '
                             'declared start pose is only true before the vehicle moves, and this '
                             'AMCL has recovery_alpha_fast/slow = 0.0, i.e. no way to recover from '
                             'a start pose that has gone stale')
    parser.add_argument('--cmd-vel-type', default='twist_stamped',
                        choices=['twist_stamped', 'twist'],
                        help='message type of the incoming command topic. nav2 in this release '
                             'publishes TwistStamped from controller_server, velocity_smoother '
                             'and collision_monitor; a subscriber of the other type receives '
                             'nothing and DDS reports no error for it')
    parser.add_argument('--command-source', default='profile',
                        choices=['profile', 'cmd_vel'],
                        help='profile: the built-in P1-N-06 velocity profile (default, '
                             'unchanged); cmd_vel: forward /cmd_vel from a controller through '
                             'the same command gate (P1-N-07)')
    parser.add_argument('--command', default='trapezoid',
                        choices=['trapezoid', 'spin', 'idle'])
    parser.add_argument('--wait-for-state', type=float, default=15.0,
                        help='seconds to wait for the first state datagram before giving up; '
                             'the simulator is paced to the wall clock, so this side may '
                             'start first')
    args = parser.parse_args()
    if Path(args.run_id).name != args.run_id:
        raise ValueError('run id must be a bare name')

    import yaml
    if args.command_source != 'cmd_vel' or args.cmd_vel_type != 'twist_stamped':
        parser.error('shared-world candidate requires fresh TwistStamped cmd_vel; no built-in profile')
    config_path = (ROOT / args.config).resolve()
    if not config_path.is_relative_to(ROOT / 'config'):
        parser.error('config must belong to 010/config')
    cfg = yaml.safe_load(config_path.read_text(encoding='utf-8'))
    if int(cfg['ros']['domain_id']) == 42:
        raise SystemExit('config refuses ROS domain 42: that is 009 (009/scripts/env.sh:28)')
    duration = float(args.duration if args.duration is not None
                     else cfg['sim']['duration_s'])
    out = ROOT / 'reports' / args.run_id
    out.mkdir(parents=True, exist_ok=True)
    if (out / 'ros_report.json').is_file():
        raise SystemExit(f'{args.run_id} already has a ROS report; choose a new run id')

    radius = float(cfg['odom']['wheel_radius_m'])
    track = float(cfg['odom']['track_m'])
    scan_cfg = cfg['scan']
    nrays = int(scan_cfg['rays'])
    angle_min = math.radians(float(scan_cfg['angle_min_deg']))
    angle_max = math.radians(float(scan_cfg['angle_max_deg']))
    frames = cfg['frames']
    ipc = cfg['ipc']
    topics = cfg['ros']['topics']
    v_max = float(cfg['command']['v_max_mps'])
    w_max = float(cfg['command']['w_max_radps'])
    profile_start = float(args.profile_start_sim_s)

    report = dict(scope='SHARED_WORLD_ROS_CANDIDATE_NOT_NAV2_ACCEPTANCE', probe='N', side='ros', run_id=args.run_id,
                  command=sys.argv, status='ERROR',
                  command_profile=args.command, command_stopped_at=args.stop_commands_at,
                  source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
                  ros_domain_id=int(cfg['ros']['domain_id']),
                  config_sha256=hashlib.sha256(
                      config_path.read_bytes()).hexdigest())

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    # Ask for a bigger receive buffer AND READ BACK WHAT WAS GRANTED.
    #
    # It is NOT the fix, and the readback is why that is known: this host's
    # /proc/sys/net/core/rmem_max is 212992 bytes, which is also the Linux default, so the
    # request is CLAMPED and the effective buffer is unchanged. Recording the achieved
    # value means 'we asked for 8 MiB' cannot be mistaken for 'we got 8 MiB'.
    #
    # The measured loss was NOT a buffer overflow: the simulator sent 990 datagrams over
    # 143.9 s (6.9/s) and this side received 223 of the ~275 sent during the 40.3 s it
    # lived -- 81%, i.e. roughly the rate it was keeping up with. The failure was that it
    # STOPPED, not that it overflowed.
    rbuf = None
    try:
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 8 * 1024 * 1024)
        rbuf = int(sock.getsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF))
    except OSError as exc:
        rbuf = f'failed: {exc!r}'
    sock.bind((ipc['host'], int(ipc['state_port'])))
    sock.settimeout(float(ipc['socket_timeout_s']))
    started = time.monotonic()

    rclpy.init()
    node = Node('workcell010_n_bridge')
    clock_pub = node.create_publisher(Clock, topics['clock'], 10)
    scan_qos = QoSProfile(depth=5, reliability=QoSReliabilityPolicy.RELIABLE,
                          durability=QoSDurabilityPolicy.VOLATILE,
                          history=QoSHistoryPolicy.KEEP_LAST)
    scan_pub = node.create_publisher(LaserScan, topics['scan'], scan_qos)
    odom_pub = node.create_publisher(Odometry, topics['odom'], 10)
    # P1-N-07: optional /cmd_vel intake. This does not make the ROS side a second command
    # writer -- it converts a Twist into the same (v, w) datagram the profile branch sends, and
    # the simulator's command gate stays the only thing that decides what the wheels are told.
    # Clipping is counted rather than hidden: a controller asking for more than the plant
    # allows must appear as a number in the report, not as a silent rescale.
    cmd_vel = {'v': 0.0, 'w': 0.0, 'count': 0, 'clipped': 0,
               'type_checked_at': 0.0, 'publisher_types': None, 'type_mismatch': None}
    controller_lease = ControllerLease(ttl_s=float(cfg['command']['ttl_s']),
                                     v_max=v_max, w_max=w_max)
    controller_refusals = {}
    if args.command_source == 'cmd_vel':
        from geometry_msgs.msg import Twist, TwistStamped

        def on_cmd_vel(message):
            # MEASURED (P1-N-07 run 04): this nav2 publishes geometry_msgs/msg/TwistStamped on
            # the whole command chain -- controller_server, velocity_smoother and
            # collision_monitor all declare it -- while older releases published plain Twist. A
            # subscriber of the wrong type receives exactly nothing, and DDS warns about a QoS
            # mismatch but not about a type mismatch, so there is no log line to find. Hence:
            # both classes are imported, the field is read off the chosen one, and the type seen
            # on the wire is recorded in the report.
            twist = message.twist if args.cmd_vel_type == 'twist_stamped' else message
            v, w = float(twist.linear.x), float(twist.angular.z)
            cmd_vel['count'] += 1
            if sim_time is None:
                return
            issued = float(message.header.stamp.sec) + message.header.stamp.nanosec / 1e9
            accepted, reason = controller_lease.offer(v, w, issued, sim_time)
            if accepted:
                cmd_vel['v'], cmd_vel['w'] = v, w
            else:
                controller_refusals[reason] = controller_refusals.get(reason, 0) + 1

        # BEST_EFFORT + VOLATILE is compatible with every publisher QoS combination, so the
        # only failure this subscription can have is a dropped datagram -- and the command
        # gate's ttl (0.5 s) covers that. It is NOT a workaround for one publisher: three nodes
        # declare a publisher on /cmd_vel (collision_monitor, docking_server, following_server)
        # and nav2's own subscriptions to it log "incompatible QoS" against all three, which is
        # a property of this nav2 release rather than of this bridge. Run 01's zero-command
        # count was blamed on that warning and it was wrong: the cause was the message TYPE,
        # measured in run 04 (see the --cmd-vel-type help text and the report's
        # `publisher_types_seen`).
        cmd_vel_qos = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.BEST_EFFORT,
                                 durability=QoSDurabilityPolicy.VOLATILE)
        expected_type = ('geometry_msgs/msg/TwistStamped' if args.cmd_vel_type == 'twist_stamped'
                         else 'geometry_msgs/msg/Twist')
        msg_type = TwistStamped if args.cmd_vel_type == 'twist_stamped' else Twist
        node.create_subscription(msg_type, '/cmd_vel', on_cmd_vel, cmd_vel_qos)
    tf_broadcaster = TransformBroadcaster(node)
    static_broadcaster = StaticTransformBroadcaster(node)
    static = TransformStamped()
    static.header.stamp = Time()
    static.header.frame_id = frames['base']
    static.child_frame_id = frames['lidar']
    static.transform.translation.x = 0.02
    static.transform.translation.z = 0.17
    static.transform.rotation.w = 1.0
    static_broadcaster.sendTransform(static)

    def stamp(sim_time):
        """A ROS stamp taken from the simulator's clock, not from the wall clock."""
        seconds = int(sim_time)
        return Time(sec=seconds, nanosec=int(round((sim_time - seconds) * 1e9)))

    odom_x = odom_y = odom_yaw = 0.0
    counters = dict(states_received=0, decode_refusals=0, clocks=0, scans=0, odoms=0,
                    commands_sent=0, command_refusals=0, bytes_in=0)
    refusal_reasons = {}
    last_sim_time = None
    clock_monotonic = True
    seq_gaps = 0
    expected_seq = 0
    next_command_sim_time = None
    last_state_wall = time.monotonic()
    sim_time = None
    sim_dt = None

    try:
        while True:
            if sim_time is not None and sim_time >= duration:
                report['status'] = 'COMPLETED'
                break
            if time.monotonic() - started > duration * 4.0 + 30.0:
                report['status'] = 'WALL_TIMEOUT'
                break
            try:
                raw, _peer = sock.recvfrom(int(ipc['max_datagram_bytes']))
            except socket.timeout:
                idle = time.monotonic() - last_state_wall
                if counters['states_received'] == 0:
                    if idle > args.wait_for_state:
                        report['status'] = 'NO_STATE_FROM_SIM'
                        break
                elif idle > STATE_GAP_TOLERANCE_S and sim_time is not None:
                    # The simulator exits when its scenario ends, and to a receiver that is
                    # indistinguishable from a simulator that never started -- both are
                    # silence. Measured on the first working run: 446 states arrived, the run
                    # was complete, and this side reported NO_STATE_FROM_SIM.
                    report['status'] = ('COMPLETED' if sim_time >= duration - 0.25
                                        else 'STATE_STREAM_STOPPED_EARLY')
                    break
                continue
            counters['states_received'] += 1
            counters['bytes_in'] += len(raw)
            # THE LIVENESS CLOCK IS REFRESHED BY ANY ARRIVAL, REFUSED OR NOT.
            #
            # It used to be refreshed only after a SUCCESSFUL decode, which made a burst
            # of refused datagrams indistinguishable from silence: `idle` grew past the
            # tolerance, the bridge concluded the stream had ENDED while datagrams were
            # arriving perfectly, and every P3 navigation round then died as GOAL_TIMEOUT
            # with nothing to suggest the transport was fine.
            #
            # A refused datagram is still PROOF THE SENDER IS ALIVE. Whether it is USABLE
            # is a separate question, answered by the refusal counter and its reasons.
            last_state_wall = time.monotonic()
            try:
                payload = decode('state', raw, expect_ranges=nrays)
            except Refused as exc:
                counters['decode_refusals'] += 1
                refusal_reasons[exc.reason] = refusal_reasons.get(exc.reason, 0) + 1
                continue
            previous = sim_time
            sim_time = float(payload['sim_time'])
            if previous is not None:
                sim_dt = sim_time - previous
                if sim_time <= previous:
                    clock_monotonic = False
            if payload['seq'] != expected_seq:
                seq_gaps += 1
            expected_seq = payload['seq'] + 1

            message_stamp = stamp(sim_time)

            # /clock: this process owns it. One publisher, sim time as the origin.
            clock = Clock()
            clock.clock = message_stamp
            clock_pub.publish(clock)
            counters['clocks'] += 1

            # /scan: republished from the simulator's rays, no world knowledge here.
            scan = LaserScan()
            scan.header.stamp = message_stamp
            scan.header.frame_id = frames['lidar']
            scan.angle_min = angle_min
            scan.angle_max = angle_max
            scan.angle_increment = (angle_max - angle_min) / max(nrays - 1, 1)
            scan.range_min = float(scan_cfg['range_min_m'])
            scan.range_max = float(scan_cfg['range_max_m'])
            scan.ranges = [float(value) for value in payload['ranges']]
            scan_pub.publish(scan)
            counters['scans'] += 1

            # /odom: integrated from wheel rates. The datagram has no pose to copy.
            left, right = payload['wheel_rate']
            linear = 0.5 * radius * (left + right)
            angular = radius * (right - left) / track
            odom_x += linear * math.cos(odom_yaw) * sim_dt if sim_dt else 0.0
            odom_y += linear * math.sin(odom_yaw) * sim_dt if sim_dt else 0.0
            odom_yaw += angular * sim_dt if sim_dt else 0.0
            odom = Odometry()
            odom.header.stamp = message_stamp
            odom.header.frame_id = frames['odom']
            odom.child_frame_id = frames['base']
            odom.pose.pose.position.x = odom_x
            odom.pose.pose.position.y = odom_y
            odom.pose.pose.orientation.z = math.sin(odom_yaw / 2.0)
            odom.pose.pose.orientation.w = math.cos(odom_yaw / 2.0)
            odom.twist.twist.linear.x = linear
            odom.twist.twist.angular.z = angular
            odom_pub.publish(odom)
            counters['odoms'] += 1

            transform = TransformStamped()
            transform.header.stamp = message_stamp
            transform.header.frame_id = frames['odom']
            transform.child_frame_id = frames['base']
            transform.transform.translation.x = odom_x
            transform.transform.translation.y = odom_y
            transform.transform.rotation.z = math.sin(odom_yaw / 2.0)
            transform.transform.rotation.w = math.cos(odom_yaw / 2.0)
            tf_broadcaster.sendTransform(transform)

            # Command, through the gate. Issued with the sim time it was computed at, so the
            # simulator can refuse it as expired instead of acting on a stale request.
            if args.command_source == 'cmd_vel' and sim_time - cmd_vel['type_checked_at'] > 1.0:
                # A type mismatch is silent, so it has to be looked for. This is the check that
                # would have caught run 04 in one second instead of four runs: with the wrong
                # message class the count stays 0 and nothing anywhere says why.
                cmd_vel['type_checked_at'] = sim_time
                try:
                    seen = sorted({info.topic_type for info in
                                   node.get_publishers_info_by_topic('/cmd_vel')})
                except Exception as exc:                            # noqa: BLE001
                    seen = [f'error: {type(exc).__name__}: {exc}']
                if seen:
                    cmd_vel['publisher_types'] = seen
                    cmd_vel['type_mismatch'] = expected_type not in seen
            stopped = (args.stop_commands_at is not None
                       and sim_time >= args.stop_commands_at)
            if args.command_source == 'cmd_vel':
                # One datagram per state datagram. The gate's ttl is 0.5 s and the state rate
                # is 20 Hz, so one dropped message does not stop the robot, and a controller
                # that dies is stopped by the gate's silence fallback rather than by this file.
                fresh_command = controller_lease.value(sim_time)
                active = not stopped and fresh_command is not None
            else:
                # profile_start holds the built-in profile until the caller says the observer is
                # ready. It is 0 by default, so the P1-N-06 behaviour is byte for byte the same.
                active = (args.command != 'idle' and not stopped
                          and sim_time >= profile_start)
            if active:
                if next_command_sim_time is None or sim_time >= next_command_sim_time:
                    next_command_sim_time = sim_time + 1.0 / float(cfg['command']['send_rate_hz'])
                    if args.command_source == 'cmd_vel':
                        v, w, issued_sim_time = fresh_command
                    elif args.command == 'spin':
                        v, w = 0.0, 0.6 * w_max
                    else:                      # trapezoid: forward, turn, come back
                        phase = (sim_time - profile_start) % 12.0
                        v = 0.5 * v_max if phase < 4.0 else (0.0 if phase < 6.0 else
                                                             (0.45 * v_max if phase < 10.0
                                                              else 0.0))
                        w = 0.0 if phase < 4.0 else (0.7 * w_max if phase < 6.0 else
                                                     (0.25 * w_max if phase < 10.0 else 0.0))
                    payload_out = command_payload(
                        seq=counters['commands_sent'],
                        issued_sim_time=round(issued_sim_time, 6),
                        ttl_s=float(cfg['command']['ttl_s']),
                        v=round(v, 6), w=round(w, 6))
                    try:
                        sock.sendto(encode('command', payload_out),
                                    (ipc['host'], int(ipc['command_port'])))
                    except Refused:
                        counters['command_refusals'] += 1
                    else:
                        counters['commands_sent'] += 1
            # Spin more than once per datagram: /cmd_vel arrives at about the state rate,
            # so a single spin per datagram sits at the edge of dropping messages -- and the
            # loss would read as "the controller stopped", not "the bridge was busy".
            for _ in range(4):
                rclpy.spin_once(node, timeout_sec=0.0)
    except Exception as exc:
        report['error'] = f'{type(exc).__name__}: {exc}'
    finally:
        report['sim_seconds'] = round(sim_time, 6) if sim_time is not None else None
        report['sim_state_dt_s'] = round(sim_dt, 6) if sim_dt else None
        report['clock_monotonic'] = clock_monotonic
        report['seq_gaps'] = seq_gaps
        report['counters'] = counters
        report['recv_buffer_bytes'] = rbuf
        report['state_gap_tolerance_s'] = STATE_GAP_TOLERANCE_S
        report['decode_refusal_reasons'] = dict(sorted(refusal_reasons.items()))
        report['controller_refusal_reasons'] = dict(sorted(controller_refusals.items()))
        report['lease_source'] = 'original_controller_stamp_and_callback_wall_time'
        report['odom_final'] = {'x': round(odom_x, 6), 'y': round(odom_y, 6),
                                'yaw': round(odom_yaw, 6)}
        report['command_source'] = args.command_source
        report['cmd_vel'] = {'topic': ('/cmd_vel' if args.command_source == 'cmd_vel'
                                       else None),
                             'received': cmd_vel['count'], 'clipped': cmd_vel['clipped'],
                             'last_v': round(cmd_vel['v'], 6),
                             'last_w': round(cmd_vel['w'], 6),
                             'limits': {'v_max_mps': v_max, 'w_max_radps': w_max}}
        report['command_source'] = args.command_source
        report['cmd_vel'] = {'topic': ('/cmd_vel' if args.command_source == 'cmd_vel'
                                       else None),
                             'received': cmd_vel['count'], 'clipped': cmd_vel['clipped'],
                             'last_v': round(cmd_vel['v'], 6),
                             'last_w': round(cmd_vel['w'], 6),
                             'limits': {'v_max_mps': v_max, 'w_max_radps': w_max},
                             'type': args.cmd_vel_type,
                             'expected_topic_type': ('geometry_msgs/msg/TwistStamped'
                                                     if args.cmd_vel_type == 'twist_stamped'
                                                     else 'geometry_msgs/msg/Twist'),
                             'publisher_types_seen': cmd_vel['publisher_types'],
                             'type_mismatch': cmd_vel['type_mismatch']}
        report['wall_seconds'] = round(time.monotonic() - started, 3)
        node.destroy_node()
        rclpy.shutdown()
        sock.close()
        (out / 'ros_report.json').write_text(json.dumps(report, indent=2) + '\n')
        print(json.dumps(report, indent=2, ensure_ascii=False), flush=True)
    return 0 if report['status'] == 'COMPLETED' else 1


if __name__ == '__main__':
    raise SystemExit(main())
