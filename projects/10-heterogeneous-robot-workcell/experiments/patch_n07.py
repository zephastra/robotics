"""P1-N-07 patches to the two existing N probe scripts.

Two additions, both opt-in, so the P1-N-06 verdict (reports/p1-n-ipc-07/08) is reproducible from
the same files with the same defaults:

  probe_n_ros.py  --command-source {profile,cmd_vel}
      `profile` is the default and is the P1-N-06 behaviour byte for byte. `cmd_vel` subscribes
      /cmd_vel and forwards the controller's command through the same simulator-side gate; the
      bridge does not become a second command writer, it becomes the one converter.

  probe_n_sim.py  --allow-stop-signal
      SIGUSR1 ends the stepping loop *gracefully* and still writes the report. The P1-N-07 run
      length is set by when the goal is reached, and the truth trajectory lives in that report,
      so the alternative (killing the process) would throw the evidence away. Off by default.

Run:  .venv/bin/python <this file> [--check]
       --check verifies the patches are already applied and the files parse.
"""
import argparse
import ast
import hashlib
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]

# The sha256 each file must have once this script has been applied. Recorded rather than
# recomputed because several patches are superseded by later ones -- e.g. the /cmd_vel
# subscription line is rewritten twice -- so "is this patch present?" cannot be answered by
# searching for its own output. A fingerprint also turns "someone edited the bridge by hand"
# into a failing check, which is the point: 010 has no git.
EXPECTED_SHA256 = {
    'experiments/probe_n_ros.py':
        '4efd133e0217a904f10f9a9f879df085d9a0160e1362184f403d1d4f13c5deae',
    'experiments/probe_n_sim.py':
        'c4d3aa196ba837cda20323dd2e04c7cad8889083078e4d112ca2e6849db78c0a',
}

PATCHES = {
    'experiments/probe_n_ros.py': [
        (
            'CLI: --cmd-vel-type',
            """    parser.add_argument('--command-source', default='profile',
""",
            """    parser.add_argument('--cmd-vel-type', default='twist_stamped',
                        choices=['twist_stamped', 'twist'],
                        help='message type of the incoming command topic. nav2 in this release '
                             'publishes TwistStamped from controller_server, velocity_smoother '
                             'and collision_monitor; a subscriber of the other type receives '
                             'nothing and DDS reports no error for it')
    parser.add_argument('--command-source', default='profile',
""",
        ),
        (
            'CLI: --command-source',
            """    parser.add_argument('--command', default='trapezoid',
                        choices=['trapezoid', 'spin', 'idle'])
""",
            """    parser.add_argument('--command-source', default='profile',
                        choices=['profile', 'cmd_vel'],
                        help='profile: the built-in P1-N-06 velocity profile (default, '
                             'unchanged); cmd_vel: forward /cmd_vel from a controller through '
                             'the same command gate (P1-N-07)')
    parser.add_argument('--command', default='trapezoid',
                        choices=['trapezoid', 'spin', 'idle'])
""",
        ),
        (
            'subscribe /cmd_vel and remember the last command',
            """    odom_pub = node.create_publisher(Odometry, topics['odom'], 10)
    tf_broadcaster = TransformBroadcaster(node)
""",
            """    odom_pub = node.create_publisher(Odometry, topics['odom'], 10)
    # P1-N-07: optional /cmd_vel intake. This does not make the ROS side a second command
    # writer -- it converts a Twist into the same (v, w) datagram the profile branch sends, and
    # the simulator's command gate stays the only thing that decides what the wheels are told.
    # Clipping is counted rather than hidden: a controller asking for more than the plant
    # allows must appear as a number in the report, not as a silent rescale.
    cmd_vel = {'v': 0.0, 'w': 0.0, 'count': 0, 'clipped': 0}
    if args.command_source == 'cmd_vel':
        from geometry_msgs.msg import Twist

        def on_cmd_vel(message):
            v, w = float(message.linear.x), float(message.angular.z)
            clipped_v = min(v_max, max(-v_max, v))
            clipped_w = min(w_max, max(-w_max, w))
            if clipped_v != v or clipped_w != w:
                cmd_vel['clipped'] += 1
            cmd_vel['v'], cmd_vel['w'] = clipped_v, clipped_w
            cmd_vel['count'] += 1

        node.create_subscription(Twist, '/cmd_vel', on_cmd_vel, 10)
    tf_broadcaster = TransformBroadcaster(node)
""",
        ),
        (
            'command source selection',
            """            if args.command != 'idle' and not stopped:
                if next_command_sim_time is None or sim_time >= next_command_sim_time:
                    next_command_sim_time = sim_time + 1.0 / float(cfg['command']['send_rate_hz'])
                    if args.command == 'spin':
""",
            """            if args.command_source == 'cmd_vel':
                # One datagram per state datagram. The gate's ttl is 0.5 s and the state rate
                # is 20 Hz, so one dropped message does not stop the robot, and a controller
                # that dies is stopped by the gate's silence fallback rather than by this file.
                active = not stopped and cmd_vel['count'] > 0
            else:
                active = args.command != 'idle' and not stopped
            if active:
                if next_command_sim_time is None or sim_time >= next_command_sim_time:
                    next_command_sim_time = sim_time + 1.0 / float(cfg['command']['send_rate_hz'])
                    if args.command_source == 'cmd_vel':
                        v, w = cmd_vel['v'], cmd_vel['w']
                    elif args.command == 'spin':
""",
        ),
        (
            'subscribe /cmd_vel with a QoS that any publisher can serve',
            """        node.create_subscription(Twist, '/cmd_vel', on_cmd_vel, 10)
""",
            """        # BEST_EFFORT + VOLATILE is compatible with every publisher QoS combination, and
        # this subscription has to be: as nav2 wires the stack, collision_monitor is the LAST
        # writer on /cmd_vel, and a reliable subscriber silently receives zero messages from
        # it. Measured on the first P1-N-07 run -- collision_monitor logged "New subscription
        # discovered on topic '/cmd_vel', requesting incompatible QoS. No messages will be sent
        # to it", and the bridge counted 0 commands while the robot never moved. nav2's own
        # docking_server and following_server subscriptions hit the same wall. The command
        # gate's ttl (0.5 s) covers any datagram a best-effort link drops.
        cmd_vel_qos = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.BEST_EFFORT,
                                 durability=QoSDurabilityPolicy.VOLATILE)
        node.create_subscription(Twist, '/cmd_vel', on_cmd_vel, cmd_vel_qos)
""",
        ),
        (
            'read the command fields from the type that was actually measured',
            """        from geometry_msgs.msg import Twist

        def on_cmd_vel(message):
            v, w = float(message.linear.x), float(message.angular.z)
""",
            """        from geometry_msgs.msg import Twist, TwistStamped

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
""",
        ),
        (
            'pick the message class from --cmd-vel-type',
            """        cmd_vel_qos = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.BEST_EFFORT,
                                 durability=QoSDurabilityPolicy.VOLATILE)
        node.create_subscription(Twist, '/cmd_vel', on_cmd_vel, cmd_vel_qos)
""",
            """        cmd_vel_qos = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.BEST_EFFORT,
                                 durability=QoSDurabilityPolicy.VOLATILE)
        expected_type = ('geometry_msgs/msg/TwistStamped' if args.cmd_vel_type == 'twist_stamped'
                         else 'geometry_msgs/msg/Twist')
        msg_type = TwistStamped if args.cmd_vel_type == 'twist_stamped' else Twist
        node.create_subscription(msg_type, '/cmd_vel', on_cmd_vel, cmd_vel_qos)
""",
        ),
        (
            'record the command type seen on the wire, once a sim second',
            """            stopped = (args.stop_commands_at is not None
                       and sim_time >= args.stop_commands_at)
""",
            """            if args.command_source == 'cmd_vel' and sim_time - cmd_vel['type_checked_at'] > 1.0:
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
""",
        ),
        (
            'report the command type and any mismatch',
            """        report['cmd_vel'] = {'topic': ('/cmd_vel' if args.command_source == 'cmd_vel'
                                       else None),
                             'received': cmd_vel['count'], 'clipped': cmd_vel['clipped'],
                             'last_v': round(cmd_vel['v'], 6),
                             'last_w': round(cmd_vel['w'], 6),
                             'limits': {'v_max_mps': v_max, 'w_max_radps': w_max}}
""",
            """        report['cmd_vel'] = {'topic': ('/cmd_vel' if args.command_source == 'cmd_vel'
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
""",
        ),
        (
            'state the command chain and the type mismatch in the QoS comment',
            """        # BEST_EFFORT + VOLATILE is compatible with every publisher QoS combination, and
        # this subscription has to be: as nav2 wires the stack, collision_monitor is the LAST
        # writer on /cmd_vel, and a reliable subscriber silently receives zero messages from
        # it. Measured on the first P1-N-07 run -- collision_monitor logged "New subscription
        # discovered on topic '/cmd_vel', requesting incompatible QoS. No messages will be sent
        # to it", and the bridge counted 0 commands while the robot never moved. nav2's own
        # docking_server and following_server subscriptions hit the same wall. The command
        # gate's ttl (0.5 s) covers any datagram a best-effort link drops.
""",
            """        # BEST_EFFORT + VOLATILE is compatible with every publisher QoS combination, so the
        # only failure this subscription can have is a dropped datagram -- and the command
        # gate's ttl (0.5 s) covers that. It is NOT a workaround for one publisher: three nodes
        # declare a publisher on /cmd_vel (collision_monitor, docking_server, following_server)
        # and nav2's own subscriptions to it log "incompatible QoS" against all three, which is
        # a property of this nav2 release rather than of this bridge. Run 01's zero-command
        # count was blamed on that warning and it was wrong: the cause was the message TYPE,
        # measured in run 04 (see the --cmd-vel-type help text and the report's
        # `publisher_types_seen`).
""",
        ),
        (
            'track the type-check state on the command dict',
            """    cmd_vel = {'v': 0.0, 'w': 0.0, 'count': 0, 'clipped': 0}
""",
            """    cmd_vel = {'v': 0.0, 'w': 0.0, 'count': 0, 'clipped': 0,
               'type_checked_at': 0.0, 'publisher_types': None, 'type_mismatch': None}
""",
        ),
        (
            'CLI: --profile-start-sim-s',
            """    parser.add_argument('--cmd-vel-type', default='twist_stamped',
""",
            """    parser.add_argument('--profile-start-sim-s', type=float, default=0.0,
                        help='hold the built-in profile until this sim second. P1-N-08 needs the '
                             'vehicle to stand still until AMCL is up: with set_initial_pose the '
                             'declared start pose is only true before the vehicle moves, and this '
                             'AMCL has recovery_alpha_fast/slow = 0.0, i.e. no way to recover from '
                             'a start pose that has gone stale')
    parser.add_argument('--cmd-vel-type', default='twist_stamped',
""",
        ),
        (
            'honour the profile start offset',
            """            else:
                active = args.command != 'idle' and not stopped
""",
            """            else:
                # profile_start holds the built-in profile until the caller says the observer is
                # ready. It is 0 by default, so the P1-N-06 behaviour is byte for byte the same.
                active = (args.command != 'idle' and not stopped
                          and sim_time >= profile_start)
""",
        ),
        (
            'phase the profile from its start offset',
            """                        phase = sim_time % 12.0
""",
            """                        phase = (sim_time - profile_start) % 12.0
""",
        ),
        (
            'define profile_start next to the other command limits',
            """    v_max = float(cfg['command']['v_max_mps'])
    w_max = float(cfg['command']['w_max_radps'])
""",
            """    v_max = float(cfg['command']['v_max_mps'])
    w_max = float(cfg['command']['w_max_radps'])
    profile_start = float(args.profile_start_sim_s)
""",
        ),
        (
            'spin several times per datagram so /cmd_vel is not dropped',
            """            rclpy.spin_once(node, timeout_sec=0.0)
""",
            """            # Spin more than once per datagram: /cmd_vel arrives at about the state rate,
            # so a single spin per datagram sits at the edge of dropping messages -- and the
            # loss would read as "the controller stopped", not "the bridge was busy".
            for _ in range(4):
                rclpy.spin_once(node, timeout_sec=0.0)
""",
        ),
        (
            'report the command source and the cmd_vel counters',
            """        report['odom_final'] = {'x': round(odom_x, 6), 'y': round(odom_y, 6),
                                'yaw': round(odom_yaw, 6)}
""",
            """        report['odom_final'] = {'x': round(odom_x, 6), 'y': round(odom_y, 6),
                                'yaw': round(odom_yaw, 6)}
        report['command_source'] = args.command_source
        report['cmd_vel'] = {'topic': ('/cmd_vel' if args.command_source == 'cmd_vel'
                                       else None),
                             'received': cmd_vel['count'], 'clipped': cmd_vel['clipped'],
                             'last_v': round(cmd_vel['v'], 6),
                             'last_w': round(cmd_vel['w'], 6),
                             'limits': {'v_max_mps': v_max, 'w_max_radps': w_max}}
""",
        ),
    ],
    'experiments/probe_n_sim.py': [
        (
            'import signal',
            """import os
from pathlib import Path
""",
            """import os
import signal
from pathlib import Path
""",
        ),
        (
            'CLI: --allow-stop-signal',
            """    parser.add_argument('--idle', action='store_true',
                        help='do not open the command socket; the gate must fall back to zero')
""",
            """    parser.add_argument('--idle', action='store_true',
                        help='do not open the command socket; the gate must fall back to zero')
    parser.add_argument('--allow-stop-signal', action='store_true',
                        help='let SIGUSR1 end the stepping loop early and still write the '
                             'report, so a run whose length is set by an event does not have '
                             'to wait out the whole duration (P1-N-07)')
""",
        ),
        (
            'arm the graceful stop and honour it in the loop condition',
            """        while d.time < duration:
""",
            """        # P1-N-07 sets the run length by when the goal is reached, and the simulator's
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
""",
        ),
        (
            'record why the loop ended',
            """        else:
            report['status'] = 'COMPLETED'
""",
            """        else:
            report['status'] = 'COMPLETED'
            report['stop_reason'] = ('SIGUSR1' if stop_requested['requested']
                                     else 'DURATION_ELAPSED')
""",
        ),
    ],
}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--check', action='store_true')
    parser.add_argument('--root', default=str(ROOT))
    args = parser.parse_args()
    root = Path(args.root)

    failures = []
    for relative, patches in PATCHES.items():
        path = root / relative
        text = path.read_text(encoding='utf-8')
        before = hashlib.sha256(text.encode()).hexdigest()
        if args.check:
            expected = EXPECTED_SHA256.get(relative)
            if expected and before == expected:
                print(f'{relative}: {before[:16]} matches the patched fingerprint')
            else:
                failures.append(f'{relative}: sha256 {before[:16]}, expected '
                                f'{(expected or "<unset>")[:16]} -- the patches are not in the '
                                f'state this script produces')
            continue
        applied, skipped, unreachable = [], [], []
        for label, old, new in patches:
            if new in text:
                skipped.append(label)
                continue
            count = text.count(old)
            if count != 1:
                # Several patches are superseded by later ones: once a later patch rewrites the
                # lines an earlier one introduced, the earlier patch's own `new` text is no
                # longer in the file and its anchor is gone. That is NOT an error -- but it is
                # also not proof that it was applied, so the patch is recorded as unreachable
                # and the FINGERPRINT decides at the end whether the file is in the right state.
                unreachable.append(f'{label} (anchor appears {count}x)')
                continue
            text = text.replace(old, new)
            applied.append(label)
        try:
            ast.parse(text)
        except SyntaxError as exc:
            failures.append(f'{relative}: patched file does not parse: {exc}')
            continue
        after = hashlib.sha256(text.encode()).hexdigest()
        expected = EXPECTED_SHA256.get(relative)
        if applied:
            path.write_text(text, encoding='utf-8')
            print(f'{relative}: applied {len(applied)}, skipped {len(skipped)}, '
                  f'not reachable {len(unreachable)}')
            if unreachable:
                for item in unreachable:
                    print(f'    (superseded) {item}')
            print(f'  sha256 {before} -> {after}')
        if after == expected:
            if not applied:
                print(f'{relative}: already in the recorded state ({after[:16]})')
            continue
        if not applied:
            failures.append(f'{relative}: sha256 {after[:16]} does not match the recorded '
                            f'{(expected or "<unset>")[:16]} and there was nothing to apply; '
                            f'the file was edited by hand or a patch script changed')
            continue
        # The file changed because this script changed it. That is expected whenever the patch
        # set itself grows, so the recorded fingerprint has to be updated deliberately.
        print(f'  NOTE: record {after} as EXPECTED_SHA256 for {relative}')

    if failures:
        for failure in failures:
            print(f'[FAIL] {failure}', flush=True)
        return 1
    print('[OK] both files are in the state this script produces and parse', flush=True)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
