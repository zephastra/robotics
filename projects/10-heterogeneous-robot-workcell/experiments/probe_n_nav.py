"""P1-N-07: start the world, the bridge and Nav2, send one goal, record what actually happened.

Task board P1-N-07: "single robot Nav2 actually arriving (same world, same bridge): a map or a
static map, AMCL or a known initial pose, a controller that actually drives the robot to the
goal, and the arrival error recorded."

This file is the orchestrator, and it deliberately owns only orchestration and evidence
collection -- every judgement lives in `evaluate_n_nav.py`, which is plain Python over the
written reports and therefore testable.

HOW TO RUN IT -- the interpreter is not a detail
-----------------------------------------------
    set +u                                   # `source /opt/ros/*/setup.bash` kills a `set -u` shell
    source scripts/env.sh
    /usr/bin/python3 experiments/probe_n_nav.py --run-id <id> --goal X Y YAW

The orchestrator must run under the SYSTEM interpreter, never the project `.venv`. It is itself a
ROS client -- `rclpy.init()`, an ActionClient for the goal, the lifecycle `get_state` services --
and the `.venv` is deliberately ROS-free, because that separation is what lets the core be tested
without ROS at all. Run it under the venv and it dies with

    ModuleNotFoundError: No module named 'rclpy._rclpy_pybind11'
    the C extension '.../python3.14/site-packages/_rclpy_pybind11.cpython-312-...so' isn't present

which reads exactly like a broken ROS installation. It is not one: `env.sh` puts
`/opt/ros/lyrical/lib/python3.14/site-packages` on `sys.path`, those extensions are built for
Python 3.14, and the venv is 3.12. Measured 2026-09-25: the same command under `/usr/bin/python3`
succeeds. The SIM half is launched by this file itself with `.venv/bin/python`, which is correct
because MuJoCo lives there and rclpy does not.

Five decisions worth stating, each of which could be made the other way:

  * The orchestrator does NOT set `use_sim_time`. It measures the simulation's clock by
    subscribing to /clock and keeps its own deadlines in wall time. A supervisor that uses the
    clock it is supervising cannot notice that clock stopping.
  * "Nav2 is up" is decided by the lifecycle states of the nodes, read over the get_state
    service, never by "it did not crash". 009's bringup failures were silent -- the log line
    said `async_send_request failed` and the cause was a slow machine -- so the acceptance
    criterion here is `reached ACTIVE`, and the node list is discovered from the graph and
    cross-checked against the lifecycle manager's own `node_names` parameter.
  * The run ends by SIGUSR1 to the simulator, not by killing it. The truth trajectory is written
    in the simulator's report on the way out, so a kill would throw away the evidence the whole
    round exists to collect. The signal is opt-in AT THE SIMULATOR
    (`probe_n_sim.py --allow-stop-signal`), and this orchestrator always passes that flag --
    so from the caller's side it is not opt-in, which is what an earlier wording implied.
  * Every process this file stops is first verified by reading its own /proc/<pid>/cmdline.
    There is no pattern kill: a `pkill -f` on this host would match the running 009 project.
  * Paths are resolved from the project root, never from the caller's cwd, and the ROS domain
    is taken from config -- the same config the two probe halves use.
"""
import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[1]

STATE_LABELS = {0: 'unknown', 1: 'unconfigured', 2: 'inactive', 3: 'active', 4: 'finalized'}
# Everything P1-N-07 needs to be up. Discovered states are recorded for all lifecycle nodes;
# these are the ones whose absence makes the verdict meaningless.
REQUIRED_ACTIVE = ('map_server', 'amcl', 'planner_server', 'controller_server', 'bt_navigator')


def read_cmdline(pid):
    try:
        raw = Path(f'/proc/{pid}/cmdline').read_bytes()
    except OSError:
        return None
    return raw.replace(b'\x00', b' ').decode('utf-8', 'replace').strip() or None


def leftover_processes(markers):
    """Processes still alive whose own cmdline names one of this round's components.

    Markers are the component paths, not the run id: this orchestrator's own command line
    contains the run id, and so does the shell that launched it, so matching on the run id would
    report the observer as a leak.
    """
    found, me = [], os.getpid()
    for entry in Path('/proc').iterdir():
        if not entry.name.isdigit():
            continue
        pid = int(entry.name)
        if pid == me:
            continue
        cmdline = read_cmdline(pid)
        if not cmdline:
            continue
        if any(marker in cmdline for marker in markers):
            found.append({'pid': pid, 'cmdline': cmdline[:400]})
    return sorted(found, key=lambda item: item['pid'])


class Processes:
    """Spawn, verify, and stop only what this run started."""

    def __init__(self, log_dir):
        self.log_dir = log_dir
        self.handles = {}
        self.logs = {}
        self.record = {}

    def spawn(self, name, cmd):
        log_path = self.log_dir / f'{name}.log'
        handle = open(log_path, 'wb')
        process = subprocess.Popen(cmd, stdout=handle, stderr=subprocess.STDOUT, cwd=str(ROOT),
                                   start_new_session=True)
        self.handles[name] = process
        self.logs[name] = handle
        self.record[name] = {'pid': process.pid, 'cmd': cmd, 'log': log_path.name,
                             'started_wall': round(time.monotonic(), 3)}
        return process

    def stop(self, name, must_contain):
        process = self.handles.get(name)
        entry = self.record.setdefault(name, {})
        if process is None:
            entry['stop'] = 'never started'
            return entry
        if process.poll() is not None:
            entry['stop'] = 'already exited'
            entry['exit_code'] = process.returncode
            return entry
        cmdline = read_cmdline(process.pid)
        if cmdline is None:
            entry['stop'] = 'gone before signal'
            entry['exit_code'] = process.wait()
            return entry
        if must_contain not in cmdline:
            # Refusing is the point: signalling something else on this host would stop another
            # project's work rather than ours.
            entry['stop'] = 'REFUSED: cmdline does not match this run'
            entry['cmdline'] = cmdline[:400]
            return entry
        signals = ((signal.SIGINT, 12.0), (signal.SIGTERM, 6.0), (signal.SIGKILL, 5.0))
        used = []
        for sig, timeout in signals:
            try:
                os.killpg(os.getpgid(process.pid), sig)
            except (ProcessLookupError, PermissionError):
                break
            used.append({'signal': sig.name, 'timeout_s': timeout})
            deadline = time.monotonic() + timeout
            while time.monotonic() < deadline and process.poll() is None:
                time.sleep(0.05)
            if process.poll() is not None:
                break
        entry['stop'] = 'signalled'
        entry['signals'] = used
        entry['exit_code'] = process.wait()
        return entry

    def close(self):
        for handle in self.logs.values():
            try:
                handle.close()
            except OSError:
                pass


def spin_until(node, predicate, timeout, on_tick=None):
    import rclpy
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
        if on_tick is not None:
            on_tick()
        if predicate():
            return True
    return False


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--run-id', required=True)
    parser.add_argument('--duration', type=float, default=150.0,
                        help='upper bound on sim seconds; the run normally ends earlier, when '
                             'the goal is reached and the settle window has passed')
    parser.add_argument('--goal', nargs=3, type=float, default=[2.6, 2.4, 0.0],
                        metavar=('X', 'Y', 'YAW'))
    parser.add_argument('--map', default='assets/maps/arena_n_probe.yaml')
    parser.add_argument('--params', default='config/nav2_n_probe.yaml')
    parser.add_argument('--sim-config', default=None,
                        help='config for probe_n_sim.py (default config/n_probe.yaml). The P3 nav arena passes its own, which differs from the P1 one only in `world`')
    parser.add_argument('--settle-sim-s', type=float, default=4.0,
                        help='sim seconds to keep watching after the goal result, so the plant '
                             'is seen to come to rest and the gate is seen to lapse')
    parser.add_argument('--bringup-timeout', type=float, default=90.0)
    parser.add_argument('--goal-timeout', type=float, default=90.0)
    parser.add_argument('--smoke', action='store_true',
                        help='bring everything up and stop without sending a goal')
    parser.add_argument('--bridge-command-source', default='cmd_vel',
                        choices=['cmd_vel', 'profile'],
                        help="where the bridge gets its velocity command. 'cmd_vel' is the "
                             "P1-N-07 wiring (Nav2 drives). 'profile' uses the bridge's built-in "
                             "P1-N-06 profile instead, which with --smoke gives a "
                             "LOCALISATION-ONLY experiment: AMCL runs against the same world "
                             "and the same bridge while the robot executes a known motion, with "
                             "no planner and no controller in the loop. That is what P1-N-08 "
                             "needs to attribute the pose error")
    parser.add_argument('--profile-start-sim-s', type=float, default=0.0,
                        help='with --bridge-command-source profile, hold the built-in profile '
                             'until this sim second, so the localisation-only experiment starts '
                             'the vehicle only after AMCL is up')
    parser.add_argument('--observe-sim-s', type=float, default=0.0,
                        help='with --smoke, keep observing for this many sim seconds after '
                             'localisation is up instead of stopping immediately, so a '
                             'localisation-only experiment has a motion to watch')
    parser.add_argument('--keep-processes', action='store_true',
                        help='do not stop anything on the way out (for interactive debugging)')
    args = parser.parse_args()
    if Path(args.run_id).name != args.run_id:
        raise ValueError('run id must be a bare name')

    import rclpy
    from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
    from lifecycle_msgs.srv import GetState
    from nav2_msgs.action import NavigateToPose
    from nav_msgs.msg import OccupancyGrid
    from rcl_interfaces.srv import GetParameters
    from rclpy.action import ActionClient
    from rclpy.node import Node
    from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
    from rosgraph_msgs.msg import Clock
    from sensor_msgs.msg import LaserScan

    import yaml
    cfg = yaml.safe_load((ROOT / 'config' / 'n_probe.yaml').read_text(encoding='utf-8'))
    if int(cfg['ros']['domain_id']) == 42:
        raise SystemExit('config refuses ROS domain 42: that is 009 (009/scripts/env.sh:28)')
    if os.environ.get('ROS_DOMAIN_ID') and int(os.environ['ROS_DOMAIN_ID']) != int(
            cfg['ros']['domain_id']):
        raise SystemExit(f"ROS_DOMAIN_ID={os.environ['ROS_DOMAIN_ID']} does not match "
                         f"config's {cfg['ros']['domain_id']}; source scripts/env.sh")

    map_path = ROOT / args.map
    params_path = ROOT / args.params
    for path in (map_path, params_path):
        if not path.is_file():
            raise SystemExit(f'missing {path}')

    run = ROOT / 'reports' / args.run_id
    run.mkdir(parents=True, exist_ok=True)
    if (run / 'nav_report.json').is_file():
        raise SystemExit(f'{args.run_id} already has a nav report; choose a new run id')

    started_wall = time.monotonic()
    venv_python = ROOT / '.venv' / 'bin' / 'python'
    sim_python = str(venv_python) if venv_python.is_file() else sys.executable

    plan = {
        'run_id': args.run_id,
        'declared_before_running': True,
        'goal': {'frame_id': 'map', 'x': args.goal[0], 'y': args.goal[1], 'yaw': args.goal[2]},
        # The tolerance is NOT restated here as numbers. It lives in the params file named by
        # `params.path` above, whose sha256 is recorded beside it, and the judge derives it from
        # there. This block used to carry typed constants -- xy 0.15 / yaw 0.20 -- and it omitted
        # `stateful`, which is the key that changes what 0.15 MEANS: a latched checker asks "was
        # this inside the window at some point", not "is it inside now". A number written twice can
        # only ever disagree with itself; the params file is the source of this one.
        'goal_tolerance_source': 'params.path -> controller_server.goal_checker_plugins[0], '
                                 'derived by evaluate_n_nav.py',
        'map': {'yaml': args.map,
                'sha256': hashlib.sha256(map_path.read_bytes()).hexdigest(),
                'pgm': 'assets/maps/arena_n_probe.pgm',
                'source': 'world_geometry_prior',
                'slam_exercised': False},
        'params': {'path': args.params,
                   'sha256': hashlib.sha256(params_path.read_bytes()).hexdigest()},
        'duration_upper_bound_sim_s': args.duration,
        'controller_plugin': 'nav2_regulated_pure_pursuit_controller'
                             '::RegulatedPurePursuitController',
        'settle_sim_s': args.settle_sim_s,
        'ros_domain_id': int(cfg['ros']['domain_id']),
        'topics': dict(cfg['ros']['topics']),
        'config_sha256': hashlib.sha256(
            (ROOT / 'config' / 'n_probe.yaml').read_bytes()).hexdigest(),
        # The sim side may be reading a DIFFERENT config (P3's arena). Recording only the
        # P1 one would make the report look like it describes a run it does not describe.
        'sim_config': args.sim_config,
        'sim_config_sha256': (hashlib.sha256(
            (ROOT / args.sim_config).read_bytes()).hexdigest()
            if args.sim_config else None),
        'smoke': bool(args.smoke),
        'component_sha256': {name: hashlib.sha256((ROOT / 'experiments' / name).read_bytes())
                             .hexdigest()
                             for name in ('probe_n_nav.py', 'probe_n_sim.py', 'probe_n_ros.py',
                                          'check_n_nav.py', 'make_map_n.py',
                                          'make_nav2_params.py')},
    }
    (run / 'nav_plan.json').write_text(json.dumps(plan, indent=2) + '\n')

    report = dict(scope='PROBE_N_NAV', probe='N', side='orchestrator', run_id=args.run_id,
                  command=sys.argv, status='ERROR', plan=plan,
                  source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
                  stages=[], lifecycle={}, action=None, teardown={}, leftovers=[])

    def stage(name, detail=None):
        sim_time = observed['clock']
        report['stages'].append({
            'stage': name,
            'wall_s': round(time.monotonic() - started_wall, 3),
            'sim_s': None if sim_time is None else round(sim_time, 3),
            'detail': detail,
        })
        print(f'[{round(time.monotonic() - started_wall, 1):7.1f}s] {name}'
              + (f' -- {detail}' if detail else ''), flush=True)

    processes = Processes(run)
    observed = {'clock': None, 'scans': 0, 'maps': 0, 'amcl': 0, 'map_info': None}
    lifecycle_clients = {}
    lifecycle_states = {}
    manager_names = None
    feedback = {'count': 0, 'last': None}
    action_result = {}

    rclpy.init()
    node = Node('workcell010_n_nav_orchestrator')

    def on_clock(message):
        observed['clock'] = float(message.clock.sec) + float(message.clock.nanosec) * 1e-9

    def on_scan(_message):
        observed['scans'] += 1

    def on_map(message):
        observed['maps'] += 1
        if observed['map_info'] is None:
            observed['map_info'] = {
                'width': message.info.width, 'height': message.info.height,
                'resolution': message.info.resolution,
                'origin': [message.info.origin.position.x, message.info.origin.position.y],
                'frame_id': message.header.frame_id,
            }

    def on_amcl(_message):
        observed['amcl'] += 1

    topics = cfg['ros']['topics']
    node.create_subscription(Clock, topics['clock'], on_clock,
                             QoSProfile(depth=50,
                                        reliability=QoSReliabilityPolicy.BEST_EFFORT))
    node.create_subscription(LaserScan, topics['scan'], on_scan, 50)
    node.create_subscription(OccupancyGrid, topics['map'], on_map,
                             QoSProfile(depth=5, reliability=QoSReliabilityPolicy.RELIABLE,
                                        durability=QoSDurabilityPolicy.TRANSIENT_LOCAL))
    node.create_subscription(PoseWithCovarianceStamped, topics['amcl_pose'], on_amcl, 50)

    def lifecycle_state(name):
        client = lifecycle_clients.get(name)
        if client is None:
            client = node.create_client(GetState, f'/{name}/get_state')
            lifecycle_clients[name] = client
        if not client.service_is_ready():
            return None
        future = client.call_async(GetState.Request())
        deadline = time.monotonic() + 1.0
        while not future.done() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.02)
        if not future.done():
            return None
        return int(future.result().current_state.id)

    def poll_lifecycle(names):
        for name in sorted(names):
            if lifecycle_states.get(name) == 3:
                continue
            state = lifecycle_state(name)
            if state is not None:
                lifecycle_states[name] = state
        return lifecycle_states

    try:
        stage('starting checker and bridge')
        processes.spawn('checker', ['/usr/bin/python3', 'experiments/check_n_nav.py',
                                    '--run-id', args.run_id,
                                    '--seconds', str(args.duration + 45.0)])
        processes.spawn('bridge', ['/usr/bin/python3', 'experiments/probe_n_ros.py',
                                   '--run-id', args.run_id, '--duration', str(args.duration),
                                   '--command-source', args.bridge_command_source,
                                   '--profile-start-sim-s', str(args.profile_start_sim_s),
                                   '--wait-for-state', '60'])
        # The bridge binds the state port only after it has imported rclpy and its message
        # types, so starting the simulator immediately loses the first datagrams and shows up
        # as a sequence gap with no other symptom (measured: seq_gaps 1 on the smoke run, while
        # P1-N-06's manually sequenced runs had none). Waiting for the bridge's node to appear
        # in the graph is a real readiness test; a fixed sleep would only be a guess.
        bridge_ready = spin_until(node, lambda: 'workcell010_n_bridge' in node.get_node_names(),
                                  30.0)
        report['bridge_ready_before_sim'] = bool(bridge_ready)
        if not bridge_ready:
            # Do not go on to start the simulator: without a bridge there is no clock and no
            # scan, and the run would spend its whole budget producing truth that nothing
            # consumed. Worse, it would look like a scenario failure. The bridge's own exit code
            # is the useful evidence -- an argument it did not understand kills it immediately.
            dead = processes.handles['bridge'].poll()
            report['status'] = 'BRIDGE_NEVER_APPEARED'
            report['bridge_exit_code'] = dead
            raise RuntimeError(f'the bridge node never appeared in the graph within 30 s '
                               f'(exit code {dead}; see bridge.log in the run directory)')
        stage('starting the simulator', {'bridge_ready': bool(bridge_ready)})
        sim_cmd = [sim_python, 'experiments/probe_n_sim.py',
                   '--run-id', args.run_id, '--duration', str(args.duration),
                   '--allow-stop-signal']
        if args.sim_config:
            sim_cmd += ['--config', args.sim_config]
        processes.spawn('sim', sim_cmd)

        stage('waiting for the simulator clock')
        got_clock = spin_until(node, lambda: observed['clock'] is not None, 30.0)
        if not got_clock:
            report['status'] = 'NO_CLOCK'
            raise RuntimeError('no /clock from the simulator within 30 s; the bridge or the '
                               'simulator did not start (see their logs in the run directory)')
        stage('clock is running', {'sim_s': round(observed['clock'], 3),
                                   'scans': observed['scans']})

        stage('starting Nav2')
        processes.spawn('nav2', ['ros2', 'launch', 'nav2_bringup', 'bringup_launch.py',
                                 f'map:={map_path}', f'params_file:={params_path}',
                                 'use_sim_time:=true', 'autostart:=true',
                                 'use_composition:=false', 'use_localization:=true',
                                 'slam:=false', 'use_keepout_zones:=false',
                                 'use_speed_zones:=false'])

        stage('waiting for lifecycle nodes to reach ACTIVE')
        deadline = time.monotonic() + args.bringup_timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            poll_lifecycle(node.get_node_names())
            if all(lifecycle_states.get(name) == 3 for name in REQUIRED_ACTIVE):
                break
        report['lifecycle']['states'] = {name: STATE_LABELS.get(value, str(value))
                                         for name, value in sorted(lifecycle_states.items())}
        report['lifecycle']['required'] = list(REQUIRED_ACTIVE)
        missing = [name for name in REQUIRED_ACTIVE if lifecycle_states.get(name) != 3]
        report['lifecycle']['not_active'] = missing
        if missing:
            report['status'] = 'BRINGUP_INCOMPLETE'
            stage('bringup incomplete', {'not_active': missing})
            raise RuntimeError(f'lifecycle nodes not active: {missing}; states '
                               f'{report["lifecycle"]["states"]}')
        stage('all required lifecycle nodes are active', {'states_seen': len(lifecycle_states)})

        # The managers' own declared lists, so "what should be up" comes from the launch rather
        # than from our assumptions about the launch. There are TWO managers -- localization
        # (map_server, amcl) and navigation (the rest) -- and comparing one of them with the
        # whole discovered graph is wrong, which the first judged run showed by flagging amcl
        # and map_server as "active but undeclared".
        managers = {}
        for manager_name in ('lifecycle_manager_localization', 'lifecycle_manager_navigation'):
            client = node.create_client(GetParameters, f'/{manager_name}/get_parameters')
            if not spin_until(node, client.service_is_ready, 5.0):
                managers[manager_name] = None
                continue
            request = GetParameters.Request()
            request.names = ['node_names']
            future = client.call_async(request)
            spin_until(node, future.done, 5.0)
            if future.done():
                values = future.result().values
                managers[manager_name] = (list(values[0].string_array_value)
                                          if values and values[0].string_array_value else None)
            else:
                managers[manager_name] = None
        declared = sorted({name for names in managers.values() if names for name in names})
        report['lifecycle']['managers'] = managers
        report['lifecycle']['manager_node_names'] = declared
        stage('read the lifecycle managers\' declared node lists',
              {name: (len(names) if names else 0) for name, names in managers.items()})

        stage('waiting for /map and /amcl_pose')
        spin_until(node, lambda: observed['maps'] > 0 and observed['amcl'] > 0, 30.0)
        report['topics'] = {'clock_s': observed['clock'], 'scans': observed['scans'],
                            'maps': observed['maps'], 'amcl_poses': observed['amcl'],
                            'map_info': observed['map_info']}
        stage('localisation is publishing', {'maps': observed['maps'],
                                            'amcl_poses': observed['amcl']})

        if args.smoke:
            if args.observe_sim_s > 0:
                target = (observed['clock'] or 0.0) + args.observe_sim_s
                stage('observing with no navigation in the loop',
                      {'until_sim_s': round(target, 2)})
                spin_until(node, lambda: (observed['clock'] or 0.0) >= target,
                           args.observe_sim_s * 4.0 + 30.0)
                stage('observation complete', {'sim_s': round(observed['clock'] or 0.0, 2)})
            report['status'] = 'SMOKE_OK'
        else:
            stage('sending the goal')
            action_client = ActionClient(node, NavigateToPose, 'navigate_to_pose')
            if not spin_until(node, action_client.server_is_ready, 30.0):
                report['status'] = 'NO_ACTION_SERVER'
                raise RuntimeError('navigate_to_pose action server never became ready')
            goal = NavigateToPose.Goal()
            goal.pose = PoseStamped()
            goal.pose.header.frame_id = 'map'
            goal.pose.header.stamp = node.get_clock().now().to_msg()
            goal.pose.pose.position.x = args.goal[0]
            goal.pose.pose.position.y = args.goal[1]
            goal.pose.pose.orientation.z = math.sin(args.goal[2] / 2.0)
            goal.pose.pose.orientation.w = math.cos(args.goal[2] / 2.0)

            def on_feedback(message):
                feedback['count'] += 1
                feedback['last'] = {
                    'distance_remaining': getattr(message.feedback, 'distance_remaining', None),
                    'number_of_recoveries': getattr(message.feedback, 'number_of_recoveries',
                                                    None),
                    'sim_s': observed['clock'],
                }

            send_future = action_client.send_goal_async(goal, feedback_callback=on_feedback)
            spin_until(node, send_future.done, 20.0)
            if not send_future.done():
                report['status'] = 'GOAL_SEND_TIMEOUT'
                raise RuntimeError('the goal was not accepted or rejected within 20 s')
            goal_handle = send_future.result()
            action_result['accepted'] = bool(goal_handle.accepted)
            action_result['sent_sim_s'] = observed['clock']
            if not goal_handle.accepted:
                report['status'] = 'GOAL_REJECTED'
                raise RuntimeError('the action server rejected the goal')

            stage('goal accepted', {'sim_s': round(observed['clock'], 3)})
            result_future = goal_handle.get_result_async()
            timed_out = not spin_until(node, result_future.done, args.goal_timeout)
            if timed_out:
                action_result['timed_out'] = True
                cancel = goal_handle.cancel_goal_async()
                spin_until(node, cancel.done, 10.0)
                report['status'] = 'GOAL_TIMEOUT'
            else:
                result = result_future.result()
                action_result['status'] = int(result.status)
                action_result['error_code'] = int(getattr(result.result, 'error_code', -1))
                report['status'] = ('GOAL_SUCCEEDED' if int(result.status) == 4
                                    else 'GOAL_NOT_SUCCEEDED')
            action_result['result_sim_s'] = observed['clock']
            action_result['feedback_count'] = feedback['count']
            action_result['last_feedback'] = feedback['last']
            stage('goal finished', {'action_status': action_result.get('status'),
                                    'sim_s': round(observed['clock'], 3)})

            stage('letting the plant settle')
            settle_until = (observed['clock'] or 0.0) + args.settle_sim_s
            spin_until(node, lambda: (observed['clock'] or 0.0) >= settle_until,
                       args.settle_sim_s * 4.0 + 15.0)
            action_result['settled_sim_s'] = observed['clock']
            stage('settled', {'sim_s': round(observed['clock'], 3)})

        report['action'] = action_result
    except Exception as exc:                                        # noqa: BLE001
        report.setdefault('error', f'{type(exc).__name__}: {exc}')
        if report['status'] == 'ERROR':
            report['status'] = 'ERROR'
    finally:
        sim = processes.handles.get('sim')
        if not args.keep_processes and sim is not None and sim.poll() is None:
            cmdline = read_cmdline(sim.pid) or ''
            if 'experiments/probe_n_sim.py' in cmdline:
                os.kill(sim.pid, signal.SIGUSR1)
                stage('asked the simulator to stop gracefully (SIGUSR1)')
                deadline = time.monotonic() + 20.0
                while time.monotonic() < deadline and sim.poll() is None:
                    time.sleep(0.1)
            else:
                stage('refusing to signal a process that is not this run\'s simulator',
                      {'cmdline': cmdline[:200]})

        if not args.keep_processes:
            stage('stopping Nav2')
            report['teardown']['nav2'] = processes.stop('nav2', 'nav2_bringup')
            stage('stopping the bridge')
            report['teardown']['bridge'] = processes.stop('bridge', 'experiments/probe_n_ros.py')
            report['teardown']['sim'] = processes.stop('sim', 'experiments/probe_n_sim.py')
            report['teardown']['checker'] = processes.stop('checker',
                                                           'experiments/check_n_nav.py')

        report['leftovers'] = leftover_processes((
            'experiments/probe_n_sim.py', 'experiments/probe_n_ros.py',
            'experiments/check_n_nav.py', 'nav2_bringup', 'nav2_amcl', 'nav2_controller',
            'nav2_planner', 'nav2_map_server', 'nav2_bt_navigator', 'nav2_behaviors',
            'nav2_lifecycle_manager', 'nav2_velocity_smoother', 'nav2_costmap_2d',
            'nav2_collision_monitor', 'nav2_smoother', 'nav2_waypoint_follower', 'nav2_route',
            'opennav_docking', 'opennav_following',
        ))
        report['processes'] = processes.record
        report['observed'] = observed
        report['wall_seconds'] = round(time.monotonic() - started_wall, 3)
        node.destroy_node()
        rclpy.shutdown()
        processes.close()
        (run / 'nav_report.json').write_text(json.dumps(report, indent=2) + '\n')
        print(json.dumps({k: v for k, v in report.items()
                          if k not in ('stages', 'processes')},
                         indent=2, ensure_ascii=False), flush=True)

    ok = report['status'] in ('GOAL_SUCCEEDED', 'SMOKE_OK')
    return 0 if ok else 1


if __name__ == '__main__':
    try:
        raise SystemExit(main())
    except SystemExit:
        raise
    except Exception as exc:                                        # noqa: BLE001
        # An orchestrator that crashes must not look like a pass, and it must say where to look.
        print(json.dumps({'status': 'FAIL', 'orchestrator_error': f'{type(exc).__name__}: {exc}'},
                         indent=2), flush=True)
        raise SystemExit(1)
