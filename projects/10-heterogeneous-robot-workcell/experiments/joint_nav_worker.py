"""ROS-only commissioning worker; no MuJoCo, simulator, or robot pose oracle.

Owns a minimal Nav2 launch and sends one declared map-frame goal. Wall deadlines
remain live when /clock stops. The parent owns the one shared physical world.
"""
import argparse
import json
import math
import os
from pathlib import Path
import signal
import time

ROOT = Path(__file__).resolve().parents[1]
NODES = ('map_server', 'amcl', 'planner_server', 'controller_server',
         'smoother_server', 'behavior_server', 'bt_navigator',
         'velocity_smoother', 'collision_monitor')


def declared_goal(values, default):
    """Map-frame task declaration, never a simulator pose or inferred cargo goal."""
    goal = list(default if values is None else values)
    if len(goal) != 3 or not all(math.isfinite(v) for v in goal):
        raise ValueError('finite map x/y/yaw goal required')
    if any(abs(v) > 50 for v in goal[:2]) or abs(goal[2]) > math.pi:
        raise ValueError('goal outside declared bounded workcell domain')
    return goal


def speed_overrides(name, speed):
    """Single-variable speed candidate; no goal, safety or acceleration changes."""
    if speed is None:return {}
    if not math.isfinite(speed) or not .05<=speed<=.15:
        raise ValueError('candidate speed must be finite within [0.05,0.15] m/s')
    if name=='controller_server':return {'FollowPath.desired_linear_vel':speed}
    if name=='velocity_smoother':return {'max_velocity':[speed,0.,.15],'min_velocity':[-speed,0.,-.15]}
    return {}


def launch_navigation(loaded=False, centered=False, start=None, retained=False, linear_speed=None, candidate='v5', turn=False):
    from launch import LaunchDescription, LaunchService
    from launch_ros.actions import Node
    from ament_index_python.packages import get_package_share_directory
    if retained and not loaded:raise ValueError('retained motion requires a loaded profile')
    if turn and candidate != 'v7':raise ValueError('turn candidate is v7-only')
    if turn and retained:raise ValueError('turn and retained are mutually exclusive')
    if linear_speed is not None and not retained:raise ValueError('speed candidate requires retained profile')
    if candidate == 'v7':
        if centered:raise ValueError('v7 commissions the base loaded transport identity only; centered load is v5 lineage')
        params = str(ROOT / ('config/nav2_joint_world_v7_retained.yaml' if retained else
                             'config/nav2_joint_world_v7_turn.yaml' if turn else
                             'config/nav2_joint_world_v7.yaml'))
    else:
        if retained and not centered:raise ValueError('retained motion requires centered loaded profile')
        params = str(ROOT / ('config/nav2_joint_world_v5_retained.yaml' if retained else
                             'config/nav2_joint_world_v5_centered.yaml' if centered else
                             'config/nav2_joint_world_v5_loaded.yaml' if loaded else
                             'config/nav2_joint_world_v5.yaml'))
    packages = dict(map_server='nav2_map_server', amcl='nav2_amcl',
        planner_server='nav2_planner', controller_server='nav2_controller',
        smoother_server='nav2_smoother', behavior_server='nav2_behaviors',
        bt_navigator='nav2_bt_navigator', velocity_smoother='nav2_velocity_smoother',
        collision_monitor='nav2_collision_monitor')
    actions = []
    for name in NODES:
        overrides = {'use_sim_time': True}
        overrides.update(speed_overrides(name,linear_speed))
        if name == 'amcl' and start is not None:
            x,y,yaw=declared_goal(start,[0.,0.,0.])
            overrides.update({'initial_pose.x':x,'initial_pose.y':y,
                              'initial_pose.yaw':yaw})
        if name == 'map_server': overrides['yaml_filename'] = str(ROOT / ('assets/maps/joint_world_v7.yaml' if candidate == 'v7' else 'assets/maps/joint_world_v5.yaml'))
        if name == 'bt_navigator':
            # Stock bringup expands find-pkg-share with ParameterFile; this
            # minimal launch resolves the installed package explicitly instead.
            overrides['bt_search_directories'] = [str(Path(get_package_share_directory('nav2_bt_navigator'))/'behavior_trees')]
        remaps = [('cmd_vel', 'cmd_vel_nav')] if name in ('controller_server', 'behavior_server', 'velocity_smoother') else []
        actions.append(Node(package=packages[name], executable=name, name=name,
            output='screen', parameters=[params, overrides], remappings=remaps))
    actions.append(Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
        name='joint_world_lifecycle_manager', output='screen',
        parameters=[{'use_sim_time': True, 'autostart': True, 'node_names': list(NODES),
                     'bond_timeout': 4.0}]))
    service = LaunchService(); service.include_launch_description(LaunchDescription(actions))
    return service.run()


def client(args):
    import rclpy
    from rclpy.action import ActionClient
    from lifecycle_msgs.srv import GetState
    from nav2_msgs.action import NavigateToPose
    from rosgraph_msgs.msg import Clock
    from geometry_msgs.msg import TwistStamped
    from probe_n_nav import Processes
    out = ROOT / 'reports' / args.run_id
    profile = json.loads((ROOT / ('config/joint_world_v7.profile.json' if args.candidate == 'v7' else 'config/joint_world_v5.profile.json')).read_text())
    profile['goal'] = declared_goal(args.goal, profile['goal'])
    report = dict(scope=('INITIALIZED_LOAD_SHARED_WORLD_NAV2_ROS_EVIDENCE' if args.loaded else
                        'EMPTY_SHARED_WORLD_NAV2_ROS_EVIDENCE'), status='ERROR',
                  active={}, command_count=0, nonzero_command_count=0, goal=profile['goal'])
    report['declared_localization_prior']=declared_goal(args.start,profile['start']+[0.])
    report['retained_motion_candidate']=args.retained_motion
    report['linear_speed_candidate_mps']=getattr(args,'linear_speed',None)
    owned = Processes(out); node = None; started = time.monotonic()
    def interrupted(signum, frame): raise KeyboardInterrupt('owned worker interrupted')
    signal.signal(signal.SIGTERM, interrupted)
    try:
        if os.environ.get('ROS_DOMAIN_ID') != '43': raise RuntimeError('isolated ROS domain 43 required')
        rclpy.init(); node = rclpy.create_node('joint_world_commission_observer')
        last_clock = [None]
        def clock_cb(msg): last_clock[0] = msg.clock
        node.create_subscription(Clock, '/clock', clock_cb, 10)
        def command_cb(msg):
            report['command_count'] += 1
            if abs(msg.twist.linear.x) + abs(msg.twist.angular.z) > 1e-4:
                report['nonzero_command_count'] += 1
        node.create_subscription(TwistStamped, '/cmd_vel', command_cb, 10)
        # Refuse another active producer; do not stop or disturb it.
        for _ in range(10): rclpy.spin_once(node, timeout_sec=.1)
        if node.get_publishers_info_by_topic('/cmd_vel'):
            raise RuntimeError('existing /cmd_vel publisher: domain not exclusive')
        owned.spawn('nav2', ['/usr/bin/python3', 'experiments/joint_nav_worker.py', '--launch']+
                    (['--loaded'] if args.loaded else [])+
                    (['--centered-load'] if args.centered_load else [])+
                    (['--retained-motion'] if args.retained_motion else [])+
                    (['--turn-conservative'] if args.turn_conservative else [])+
                    (['--linear-speed',str(args.linear_speed)] if getattr(args,'linear_speed',None) is not None else [])+
                    (['--candidate',args.candidate] if args.candidate != 'v5' else [])+
                    (['--start']+[str(v) for v in report['declared_localization_prior']] if args.start is not None else []))
        clients = {name: node.create_client(GetState, '/'+name+'/get_state') for name in NODES}
        pending = {}; next_query = 0.
        def spin_until(predicate, budget):
            deadline = time.monotonic() + budget
            while not predicate():
                if time.monotonic() > deadline or time.monotonic()-started > args.wall_budget:
                    raise TimeoutError('ROS commissioning wall deadline')
                if owned.handles['nav2'].poll() is not None: raise RuntimeError('Nav2 launch exited')
                rclpy.spin_once(node, timeout_sec=.05)
        def ready():
            nonlocal next_query
            for name, future in list(pending.items()):
                if future.done():
                    result = future.result()
                    report['active'][name] = result.current_state.id if result else None
                    del pending[name]
            if time.monotonic() >= next_query:
                for name, service in clients.items():
                    if name not in pending and service.service_is_ready():
                        pending[name] = service.call_async(GetState.Request())
                next_query = time.monotonic()+1.
            return last_clock[0] is not None and all(report['active'].get(n)==3 for n in NODES)
        spin_until(ready, 90.)
        publishers = node.get_publishers_info_by_topic('/cmd_vel')
        report['cmd_vel_publishers'] = [p.node_name for p in publishers]
        if report['cmd_vel_publishers'] != ['collision_monitor']:
            raise RuntimeError('final command producer must be collision_monitor only')
        action = ActionClient(node, NavigateToPose, '/navigate_to_pose')
        spin_until(lambda: action.server_is_ready(), 10.)
        goal = NavigateToPose.Goal(); goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = last_clock[0]  # NOT the observer's wall clock
        x, y, yaw = profile['goal']; goal.pose.pose.position.x = x; goal.pose.pose.position.y = y
        goal.pose.pose.orientation.z = math.sin(yaw/2); goal.pose.pose.orientation.w = math.cos(yaw/2)
        future = action.send_goal_async(goal); spin_until(future.done, 15.)
        handle = future.result(); report['goal_accepted'] = bool(handle and handle.accepted)
        if not report['goal_accepted']: raise RuntimeError('goal rejected')
        result = handle.get_result_async(); spin_until(result.done, args.wall_budget-100.)
        report['action_status'] = result.result().status
        report['action_error_code'] = result.result().result.error_code
        report['status'] = 'SUCCEEDED' if report['action_status']==4 else 'GOAL_FAILED'
    except (Exception, KeyboardInterrupt) as exc:
        report['error'] = type(exc).__name__+': '+str(exc)
    finally:
        owned.stop('nav2', 'joint_nav_worker.py')
        for log in owned.logs.values(): log.close()
        report['processes'] = owned.record
        if node is not None: node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
        report['wall_s'] = time.monotonic()-started
        target = out / 'nav2_worker_report.json'; temporary = out / 'nav2_worker_report.tmp'
        temporary.write_text(json.dumps(report, indent=2)+'\n'); temporary.replace(target)
        print(json.dumps(report, indent=2), flush=True)
    return 0 if report['status']=='SUCCEEDED' else 1


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--launch', action='store_true'); parser.add_argument('--run-id')
    parser.add_argument('--wall-budget', type=float, default=300.)
    parser.add_argument('--loaded', action='store_true', help='independent conservative loaded footprint candidate')
    parser.add_argument('--centered-load',action='store_true',help='stricter +/-60mm load-centre region candidate')
    parser.add_argument('--retained-motion',action='store_true',help='independent low-acceleration CONTROL candidate, not qualified')
    parser.add_argument('--linear-speed',type=float,help='single-variable retained speed diagnostic [0.05,0.15]')
    parser.add_argument('--goal', nargs=3, type=float, help='declared station approach in map x/y/yaw')
    parser.add_argument('--start', nargs=3, type=float, help='declared parked station localization prior, never runtime truth')
    parser.add_argument('--candidate',choices=['v5','v7'],default='v5',help='tested candidate selection; v7 uses the independently minted v7 identity')
    parser.add_argument('--turn-conservative',action='store_true',help='v7-only angular-axis conservative candidate; linear speed unchanged')
    args = parser.parse_args()
    if args.centered_load and not args.loaded:parser.error('--centered-load requires --loaded')
    if args.retained_motion and not args.loaded:parser.error('--retained-motion requires --loaded')
    if args.candidate == 'v5' and args.retained_motion and not args.centered_load:parser.error('--retained-motion requires --centered-load')
    if args.turn_conservative and args.candidate != 'v7':parser.error('--turn-conservative is v7-only')
    if args.turn_conservative and args.retained_motion:parser.error('--turn-conservative and --retained-motion are mutually exclusive')
    if args.turn_conservative and not args.loaded:parser.error('--turn-conservative requires --loaded')
    if args.linear_speed is not None:
        if not args.retained_motion:parser.error('--linear-speed requires --retained-motion')
        speed_overrides('controller_server',args.linear_speed)
    if args.launch: return launch_navigation(args.loaded,args.centered_load,args.start,args.retained_motion,args.linear_speed,args.candidate,args.turn_conservative)
    if not args.run_id or Path(args.run_id).name != args.run_id or not 110 < args.wall_budget <= 600:
        parser.error('bare run ID and wall budget (110,600] required')
    return client(args)


if __name__=='__main__': raise SystemExit(main())
