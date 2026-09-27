"""P1-N-07 acceptance side: watch the ROS graph while Nav2 drives the robot, record raw facts.

Same separation as P1-N-06: this file counts and timestamps, and `evaluate_n_nav.py` decides.
The checker is a pure observer -- it publishes nothing, commands nothing and cannot start or
stop anything.

What it adds over `check_n_bridge.py`, and why each addition is a question that must be
answered by measurement rather than by reading the launch file:

  * `/cmd_vel` **publisher node names**. nav2 wires the chain
    controller_server -> velocity_smoother -> collision_monitor, and the last writer is the one
    that decides what the plant is told. AGENTS.md requires a single final command writer, so
    who that is has to be observed, not assumed.
  * `/map` with TRANSIENT_LOCAL. map_server latches its map; a VOLATILE subscriber receives
    zero messages and the symptom is identical to "the map was never published" (009 measured
    this on costmap snapshots).
  * the dynamic TF edge set, as (parent, child) pairs. P1-N-06 asserted "exactly one /tf
    publisher"; with AMCL added that is false and the invariant that still holds is one
    publisher per EDGE -- so the edges are recorded, not just the child frames.
  * `/amcl_pose` samples with the simulator stamp on the header. This is the only way the judge
    can measure localisation error against truth. 009's lesson is that a self-reported pose
    with `localization_valid=True` was 8 m out, so "the localiser is fine" is never inferred
    from the localiser's own opinion.
  * the first complete scan. The judge raycasts it through the static map image, which is the
    only check that a wrong map origin, resolution or flip would fail.
"""
import argparse
import json
import math
import time
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]

TOPICS = ('clock', 'scan', 'odom', 'tf', 'tf_static', 'map', 'amcl_pose', 'cmd_vel',
          'cmd_vel_nav', 'cmd_vel_smoothed')
MAX_AMCL_SAMPLES = 5000


def main():
    import rclpy
    from geometry_msgs.msg import PoseWithCovarianceStamped, TwistStamped
    from nav_msgs.msg import OccupancyGrid, Odometry
    from rclpy.node import Node
    from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                           QoSReliabilityPolicy)
    from rosgraph_msgs.msg import Clock
    from sensor_msgs.msg import LaserScan
    from tf2_msgs.msg import TFMessage

    parser = argparse.ArgumentParser()
    parser.add_argument('--run-id', required=True)
    parser.add_argument('--seconds', type=float, default=120.0)
    parser.add_argument('--out-name', default='checker_nav_raw.json')
    args = parser.parse_args()

    import yaml
    cfg = yaml.safe_load((ROOT / 'config' / 'n_probe.yaml').read_text(encoding='utf-8'))
    topics = cfg['ros']['topics']
    topic_of = {name: topics[name] for name in TOPICS}
    out = ROOT / 'reports' / args.run_id
    out.mkdir(parents=True, exist_ok=True)

    rclpy.init()
    node = Node('workcell010_n_nav_check')
    best_effort = QoSProfile(depth=50, reliability=QoSReliabilityPolicy.BEST_EFFORT)
    reliable = QoSProfile(depth=50, reliability=QoSReliabilityPolicy.RELIABLE)
    latched = QoSProfile(depth=5, reliability=QoSReliabilityPolicy.RELIABLE,
                         durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                         history=QoSHistoryPolicy.KEEP_LAST)

    raw = dict(scope='PROBE_N_NAV', probe='N', side='checker', run_id=args.run_id,
               command=sys.argv, status='ERROR', rows=[],
               counts={name: 0 for name in TOPICS})
    state = {
        'clock_stamp': None, 'scan_stamp': None, 'odom_stamp': None, 'map_stamp': None,
        'scan_first': None, 'scan_first_stamp': None,
        'map_info': None, 'odom_child': None, 'odom_first': None, 'odom_last': None,
        'tf_edges': [], 'tf_static_edges': [],
        'amcl_samples': [], 'amcl_first_wall': None, 'amcl_last': None,
        'odom_samples': [], 'odom_first_wall': None,
        'cmd_vel_first': None, 'cmd_vel_last': None, 'cmd_vel_nonzero': 0,
        'cmd_vel_max_v': 0.0, 'cmd_vel_max_w': 0.0,
        'publisher_nodes': {name: [] for name in TOPICS},
        'publisher_unresolved': {name: 0 for name in TOPICS},
        'publisher_qos': {name: [] for name in TOPICS},
        'subscriber_qos': {name: [] for name in TOPICS},
    }

    def qos_of(profile):
        """The QoS fields that decide whether two endpoints can talk to each other."""
        return {'reliability': str(profile.reliability), 'durability': str(profile.durability),
                'history': str(getattr(profile, 'history', None)),
                'depth': getattr(profile, 'depth', None)}

    def to_seconds(stamp):
        return float(stamp.sec) + float(stamp.nanosec) * 1e-9

    def on_clock(message):
        raw['counts']['clock'] += 1
        state['clock_stamp'] = to_seconds(message.clock)

    def on_scan(message):
        raw['counts']['scan'] += 1
        state['scan_stamp'] = to_seconds(message.header.stamp)
        if state['scan_first'] is None:
            state['scan_first'] = [float(value) for value in message.ranges]
            state['scan_first_stamp'] = state['scan_stamp']

    def on_odom(message):
        raw['counts']['odom'] += 1
        state['odom_stamp'] = to_seconds(message.header.stamp)
        state['odom_child'] = message.child_frame_id
        pose = (message.pose.pose.position.x, message.pose.pose.position.y)
        if state['odom_first'] is None:
            state['odom_first'] = pose
        state['odom_last'] = pose
        # Sampled over time, not just first/last: P1-N-08 has to tell "AMCL invents the lead"
        # apart from "AMCL faithfully propagates an odometry that leads the truth", and that
        # needs the odometry trajectory, not its endpoint.
        orientation = message.pose.pose.orientation
        yaw = math.atan2(2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
                         1.0 - 2.0 * (orientation.y ** 2 + orientation.z ** 2))
        if len(state['odom_samples']) < MAX_AMCL_SAMPLES:
            state['odom_samples'].append([state['odom_stamp'], pose[0], pose[1], yaw])

    def on_map(message):
        raw['counts']['map'] += 1
        state['map_stamp'] = to_seconds(message.header.stamp)
        if state['map_info'] is None:
            state['map_info'] = {
                'frame_id': message.header.frame_id,
                'width': message.info.width, 'height': message.info.height,
                'resolution': message.info.resolution,
                'origin': [message.info.origin.position.x, message.info.origin.position.y],
                'cells': len(message.data),
                'occupied': sum(1 for v in message.data if v >= 65),
                'free': sum(1 for v in message.data if v == 0),
                'unknown': sum(1 for v in message.data if v == -1),
            }

    def on_tf(message):
        raw['counts']['tf'] += 1
        for transform in message.transforms:
            edge = [transform.header.frame_id, transform.child_frame_id]
            if edge not in state['tf_edges']:
                state['tf_edges'].append(edge)

    def on_tf_static(message):
        raw['counts']['tf_static'] += 1
        for transform in message.transforms:
            edge = [transform.header.frame_id, transform.child_frame_id]
            if edge not in state['tf_static_edges']:
                state['tf_static_edges'].append(edge)

    def on_amcl(message):
        raw['counts']['amcl_pose'] += 1
        now = time.monotonic()
        if state['amcl_first_wall'] is None:
            state['amcl_first_wall'] = round(now, 4)
        position = message.pose.pose.position
        orientation = message.pose.pose.orientation
        yaw = math.atan2(2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
                         1.0 - 2.0 * (orientation.y ** 2 + orientation.z ** 2))
        sample = [to_seconds(message.header.stamp), position.x, position.y, yaw]
        state['amcl_last'] = sample
        if len(state['amcl_samples']) < MAX_AMCL_SAMPLES:
            state['amcl_samples'].append(sample)

    def on_cmd_vel(message):
        raw['counts']['cmd_vel'] += 1
        v, w = float(message.twist.linear.x), float(message.twist.angular.z)
        state['cmd_vel_max_v'] = max(state['cmd_vel_max_v'], abs(v))
        state['cmd_vel_max_w'] = max(state['cmd_vel_max_w'], abs(w))
        if v != 0.0 or w != 0.0:
            state['cmd_vel_nonzero'] += 1
        if state['cmd_vel_first'] is None:
            state['cmd_vel_first'] = [v, w]
        state['cmd_vel_last'] = [v, w]

    def publisher_snapshot(name):
        try:
            infos = node.get_publishers_info_by_topic(topic_of[name])
        except Exception as exc:                    # noqa: BLE001
            return {'named': [], 'unresolved': 0, 'error': str(exc)}
        named, unresolved, qos = [], 0, []
        for info in infos:
            node_name = info.node_name
            # DDS can report a publisher whose node name is not in the discovered graph. Those
            # entries are counted separately instead of being dropped: dropping them would let
            # "exactly one publisher" pass on a graph that actually has two.
            if node_name and node_name != '_NODE_NAME_UNKNOWN_':
                if node_name not in named:
                    named.append(node_name)
            else:
                unresolved += 1
            qos.append(dict(qos_of(info.qos_profile), type=info.topic_type))
        return {'named': sorted(named), 'unresolved': unresolved, 'qos': qos}

    def subscriber_qos(name):
        try:
            infos = node.get_subscriptions_info_by_topic(topic_of[name])
        except Exception as exc:                    # noqa: BLE001
            return [{'error': str(exc)}]
        return [dict(qos_of(info.qos_profile), node=info.node_name, type=info.topic_type)
                for info in infos]

    node.create_subscription(Clock, topic_of['clock'], on_clock, best_effort)
    node.create_subscription(LaserScan, topic_of['scan'], on_scan, reliable)
    node.create_subscription(Odometry, topic_of['odom'], on_odom, reliable)
    node.create_subscription(TFMessage, topic_of['tf'], on_tf, best_effort)
    # latched: a VOLATILE subscriber silently receives zero messages
    node.create_subscription(TFMessage, topic_of['tf_static'], on_tf_static, latched)
    # latched: map_server publishes its map with TRANSIENT_LOCAL durability
    node.create_subscription(OccupancyGrid, topic_of['map'], on_map, latched)
    node.create_subscription(PoseWithCovarianceStamped, topic_of['amcl_pose'], on_amcl, reliable)
    # The command topics carry geometry_msgs/msg/TwistStamped in this nav2 release (measured in
    # P1-N-07 run 04 on controller_server, velocity_smoother and collision_monitor). The assumed
    # type is written into the report and the judge compares it with the type each publisher
    # actually declares, so a release that goes back to plain Twist fails loudly instead of
    # producing another silent zero.
    raw['command_topic_type_assumed'] = 'geometry_msgs/msg/TwistStamped'
    node.create_subscription(TwistStamped, topic_of['cmd_vel'], on_cmd_vel, best_effort)
    # The two intermediate hops. They are counted only: the question they answer is which hop in
    # controller -> cmd_vel_nav -> cmd_vel_smoothed -> cmd_vel is silent, because "the robot did
    # not move" is the same symptom for all four.
    for stage in ('cmd_vel_nav', 'cmd_vel_smoothed'):
        def make_counter(key):
            def counter(_message):
                raw['counts'][key] += 1
            return counter

        node.create_subscription(TwistStamped, topic_of[stage], make_counter(stage), best_effort)

    started = time.monotonic()
    try:
        while time.monotonic() - started < args.seconds:
            rclpy.spin_once(node, timeout_sec=0.25)
            publishers = {}
            for name in TOPICS:
                snapshot = publisher_snapshot(name)
                publishers[name] = {'named': len(snapshot['named']),
                                    'unresolved': snapshot['unresolved']}
                if 'error' in snapshot:
                    publishers[name]['error'] = snapshot['error']
                known = state['publisher_nodes'][name]
                for entry in snapshot['named']:
                    if entry not in known:
                        known.append(entry)
                state['publisher_unresolved'][name] = max(
                    state['publisher_unresolved'][name], snapshot['unresolved'])
                if snapshot['qos'] and not state['publisher_qos'][name]:
                    state['publisher_qos'][name] = snapshot['qos']
                if not state['subscriber_qos'][name]:
                    state['subscriber_qos'][name] = subscriber_qos(name)
            raw['rows'].append({
                'wall': round(time.monotonic() - started, 4),
                'counts': dict(raw['counts']),
                'publishers': publishers,
                'clock_stamp': state['clock_stamp'],
                'scan_stamp': state['scan_stamp'],
                'odom_stamp': state['odom_stamp'],
                'map_stamp': state['map_stamp'],
                'amcl_pose_count': raw['counts']['amcl_pose'],
            })
        raw['status'] = 'COMPLETED'
    except Exception as exc:                        # noqa: BLE001
        raw['error'] = f'{type(exc).__name__}: {exc}'
    finally:
        raw['observed'] = state
        raw['wall_seconds'] = round(time.monotonic() - started, 3)
        node.destroy_node()
        rclpy.shutdown()
        (out / args.out_name).write_text(json.dumps(raw, indent=2) + '\n')
        summary = {k: v for k, v in raw.items() if k not in ('rows', 'observed')}
        print(json.dumps(summary, indent=2, ensure_ascii=False), flush=True)
        brief = {k: v for k, v in state.items()
                 if k not in ('amcl_samples', 'odom_samples', 'scan_first', 'rows',
                              'publisher_qos', 'subscriber_qos')}
        print('observed:', json.dumps(brief, ensure_ascii=False), flush=True)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
