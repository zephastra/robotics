"""P1-N probe, acceptance side: observe the ROS graph while the probe runs, record raw facts.

This is the independent observer. It publishes nothing and commands nothing; it only counts
and timestamps what appears, because the checks that matter here are about the *shape of the
graph*, not about behaviour:

  * how many publishers /clock has (the simulator's clock has to have exactly one owner, and
    009 already measured what several clock publishers do to a tf2 buffer),
  * how many publishers /tf and /tf_static have,
  * whether the stamps are simulation time (small numbers, advancing by the sim rate) rather
    than the wall clock (1.7e9 seconds),
  * whether the scan geometry the subscriber sees matches config,
  * whether the rates actually achieved match the rates configured.

It writes raw observations only. Judging them is `experiments/evaluate_n_ipc.py`, which is
plain Python and therefore testable without a ROS graph in the room.
"""
import argparse
import json
import math
from pathlib import Path
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src'))


def main():
    import rclpy
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from rclpy.qos import (QoSDurabilityPolicy, QoSProfile,
                           ReliabilityPolicy)
    from rosgraph_msgs.msg import Clock
    from sensor_msgs.msg import LaserScan
    from tf2_msgs.msg import TFMessage

    parser = argparse.ArgumentParser()
    parser.add_argument('--run-id', required=True)
    parser.add_argument('--seconds', type=float, default=30.0)
    args = parser.parse_args()

    import yaml
    cfg = yaml.safe_load((ROOT / 'config' / 'n_probe.yaml').read_text(encoding='utf-8'))
    topics = cfg['ros']['topics']
    out = ROOT / 'reports' / args.run_id
    out.mkdir(parents=True, exist_ok=True)

    rclpy.init()
    node = Node('workcell010_n_check')
    best_effort = QoSProfile(depth=50, reliability=ReliabilityPolicy.BEST_EFFORT)
    reliable = QoSProfile(depth=50, reliability=ReliabilityPolicy.RELIABLE)

    raw = dict(scope='PROBE_N_IPC', probe='N', side='checker', run_id=args.run_id,
               command=sys.argv, status='ERROR', rows=[],
               counts={'clock': 0, 'scan': 0, 'odom': 0, 'tf': 0, 'tf_static': 0})
    state = {'clock_stamp': None, 'scan_stamp': None, 'odom_stamp': None,
             'scan_geometry': None, 'scan_range_min': None, 'scan_range_max': None,
             'scan_finite': 0, 'scan_rays': 0, 'clock_child': [], 'tf_children': [],
             'static_children': [], 'odom_child': None, 'odom_first': None, 'odom_last': None}

    def to_seconds(stamp):
        return float(stamp.sec) + float(stamp.nanosec) * 1e-9

    def on_clock(message):
        raw['counts']['clock'] += 1
        state['clock_stamp'] = to_seconds(message.clock)

    def on_scan(message):
        raw['counts']['scan'] += 1
        state['scan_stamp'] = to_seconds(message.header.stamp)
        if state['scan_geometry'] is None:
            state['scan_geometry'] = {
                'frame_id': message.header.frame_id,
                'angle_min': message.angle_min, 'angle_max': message.angle_max,
                'angle_increment': message.angle_increment,
                'range_min': message.range_min, 'range_max': message.range_max,
                'rays': len(message.ranges),
            }
            finite = [r for r in message.ranges
                      if not math.isnan(r) and not math.isinf(r)]
            state['scan_finite'] = len(finite)
            state['scan_rays'] = len(message.ranges)
            state['scan_range_min'] = min(finite) if finite else None
            state['scan_range_max'] = max(finite) if finite else None

    def on_odom(message):
        raw['counts']['odom'] += 1
        state['odom_stamp'] = to_seconds(message.header.stamp)
        state['odom_child'] = message.child_frame_id
        pose = (message.pose.pose.position.x, message.pose.pose.position.y)
        if state['odom_first'] is None:
            state['odom_first'] = pose
        state['odom_last'] = pose

    def on_tf(message):
        raw['counts']['tf'] += 1
        for transform in message.transforms:
            if transform.child_frame_id not in state['tf_children']:
                state['tf_children'].append(transform.child_frame_id)

    def on_tf_static(message):
        raw['counts']['tf_static'] += 1
        for transform in message.transforms:
            if transform.child_frame_id not in state['static_children']:
                state['static_children'].append(transform.child_frame_id)

    node.create_subscription(Clock, topics['clock'], on_clock, best_effort)
    node.create_subscription(LaserScan, topics['scan'], on_scan, reliable)
    node.create_subscription(Odometry, topics['odom'], on_odom, reliable)
    node.create_subscription(TFMessage, topics['tf'], on_tf, best_effort)
    # latched: a VOLATILE subscriber receives zero messages and the symptom is
    # indistinguishable from "the topic does not exist" (009 measured this on costmap
    # snapshots). Static transforms are latched, so this subscription must be too.
    node.create_subscription(TFMessage, topics['tf_static'], on_tf_static,
                             QoSProfile(depth=50, reliability=ReliabilityPolicy.RELIABLE,
                                        durability=QoSDurabilityPolicy.TRANSIENT_LOCAL))

    started = time.monotonic()
    try:
        while time.monotonic() - started < args.seconds:
            rclpy.spin_once(node, timeout_sec=0.25)
            publishers = {}
            for name in ('clock', 'scan', 'odom', 'tf', 'tf_static'):
                try:
                    publishers[name] = len(node.get_publishers_info_by_topic(topics[name]))
                except Exception as exc:                    # noqa: BLE001
                    publishers[name] = f'error: {exc}'
            raw['rows'].append({
                'wall': round(time.monotonic() - started, 4),
                'counts': dict(raw['counts']),
                'publishers': publishers,
                'clock_stamp': state['clock_stamp'],
                'scan_stamp': state['scan_stamp'],
                'odom_stamp': state['odom_stamp'],
            })
        raw['status'] = 'COMPLETED'
    except Exception as exc:                                # noqa: BLE001
        raw['error'] = f'{type(exc).__name__}: {exc}'
    finally:
        raw['observed'] = {key: value for key, value in state.items()}
        raw['wall_seconds'] = round(time.monotonic() - started, 3)
        node.destroy_node()
        rclpy.shutdown()
        (out / 'checker_raw.json').write_text(json.dumps(raw, indent=2) + '\n')
        summary = {k: v for k, v in raw.items() if k not in ('rows', 'observed')}
        print(json.dumps(summary, indent=2, ensure_ascii=False), flush=True)
        print('observed:', json.dumps(state, ensure_ascii=False), flush=True)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
