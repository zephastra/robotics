#!/usr/bin/env python3
"""List topics with publisher/subscriber counts and QoS -- from inside the node graph.

Why this exists: `ros2 topic info` and `ros2 topic list` are not trustworthy on this
machine. In one run they reported "Unknown topic '/r01/cmd_vel'" for a topic that the
safety gate was demonstrably publishing on, and in another they reported "Unknown
topic '/r01/scan'" while `ros2 topic hz` measured it at 7 Hz. The CLI talks to a
long-lived daemon that can serve a partial or stale graph, and a partial graph looks
exactly like a broken system.

rclpy's own discovery calls run in this process, use the same DDS configuration and
the same daemon-free path as the nodes under test, so they cannot be stale in that way.
That is why every graph claim in the 009 report is made with this tool and not with
the CLI.

Exit codes: 0 always (it is a reporting tool); use --require to make it assert.
"""

from __future__ import annotations

import argparse
import sys
import time

import rclpy
from rclpy.node import Node


def _qos_summary(info) -> str:
    q = info.qos_profile
    return (f"{q.reliability.name.replace('Policy', '')}/"
            f"{q.durability.name.replace('Policy', '')}/"
            f"{q.history.name.replace('Policy', '')}({q.depth})")


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--filter", default="", help="only topics containing this substring")
    ap.add_argument("--settle", type=float, default=6.0,
                    help="seconds to let discovery converge before reporting")
    ap.add_argument("--require", action="append", default=[],
                    help="topic that MUST have >=1 publisher and >=1 subscriber; "
                         "repeatable. Non-zero exit if any is not satisfied.")
    ap.add_argument("--verbose", action="store_true", help="print every endpoint's QoS")
    args, ros_args = ap.parse_known_args(argv)

    rclpy.init(args=ros_args)
    node = Node("graph_inspector")
    # Discovery is asynchronous: asking immediately returns the partial graph and looks
    # like missing nodes. Waiting is not politeness, it is correctness.
    deadline = time.monotonic() + args.settle
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.2)

    topics = sorted(node.get_topic_names_and_types())
    print("=" * 84)
    print("  009 ROS graph, measured in-process (daemon-free)")
    print("=" * 84)
    print(f"  {'topic':46s} {'pub':>4s} {'sub':>4s}  type")
    print("  " + "-" * 80)

    failures: list[str] = []
    filtered = [(t, ty) for t, ty in topics if args.filter in t]
    for topic, types in filtered:
        pubs = node.get_publishers_info_by_topic(topic)
        subs = node.get_subscriptions_info_by_topic(topic)
        print(f"  {topic:46s} {len(pubs):4d} {len(subs):4d}  "
              f"{','.join(t.split('/')[-1] for t in types)}")
        if args.verbose and (pubs or subs):
            for p in pubs:
                print(f"        PUB  {p.node_name:32s} {_qos_summary(p)}")
            for s in subs:
                print(f"        SUB  {s.node_name:32s} {_qos_summary(s)}")
        if topic in args.require:
            if not pubs:
                failures.append(f"{topic}: no publisher")
            if not subs:
                failures.append(f"{topic}: no subscriber")

    print()
    print(f"  topics listed: {len(filtered)}")
    if failures:
        print("  REQUIREMENTS NOT MET:")
        for f in failures:
            print(f"    - {f}")
    else:
        print("  all required topics have both a publisher and a subscriber")

    node.destroy_node()
    rclpy.shutdown()
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
