"""P1-N-07 config patches: two additions to config/n_probe.yaml, both opt-in and additive.

1. `sim.wall_deadline_s` 90.0 -> 240.0.
   It is a safety stop, not an acceptance threshold: it exists so a wedged run does not spin
   forever. P1-N-06's scenario was 24 s of sim time, so 90 s of wall clock gave ~3.7x headroom.
   P1-N-07's run length is set by when a goal is reached and includes a Nav2 bringup, so 90 s
   could cut a healthy run short -- which would read as a scenario failure rather than as a
   budget failure.

2. three localisation/command topics added to `ros.topics`.
   They are what a Nav2 stack adds on top of the P1-N-06 bridge: `/map` (map_server),
   `/amcl_pose` (amcl) and `/cmd_vel` (the final command writer). The checker and the
   orchestrator both read them from here instead of repeating the strings, for the same reason
   the rest of the file exists: two places that default the same value independently is how
   009's `robots:=` default came to disagree with the case that declared three robots.

Nothing else in the file changes, and `duration_s` keeps its P1-N-06 default -- the N-07
orchestrator passes the duration on the command line.
"""
import argparse
import hashlib
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]

PATCHES = [
    (
        'wall deadline 90 -> 240 s',
        """  duration_s: 24.0
  wall_deadline_s: 90.0
""",
        """  duration_s: 24.0
  # Safety deadline, not an acceptance threshold. P1-N-06's scenario was 24 s of sim time, so
  # 90 s of wall clock gave ~3.7x headroom. P1-N-07's run length is set by when a goal is
  # reached and it includes a Nav2 bringup, so 90 s could cut a healthy run short -- which would
  # read as a scenario failure rather than as a budget failure. Raised to 240 s for that reason.
  wall_deadline_s: 240.0
""",
    ),
    (
        'topics: map, amcl_pose, cmd_vel',
        """    tf: /tf
    tf_static: /tf_static
""",
        """    tf: /tf
    tf_static: /tf_static
    # P1-N-07: the localisation and command topics a Nav2 stack adds on top of the N-06 bridge.
    # The checker and the orchestrator both read them from here rather than repeating strings.
    map: /map
    amcl_pose: /amcl_pose
    cmd_vel: /cmd_vel
""",
    ),
    (
        'topics: the two hops between the controller and the final command writer',
        """    cmd_vel: /cmd_vel
""",
        """    cmd_vel: /cmd_vel
    # The two hops between controller_server and the final writer, added after P1-N-07 run 02
    # showed /cmd_vel carrying nothing at all: with three publishers declared on /cmd_vel
    # (collision_monitor, docking_server, following_server) the interesting question is not
    # "who writes /cmd_vel" but "which hop is silent".
    cmd_vel_nav: /cmd_vel_nav
    cmd_vel_smoothed: /cmd_vel_smoothed
""",
    ),
]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--check', action='store_true')
    parser.add_argument('--root', default=str(ROOT))
    args = parser.parse_args()
    path = Path(args.root) / 'config' / 'n_probe.yaml'
    text = path.read_text(encoding='utf-8')
    before = hashlib.sha256(text.encode()).hexdigest()
    applied, pending = [], []
    for label, old, new in PATCHES:
        if new in text:
            applied.append(label)
            continue
        pending.append(label)
        if args.check:
            continue
        count = text.count(old)
        if count != 1:
            raise SystemExit(f'[FAIL] anchor for "{label}" appears {count} time(s), expected 1')
        text = text.replace(old, new)
    if args.check and pending:
        for label in pending:
            print(f'[FAIL] not applied: {label}', flush=True)
        return 1
    if not args.check and pending:
        path.write_text(text, encoding='utf-8')
        after = hashlib.sha256(text.encode()).hexdigest()
        print(f'{path.name}: applied {len(pending)}, already present {len(applied)}')
        print(f'  sha256 {before} -> {after}')
    else:
        print(f'{path.name}: all {len(applied)} patches already present')
    print('[OK] config patches present', flush=True)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
