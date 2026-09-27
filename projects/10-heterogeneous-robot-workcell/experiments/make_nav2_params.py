"""Derive `config/nav2_n_probe.yaml` from nav2's own default parameter file.

Why derive instead of hand-writing: nav2's stock file is 604 lines and covers ~20 nodes
(controller, planner, smoother, behaviour, BT navigator, costmaps, collision monitor, docking,
route, following). Hand-copying it would be a transcription exercise with a very high chance of
one silently wrong key, and a missing section does not error -- the node just runs on defaults.
Deriving it here means every deviation from stock is an *entry in the list below*, and nothing
else differs.

The generated file is committed to the repository so it can be read without running this, and
`--check` fails if the installed stock file changed enough that an anchor no longer matches.
That is deliberate: after a nav2 upgrade a human must re-read this list rather than have the
probe quietly inherit new defaults.

Every edit states its reason. The two that are load-bearing for the N probe's honesty are:

  * `use_sim_time: true` is written into EVERY section, including the costmaps. The probe's
    clock is the simulator's, and a node that silently runs on the wall clock would make a
    timeout look like a policy decision.
  * the command chain is left exactly as nav2 wires it (controller -> `cmd_vel_nav` ->
    velocity_smoother -> `cmd_vel_smoothed` -> collision_monitor -> `cmd_vel`), and only the
    plant limits are tightened to what the stand-in chassis can actually do (0.6 m/s, 1.2 rad/s,
    from config/n_probe.yaml). Whoever ends up publishing `/cmd_vel` is *measured* by the
    checker, not assumed here.
"""
import argparse
import hashlib
import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
STOCK_DEFAULT = Path('/opt/nav2/nav2_bringup/share/nav2_bringup/params/nav2_params.yaml')

RPP_BLOCK = '''    FollowPath:
      # P1-N-07 uses Regulated Pure Pursuit rather than the stock MPPI. Stock MPPI is
      # configured with batch_size 2000 x 56 time steps at 20 Hz, which is a real CPU budget
      # on a machine that is also stepping MuJoCo at 1x real time; and this round is about
      # the wiring and the measured arrival error, not about controller tuning. The choice is
      # written into the run report, so a pass cannot be read as "MPPI works here".
      plugin: "nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController"
      desired_linear_vel: 0.35
      lookahead_dist: 0.45
      min_lookahead_dist: 0.25
      max_lookahead_dist: 0.75
      lookahead_time: 1.5
      use_velocity_scaled_lookahead_dist: false
      transform_tolerance: 0.1
      min_approach_linear_velocity: 0.05
      approach_velocity_scaling_dist: 0.6
      use_collision_detection: true
      max_allowed_time_to_collision_up_to_carrot: 1.5
      use_regulated_linear_velocity_scaling: true
      use_cost_regulated_linear_velocity_scaling: false
      regulated_linear_scaling_min_radius: 0.6
      regulated_linear_scaling_min_speed: 0.15
      use_rotate_to_heading: true
      allow_reversing: false
      rotate_to_heading_min_angle: 0.5
      rotate_to_heading_angular_vel: 0.8
      max_angular_accel: 1.5
      use_interpolation: false
      max_robot_pose_search_dist: 10.0
'''

# (label, old, new, expected count of `old`)
LITERAL_EDITS = [
    (
        'amcl: motion-noise vector alpha1..5 0.2 -> 0.02. Measured reason: wheel odometry on this '
        'plant tracks truth to 0.005 m at the median over a 3.7 m localisation-only run '
        '(reports/p1-n-08-loc-03) -- the chassis is a rigid stand-in with an ideal velocity '
        'source and no slip model. The stock alphas describe a real robot whose odometry is '
        'noisy to roughly 10% of travel, so AMCL injects about 100x this plant\'s actual '
        'odometry noise, and with update_min_d 0.25 that becomes a ~0.05 m random step per '
        'update, i.e. ~0.19 m of random walk over 15 updates. The likelihood field here is too '
        'flat to pull it back (walls 4 m away, z_rand 0.5), so the walk survives. Measured '
        'effect of the change: docs/history/P1_N_AMCL_SESSION.md',
        '    alpha1: 0.2\n    alpha2: 0.2\n    alpha3: 0.2\n    alpha4: 0.2\n    alpha5: 0.2\n',
        '    alpha1: 0.02\n    alpha2: 0.02\n    alpha3: 0.02\n    alpha4: 0.02\n    alpha5: 0.02\n',
        1,
    ),
    (
        'bt_navigator: server timeout 20 -> 1000 and cancel timeout 50 -> 500. The N probe '
        'publishes /clock at 20 Hz (config state_rate_hz), i.e. one clock tick every 50 ms, and '
        'the whole stack runs on sim time. Measured on the first P1-N-07 run: the behaviour tree '
        'gave up waiting for controller_server to acknowledge the follow_path goal 206 ms after '
        'sending it, and controller_server received that goal 52 ms after the abort -- so the '
        'handshake needs about 260 ms and stock\'s 20 is shorter than one clock tick. This is a '
        'patience setting, not an acceptance threshold: acceptance is the measured arrival error',
        '    default_server_timeout: 20\n    default_cancel_timeout: 50\n',
        '    default_server_timeout: 1000\n    default_cancel_timeout: 500\n',
        1,
    ),
    (
        'amcl: base_frame_id base_footprint -> base_link (the probe publishes base_link)',
        '    base_frame_id: "base_footprint"\n    beam_skip_distance: 0.5\n',
        '    base_frame_id: "base_link"\n    beam_skip_distance: 0.5\n',
        1,
    ),
    (
        'amcl: start from the known arena origin instead of a pose it has to find. The arena is '
        '010-authored and the robot spawns at (0, 0, 0), so the start pose is known by '
        'construction -- the "known initial pose" option in TASK_BOARD P1-N-07, not a truth '
        'value read at run time',
        '    scan_topic: scan\n',
        '    scan_topic: scan\n'
        '    set_initial_pose: true\n'
        '    initial_pose:\n'
        '      x: 0.0\n'
        '      y: 0.0\n'
        '      z: 0.0\n'
        '      yaw: 0.0\n',
        1,
    ),
    (
        'controller_server: goal checker 0.25/0.25 -> 0.15/0.20, so the measured arrival error '
        'is a tight number rather than a third of a metre (acceptance allows tolerance + 0.10 m '
        'of stopping drift)',
        '      xy_goal_tolerance: 0.25\n      yaw_goal_tolerance: 0.25\n',
        '      xy_goal_tolerance: 0.15\n      yaw_goal_tolerance: 0.20\n',
        1,
    ),
    (
        'local_costmap: use the 2D obstacle layer, not voxels -- the only observation source is '
        'a 2D LaserScan, so a voxel grid is pure overhead',
        '      plugins: ["voxel_layer", "inflation_layer"]\n',
        '      plugins: ["obstacle_layer", "inflation_layer"]\n',
        1,
    ),
    (
        'local_costmap: VoxelLayer -> ObstacleLayer',
        '      voxel_layer:\n'
        '        plugin: "nav2_costmap_2d::VoxelLayer"\n'
        '        enabled: true\n'
        '        publish_voxel_map: true\n'
        '        origin_z: 0.0\n'
        '        z_resolution: 0.05\n'
        '        z_voxels: 16\n'
        '        max_obstacle_height: 2.0\n'
        '        mark_threshold: 0\n'
        '        observation_sources: scan\n',
        '      obstacle_layer:\n'
        '        plugin: "nav2_costmap_2d::ObstacleLayer"\n'
        '        enabled: true\n'
        '        observation_sources: scan\n',
        1,
    ),
    (
        'costmaps: no keepout/speed filter plugins (local). The KEY IS OMITTED, not emptied: an '
        'empty YAML sequence is rejected by rclcpp while it applies the parameter overrides, and '
        'the node aborts during construction -- controller_server and planner_server both died '
        'with "parameter_value_from failed for parameter \'filters\': rcl parameter structure '
        'contains no value". Omitting the key leaves the costmap on its own empty default',
        '      filters: ["keepout_filter"]\n',
        '      # filters: omitted on purpose; an empty YAML sequence [] aborts rclcpp at node\n'
        '      # construction, so the key is removed instead of emptied.\n',
        1,
    ),
    (
        'costmaps: no keepout/speed filter plugins (global), for the same reason',
        '      filters: ["keepout_filter", "speed_filter"]\n',
        '      # filters: omitted on purpose; an empty YAML sequence [] aborts rclcpp at node\n'
        '      # construction, so the key is removed instead of emptied.\n',
        1,
    ),
    (
        'costmaps: inscribed radius 0.22 -> 0.26 m, the circumscribed radius of the stand-in '
        'chassis (box half-sizes 0.19 x 0.13, outer wheel extent 0.17)',
        '      robot_radius: 0.22\n',
        '      robot_radius: 0.26\n',
        2,
    ),
    (
        'local_costmap: inflation radius 0.70 -> 0.35 m. 0.70 is a TurtleBot4-sized value that '
        'would make the 1.2 m gaps in this arena look closed',
        '        inflation_radius: 0.70\n',
        '        inflation_radius: 0.35\n',
        1,
    ),
    (
        'global_costmap: inflation radius 0.7 -> 0.35 m (same reason)',
        '        inflation_radius: 0.7\n',
        '        inflation_radius: 0.35\n',
        1,
    ),
    (
        'costmaps: read the scan on the absolute topic /scan. The stock file uses a relative '
        '`scan` with a comment about the parent namespace; with an empty namespace both resolve '
        'to /scan, and writing it absolutely removes the ambiguity',
        '          topic: scan\n',
        '          topic: /scan\n',
        2,
    ),
    (
        'costmaps: clearing ranges out to the configured scan maximum. Stock raytraces 3.0 m '
        'and marks obstacles out to 2.5 m, which in an 8x8 m arena means the walls are never '
        'cleared from the live scan and the local costmap is permanently stale',
        '          raytrace_max_range: 3.0\n'
        '          raytrace_min_range: 0.0\n'
        '          obstacle_max_range: 2.5\n'
        '          obstacle_min_range: 0.0\n',
        '          raytrace_max_range: 8.0\n'
        '          raytrace_min_range: 0.0\n'
        '          obstacle_max_range: 6.0\n'
        '          obstacle_min_range: 0.0\n',
        2,
    ),
    (
        'velocity_smoother: clamp to the plant. n_probe.yaml declares v_max 0.60 m/s and w_max '
        '1.20 rad/s; stock clamps to 0.5/2.0, i.e. it would allow a yaw rate the stand-in '
        'chassis cannot track',
        '    max_velocity: [0.5, 0.0, 2.0]\n'
        '    min_velocity: [-0.5, 0.0, -2.0]\n'
        '    max_accel: [2.5, 0.0, 3.2]\n'
        '    max_decel: [-2.5, 0.0, -3.2]\n',
        '    max_velocity: [0.6, 0.0, 1.2]\n'
        '    min_velocity: [-0.6, 0.0, -1.2]\n'
        '    max_accel: [1.0, 0.0, 2.0]\n'
        '    max_decel: [-1.0, 0.0, -2.0]\n',
        1,
    ),
    (
        'collision_monitor: base frame base_footprint -> base_link (the probe has no '
        'base_footprint frame). collision_monitor is the LAST writer on /cmd_vel as nav2 wires '
        'it, so its frame matters',
        '    base_frame_id: "base_footprint"\n'
        '    odom_frame_id: "odom"\n'
        '    cmd_vel_in_topic: "cmd_vel_smoothed"\n',
        '    base_frame_id: "base_link"\n'
        '    odom_frame_id: "odom"\n'
        '    cmd_vel_in_topic: "cmd_vel_smoothed"\n',
        1,
    ),
    (
        'costmaps: the KEEPOUT_ZONE_ENABLED placeholder becomes false here rather than at launch '
        'time, so the file means the same thing however it is launched',
        'enabled: KEEPOUT_ZONE_ENABLED',
        'enabled: false',
        2,
    ),
    (
        'costmaps: the SPEED_ZONE_ENABLED placeholder becomes false here (same reason)',
        'enabled: SPEED_ZONE_ENABLED',
        'enabled: false',
        1,
    ),
]

SLICE_EDITS = [
    (
        'controller_server: MPPI -> Regulated Pure Pursuit (see the block comment in the result)',
        '    FollowPath:\n',
        '\nlocal_costmap:\n',
        RPP_BLOCK,
    ),
]

USE_SIM_TIME_PATTERN = re.compile(r'^(?P<indent>[ ]*)ros__parameters:[ ]*$', re.M)
EXPECTED_SECTIONS = 20
# The stock file indents in steps of two, so a parameter goes one step deeper than the
# `ros__parameters:` line that owns it. Inserting it at the SAME indent makes it a sibling of
# `ros__parameters:` and the following keys (indent + 2) become children of a scalar -- which
# yaml rejects outright. That is how this was found, so the step is named rather than inlined.
INDENT_STEP = '  '

HEADER = """# P1-N-07 Nav2 parameters.  GENERATED -- do not hand-edit.
#
# Regenerate:  .venv/bin/python experiments/make_nav2_params.py
# Verify:      .venv/bin/python experiments/make_nav2_params.py --check
#
# Derived from {stock} (sha256 {stock_hash}).
# Generator sha256: {generator_hash}
#
# Every deviation from stock is one entry in experiments/make_nav2_params.py; this file has no
# other differences. The reasons are in that list, next to the edit.
#
# What is deliberately NOT changed: the nav2 command chain
#   controller_server --(cmd_vel -> cmd_vel_nav)--> velocity_smoother
#     --(cmd_vel_smoothed)--> collision_monitor --(cmd_vel)--> plant
# so the probe measures nav2 as shipped, including which node is the final writer on /cmd_vel.
"""


def apply_literal(text, label, old, new, expected):
    found = text.count(old)
    if found != expected:
        raise SystemExit(f'[FAIL] anchor for "{label}" appears {found} time(s), expected '
                         f'{expected}. The stock file changed; re-read this edit.')
    return text.replace(old, new)


def apply_slice(text, label, start_anchor, end_anchor, new):
    if text.count(start_anchor) != 1:
        raise SystemExit(f'[FAIL] slice start for "{label}" is not unique')
    if text.count(end_anchor) != 1:
        raise SystemExit(f'[FAIL] slice end for "{label}" is not unique')
    start = text.index(start_anchor) + len(start_anchor)
    end = text.index(end_anchor)
    if end < start:
        raise SystemExit(f'[FAIL] slice end for "{label}" precedes its start')
    return text[:start] + new + text[end:]


def build(stock_path):
    text = Path(stock_path).read_text(encoding='utf-8')
    for label, old, new, expected in LITERAL_EDITS:
        text = apply_literal(text, label, old, new, expected)
    for label, start_anchor, end_anchor, new in SLICE_EDITS:
        text = apply_slice(text, label, start_anchor, end_anchor, new)

    text, count = USE_SIM_TIME_PATTERN.subn(
        lambda m: f'{m.group(0)}\n{m.group("indent")}{INDENT_STEP}use_sim_time: true', text)
    if count != EXPECTED_SECTIONS:
        raise SystemExit(f'[FAIL] found {count} ros__parameters sections, expected '
                         f'{EXPECTED_SECTIONS}; a section was added or removed upstream')

    # Refuse to emit an empty YAML sequence. rclcpp parses parameter overrides from YAML while
    # it constructs the node, and a `key: []` becomes "rcl parameter structure contains no
    # value" -- an exception thrown out of the constructor, so the process aborts before it can
    # report anything. controller_server and planner_server both died this way in the first
    # P1-N-07 smoke run, and the only visible symptom was a lifecycle node stuck at
    # "unconfigured" plus a lifecycle manager spinning on "Waiting for service".
    offenders = [(number, line.strip()) for number, line in enumerate(text.splitlines(), 1)
                 if re.search(r':\s*\[\s*\]\s*$', line) and not line.lstrip().startswith('#')]
    if offenders:
        raise SystemExit(f'[FAIL] the generated file has empty YAML sequences, which rclcpp '
                         f'rejects at node construction: {offenders}')
    return text


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--stock', default=str(STOCK_DEFAULT))
    parser.add_argument('--out', default=str(ROOT / 'config' / 'nav2_n_probe.yaml'))
    parser.add_argument('--initial-pose', nargs=3, type=float, default=[0.0, 0.0, 0.0],
                        metavar=('X', 'Y', 'YAW'),
                        help='AMCL initial pose. 0,0,0 is the P1 arena, where the chassis spawns at the origin; a world whose robot spawns elsewhere overrides it here rather than in the generated file')
    parser.add_argument('--check', action='store_true')
    args = parser.parse_args()

    stock = Path(args.stock)
    if not stock.is_file():
        raise SystemExit(f'{stock} not found; nav2 is a source build in /opt/nav2, and this '
                         f'generator is anchored to its default parameter file')
    body = build(stock)
    # The start pose is one of the slice edits, inserted as 0,0,0. A world whose robot
    # spawns elsewhere overrides those four lines HERE, in the generator, rather than in
    # the generated artefact -- editing the artefact is what the header forbids.
    _x, _y, _yaw = (float(v) for v in args.initial_pose)
    if (_x, _y, _yaw) != (0.0, 0.0, 0.0):
        _old = '      x: 0.0\n      y: 0.0\n      z: 0.0\n      yaw: 0.0\n'
        _n = body.count(_old)
        assert _n == 1, f'initial_pose block occurs {_n}x in the generated body'
        body = body.replace(_old, f'      x: {_x}\n      y: {_y}\n      z: 0.0\n'
                                     f'      yaw: {_yaw}\n')
    stock_hash = hashlib.sha256(stock.read_bytes()).hexdigest()
    generator_hash = hashlib.sha256(Path(__file__).read_bytes()).hexdigest()
    header = HEADER.format(stock=stock, stock_hash=stock_hash, generator_hash=generator_hash)
    result = header + body

    out = Path(args.out)
    print(f'{len(result.splitlines())} lines, {len(LITERAL_EDITS)} literal edits, '
          f'{len(SLICE_EDITS)} block edit, use_sim_time in {EXPECTED_SECTIONS} sections',
          flush=True)
    if args.check:
        if not out.is_file():
            print(f'[FAIL] missing: {out}', flush=True)
            return 1
        if out.read_text(encoding='utf-8') != result:
            print(f'[FAIL] out of date: {out}', flush=True)
            return 1
        print(f'[OK] {out} is current', flush=True)
        return 0
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(result, encoding='utf-8')
    print(f'[OK] wrote {out}', flush=True)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
