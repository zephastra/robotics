"""Judge P1-N-07 from the files the run wrote. Plain Python: no ROS, no sockets, no simulation.

Inputs, all under `reports/<run-id>/`:
  * `nav_plan.json`      -- the scenario as DECLARED before the run started
  * `nav_report.json`    -- the orchestrator: stages, lifecycle states, the action result
  * `sim_report.json`    -- MuJoCo: the truth trajectory, the command gate, the stop reason
  * `ros_report.json`    -- the bridge: what it published and what it received on /cmd_vel
  * `checker_nav_raw.json` -- the independent observer: topic counts, publisher identities,
                              QoS and message types, TF edges, the first scan, amcl_pose samples

Judgement is a pure function of those files, so it can be tested on synthetic ones -- including
the ones that must FAIL. That matters here more than usual, because the interesting failure of
this round is a *silent* one: a subscriber of the wrong message type receives nothing, DDS
reports no error, and the only place the type appears is the graph introspection in
`checker_nav_raw.json`.

Two rules this file follows deliberately:

  * **Arrival is measured against truth, never against nav2's own opinion.** The run that
    passed this round had `navigate_to_pose` return SUCCEEDED while the vehicle stopped 0.43 m
    from the goal, because AMCL believed it was 0.13 m away. 009's protected-zone failures had
    the same shape: a self-reported pose with `localization_valid=True` that was metres out. So
    the goal checker is evidence that the *controller* finished, not that the *robot* arrived.
  * **A check whose input is missing is NOT_RUN, never PASS.** Every check names the fact it
    measures and the number it found.
"""
import argparse
import json
import math
from pathlib import Path
import re
import sys
from xml.etree import ElementTree

import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'experiments'))

from make_map_n import FREE, read_map, read_world_geoms, raycast_grid  # noqa: E402

REQUIRED_ACTIVE = ('map_server', 'amcl', 'planner_server', 'controller_server', 'bt_navigator')
COMMAND_TYPE = 'geometry_msgs/msg/TwistStamped'
COMMAND_TOPICS = ('cmd_vel_nav', 'cmd_vel_smoothed', 'cmd_vel')
# The arena is DERIVED from the world, and from the map that is a map of that world. These were the
# typed constants PILLAR_A = (2.00, 1.50, 0.15), ROBOT_CIRCUMSCRIBED_RADIUS = 0.23 and
# WALL_INNER_FACE = 3.95.
#
# PILLAR_A was the expensive one. The detour check compared the path's closest approach to a fixed
# coordinate AND required the route to come within 0.9 m of it, so a goal in another quadrant --
# where a different obstacle is the one that matters -- produced a FAIL that had nothing to do with
# the vehicle. A typed obstacle can only judge the route it was typed for. Repeatability needs
# several goals, so the obstacle is now whichever one the straight line to THIS goal is obstructed
# by, and "no obstacle on this route" is NOT_RUN with the reason stated, not a silent PASS.
#
# The robot radius is derived too: the AMR is a declared stand-in (P1-ENV-01), and a typed radius
# would judge the next chassis by the old footprint.
def obstacle_footprint_distance(obstacle, x, y):
    """Distance from (x, y) to the obstacle's own FOOTPRINT: exact for a circle, and for a box the
    distance to the RECTANGLE rather than to a circle drawn around it.

    The distinction is not cosmetic. `pillar_b` is a 0.6 x 0.6 box whose circumscribed radius is
    0.4243, so treating it as a circle demands 0.7045 m of centre clearance to pass a flat side
    that really needs 0.2802 m -- and an over-conservative check turns a legitimate route into a
    FAIL, which is the same class of error as the typed constant this replaced, in the other
    direction. Measured: the derived features reproduce the arena exactly (pillar_a r = 0.150,
    wall inner face 3.950, robot 0.2302), and none of them is typed.

    A point inside a footprint returns 0, so a straight line through the interior reads as
    obstructed rather than as a negative gap.
    """
    dx, dy = x - obstacle['x'], y - obstacle['y']
    if obstacle['kind'] == 'circle':
        return math.hypot(dx, dy) - obstacle['radius']
    half_x, half_y = obstacle['half']
    return math.hypot(max(abs(dx) - half_x, 0.0), max(abs(dy) - half_y, 0.0))


def body_geom_boxes(world_path):
    """BOX geoms that live inside a `<body>` -- the robot's own parts, which the map skips.

    This parser sits here, and NOT in `make_map_n`, on purpose: that generator writes its OWN
    sha256 into the map it produces (`generator_sha256`), so any edit to it invalidates a frozen
    map artefact by design. Meeting this judge's need by editing a frozen artefact's producer
    points the dependency the wrong way. Measured: touching `make_map_n.py` immediately turned
    `make_map_n.py --check` red, on a map whose PGM was byte-identical.
    """
    text = Path(world_path).read_text(encoding='utf-8')
    text = re.sub(r'<!--.*?-->', '', text, flags=re.S)
    root = ElementTree.fromstring(text)
    found = []

    def walk(node, in_body):
        for child in node:
            if child.tag == 'body':
                walk(child, True)
            elif child.tag == 'geom':
                if in_body and child.get('type', 'sphere') == 'box':
                    found.append({'name': child.get('name') or '<unnamed>',
                                  'size': [float(v) for v in (child.get('size') or '').split()]})
            else:
                walk(child, in_body)

    walk(root.find('worldbody'), False)
    return found


def robot_footprint_radius(world_path):
    """(circumscribed radius, geom name) of the chassis, read out of the world the run used.

    The chassis is the only BOX geom inside a `<body>` in this arena -- the wheels are cylinders --
    so it is identifiable from the geometry rather than by name. A second box raises instead of
    picking one: guessing which box is the footprint is how a judge starts measuring the wrong
    object, and every check here is a length.
    """
    boxes = body_geom_boxes(world_path)
    if len(boxes) != 1:
        raise SystemExit(
            f'{world_path}: expected exactly one BOX geom inside a <body> (the chassis footprint), '
            f'found {[g["name"] for g in boxes] or "none"}. The judge derives the robot radius from '
            f'the geometry, so which box is the footprint has to be unambiguous.')
    size = boxes[0]['size']
    return math.hypot(size[0], size[1]), boxes[0]['name']


def arena_features(world_path, map_yaml):
    """Obstacles, wall inner face, robot radius and interior extent -- all derived.

    Nothing is inferred from a name. The map's free cells give the arena's interior extent; a
    static non-plane geom whose centre lies OUTSIDE that extent is a boundary wall; one inside it
    is an obstacle. A wall's inner face is then read off its thin axis, which is why this asks the
    geometry rather than trusting the map's half-cell resolution.
    """
    built = read_map(map_yaml)
    width, res = built['width'], built['resolution']
    cells = [(index % width, index // width) for index, value in enumerate(built['grid'])
             if value == FREE]
    if not cells:
        raise SystemExit(f'{map_yaml}: no free cells, so the arena has no interior to measure')
    ox, oy = built['origin']
    extent = max(max(abs(ox + (col + 0.5) * res) for col, _row in cells),
                 max(abs(oy + (row + 0.5) * res) for _col, row in cells))

    static, _moving = read_world_geoms(world_path)
    obstacles, wall_faces = [], []
    for geom in static:
        if geom['type'] == 'plane':
            continue
        pos = (geom['pos'] + [0.0, 0.0, 0.0])[:3]
        size = geom['size']
        if max(abs(pos[0]), abs(pos[1])) > extent:
            axis = 0 if size[0] < size[1] else 1
            wall_faces.append(abs(pos[axis]) - size[axis])
            continue
        if geom['type'] == 'cylinder':
            obstacles.append({'name': geom['name'], 'x': pos[0], 'y': pos[1],
                              'kind': 'circle', 'radius': size[0]})
        else:
            obstacles.append({'name': geom['name'], 'x': pos[0], 'y': pos[1],
                              'kind': 'rect', 'half': [size[0], size[1]]})
    if not wall_faces:
        raise SystemExit(f'{world_path}: no boundary wall found outside the interior extent '
                         f'{extent:.3f} m')
    robot, robot_geom = robot_footprint_radius(world_path)
    return {'obstacles': obstacles, 'wall_inner_face': min(wall_faces),
            'robot_radius': robot, 'robot_geom': robot_geom, 'interior_extent_m': extent,
            'world': str(world_path), 'map': str(map_yaml)}


def wrapped(angle):
    """Angular difference on the circle."""
    return abs((angle + math.pi) % (2.0 * math.pi) - math.pi)


def load(path):
    return json.loads(path.read_text(encoding='utf-8')) if path.is_file() else None


def goal_checker_config(params_path):
    """What the arrival number was compared against: tolerance AND whether it latches.

    `nav_plan.json` used to restate this as typed constants -- `xy_m: 0.15, yaw_rad: 0.20` -- and
    that block omitted `stateful`, which is the one key that changes what 0.15 MEANS. A latched
    checker asks "was this inside the window at some point", not "is it inside now", so a reader
    who took the block at face value would explain a 0.28 m arrival as drift out of a 0.15 m
    window. Measured, the vehicle never entered the window at all: the arrival error is AMCL's.

    This reads the params file the run itself named (`params.path`, sha256 recorded next to it),
    so there is ONE copy of the number and no second copy to disagree with it.

    It cannot live in `make_nav2_params.py`, which is the natural owner of "what the params mean":
    that generator writes its own sha256 into the file it produces, so any edit to it invalidates a
    frozen artefact by design. Meeting this judge's need by editing a frozen artefact's producer
    points the dependency the wrong way -- the same trap `body_geom_boxes` documents for the map.
    """
    doc = yaml.safe_load(Path(params_path).read_text(encoding='utf-8'))
    node = (doc.get('controller_server') or {}).get('ros__parameters') or {}
    names = list(node.get('goal_checker_plugins') or [])
    if not names:
        return None
    name = names[0]
    block = node.get(name) or {}
    if block.get('xy_goal_tolerance') is None:
        return None
    return {'plugin': name, 'declared_plugins': names,
            'stateful': bool(block.get('stateful', False)),
            'xy_m': float(block['xy_goal_tolerance']),
            'yaw_rad': block.get('yaw_goal_tolerance'),
            'path': str(params_path)}


def truth_at(samples, when):
    """Linear interpolation of [t, x, y, yaw] samples, with yaw unwrapped."""
    if not samples:
        return None
    if when <= samples[0][0]:
        return samples[0][1:]
    if when >= samples[-1][0]:
        return samples[-1][1:]
    for previous, current in zip(samples, samples[1:]):
        if previous[0] <= when <= current[0]:
            span = current[0] - previous[0]
            fraction = 0.0 if span <= 0 else (when - previous[0]) / span
            dx = current[1] - previous[1]
            dy = current[2] - previous[2]
            dyaw = wrapped(current[3] - previous[3])
            return [previous[1] + fraction * dx, previous[2] + fraction * dy,
                    previous[3] + fraction * dyaw]
    return samples[-1][1:]


def path_length(samples):
    total = 0.0
    for previous, current in zip(samples, samples[1:]):
        total += math.hypot(current[1] - previous[1], current[2] - previous[2])
    return total


def speed_at(samples, when, window=0.5):
    """Mean speed over [when - window, when], from truth samples."""
    points = [row for row in samples if when - window <= row[0] <= when]
    if len(points) < 2:
        return None
    return path_length(points) / (points[-1][0] - points[0][0])


def percentile(values, fraction):
    if not values:
        return None
    ordered = sorted(values)
    index = min(len(ordered) - 1, max(0, int(round(fraction * (len(ordered) - 1)))))
    return ordered[index]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--run-id', required=True)
    parser.add_argument('--world', default=None,
                        help='the world the run used; defaults to config/n_probe.yaml, the same source the map generator reads')
    parser.add_argument('--reports-dir', default=None,
                        help='defaults to <root>/reports; exists so tests can point the judge '
                             'at synthetic inputs')
    parser.add_argument('--arrival-limit-m', type=float, default=0.25,
                        help='declared goal tolerance 0.15 m plus 0.10 m of stopping drift')
    parser.add_argument('--arrival-yaw-limit-rad', type=float, default=0.30)
    parser.add_argument('--clearance-margin-m', type=float, default=0.05,
                        help='added to pillar radius + robot circumscribed radius')
    parser.add_argument('--map-scan-tolerance-m', type=float, default=0.20)
    parser.add_argument('--map-scan-min-agreement', type=float, default=0.9)
    args = parser.parse_args()

    cfg = yaml.safe_load((ROOT / 'config' / 'n_probe.yaml').read_text(encoding='utf-8'))
    nav2 = yaml.safe_load((ROOT / 'config' / 'nav2_n_probe.yaml').read_text(encoding='utf-8'))
    out = (Path(args.reports_dir) if args.reports_dir else ROOT / 'reports') / args.run_id
    plan = load(out / 'nav_plan.json')
    # Derive the arena only when the plan declares the map it planned against. A plan that does
    # not declare one is thin, and this says so as a FAIL rather than raising on a key lookup.
    features = {}
    map_entry = (plan or {}).get('map') or {}
    if map_entry.get('yaml'):
        features = arena_features(ROOT / (args.world or cfg['world']), ROOT / map_entry['yaml'])
    nav = load(out / 'nav_report.json')
    sim = load(out / 'sim_report.json')
    ros = load(out / 'ros_report.json')
    checker = load(out / 'checker_nav_raw.json')

    checks = []

    def check(name, verdict, detail):
        checks.append({'check': name, 'verdict': verdict, 'detail': detail})

    for label, data in (('nav_plan', plan), ('nav_report', nav), ('sim_report', sim),
                        ('ros_report', ros), ('checker_nav_raw', checker)):
        check(f'{label} exists', 'PASS' if data else 'FAIL',
              'present' if data else f'missing: {out / (label + ".json")}')
    if not all((plan, nav, sim, ros, checker)):
        return write(out, args.run_id, checks, features)

    if not features:
        # Every geometric check below is a length, and every one of those lengths is anchored to
        # the arena. Without a declared map there is nothing to anchor them to, so say that
        # instead of measuring against an empty dict.
        check('the plan declares the map the arena is derived from', 'FAIL',
              "nav_plan.json carries no map.yaml, so the obstacle set, the wall inner face and the "
              "robot radius cannot be derived and the geometric checks below would be unanchored")
        return write(out, args.run_id, checks, features)

    # ---- the stack was actually up, decided by lifecycle state not by absence of crashes ----
    states = nav['lifecycle']['states']
    inactive = [name for name in REQUIRED_ACTIVE if states.get(name) != 'active']
    check('every required lifecycle node reached ACTIVE', 'PASS' if not inactive else 'FAIL',
          f"{len(states)} lifecycle nodes seen; not active: {inactive or 'none'}")
    declared = nav['lifecycle'].get('manager_node_names')
    managers = nav['lifecycle'].get('managers') or {}
    if not declared:
        check('the lifecycle managers declared the same node list', 'NOT_RUN',
              'node_names parameter not readable for either manager')
    else:
        missing = [name for name in declared if states.get(name) != 'active']
        extra = [name for name in states if name not in declared]
        counts = {name: (len(names) if names else 0) for name, names in managers.items()}
        check('the lifecycle managers declared the same node list',
              'PASS' if not missing and not extra else 'FAIL',
              f'the two managers declared {counts}, i.e. {len(declared)} distinct nodes; not '
              f'active among them: {missing or "none"}; active but undeclared: {extra or "none"} '
              f'(map_server and amcl belong to the localization manager, the rest to the '
              f'navigation manager, so the comparison is against their union)')

    # ---- one clock owner, one publisher per TF edge ------------------------------------------
    nodes = checker['observed']['publisher_nodes']
    check('exactly one /clock publisher', 'PASS' if nodes['clock'] == ['workcell010_n_bridge']
          else 'FAIL', f"{nodes['clock']} (the simulator's bridge owns time)")
    edges = sorted(tuple(edge) for edge in checker['observed']['tf_edges'])
    expected_edges = sorted({('odom', cfg['frames']['base']), ('map', 'odom')})
    check('dynamic TF carries exactly two edges, one per hop',
          'PASS' if edges == expected_edges else 'FAIL',
          f'edges seen: {edges}, expected {expected_edges}')
    check('the /tf publisher count matches the number of edges (one writer per edge)',
          'PASS' if len(nodes['tf']) == 2 else 'FAIL',
          f"{len(nodes['tf'])} publishers on /tf: {nodes['tf']}")
    static_edges = sorted(tuple(edge) for edge in checker['observed']['tf_static_edges'])
    check('static TF carries exactly the base->lidar edge',
          'PASS' if static_edges == [(cfg['frames']['base'], cfg['frames']['lidar'])] else 'FAIL',
          f'edges seen: {static_edges}')

    # ---- the map that was loaded is the map that was generated --------------------------------
    built = read_map(ROOT / plan['map']['yaml'])
    info = checker['observed'].get('map_info')
    if not info:
        check('/map was published and matches the generated map', 'NOT_RUN', 'no OccupancyGrid')
    else:
        same = (info['width'] == built['width'] and info['height'] == built['height']
                and abs(info['resolution'] - built['resolution']) < 1e-6
                and abs(info['origin'][0] - built['origin'][0]) < 1e-6
                and abs(info['origin'][1] - built['origin'][1]) < 1e-6)
        check('/map was published and matches the generated map', 'PASS' if same else 'FAIL',
              f"published {info['width']}x{info['height']} res {info['resolution']:.4f} origin "
              f"({info['origin'][0]}, {info['origin'][1]}); generated {built['width']}x"
              f"{built['height']} res {built['resolution']} origin {built['origin']}; "
              f"cells occupied {info['occupied']} free {info['free']} unknown {info['unknown']}")

    # ---- the map agrees with what the robot SAW, which is the only check a wrong origin,
    #      resolution or image flip would fail ---------------------------------------------------
    scan = checker['observed'].get('scan_first')
    if not scan or sim.get('truth_samples'):
        if not scan:
            check('the static map agrees with the robot\'s first scan', 'NOT_RUN',
                  'no scan recorded')
        else:
            stamp = checker['observed']['scan_first_stamp']
            pose = truth_at(sim['truth_samples'], stamp)
            scan_cfg = cfg['scan']
            nrays = int(scan_cfg['rays'])
            angles = [math.radians(float(scan_cfg['angle_min_deg'])
                                   + index * (float(scan_cfg['angle_max_deg'])
                                              - float(scan_cfg['angle_min_deg'])) / (nrays - 1))
                      for index in range(nrays)]
            # The lidar sits 0.02 m ahead of the body origin, as the bridge's static transform
            # says; cast from there so the two sides describe the same sensor.
            lidar_pose = (pose[0] + 0.02 * math.cos(pose[2]), pose[1] + 0.02 * math.sin(pose[2]),
                          pose[2])
            expected = raycast_grid(built, lidar_pose, angles, float(scan_cfg['range_max_m']))
            if len(scan) != len(expected):
                check('the static map agrees with the robot\'s first scan', 'FAIL',
                      f'scan has {len(scan)} rays, map raycast produced {len(expected)}')
            else:
                agree = sum(1 for got, want in zip(scan, expected)
                            if abs(got - want) <= args.map_scan_tolerance_m)
                fraction = agree / len(expected)
                worst = max(abs(got - want) for got, want in zip(scan, expected))
                check('the static map agrees with the robot\'s first scan',
                      'PASS' if fraction >= args.map_scan_min_agreement else 'FAIL',
                      f'{fraction:.1%} of {len(expected)} rays within '
                      f'{args.map_scan_tolerance_m} m at sim {stamp} s, worst {worst:.3f} m '
                      f'(a wrong map origin, resolution or image flip fails this)')

    # ---- the command chain: right types, one bad hop is identifiable --------------------------
    declared_types = {}
    for topic in COMMAND_TOPICS:
        entries = checker['observed']['publisher_qos'].get(topic) or []
        declared_types[topic] = sorted({entry.get('type') for entry in entries if entry.get('type')})
    wrong = {topic: types for topic, types in declared_types.items()
             if types and types != [COMMAND_TYPE]}
    observed_types = {topic: types for topic, types in declared_types.items() if types}
    if not observed_types:
        check('every command topic carries the declared message type', 'NOT_RUN',
              'no publisher types observed')
    else:
        check('every command topic carries the declared message type',
              'PASS' if not wrong else 'FAIL',
              f'observer assumed {checker.get("command_topic_type_assumed")}; seen on the wire: '
              f'{observed_types}')
    counts = checker['counts']
    hop_detail = {topic: counts.get(topic, 0) for topic in COMMAND_TOPICS}
    check('every hop of the command chain carried traffic',
          'PASS' if all(value > 0 for value in hop_detail.values()) else 'FAIL',
          f'controller -> cmd_vel_nav -> cmd_vel_smoothed -> cmd_vel counts: {hop_detail}; a '
          f'zero identifies the broken hop')
    received = (ros.get('cmd_vel') or {}).get('received', 0)
    check('the bridge received the command stream', 'PASS' if received > 0 else 'FAIL',
          f"{received} /cmd_vel messages received, {(ros.get('cmd_vel') or {}).get('clipped')} "
          f"clipped, type {(ros.get('cmd_vel') or {}).get('type')}, publisher types seen "
          f"{(ros.get('cmd_vel') or {}).get('publisher_types_seen')}")
    check('the bridge saw no type mismatch', 'PASS'
          if (ros.get('cmd_vel') or {}).get('type_mismatch') is False else 'FAIL',
          f"type_mismatch={(ros.get('cmd_vel') or {}).get('type_mismatch')}")

    # ---- the gate held, and lapsed when the controller stopped --------------------------------
    gate = sim.get('gate') or {}
    modes = gate.get('modes') or {}
    check('the simulator command gate accepted commands',
          'PASS' if gate.get('accepted', 0) > 0 else 'FAIL',
          f"accepted {gate.get('accepted', 0)}, refused {gate.get('refused', 0)}, reasons "
          f"{gate.get('reasons')}")
    check('the gate was HOLDING while commands arrived',
          'PASS' if modes.get('HOLDING', 0) > 0 else 'FAIL', f'step modes: {modes}')
    check('the gate fell back to zero after the goal was reached',
          'PASS' if modes.get('SILENT', 0) > 0 and gate.get('accepted', 0) > 0 else 'FAIL',
          f"SILENT steps {modes.get('SILENT', 0)}, final mode {gate.get('mode')!r}, reason "
          f"{gate.get('reason')!r}")

    # ---- arrival, measured against truth ------------------------------------------------------
    strategy = nav.get('action') or {}
    check('the action server accepted the goal', 'PASS' if strategy.get('accepted') else 'FAIL',
          f"accepted={strategy.get('accepted')}, sent at sim {strategy.get('sent_sim_s')} s")
    check('the goal was reported SUCCEEDED', 'PASS' if strategy.get('status') == 4 else 'FAIL',
          f"action status {strategy.get('status')} (4 = SUCCEEDED), error_code "
          f"{strategy.get('error_code')}, {strategy.get('feedback_count')} feedback messages")

    samples = sim.get('truth_samples') or []
    goal = plan['goal']
    straight = math.hypot(goal['x'], goal['y'])
    if not samples:
        for name in ('the vehicle arrived within tolerance of the goal, measured against truth',
                     'the route went around the obstacle on it rather than through it',
                     'the vehicle came to rest after the goal'):
            check(name, 'NOT_RUN', 'no truth samples')
    else:
        result_time = strategy.get('result_sim_s')
        final_pose = samples[-1][1:]
        final_distance = math.hypot(final_pose[0] - goal['x'], final_pose[1] - goal['y'])
        final_yaw_error = wrapped(final_pose[2] - goal['yaw'])
        # The CLOSEST truth approach and when. This is the quantity that answers "did it ever get
        # there", and it is reported because on the eight runs measured it EQUALS the final
        # distance in 8 of 8: the approach is monotonic, the vehicle stops at its closest point,
        # and there is no post-arrival drift for a second quantity to describe. The row that used
        # to sit here printed "the distance at the moment nav2 reported success" beside the final
        # distance and the two were the same number in 7 of 7 -- a diagnostic that cannot report
        # anything the reader does not already have.
        run_min = min((math.hypot(row[1] - goal['x'], row[2] - goal['y']) for row in samples))
        run_min_time = min(samples, key=lambda row: math.hypot(row[1] - goal['x'],
                                                              row[2] - goal['y']))[0]
        verdict = ('PASS' if final_distance <= args.arrival_limit_m
                   and final_yaw_error <= args.arrival_yaw_limit_rad else 'FAIL')
        gc = goal_checker_config(ROOT / plan['params']['path']) if (
            (plan.get('params') or {}).get('path')) else None
        belief_samples = checker['observed'].get('amcl_samples') or []
        when = result_time if result_time is not None else samples[-1][0]
        belief_pose = (min(belief_samples, key=lambda row: abs(row[0] - when))
                       if belief_samples else None)
        if belief_pose is None or gc is None:
            because = ('no amcl_pose sample was recorded' if belief_pose is None else
                       'the plan does not name the params file the tolerance lives in, so the '
                       'pose nav2 compared against cannot be recovered')
            decomposition = f'[not decomposed: {because}]'
            belief_distance = None
        else:
            stamp, bx, by, _byaw = belief_pose
            belief_distance = math.hypot(bx - goal['x'], by - goal['y'])
            reference = truth_at(samples, stamp)
            belief_error = (math.hypot(bx - reference[0], by - reference[1])
                            if reference else None)
            entered = [row for row in samples
                       if math.hypot(row[1] - goal['x'], row[2] - goal['y']) <= gc['xy_m']]
            decomposition = (
                f'; decomposed: at sim {stamp:.2f} s -- the pose nav2 BELIEVES in -- the believed '
                f'distance to the goal was {belief_distance:.3f} m with a belief error of '
                f'{belief_error:.3f} m, so {final_distance - belief_distance:+.3f} m of the arrival '
                f'number is LOCALISATION and {belief_distance:.3f} m is the vehicle stopping short '
                f'of its own believed goal. The declared tolerance is {gc["xy_m"]} m from '
                f'{gc["path"]} with stateful={gc["stateful"]}, and the truth entered it '
                + (f'at sim {entered[0][0]:.2f} s' if entered else 'NEVER'))
        check('the vehicle arrived within tolerance of the goal, measured against truth', verdict,
              f'final truth pose ({final_pose[0]:.3f}, {final_pose[1]:.3f}, yaw '
              f'{final_pose[2]:.3f}) is {final_distance:.3f} m and {final_yaw_error:.3f} rad from '
              f'the goal (limits {args.arrival_limit_m} m / {args.arrival_yaw_limit_rad} rad); the '
              f'closest truth approach over the whole run was {run_min:.3f} m at sim '
              f'{run_min_time:.2f} s' + decomposition)

        # ---- did it stop where IT thinks the goal is? The part of the arrival number that is
        #      the controller's own doing, judged against a tolerance DERIVED from the run's own
        #      params file rather than against a number restated in this judge.
        BELIEF = 'nav2 stopped inside its own declared tolerance of the goal it believes in'
        if gc is None:
            check(BELIEF, 'NOT_RUN',
                  'nav_plan.json names no params file, so the goal checker\'s xy tolerance '
                  'cannot be derived and this check would be measuring against a guess')
        elif belief_pose is None:
            check(BELIEF, 'NOT_RUN', 'no amcl_pose samples, so the believed pose is unknown')
        else:
            inside = belief_distance <= gc['xy_m']
            check(BELIEF, 'PASS' if inside else 'FAIL',
                  f'the last pose nav2 believes in (sim {stamp:.2f} s, '
                  f'{"the success stamp" if result_time is not None else "the end of the run"}) is '
                  f'{belief_distance:.3f} m from the goal, against the {gc["xy_m"]} m declared by '
                  f'{gc["path"]} ({gc["plugin"]}, stateful={gc["stateful"]}). '
                  + ('It came to rest inside its own tolerance.'
                     if inside else
                     'It came to rest OUTSIDE the tolerance it applied: nav2 reported a finished '
                     'goal while believing it was further away than the contract allows, which is '
                     'the "accepted does not mean arrived" defect in the frame nav2 itself uses.'))

        travelled = path_length(samples)
        start = (samples[0][1], samples[0][2])

        # WHICH obstacle -- derived from this run's own goal, not named. The straight line is
        # SAMPLED rather than measured centre-to-line, because a box's closest point is not on the
        # line through its centre. 5 mm steps over a route of at most 11 m is 2200 points.
        leg = math.hypot(goal['x'] - start[0], goal['y'] - start[1])
        steps = max(2, int(leg / 0.005))
        principal, line_gap = None, None
        for obstacle in features['obstacles']:
            gap = min(obstacle_footprint_distance(
                obstacle,
                start[0] + (k / steps) * (goal['x'] - start[0]),
                start[1] + (k / steps) * (goal['y'] - start[1])) for k in range(steps + 1))
            if line_gap is None or gap < line_gap:
                principal, line_gap = obstacle, gap
        need = features['robot_radius'] + args.clearance_margin_m
        census = ', '.join(
            f"{o['name']} ({o['x']:.2f}, {o['y']:.2f}) "
            + (f"r={o['radius']:.3f}" if o['kind'] == 'circle'
               else f"box {2 * o['half'][0]:.2f}x{2 * o['half'][1]:.2f}")
            for o in features['obstacles'])
        if line_gap >= need:
            # NOT_RUN, not PASS: this route never had to avoid anything, so the run says nothing
            # about planning around an obstacle. The old check called this case FAIL, because it
            # was typed to one pillar and to one goal.
            check('the route went around the obstacle on it rather than through it', 'NOT_RUN',
                  f'the straight line from ({start[0]:.2f}, {start[1]:.2f}) to the goal clears the '
                  f"nearest obstacle {principal['name']} by {line_gap:.3f} m, which is at least the "
                  f"{need:.3f} m a {features['robot_radius']:.3f} m robot needs, so there is no "
                  f'obstacle to go around on this route. Obstacles derived from the world: '
                  f'{census}. This is NOT_RUN, which is not PASS.')
        else:
            clearance = min(obstacle_footprint_distance(principal, row[1], row[2])
                            for row in samples)
            check('the route went around the obstacle on it rather than through it',
                  'PASS' if clearance >= need else 'FAIL',
                  f'truth path {travelled:.3f} m against a straight line of {straight:.3f} m '
                  f'(+{travelled - straight:.3f} m); closest approach to '
                  f"{principal['name']} ({principal['x']:.2f}, {principal['y']:.2f}) was "
                  f'{clearance:.3f} m, need >= {need:.3f} m clear of its OWN FOOTPRINT = robot '
                  f"{features['robot_radius']:.3f} m ({features['robot_geom']}, circumscribed) + "
                  f'margin {args.clearance_margin_m:.3f}. The obstacle is DERIVED, not named: it is '
                  f'whichever one the straight line to THIS goal passes closest to. All: {census}')
        outside = max(max(abs(row[1]), abs(row[2])) for row in samples)
        check('the vehicle stayed inside the walls',
              'PASS' if outside <= features['wall_inner_face'] - features['robot_radius']
              else 'FAIL',
              f"largest |x| or |y| reached {outside:.3f} m, inner wall face at "
              f"{features['wall_inner_face']:.3f} m DERIVED from the wall geoms' thin axis, body "
              f"radius {features['robot_radius']:.3f} m")

        settled = speed_at(samples, samples[-1][0], window=1.0)
        check('the vehicle came to rest after the goal',
              'PASS' if settled is not None and settled <= 0.02 else 'FAIL',
              f'mean speed {settled:.4f} m/s over the last 1.0 s of truth'
              if settled is not None else 'not enough truth samples at the end')

    # ---- localisation: amcl_pose against truth at the same stamp ------------------------------
    amcl_samples = checker['observed'].get('amcl_samples') or []
    if not amcl_samples or not samples:
        check('AMCL agrees with truth at the same simulator time', 'NOT_RUN',
              f'{len(amcl_samples)} amcl_pose samples, {len(samples)} truth samples')
    else:
        errors, yaw_errors, pairs = [], [], []
        for stamp, x, y, yaw in amcl_samples:
            reference = truth_at(samples, stamp)
            if reference is None:
                continue
            errors.append(math.hypot(x - reference[0], y - reference[1]))
            yaw_errors.append(wrapped(yaw - reference[2]))
            pairs.append((stamp, x - reference[0], y - reference[1], wrapped(yaw - reference[2])))
        if not errors:
            check('AMCL agrees with truth at the same simulator time', 'NOT_RUN',
                  'no amcl sample fell inside the truth window')
        else:
            median = percentile(errors, 0.5)
            p95 = percentile(errors, 0.95)
            median_yaw = percentile(yaw_errors, 0.5)
            verdict = 'PASS' if (median <= 0.15 and p95 <= 0.30 and median_yaw <= 0.15) else 'FAIL'
            check('AMCL agrees with truth at the same simulator time', verdict,
                  f'{len(errors)} paired samples: position median {median:.3f} m / p95 '
                  f'{p95:.3f} m, yaw median {median_yaw:.3f} rad (limits 0.15 / 0.30 / 0.15); '
                  f'last pair dt={pairs[-1][0]:.2f}s dx={pairs[-1][1]:+.3f} dy={pairs[-1][2]:+.3f} '
                  f'dyaw={pairs[-1][3]:+.3f}')

    # ---- odometry against truth, for the N-06 comparison ---------------------------------------
    truth_final = (sim.get('truth') or {}).get('final')
    odom_final = ros.get('odom_final')
    if truth_final and odom_final:
        position_error = math.hypot(odom_final['x'] - truth_final[0],
                                    odom_final['y'] - truth_final[1])
        yaw_error = wrapped(odom_final['yaw'] - truth_final[2])
        check('wheel odometry tracks truth over the route',
              'PASS' if position_error <= 0.30 and yaw_error <= 0.25 else 'FAIL',
              f'position {position_error:.4f} m, yaw {yaw_error:.4f} rad over '
              f"{(sim.get('truth') or {}).get('travelled_m')} m driven")
    else:
        check('wheel odometry tracks truth over the route', 'NOT_RUN', 'no truth or odom pose')

    # ---- the two halves finished, and the run left nothing behind -----------------------------
    check('the simulator completed and said why it stopped',
          'PASS' if sim.get('status') == 'COMPLETED' and sim.get('stop_reason') else 'FAIL',
          f"status {sim.get('status')}, stop_reason {sim.get('stop_reason')}, "
          f"{sim.get('sim_seconds')} s sim")
    gap = (abs(ros.get('sim_seconds', 0) - sim.get('sim_seconds', 0))
           if ros.get('sim_seconds') is not None else None)
    check('the bridge followed the simulation to its end',
          'PASS' if gap is not None and gap <= 0.6 else 'FAIL',
          f"bridge last sim time {ros.get('sim_seconds')} s vs simulator "
          f"{sim.get('sim_seconds')} s (difference {gap}); the bridge's own status is "
          f"{ros.get('status')!r}, which reads STATE_STREAM_STOPPED_EARLY when the run ends by "
          f"SIGUSR1 before its duration")
    check('the bridge refused no state datagrams and lost no sequence numbers',
          'PASS' if (ros['counters']['decode_refusals'] == 0 and ros.get('seq_gaps') == 0
                     and ros['counters']['states_received'] == sim['ipc']['states_sent'])
          else 'FAIL',
          f"received {ros['counters']['states_received']} of {sim['ipc']['states_sent']} sent, "
          f"seq gaps {ros.get('seq_gaps')}, decode refusals "
          f"{ros['counters']['decode_refusals']}")
    leftovers = nav.get('leftovers') or []
    check('the run left no processes behind', 'PASS' if not leftovers else 'FAIL',
          f'{len(leftovers)} leftover processes: {leftovers[:4]}')

    return write(out, args.run_id, checks, features)


def write(out, run_id, checks, features=None):
    failed = [item for item in checks if item['verdict'] == 'FAIL']
    not_run = [item for item in checks if item['verdict'] == 'NOT_RUN']
    verdict = 'FAIL' if failed else ('INCOMPLETE' if not_run else 'PASS')
    result = {'run_id': run_id, 'verdict': verdict, 'checks': checks,
              'failed': [item['check'] for item in failed],
              'not_run': [item['check'] for item in not_run],
              'arena_features': features or {}}
    (out / 'acceptance.json').write_text(json.dumps(result, indent=2) + '\n')

    lines = [f'# P1-N-07 acceptance -- {run_id}', '',
             f'**verdict: {verdict}**  ({len(checks)} checks: '
             f'{sum(1 for c in checks if c["verdict"] == "PASS")} PASS, {len(failed)} FAIL, '
             f'{len(not_run)} NOT_RUN)', '']
    for item in checks:
        lines.append(f'- `{item["verdict"]}` **{item["check"]}** -- {item["detail"]}')
    (out / 'acceptance.md').write_text('\n'.join(lines) + '\n')

    print(json.dumps({'run_id': run_id, 'verdict': verdict,
                      'failed': result['failed'], 'not_run': result['not_run']}, indent=2))
    for item in checks:
        print(f"  {item['verdict']:<8} {item['check']}: {item['detail']}")
    return 0 if verdict == 'PASS' else 1


if __name__ == '__main__':
    # A judge that crashes must not look like a pass (the P1-N-06 judge made that mistake once).
    try:
        raise SystemExit(main())
    except SystemExit:
        raise
    except Exception as exc:                                        # noqa: BLE001
        print(json.dumps({'verdict': 'FAIL',
                          'judge_error': f'{type(exc).__name__}: {exc}'}, indent=2))
        raise SystemExit(1)
