"""P1-N-07 tests: the map generator, the nav2 parameter generator, and the judge.

The judge tests are the point. A judge that cannot be shown to FAIL is not a judge, and this
round's actual defect was a *silent* one -- a subscriber of the wrong message type receives
nothing and DDS reports no error -- so the synthetic runs below include exactly that case, plus
the case where nav2 says SUCCEEDED while the vehicle stopped 0.43 m short of the goal.
"""
import json
import math
from pathlib import Path
import sys
import xml.etree.ElementTree as ElementTree

import pytest
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src'))
sys.path.insert(0, str(ROOT / 'experiments'))

import make_map_n  # noqa: E402
import make_nav2_params  # noqa: E402
import evaluate_n_nav  # noqa: E402

WORLD = ROOT / 'assets' / 'worlds' / 'world_n_probe.xml'
MAP_YAML = ROOT / 'assets' / 'maps' / 'arena_n_probe.yaml'


# ---------------------------------------------------------------- map generator

def test_the_start_pose_lands_on_the_centre_cell():
    built = make_map_n.build_grid(WORLD)
    assert built['width'] == built['height'] == 181
    row, col = make_map_n.to_cell(0.0, 0.0, built['origin'], built['resolution'],
                                 built['height'])
    assert (row, col) == (90, 90)
    # and the cell centre is the start pose itself, which is what makes a raycast through the
    # map comparable with a real scan instead of being half a cell out
    assert make_map_n.to_world(row, col, built['origin'], built['resolution'],
                               built['height']) == (0.0, 0.0)
    assert built['grid'][row * built['width'] + col] == make_map_n.FREE


def test_cell_and_world_conversions_round_trip():
    built = make_map_n.build_grid(WORLD)
    for x, y in ((0.0, 0.0), (2.0, 1.5), (-2.6, 2.2), (3.9, -3.9)):
        row, col = make_map_n.to_cell(x, y, built['origin'], built['resolution'],
                                      built['height'])
        centre_x, centre_y = make_map_n.to_world(row, col, built['origin'],
                                                 built['resolution'], built['height'])
        assert abs(centre_x - x) <= built['resolution']
        assert abs(centre_y - y) <= built['resolution']


def test_the_map_agrees_with_hand_computed_geometry():
    """The walls' inner faces are at +-3.95 m, so a ray along +x or +-y reports about that and
    a 45 degree ray reports 3.95 / cos(45). A wrong origin, resolution or row order fails this."""
    built = make_map_n.build_grid(WORLD)
    angles = [math.radians(value) for value in (-90, -45, 0, 45, 90)]
    ranges = make_map_n.raycast_grid(built, (0.0, 0.0, 0.0), angles, 8.0)
    expected = {0: 3.95, 45: 5.586, 90: 3.95, -45: 5.586, -90: 3.95}
    for angle, value in zip((int(round(math.degrees(a))) for a in angles), ranges):
        assert abs(value - expected[angle]) < 0.06


def test_the_band_outside_the_walls_is_not_free():
    """Free space is the reachable space: the corners outside the wall ring must be occupied,
    otherwise a planner could route around the outside of the arena."""
    built = make_map_n.build_grid(WORLD)
    for x, y in ((4.20, 4.20), (-4.20, 4.20), (4.20, -4.20), (-4.20, -4.20)):
        row, col = make_map_n.to_cell(x, y, built['origin'], built['resolution'],
                                      built['height'])
        assert built['grid'][row * built['width'] + col] == make_map_n.OCCUPIED, (x, y)


def test_the_movable_geoms_are_skipped_and_recorded():
    built = make_map_n.build_grid(WORLD)
    assert 'chassis' not in built['included_geoms']
    assert set(built['skipped_moving_geoms']) == {'chassis', 'wheel_left_geom',
                                                  'wheel_right_geom'}
    assert 'floor' in built['floor_geoms']


def test_a_rotated_static_geom_is_refused_rather_than_misplaced(tmp_path):
    """A rotated obstacle cannot be rasterised by an axis-aligned rasteriser, and drawing it in
    the wrong place is exactly the failure a map must not have -- so it raises."""
    broken = tmp_path / 'world.xml'
    broken.write_text("""<mujoco>
      <worldbody>
        <geom name="floor" type="plane" size="12 12 0.05"/>
        <geom name="rotated_wall" type="box" pos="1 1 0.3" size="0.3 0.3 0.3"
              quat="0.9238795 0 0 0.3826834"/>
      </worldbody>
    </mujoco>
    """, encoding='utf-8')
    with pytest.raises(SystemExit) as excinfo:
        make_map_n.build_grid(broken)
    assert 'rotated_wall' in str(excinfo.value)


def test_the_real_world_file_parses_only_after_comments_are_stripped():
    """MuJoCo's parser accepts `--` inside an XML comment and XML does not, so ElementTree
    rejects the world file outright. The rasteriser strips comments, and this test pins both
    halves of that: the raw file is not well-formed, and the reader copes."""
    with pytest.raises(ElementTree.ParseError):
        ElementTree.parse(WORLD)
    static, _moving = make_map_n.read_world_geoms(WORLD)
    assert [geom['name'] for geom in static] == ['floor', 'wall_north', 'wall_south',
                                                'wall_east', 'wall_west', 'pillar_a',
                                                'pillar_b', 'pillar_c']


def test_the_generated_map_on_disk_is_current():
    """`--check` is the guard that makes a hand-edited map impossible."""
    built = make_map_n.build_grid(WORLD)
    assert MAP_YAML.is_file()
    pgm = (MAP_YAML.parent / 'arena_n_probe.pgm').read_bytes()
    assert pgm == make_map_n.pgm_bytes(built)
    loaded = make_map_n.read_map(MAP_YAML)
    assert (loaded['width'], loaded['height']) == (built['width'], built['height'])
    assert loaded['grid'] == built['grid']


# ---------------------------------------------------------------- nav2 parameters

def test_the_generated_nav2_params_parse_and_carry_no_empty_sequence():
    text = (ROOT / 'config' / 'nav2_n_probe.yaml').read_text(encoding='utf-8')
    parsed = yaml.safe_load(text)
    assert len(parsed) == 20
    for number, line in enumerate(text.splitlines(), 1):
        stripped = line.lstrip()
        if stripped.startswith('#'):
            continue
        # an empty YAML sequence aborts rclcpp while it applies parameter overrides, and the
        # visible symptom is a lifecycle node stuck at "unconfigured"
        assert not line.rstrip().endswith(': []'), f'line {number}: {line}'
    assert make_nav2_params.EXPECTED_SECTIONS == 20


def test_every_nav2_section_runs_on_simulation_time():
    parsed = yaml.safe_load((ROOT / 'config' / 'nav2_n_probe.yaml').read_text(encoding='utf-8'))
    def parameters(section):
        if 'ros__parameters' in section:
            return section['ros__parameters']
        return next(iter(section.values()))['ros__parameters']

    for name, section in parsed.items():
        assert parameters(section).get('use_sim_time') is True, name


def test_the_committed_nav2_params_match_the_generator():
    stock = Path(make_nav2_params.STOCK_DEFAULT)
    if not stock.is_file():
        pytest.skip('nav2 default parameter file not installed on this machine')
    text = (ROOT / 'config' / 'nav2_n_probe.yaml').read_text(encoding='utf-8')
    assert text.endswith(make_nav2_params.build(stock))


def test_an_anchor_that_does_not_match_is_refused():
    with pytest.raises(SystemExit):
        make_nav2_params.apply_literal('this text has nothing in it', 'label', 'needle',
                                       'new', 1)
    with pytest.raises(SystemExit):
        make_nav2_params.apply_literal('one needle, one needle', 'label', 'needle', 'new', 1)


def test_the_generator_refuses_to_emit_an_empty_sequence(tmp_path):
    """An empty YAML sequence aborts rclcpp at node construction, and the symptom is a lifecycle
    node stuck at unconfigured -- so the generator refuses to produce one."""
    stock = Path(make_nav2_params.STOCK_DEFAULT)
    if not stock.is_file():
        pytest.skip('nav2 default parameter file not installed on this machine')
    patched = tmp_path / 'stock.yaml'
    patched.write_text(stock.read_text(encoding='utf-8').replace(
        '    introspection_mode: "disabled"\n', '    empty_list: []\n', 1), encoding='utf-8')
    with pytest.raises(SystemExit) as excinfo:
        make_nav2_params.build(patched)
    assert 'empty YAML sequences' in str(excinfo.value)


# ---------------------------------------------------------------- the judge

STRAIGHT_LINE_GOAL = (2.6, 2.4)
AROUND = [(0.6, 1.2), (1.4, 2.2), (2.6, 2.4)]


def _walk(waypoints, samples_per_leg=20, dt=0.1, start=(0.0, 0.0), yaw_end=0.0):
    points, t = [], 0.0
    previous = start
    for target in list(waypoints):
        for step in range(1, samples_per_leg + 1):
            fraction = step / samples_per_leg
            x = previous[0] + fraction * (target[0] - previous[0])
            y = previous[1] + fraction * (target[1] - previous[1])
            heading = math.atan2(target[1] - previous[1], target[0] - previous[0])
            t += dt
            points.append([round(t, 4), round(x, 6), round(y, 6), round(heading, 6)])
        previous = target
    for _ in range(12):                      # a stationary tail, so "came to rest" is measurable
        t += dt
        points.append([round(t, 4), points[-1][1], points[-1][2], yaw_end])
    return points


def build_run(out, *, waypoints=AROUND, goal=(2.6, 2.4), action_status=4,
              amcl_offset=(0.0, 0.0), belief_pose=None,
              params='config/nav2_n_probe.yaml',
              publisher_type='geometry_msgs/msg/TwistStamped', hop_counts=None,
              arrival_pose=None, scan_offset=0.0):
    """A complete, self-consistent run directory; individual fields can be broken on purpose."""
    out.mkdir(parents=True, exist_ok=True)
    samples = _walk(waypoints)
    if arrival_pose is not None:
        final = samples[-1]
        delta = (arrival_pose[0] - final[1], arrival_pose[1] - final[2])
        for row in samples[-13:]:
            row[1] += delta[0]
            row[2] += delta[1]
    travelled = evaluate_n_nav.path_length(samples)
    goal = {'frame_id': 'map', 'x': goal[0], 'y': goal[1], 'yaw': 0.0}
    map_path = MAP_YAML.relative_to(ROOT)
    plan = {'run_id': out.name, 'goal': goal, 'map': {'yaml': str(map_path)}}
    if params is not None:
        # A real plan names the params file the goal checker's tolerance lives in, and
        # the judge derives the tolerance from THAT file rather than from a restatement.
        plan['params'] = {'path': str(params)}
    (out / 'nav_plan.json').write_text(json.dumps(plan))
    scan_stamp = samples[0][0]
    built = make_map_n.read_map(MAP_YAML)
    nrays = 181
    angles = [math.radians(-90.0 + index * 180.0 / (nrays - 1)) for index in range(nrays)]
    reference = evaluate_n_nav.truth_at(samples, scan_stamp)
    ranges = make_map_n.raycast_grid(
        built, (reference[0] + 0.02, reference[1], reference[2]), angles, 8.0)

    (out / 'nav_report.json').write_text(json.dumps({
        'run_id': out.name, 'status': 'GOAL_SUCCEEDED',
        'lifecycle': {'states': {name: 'active' for name in (
            'map_server', 'amcl', 'planner_server', 'controller_server', 'bt_navigator')},
            'managers': {'lifecycle_manager_localization': ['map_server', 'amcl'],
                         'lifecycle_manager_navigation': ['planner_server', 'controller_server',
                                                          'bt_navigator']},
            'manager_node_names': ['map_server', 'amcl', 'planner_server', 'controller_server',
                                   'bt_navigator']},
        'action': {'accepted': True, 'status': action_status, 'error_code': 0,
                   'sent_sim_s': samples[0][0], 'result_sim_s': samples[-13][0],
                   'feedback_count': 100},
        'leftovers': [],
    }))
    (out / 'sim_report.json').write_text(json.dumps({
        'status': 'COMPLETED', 'stop_reason': 'SIGUSR1', 'sim_seconds': samples[-1][0],
        'truth_samples': samples,
        'truth': {'final': samples[-1][1:], 'travelled_m': round(travelled, 4)},
        'ipc': {'states_sent': len(samples)},
        'gate': {'accepted': 300, 'refused': 0, 'reasons': {}, 'mode': 'SILENT', 'reason':
                 'SILENT_SINCE_LAST_COMMAND', 'modes': {'HOLDING': 8000, 'SILENT': 9000}},
    }))
    (out / 'ros_report.json').write_text(json.dumps({
        'status': 'STATE_STREAM_STOPPED_EARLY', 'sim_seconds': samples[-1][0], 'seq_gaps': 0,
        'counters': {'states_received': len(samples), 'decode_refusals': 0},
        'odom_final': {'x': samples[-1][1] + 0.05, 'y': samples[-1][2] - 0.03,
                       'yaw': samples[-1][3]},
        'cmd_vel': {'received': 277, 'clipped': 0, 'type': 'twist_stamped',
                    'publisher_types_seen': [publisher_type],
                    'type_mismatch': publisher_type != 'geometry_msgs/msg/TwistStamped'},
    }))
    counts = {'clock': 700, 'scan': 700, 'odom': 700, 'tf': 1200, 'tf_static': 1, 'map': 1,
              'amcl_pose': len(samples), 'cmd_vel': 277, 'cmd_vel_nav': 267,
              'cmd_vel_smoothed': 277}
    if hop_counts:
        counts.update(hop_counts)
    types = {topic: [{'type': publisher_type, 'reliability': '1', 'durability': '2',
                      'depth': 10}] for topic in ('cmd_vel_nav', 'cmd_vel_smoothed', 'cmd_vel')}
    # The final sample is always included: the last amcl_pose of a run is the pose the
    # vehicle came to rest believing, and without it the fixture cannot express where
    # nav2 thinks it stopped.
    indices = sorted(set(list(range(0, len(samples), 40)) + [len(samples) - 1]))
    amcl_samples = [[samples[index][0], samples[index][1] + amcl_offset[0],
                     samples[index][2] + amcl_offset[1], samples[index][3]]
                    for index in indices]
    if belief_pose is not None:
        amcl_samples[-1][1], amcl_samples[-1][2] = belief_pose
    (out / 'checker_nav_raw.json').write_text(json.dumps({
        'run_id': out.name, 'counts': counts,
        'command_topic_type_assumed': 'geometry_msgs/msg/TwistStamped',
        'observed': {
            'publisher_nodes': {'clock': ['workcell010_n_bridge'],
                                'tf': ['workcell010_n_bridge', 'amcl'],
                                'cmd_vel': ['collision_monitor']},
            'publisher_qos': types,
            'tf_edges': [['odom', 'base_link'], ['map', 'odom']],
            'tf_static_edges': [['base_link', 'lidar_link']],
            'map_info': {'frame_id': 'map', 'width': built['width'], 'height': built['height'],
                         'resolution': built['resolution'],
                         'origin': [built['origin'][0], built['origin'][1]],
                         'occupied': sum(1 for value in built['grid']
                                         if value == make_map_n.OCCUPIED),
                         'free': sum(1 for value in built['grid'] if value == make_map_n.FREE),
                         'unknown': 0},
            'scan_first': [value + scan_offset for value in ranges],
            'scan_first_stamp': scan_stamp,
            'amcl_samples': amcl_samples,
        },
    }))
    return out


def run_judge(out):
    import subprocess
    script = ROOT / 'experiments' / 'evaluate_n_nav.py'
    completed = subprocess.run([sys.executable, str(script), '--run-id', out.name,
                                '--reports-dir', str(out.parent)],
                               capture_output=True, text=True)
    result = json.loads((out / 'acceptance.json').read_text())
    return completed.returncode, result


def judge(tmp_path, name, **kwargs):
    return run_judge(build_run(tmp_path / name, **kwargs))


def rows_of(result):
    return {item['check']: item['verdict'] for item in result['checks']}


ARRIVAL = 'the vehicle arrived within tolerance of the goal, measured against truth'
BELIEF = 'nav2 stopped inside its own declared tolerance of the goal it believes in'


def params_with(tmp_path, name, xy):
    """A copy of the real params file with ONE number changed, so the test can show that the
    judge follows the file rather than a constant of its own."""
    doc = yaml.safe_load((ROOT / 'config' / 'nav2_n_probe.yaml').read_text(encoding='utf-8'))
    node = doc['controller_server']['ros__parameters']
    node[node['goal_checker_plugins'][0]]['xy_goal_tolerance'] = xy
    written = tmp_path / name
    written.write_text(yaml.safe_dump(doc, sort_keys=False), encoding='utf-8')
    return written


def test_a_clean_synthetic_run_passes(tmp_path):
    code, result = judge(tmp_path, 'synthetic-pass')
    assert result['verdict'] == 'PASS', result['failed'] + result['not_run']
    assert code == 0


def test_the_judge_fails_when_nav2_says_success_but_the_vehicle_stopped_short(tmp_path):
    """The vehicle stopped 0.43 m short while nav2 reported success. This fixture moves the last
    stretch of the truth AND the belief that is sampled from it, so both rows fail -- it is the
    coarse case. `test_the_judge_separates_...` below is the shape the real runs have, where the
    belief is good and only the truth is out."""
    code, result = judge(tmp_path, 'short-arrival', arrival_pose=(2.231, 2.178))
    assert result['verdict'] == 'FAIL'
    assert 'the vehicle arrived within tolerance of the goal, measured against truth' in \
        result['failed']
    assert code == 1


def test_the_judge_fails_a_straight_line_route_that_hits_the_pillar(tmp_path):
    """The goal is placed so the straight line passes 0.2535 m from pillar_a, whose radius plus
    the body radius is 0.38 m. A run that drives straight through must fail both the detour and
    the clearance part of the check."""
    code, result = judge(tmp_path, 'through-the-pillar', waypoints=[STRAIGHT_LINE_GOAL])
    assert result['verdict'] == 'FAIL'
    assert 'the route went around the obstacle on it rather than through it' \
        in result['failed'], result
    assert code == 1


def test_the_judge_fails_when_amcl_is_metres_out(tmp_path):
    code, result = judge(tmp_path, 'amcl-off', amcl_offset=(0.25, 0.20))
    assert result['verdict'] == 'FAIL'
    assert 'AMCL agrees with truth at the same simulator time' in result['failed']


def test_the_judge_fails_when_the_scan_disagrees_with_the_map(tmp_path):
    """This is the check a wrong map origin, a wrong resolution or a vertically flipped image
    fails, and it is the only one that compares the map with what the robot actually saw."""
    code, result = judge(tmp_path, 'scan-vs-map', scan_offset=0.5)
    assert result['verdict'] == 'FAIL'
    assert "the static map agrees with the robot's first scan" in result['failed']
    assert code == 1


def test_the_judge_fails_a_wrong_command_message_type(tmp_path):
    """The silent failure of this round: a Twist subscriber against a TwistStamped publisher
    receives nothing and DDS reports no error at all."""
    code, result = judge(tmp_path, 'wrong-type',
                         publisher_type='geometry_msgs/msg/Twist')
    assert result['verdict'] == 'FAIL'
    assert 'every command topic carries the declared message type' in result['failed']
    assert 'the bridge saw no type mismatch' in result['failed']


def test_the_judge_names_the_silent_hop(tmp_path):
    code, result = judge(tmp_path, 'silent-hop', hop_counts={'cmd_vel_smoothed': 0, 'cmd_vel': 0})
    assert result['verdict'] == 'FAIL'
    failed = result['failed']
    assert 'every hop of the command chain carried traffic' in failed
    detail = next(item['detail'] for item in result['checks']
                  if item['check'] == 'every hop of the command chain carried traffic')
    assert "'cmd_vel_smoothed': 0" in detail


def test_the_judge_reports_missing_input_as_failure_not_silence(tmp_path):
    out = tmp_path / 'empty'
    out.mkdir()
    (out / 'nav_plan.json').write_text(json.dumps({'goal': {'x': 0, 'y': 0, 'yaw': 0}}))
    import subprocess
    completed = subprocess.run([sys.executable, str(ROOT / 'experiments' / 'evaluate_n_nav.py'),
                                '--run-id', 'empty', '--reports-dir', str(tmp_path)],
                               capture_output=True, text=True)
    assert completed.returncode == 1
    result = json.loads((out / 'acceptance.json').read_text())
    assert result['verdict'] == 'FAIL'
    assert 'sim_report exists' in result['failed']


def test_the_judge_survives_an_empty_directory(tmp_path):
    (tmp_path / 'nothing').mkdir()
    import subprocess
    completed = subprocess.run([sys.executable, str(ROOT / 'experiments' / 'evaluate_n_nav.py'),
                                '--run-id', 'nothing', '--reports-dir', str(tmp_path)],
                               capture_output=True, text=True)
    assert completed.returncode == 1
    assert 'judge_error' not in completed.stdout


def test_truth_interpolation_and_angle_wrapping():
    samples = [[0.0, 0.0, 0.0, 0.0], [1.0, 1.0, 0.0, 0.0], [2.0, 2.0, 0.0, 3.0]]
    assert evaluate_n_nav.truth_at(samples, 0.5)[0] == pytest.approx(0.5)
    assert evaluate_n_nav.truth_at(samples, 1.5)[0] == pytest.approx(1.5)
    # 0 -> 3.0 rad is a +3.0 step on the circle (3.0 < pi), so halfway is +1.5
    assert evaluate_n_nav.truth_at(samples, 1.5)[2] == pytest.approx(1.5)
    # and a step that really does need wrapping stays on the short arc: 3.0 -> -3.0 is only
    # 0.283 rad the short way round, not 6.0 the long way
    wrapping = [[0.0, 0.0, 0.0, 3.0], [1.0, 0.0, 0.0, -3.0]]
    middle = evaluate_n_nav.truth_at(wrapping, 0.5)[2]
    assert evaluate_n_nav.wrapped(middle - 3.0) <= 0.20
    assert evaluate_n_nav.wrapped(5.45 - (-0.98)) == pytest.approx(0.1468, abs=0.001)


# ---------------------------------------------------------------- the bridge patches

def test_the_bridge_defaults_still_reproduce_p1_n_ipc():
    """The two additions must be opt-in, or the P1-N-06 verdict stops being reproducible from
    these files."""
    text = (ROOT / 'experiments' / 'probe_n_ros.py').read_text(encoding='utf-8')
    assert "add_argument('--command-source', default='profile'" in text
    assert "add_argument('--cmd-vel-type', default='twist_stamped'" in text
    assert "TwistStamped if args.cmd_vel_type == 'twist_stamped' else Twist" in text
    assert 'type_mismatch' in text
    sim = (ROOT / 'experiments' / 'probe_n_sim.py').read_text(encoding='utf-8')
    assert "add_argument('--allow-stop-signal', action='store_true'" in sim
    assert "signal.signal(signal.SIGUSR1, _on_stop)" in sim
    assert "report['stop_reason']" in sim


def test_the_probe_reports_a_type_mismatch_instead_of_silence():
    """The check that would have turned four runs into one: with the wrong message class the
    count stays zero and nothing says why."""
    text = (ROOT / 'experiments' / 'probe_n_ros.py').read_text(encoding='utf-8')
    assert "get_publishers_info_by_topic('/cmd_vel')" in text
    assert "cmd_vel['type_mismatch'] = expected_type not in seen" in text


# ---------------------------------------------------------------- the arena, DERIVED

B_PILLAR_GOAL = (-2.6, -2.6)
AROUND_B = [(-2.6, 0.2), (-2.6, -1.2), (-2.6, -2.6)]
CLEAR_GOAL = (3.0, 0.0)
ALONG_X = [(1.5, 0.0), (3.0, 0.0)]


def test_the_derived_arena_features_reproduce_the_arena():
    """The values that used to be typed: PILLAR_A radius, the wall face and the robot radius.

    Deriving them is only honest if the derived numbers are the ones the typed constants held --
    otherwise the change is not a refactor but a silent re-parameterisation of every check that
    uses them.
    """
    features = evaluate_n_nav.arena_features(WORLD, MAP_YAML)
    by_name = {o['name']: o for o in features['obstacles']}
    assert set(by_name) == {'pillar_a', 'pillar_b', 'pillar_c'}
    assert by_name['pillar_a']['kind'] == 'circle'
    assert abs(by_name['pillar_a']['radius'] - 0.15) < 1e-12       # was PILLAR_A[2]
    assert by_name['pillar_c']['kind'] == 'circle'
    assert abs(by_name['pillar_c']['radius'] - 0.20) < 1e-12
    # pillar_b is a BOX and must not be turned into a circle: a circle round it would demand
    # 0.70 m of centre clearance to pass a flat side that needs 0.28 m
    assert by_name['pillar_b']['kind'] == 'rect'
    assert [round(v, 6) for v in by_name['pillar_b']['half']] == [0.3, 0.3]
    assert abs(features['wall_inner_face'] - 3.95) < 1e-12        # was WALL_INNER_FACE
    assert abs(features['robot_radius'] - 0.2302172887) < 1e-9    # was ROBOT_CIRCUMSCRIBED_RADIUS
    assert features['robot_geom'] == 'chassis'


def test_clearance_is_measured_to_the_footprint_not_to_a_circle():
    """A point beside a box's flat side is 0.30 m away, not 0.4243 m."""
    box = {'name': 'b', 'x': 0.0, 'y': 0.0, 'kind': 'rect', 'half': [0.3, 0.3]}
    assert abs(evaluate_n_nav.obstacle_footprint_distance(box, 0.6, 0.0) - 0.3) < 1e-12
    assert abs(evaluate_n_nav.obstacle_footprint_distance(box, 0.6, 0.6)
               - (0.3 ** 2 + 0.3 ** 2) ** 0.5) < 1e-12          # measured to the corner
    assert evaluate_n_nav.obstacle_footprint_distance(box, 0.1, 0.1) == 0.0   # inside
    circle = {'name': 'c', 'x': 0.0, 'y': 0.0, 'kind': 'circle', 'radius': 0.15}
    assert abs(evaluate_n_nav.obstacle_footprint_distance(circle, 1.0, 0.0) - 0.85) < 1e-12


def test_a_different_goal_is_judged_against_a_different_obstacle(tmp_path):
    """The check used to be typed to pillar_a, so this goal FAILED for being far from it."""
    code, result = judge(tmp_path, 'around-pillar-b', waypoints=AROUND_B, goal=B_PILLAR_GOAL)
    detail = ' '.join(item['detail'] for item in result['checks']
                      if item['check'] == 'the route went around the obstacle on it rather than through it')
    assert 'pillar_b' in detail, detail
    assert 'pillar_a' not in detail.split('All:')[0], detail
    assert result['verdict'] == 'PASS', result['failed'] + result['not_run']
    assert code == 0


def test_a_route_with_nothing_to_avoid_is_not_run_rather_than_pass(tmp_path):
    """NOT_RUN is not PASS: this route says nothing about planning around an obstacle."""
    code, result = judge(tmp_path, 'no-obstacle', waypoints=ALONG_X, goal=CLEAR_GOAL)
    item = next(item for item in result['checks']
                if item['check'] == 'the route went around the obstacle on it rather than through it')
    assert item['verdict'] == 'NOT_RUN', item
    assert 'no obstacle to go around' in item['detail']
    # and a NOT_RUN must not be promoted to a pass at the top level
    assert result['verdict'] == 'INCOMPLETE', result['verdict']
    assert code == 1


def test_the_judge_separates_a_localisation_error_from_a_stopping_error(tmp_path):
    """The shape 8 of 8 real runs have: nav2 finished its goal while believing it stood on it,
    and the truth is 0.43 m out. The arrival row must FAIL and the belief row must PASS.

    They are two different questions -- "is it where it should be" and "did it stop where it
    thinks it should be" -- and answering both with one number is what made the old row
    uninterpretable: it reported the same figure twice.
    """
    code, result = judge(tmp_path, 'belief-good-truth-bad', arrival_pose=(2.231, 2.178),
                         belief_pose=(2.6, 2.4))
    seen = rows_of(result)
    assert seen[ARRIVAL] == 'FAIL'
    assert seen[BELIEF] == 'PASS'
    assert code == 1
    detail = next(c['detail'] for c in result['checks'] if c['check'] == ARRIVAL)
    assert 'LOCALISATION' in detail
    assert 'the pose nav2 BELIEVES in' in detail
    # `NEVER` is NOT asserted: this fixture's walk passes through the tolerance at sim
    # 5.80 s and then the injected shift carries the truth away, which is not the shape a
    # real run has. The eight real runs all read NEVER; that is reported, not asserted.


def test_the_judge_fails_when_nav2_stops_outside_its_own_tolerance(tmp_path):
    """`p1-n-nav-10` in miniature: the truth is inside the round's 0.25 m limit, so the arrival
    row passes -- and nav2 itself believed it was 0.189 m from the goal against a 0.15 m
    contract. A run that comes to rest outside the tolerance it applied did not arrive, however
    good the truth looks."""
    code, result = judge(tmp_path, 'belief-outside', belief_pose=(2.45, 2.285))
    seen = rows_of(result)
    assert seen[ARRIVAL] == 'PASS'
    assert seen[BELIEF] == 'FAIL'
    assert BELIEF in result['failed']
    assert code == 1


def test_the_tolerance_is_read_from_the_params_file_the_run_named(tmp_path):
    """Two params files that differ in ONE value, one synthetic body. The verdict must follow the
    file: the same 0.189 m stop passes against a 0.30 m contract and fails against a 0.10 m one.
    This is the regression for the typed constant that omitted `stateful`."""
    wide = params_with(tmp_path, 'params_wide.yaml', 0.30)
    tight = params_with(tmp_path, 'params_tight.yaml', 0.10)
    _code, loose = judge(tmp_path, 'tolerance-wide', belief_pose=(2.45, 2.285), params=wide)
    _code2, firm = judge(tmp_path, 'tolerance-tight', belief_pose=(2.45, 2.285), params=tight)
    assert rows_of(loose)[BELIEF] == 'PASS'
    assert rows_of(firm)[BELIEF] == 'FAIL'
    detail = next(c['detail'] for c in firm['checks'] if c['check'] == BELIEF)
    assert '0.1 m declared by' in detail


def test_a_plan_that_names_no_params_file_is_not_run_not_passed(tmp_path):
    """A check whose input is missing is NOT_RUN, never PASS -- the judge's own stated rule."""
    out = build_run(tmp_path / 'no-params')
    plan = json.loads((out / 'nav_plan.json').read_text())
    plan.pop('params', None)
    (out / 'nav_plan.json').write_text(json.dumps(plan))
    _code, result = run_judge(out)
    assert rows_of(result)[BELIEF] == 'NOT_RUN'
    assert BELIEF in result['not_run']
    assert result['verdict'] == 'INCOMPLETE'


def test_the_plan_does_not_restate_the_tolerance_as_typed_constants():
    """The block that omitted `stateful` was `'goal_tolerance': {'xy_m': 0.15, ...}` in the
    orchestrator, and `stateful` is the key that changes what 0.15 MEANS. One number, one place."""
    text = (ROOT / 'experiments' / 'probe_n_nav.py').read_text(encoding='utf-8')
    assert "'goal_tolerance':" not in text
    assert 'goal_tolerance_source' in text
