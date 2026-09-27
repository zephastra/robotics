"""P3-NAV-02: judge the P3 navigation rounds -- bare and loaded -- from their own reports.

WHY THIS IS A SEPARATE JUDGE FROM `evaluate_n_nav.py`
----------------------------------------------------
`evaluate_n_nav.py` is P1's judge and it is anchored to P1's arena: it has `PILLAR_A`,
`WALL_INNER_FACE = 3.95` and a map/scan comparison against an 8x8 m box with three obstacles,
all as module constants. Pointing it at the P3 arena would produce rows about a wall that is not
there and a pillar that does not exist, and they would read `PASS` for the wrong reason. So the
P3 rows are derived here, from the arena's own frozen layout, and the P1 judge is left to the P1
round it was written for.

WHAT IT JUDGES, AND WHAT IT CANNOT
----------------------------------
  * It judges the ARRIVAL: Nav2's own goal result, and separately the TRUTH distance from the
    final pose to the goal. The truth is the chassis qpos recorded by the simulator, which never
    goes on the wire (P1-N's state datagram has no pose field), so this is an independent number.
  * It CANNOT judge the arrival YAW from truth: `sim_report.truth` records position only. The yaw
    row therefore reports AMCL's own estimate and says so. A yaw claim from an estimate is not
    the same kind of claim as a yaw claim from truth, and the difference is written into the row.
  * It is NOT a docking result for the assembled cell. The arena is a declared stand-in: the
    room is the judged world's room, but H, A, C and the second AMR are absent (their footprints
    are obstacles), and the loaded round carries the V1 tray at a derived stand-in mount.
"""
import argparse
import hashlib
import json
import math
import pathlib
import sys

ROOT = pathlib.Path(__file__).resolve().parents[1]
LAYOUT = ROOT / 'assets' / 'worlds' / 'world_p3_nav.layout.json'
MAP_YAML = ROOT / 'assets' / 'maps' / 'p3_nav.yaml'
MAP_PGM = ROOT / 'assets' / 'maps' / 'p3_nav.pgm'
FREEZE = ROOT / 'config' / 'p3_freeze.json'

#: Declared, and frozen by `make_p3_freeze.py`. 0.25 m is P1-N's arrival limit, reused so the two
#: rounds are quotable against the same bar rather than against a number chosen after the fact.
ARRIVAL_LIMIT_M = 0.25
#: The observed sim clock must reach at least this fraction of what the simulator stepped. Below
#: it, the ROS side did not keep up and the round says nothing about navigation.
CLOCK_TRACKING_MIN = 0.90

#: The acceptance PROFILE: which checks a PASS of this task requires, and which are declared NOT
#: APPLICABLE in advance rather than discovered to be missing. Both lists are needed, and the
#: difference between them is the whole point:
#:
#:   * a REQUIRED row that did not run makes the round INCOMPLETE -- it cannot pass;
#:   * a row declared NOT APPLICABLE narrows the claim and is reported, never counted as evidence.
#:
#: The aggregator this replaces dropped every NOT_RUN row from the denominator
#: (`required = [r for r in self.rows if r['status'] != 'NOT_RUN']`), so a required check that
#: never ran could not stop a PASS. Measured 2026-09-25: `reports/p3-nav-05` was 8 checks /
#: 7 PASS / 1 NOT_RUN and its verdict was PASS, the NOT_RUN being the arrival yaw. The row said the
#: yaw claim could not be made and the aggregate said the round passed: **the aggregate erased the
#: row.** The other judge in this repository (`evaluate_n_nav.write`) already returns INCOMPLETE for
#: the same situation, so the two disagreed about the same kind of evidence.
#:
#: The yaw entry is the shape the next phase's guidance asks for: the arena's lack of yaw truth is a
#: property of the ARENA, declared here before any run, not a number that went missing and got
#: demoted to optional afterwards.
PROFILE = {
    'id': 'P3-NAV-02',
    'version': 2,
    'required': (
        'the run declares which physical variant it is, and it checks out',
        'the round ran on the frozen configuration and the frozen arena',
        'the plant is paced below real time, and the ROS side kept up with it',
        'Nav2 reported the goal reached',
        'the truth arrival error is within the limit',
        'the loaded/unloaded dock error is measured against a declared dock pose',
        'the goal cell is reachable free space in the frozen map',
        'each falsifiability probe still drives its own check out of PASS',
    ),
    'not_applicable': {
        'an independent arrival YAW is available':
            'declared in advance, not discovered missing: this arena has no independent yaw truth '
            '(`sim_report.truth` records POSITION ONLY), so the profile narrows its claim to '
            'position and carries no yaw result at all',
    },
}
STATUSES = ('PASS', 'FAIL', 'NOT_RUN', 'NOT_APPLICABLE')


def verdict_of(rows, profile):
    """PASS / FAIL / INCOMPLETE / CONFIG_ERROR, over the profile's DECLARED required set.

    INCOMPLETE is the state the old aggregator could not express. CONFIG_ERROR covers a profile and
    a judge that disagree about what is judged -- an undeclared row, a duplicate id, an empty
    required set, a status value nobody defined. None of those is evidence of anything, so none of
    them may yield PASS.
    """
    names = [row['name'] for row in rows]
    problems = []
    if not profile['required']:
        problems.append('the profile declares no required checks, so PASS would mean nothing')
    duplicates = sorted({name for name in names if names.count(name) > 1})
    if duplicates:
        problems.append(f'duplicate check ids: {duplicates}')
    unknown = sorted({row['status'] for row in rows} - set(STATUSES))
    if unknown:
        problems.append(f'status value(s) this profile does not define: {unknown}')
    declared = set(profile['required']) | set(profile['not_applicable'])
    undeclared = sorted(set(names) - declared)
    if undeclared:
        problems.append(f'rows emitted that the profile does not declare: {undeclared}')
    never_emitted = sorted(set(profile['required']) - set(names))
    if never_emitted:
        problems.append(f'required checks that were never emitted: {never_emitted}')
    if problems:
        return 'CONFIG_ERROR', problems
    by_name = {row['name']: row for row in rows}
    failed = [name for name in profile['required'] if by_name[name]['status'] == 'FAIL']
    if failed:
        return 'FAIL', failed
    unfinished = [name for name in profile['required'] if by_name[name]['status'] != 'PASS']
    if unfinished:
        return 'INCOMPLETE', unfinished
    return 'PASS', []


class Results:
    def __init__(self):
        self.rows = []

    def add(self, name, status, detail):
        self.rows.append({'name': name, 'status': status, 'detail': detail})

    # There is deliberately no `verdict()` here any more. The verdict is a function of the PROFILE
    # (`verdict_of`), not of whatever rows happen to be present: the method that used to live here
    # filtered NOT_RUN out of the denominator and so could not see a missing required check.
    def counts(self):
        return {'checks': len(self.rows),
                'pass': sum(1 for r in self.rows if r['status'] == 'PASS'),
                'fail': sum(1 for r in self.rows if r['status'] == 'FAIL'),
                'not_run': sum(1 for r in self.rows if r['status'] == 'NOT_RUN'),
                'not_applicable': sum(1 for r in self.rows
                                      if r['status'] == 'NOT_APPLICABLE')}


def read_map_cell(map_yaml, map_pgm, x, y):
    """The occupancy at world (x, y), from the generated P5 map. Comments are skipped properly:
    an earlier reader of this same file split on the first three newlines and got the data offset
    wrong, which made the start and the goal both read as occupied."""
    import yaml
    meta = yaml.safe_load(map_yaml.read_text(encoding='utf-8'))
    raw = map_pgm.read_bytes()
    pos, toks = 0, []
    while len(toks) < 4:
        while pos < len(raw) and raw[pos:pos + 1].isspace():
            pos += 1
        if raw[pos:pos + 1] == b'#':
            pos = raw.index(b'\n', pos) + 1
            continue
        end = pos
        while end < len(raw) and not raw[end:end + 1].isspace():
            end += 1
        toks.append(raw[pos:end])
        pos = end
    pos += 1
    w, h = int(toks[1]), int(toks[2])
    data = raw[pos:]
    ox, oy, _ = meta['origin']
    res = float(meta['resolution'])
    col = int((x - ox) / res)
    row = h - 1 - int((y - oy) / res)
    if not (0 <= row < h and 0 <= col < w):
        return None
    return int(data[row * w + col])


def wrapped(a):
    return math.atan2(math.sin(a), math.cos(a))


def load(path):
    return json.loads(path.read_text(encoding='utf-8'))


def sha256_of(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def variant_registry(layout):
    """The layout's own variant table: variant -> the world basename and content hash it declares.

    This is what the judge should have used from the start. The line it replaces was
    `use_loaded = 'loaded' in args.run_id`, which asked the RUN ID what the physics was; the layout
    already states it, by content hash, and two variants cannot collide.
    """
    return {name: {'xml': block['xml'], 'xml_sha256': block.get('xml_sha256'), 'block': block}
            for name, block in layout.items()
            if isinstance(block, dict) and 'xml' in block}


def _refused(code, detail):
    return {'arena': None, 'variant': None, 'status': 'FAIL', 'code': code,
            'detail': f'{code}: {detail}'}


def resolve_identity(run_id, plan, sim, layout):
    """Which physical variant a run was, decided from the run's own record and NOT from its id.

    The chain the guidance asks for, in its order:
        the run's own config record  ->  the world path and its CONTENT hash  ->  the versioned
        scenario layout  ->  the load entity and how it was MODELLED.
    The last step is why the load paragraph is in the row: `loaded` in this layout is a rigid
    stand-in mount (tray_v1 at z 0.240 m, reason "raised clear of the lidar"), not a tray resting on
    the deck rollers, so no retention or settle result can be read off the loaded round.

    Measured 2026-09-25, which is why this exists: `p3-nav-06`'s records name
    `config/p3_nav_loaded.yaml` and `assets/worlds/world_p3_nav_loaded.xml` (sha256 28256307...)
    while its gate said `variant: bare` and anchored every geometric row to the bare arena.
    """
    registry = variant_registry(layout)
    plan_cfg = (plan or {}).get('sim_config')
    plan_cfg_sha = (plan or {}).get('sim_config_sha256')
    sim_cfg = sim.get('config_path')
    sim_cfg_sha = sim.get('config_sha256')
    world_path = sim.get('world')
    world_sha = sim.get('world_sha256')

    missing = []
    if not plan:
        missing.append('nav_plan.json, where the run declared its scenario before running')
    if not world_path or not world_sha:
        missing.append('sim_report.world / sim_report.world_sha256')
    if not sim_cfg or not sim_cfg_sha:
        missing.append('sim_report.config_path / sim_report.config_sha256')
    if missing:
        return _refused('EVIDENCE_MISSING',
                        '; '.join(missing) + ". The variant is NOT defaulted: a default is the same "
                        "guess as reading the run id, only quieter")

    disk = ROOT / world_path
    if not disk.is_file():
        return _refused('EVIDENCE_MISSING',
                        f'the run names {world_path}, which is not on disk, so the content hash it '
                        f'recorded ({world_sha[:12]}) cannot be checked')
    disk_sha = sha256_of(disk)
    if disk_sha != world_sha:
        return _refused('EVIDENCE_STALE',
                        f'the run recorded {world_path} sha256 {world_sha[:12]} but the file on '
                        f'disk is {disk_sha[:12]}: this run is about a world that is no longer the '
                        f'one on disk')

    hits = [name for name, entry in registry.items() if entry['xml_sha256'] == world_sha]
    if not hits:
        return _refused('EVIDENCE_UNREGISTERED',
                        f'no variant in {LAYOUT.name} declares world sha256 {world_sha[:12]} '
                        f'({world_path}); a variant the layout does not know is refused rather than '
                        f'judged against a default')

    conflicts = []
    if len(hits) > 1:
        conflicts.append(f'{len(hits)} variants declare this same world hash: {hits}')
    if plan_cfg and plan_cfg_sha and (plan_cfg != sim_cfg or plan_cfg_sha != sim_cfg_sha):
        conflicts.append(f'nav_plan declares {plan_cfg} sha {plan_cfg_sha[:12]} while the simulator '
                         f'report says it read {sim_cfg} sha {sim_cfg_sha[:12]}')
    if conflicts:
        return _refused('EVIDENCE_CONFLICT',
                        '; '.join(conflicts) + f'. The run id {run_id!r} is deliberately not '
                        f'consulted to break the tie: that would be the guess again')

    variant = hits[0]
    block = registry[variant]['block']
    load = block.get('load')
    declared_name = registry[variant]['xml']
    # The PATH is reported, never enforced. Identity is the content hash: a copy of this world under
    # another name is the same world, which is the normal case when evidence is copied into a new
    # id. Refusing on the basename would block exactly the re-judge the guidance asks for.
    path_note = ''
    if pathlib.Path(world_path).name != declared_name:
        path_note = (f". The record names the world {pathlib.Path(world_path).name!r} while "
                     f"{variant!r} declares {declared_name!r}; recorded rather than refused, "
                     f"because identity is the content hash and a copy under another name is the "
                     f"same world")
    if load:
        modelled = (f"the load is a DECLARED STAND-IN RIGID MOUNT: {load['tray_source']} (sha256 "
                    f"{str(load.get('tray_source_sha256'))[:12]}) at z {load['mount_z_m']} m, reason "
                    f"{load.get('mount_reason')!r} -- not a tray resting on the deck rollers, so no "
                    f"retention or settle result may be read off this round")
    else:
        modelled = 'no load is modelled in this variant'
    return {'arena': block, 'variant': variant, 'status': 'PASS', 'code': None,
            'detail': f'variant {variant!r} DERIVED FROM CONTENT, not from the run id: '
                      f'{world_path} sha256 {world_sha[:12]} is the hash {LAYOUT.name} declares for '
                      f'{variant!r}; the plan and the simulator agree on config {sim_cfg} (sha256 '
                      f'{sim_cfg_sha[:12]}); {modelled}{path_note}'}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--run-id', required=True)
    ap.add_argument('--reports-dir', default=None)
    args = ap.parse_args()

    reports = pathlib.Path(args.reports_dir) if args.reports_dir else ROOT / 'reports'
    run = reports / args.run_id
    res = Results()
    doc = load(LAYOUT)

    nav_path, sim_path = run / 'nav_report.json', run / 'sim_report.json'
    missing = [p.name for p in (nav_path, sim_path) if not p.is_file()]
    if missing:
        print(f'[FAIL] {args.run_id}: missing {missing}')
        return 1
    nav, sim = load(nav_path), load(sim_path)
    plan_path = run / 'nav_plan.json'
    plan = load(plan_path) if plan_path.is_file() else None

    identity = resolve_identity(args.run_id, plan, sim, doc)
    res.add('the run declares which physical variant it is, and it checks out',
            identity['status'], identity['detail'])
    arena, variant = identity['arena'], identity['variant']
    if arena is None:
        # Every geometric row below reads `arena`, so there is nothing to measure and refusing is
        # the point: the previous version fell back to the bare variant, which is exactly how the
        # loaded run came to be judged against the unloaded arena.
        return emit(args.run_id, run, res, identity, [], None, None)

    # ---- 1. the round ran on the frozen artefact, not on something that drifted ----
    frozen = load(FREEZE) if FREEZE.is_file() else None
    cfg = [a for a in (frozen or {}).get('artefacts', []) if a['path'].startswith('config/p3_nav')]
    want_cfg = {a['path']: a['sha256'] for a in cfg}
    got_cfg = sim.get('config_sha256')
    want_world = [a['sha256'] for a in (frozen or {}).get('artefacts', [])
                  if a['path'].endswith(arena['xml']) ]
    ident_bad = []
    if got_cfg is None or not want_cfg:
        ident_bad.append('no config hash to compare')
    elif got_cfg not in want_cfg.values():
        ident_bad.append(f"sim config sha256 {got_cfg[:12]} is not a frozen p3_nav config")
    if not want_world:
        ident_bad.append(f"{arena['xml']} is not in the freeze")
    res.add('the round ran on the frozen configuration and the frozen arena',
            'FAIL' if ident_bad else 'PASS',
            '; '.join(ident_bad) if ident_bad else
            f"config sha256 {str(got_cfg)[:12]} is in the freeze; {arena['xml']} sha256 "
            f"{arena['xml_sha256'][:12]} is in the freeze")

    # ---- 2. the plant was paced below real time and the ROS side kept up ----
    ratio = sim.get('achieved_realtime_ratio')
    factor = sim.get('realtime_factor')
    stepped = float(sim.get('sim_seconds') or 0.0)
    observed = float((nav.get('observed') or {}).get('clock') or 0.0)
    tracking = observed / stepped if stepped else 0.0
    pace_bad = []
    if factor is None or factor >= 1.0:
        pace_bad.append(f'realtime_factor {factor} is not below 1.0')
    if ratio is None or abs(float(ratio) - float(factor)) > 0.10:
        pace_bad.append(f'achieved ratio {ratio} does not match the {factor} target')
    if tracking < CLOCK_TRACKING_MIN:
        pace_bad.append(f'the observed clock reached only {tracking * 100:.1f}% of the '
                        f'{stepped:.3f} sim seconds the plant stepped')
    res.add('the plant is paced below real time, and the ROS side kept up with it',
            'FAIL' if pace_bad else 'PASS',
            '; '.join(pace_bad) if pace_bad else
            f'realtime_factor {factor}, achieved {ratio}, observed clock {observed:.3f} of '
            f'{stepped:.3f} sim s ({tracking * 100:.1f}%)')

    # ---- 3. Nav2 itself reported the goal reached ----
    status = nav.get('status')
    res.add('Nav2 reported the goal reached',
            'PASS' if status == 'GOAL_SUCCEEDED' else 'FAIL',
            f"orchestrator status {status!r}; action result "
            f"{(nav.get('goal') or {}).get('action_status')!r}")

    # ---- 4. the TRUTH distance from the final pose to the goal ----
    goal_x, goal_y = float(arena['goal'][0]), float(arena['goal'][1])
    truth_final = sim.get('truth', {}).get('final')
    if not truth_final:
        res.add('the truth arrival error is within the limit', 'NOT_RUN',
                'the simulator report carries no truth.final')
    else:
        # `truth.final` IS NOT A WORLD POSITION. It is [slide_x, slide_y, yaw] of the base body
        # -- JOINT coordinates -- so the world pose is `body_pos + qpos`, and the arena
        # layout records `body_pos.x` as `start_x`. Reading it as absolute made this gate
        # report 1.7786 m for a round whose true error is 0.097 m. Measured, not guessed:
        # bare start_x 1.6850 + slide_x 8.0566 = 9.7416, against a goal of 9.8350.
        tx = float(arena["start"][0]) + float(truth_final[0])
        ty = float(arena["start"][1]) + float(truth_final[1])
        err = math.hypot(tx - goal_x, ty - goal_y)
        travelled = sim.get('truth', {}).get('travelled_m')
        res.add('the truth arrival error is within the limit',
                'PASS' if err <= ARRIVAL_LIMIT_M else 'FAIL',
                f'truth final ({tx:.4f}, {ty:.4f}) vs goal ({goal_x:.4f}, {goal_y:.4f}): '
                f'{err:.4f} m (limit {ARRIVAL_LIMIT_M}); the chassis travelled {travelled} m of '
                f'the {arena["goal_x"] - arena["start_x"]:.3f} m the arena declares')

    # ---- 5. the docking number, which is the same measurement against the dock pose ----
    # The arena's GOAL IS the dock pose, so the arrival error IS the dock error. Stated as its
    # own row because "docking" is what the P3 exit gate asks for and the reader should not have
    # to know that this arena encodes the dock as the goal.
    if truth_final:
        err = math.hypot(float(arena["start"][0]) + float(truth_final[0]) - goal_x,
                        float(arena["start"][1]) + float(truth_final[1]) - goal_y)
        res.add('the loaded/unloaded dock error is measured against a declared dock pose',
                'PASS' if err <= ARRIVAL_LIMIT_M else 'FAIL',
                f'{variant} round: {err:.4f} m at the dock pose ({goal_x:.4f}, {goal_y:.4f}) '
                f'(limit {ARRIVAL_LIMIT_M}). ⚠️ the dock pose is the ARENA\'s goal, not C\'s '
                f'receiver: the C fixture is an obstacle here, not a docking target, and no '
                f'interlock or ALIGN window is in the loop')

    # ---- 6. the arrival yaw can only be quoted from an estimate, so it says so ----
    est = nav.get('localisation') or {}
    yaw_est = est.get('final_yaw_error')
    # ★ NOT_APPLICABLE, NOT NOT_RUN, and the difference is the whole argument.
    # The first version of this row read `'PASS' if True else 'FAIL'` -- a check that cannot fail,
    # the exact defect this project spends its time hunting in other people's code. It was then
    # NOT_RUN, which was better but still wrong in the aggregate: NOT_RUN means "should have run and
    # did not", and the aggregator excused it, so p3-nav-05 passed while its own row said the yaw
    # claim could not be made. The property is real and permanent -- the arena has no independent
    # yaw truth -- so the PROFILE declares it not applicable BEFORE the run and the claim is
    # narrowed on purpose. A NOT_RUN here would mean something went wrong; a NOT_APPLICABLE means
    # the profile never asked for it.
    yaw_name = 'an independent arrival YAW is available'
    res.add(yaw_name,
            'NOT_APPLICABLE',
            (f'AMCL/odometry-based final yaw error {yaw_est!r} rad is reported by the '
             f'orchestrator, but it is an ESTIMATE' if yaw_est is not None
             else 'no yaw error recorded by the orchestrator') +
            '. `sim_report.truth` records POSITION ONLY, so this arena has no independent yaw '
            'number. A yaw claim from an estimate is a weaker claim than one from truth and must '
            'not be quoted as the latter. Declared NOT APPLICABLE by the profile: '
            + PROFILE['not_applicable'][yaw_name])

    # ---- 7. the goal is inside the map's REACHABLE free space ----
    v = read_map_cell(MAP_YAML, MAP_PGM, goal_x, goal_y)
    res.add('the goal cell is reachable free space in the frozen map',
            'PASS' if v is not None and v >= 250 else 'FAIL',
            f'occupancy at the goal is {v} (>=250 is free; the map marks unreachable cells '
            f'occupied, so this is a reachability test and not just a bounds check)')

    probes = falsifiability(nav, sim, plan, arena)
    n_ok = sum(1 for p in probes if p['made_the_check_fail'])
    covered = sorted({name for probe in probes for name in probe['covers']})
    uncovered = [name for name in PROFILE['required'] if name not in covered]
    res.add('each falsifiability probe still drives its own check out of PASS',
            'PASS' if probes and n_ok == len(probes) else 'FAIL',
            f"{n_ok}/{len(probes)} probes fired; "
            + '; '.join(f"{probe['check']}={'yes' if probe['made_the_check_fail'] else 'NO'}"
                        for probe in probes)
            + f'. {len(covered)} of {len(PROFILE["required"])} required checks carry a probe; '
              f'without one: {uncovered or "none"}. The row is named for what it measures -- an '
              f'earlier version was called "every check has been shown able to fail" while covering '
              f'5 of 9 rows, and the uncovered ones were not named anywhere.')

    return emit(args.run_id, run, res, identity, probes, arena, variant)


def emit(run_id, run, res, identity, probes, arena, variant):
    """Write the gate and print it. One exit path, so a refusal cannot skip the evidence."""
    if arena is None:
        verdict = 'FAIL'
        reasons = [f"identity could not be resolved ({identity['code']}), so not one geometric row "
                   f"can be anchored to an arena"]
    else:
        verdict, reasons = verdict_of(res.rows, PROFILE)
    out = {
        'run_id': run_id, 'variant': variant, 'identity': {k: v for k, v in identity.items()
                                                          if k != 'arena'},
        'profile': {'id': PROFILE['id'], 'version': PROFILE['version'],
                    'required': list(PROFILE['required']),
                    'not_applicable': PROFILE['not_applicable']},
        'arena': (None if arena is None else
                  {'xml': arena['xml'], 'xml_sha256': arena['xml_sha256'],
                   'start': arena['start'], 'goal': arena['goal'],
                   'pose_clearance_m': arena['pose_clearance_m'],
                   'load': arena.get('load')}),
        'verdict': verdict, 'verdict_reasons': reasons,
        'counts': res.counts(), 'checks': res.rows,
        'falsifiability': probes,
        'not_established': [
            'the ASSEMBLED cell: H, A, C and the second AMR are absent from this arena; their '
            'footprints are obstacles',
            "C's docking: the dock pose is the arena's goal, the C fixture is an obstacle, and "
            "no interlock, ALIGN window or receiver crown is in the loop",
            'the arrival yaw from truth: `sim_report.truth` records position only, declared NOT '
            'APPLICABLE by the profile',
            'the load: the loaded variant carries a declared stand-in RIGID MOUNT, not a tray on '
            'the deck rollers, so no retention result comes from it',
            'SLAM: the map is an operator prior generated from the world',
        ],
    }
    run.mkdir(parents=True, exist_ok=True)
    (run / 'p3_nav_gate.json').write_text(json.dumps(out, indent=2) + '\n', encoding='utf-8')

    print('=' * 96)
    print(f'P3 NAV GATE -- {run_id} ({variant})')
    print('=' * 96)
    if arena is not None:
        print(f"  arena {arena['xml']}  sha256 {arena['xml_sha256'][:16]}")
        print(f"  start {arena['start']}  goal {arena['goal']}")
        if arena.get('load'):
            L = arena['load']
            print(f"  load  {L['tray_source']} at z {L['mount_z_m']} m, {L['load_mass_kg']} kg")
    print(f"  profile {PROFILE['id']} v{PROFILE['version']}, "
          f"{len(PROFILE['required'])} required checks")
    print()
    for row in res.rows:
        flag = {'PASS': 'ok  ', 'FAIL': 'FAIL', 'NOT_RUN': '--  ',
                'NOT_APPLICABLE': 'n/a '}.get(row['status'], '?   ')
        print(f"{flag} [{row['status']}] {row['name']}")
        print(f'            {row["detail"]}')
    counts = res.counts()
    print(f"\n{counts['pass']}/{counts['checks']} checks PASS, {counts['fail']} FAIL, "
          f"{counts['not_run']} NOT_RUN, {counts['not_applicable']} NOT_APPLICABLE")
    print(f'verdict {verdict}' + (f' -- {reasons}' if reasons else ''))
    print('\nNOT established: ' + '; '.join(out['not_established']))
    print(f'\nwrote {run}/p3_nav_gate.json')
    return 0 if verdict == 'PASS' else 1


def falsifiability(nav, sim, plan, arena):
    """Each row must be driven to a non-PASS state by a probe built from the SAME data source.

    The probes mutate the loaded report dicts in memory, so they exercise the judging code without
    touching a single file on disk. `covers` names the row(s) each probe is evidence for, so the
    gate can report which required checks have NO probe instead of implying that all of them do.
    """
    import copy
    out = []

    def run(label, what, fn, covers):
        try:
            fired = bool(fn())
        except Exception as exc:
            fired, what = False, f'{what} -- probe raised {exc!r}'
        out.append({'check': label, 'probe': what, 'made_the_check_fail': fired,
                    'covers': list(covers)})

    def r_identity():
        s = copy.deepcopy(sim)
        s['world_sha256'] = '0' * 64
        return _rerun(nav, s, plan, arena)['identity']

    def r_world():
        s = copy.deepcopy(sim)
        s['config_sha256'] = '0' * 64
        return _rerun(nav, s, plan, arena)['world']

    def r_pace():
        s = copy.deepcopy(sim)
        s['achieved_realtime_ratio'] = 1.0
        return _rerun(nav, s, plan, arena)['pace']

    def r_status():
        n = copy.deepcopy(nav)
        n['status'] = 'GOAL_TIMEOUT'
        return _rerun(n, sim, plan, arena)['status']

    def r_arrival():
        s = copy.deepcopy(sim)
        tf = s['truth']['final']
        # perturb in the JOINT frame, because that is what `truth.final` is: `_rerun` adds
        # `arena['start']` exactly as `main` does. The previous version added 5 m as if it were a
        # world coordinate, so the probe exercised a different formula than the row it guarded.
        s['truth']['final'] = [float(tf[0]) + 5.0, float(tf[1]), float(tf[2])]
        r = _rerun(nav, s, plan, arena)
        return r['arrival'] and r['dock']

    def r_map():
        # a goal whose cell is occupied: the arena's own start is inside the h obstacle's
        # inflated footprint in the map, so use a point clearly outside the room
        a = copy.deepcopy(arena)
        a['goal'] = [200.0, 200.0]
        a['goal_x'] = 200.0
        return _rerun(nav, sim, plan, a)['map']

    run('identity', 'replace the recorded world hash with zeros', r_identity,
        ['the run declares which physical variant it is, and it checks out'])
    run('world', 'replace the sim config hash with zeros', r_world,
        ['the round ran on the frozen configuration and the frozen arena'])
    run('pace', 'claim the plant achieved a realtime ratio of 1.0', r_pace,
        ['the plant is paced below real time, and the ROS side kept up with it'])
    run('status', 'report the goal as timed out', r_status,
        ['Nav2 reported the goal reached'])
    run('arrival', 'move truth 5 m past the goal, in the joint frame', r_arrival,
        ['the truth arrival error is within the limit',
         'the loaded/unloaded dock error is measured against a declared dock pose'])
    run('map', 'ask about a goal outside the map', r_map,
        ['the goal cell is reachable free space in the frozen map'])
    return out


def _rerun(nav, sim, plan, arena):
    """The same decisions as `main`, on injected data, returning which rows did NOT pass."""
    out = {}
    frozen = load(FREEZE)
    want = {a['sha256'] for a in frozen['artefacts']
            if a['path'].startswith('config/p3_nav')}
    out['identity'] = resolve_identity('probe', plan, sim, load(LAYOUT))['status'] != 'PASS'
    out['world'] = sim.get('config_sha256') not in want
    factor = sim.get('realtime_factor')
    ratio = sim.get('achieved_realtime_ratio')
    stepped = float(sim.get('sim_seconds') or 0.0)
    observed = float((nav.get('observed') or {}).get('clock') or 0.0)
    tracking = observed / stepped if stepped else 0.0
    out['pace'] = (factor is None or factor >= 1.0 or ratio is None
                   or abs(float(ratio) - float(factor)) > 0.10
                   or tracking < CLOCK_TRACKING_MIN)
    out['status'] = nav.get('status') != 'GOAL_SUCCEEDED'
    tf = sim.get('truth', {}).get('final')
    gx, gy = float(arena['goal'][0]), float(arena['goal'][1])
    # `truth.final` is a JOINT coordinate, so the world pose is start + final. `main` does this;
    # `_rerun` did not, so the arrival probe guarded a formula the row does not use.
    arrival_err = (None if not tf else
                   math.hypot(float(arena['start'][0]) + float(tf[0]) - gx,
                              float(arena['start'][1]) + float(tf[1]) - gy))
    out['arrival'] = arrival_err is None or arrival_err > ARRIVAL_LIMIT_M
    out['dock'] = out['arrival']
    v = read_map_cell(MAP_YAML, MAP_PGM, gx, gy)
    out['map'] = not (v is not None and v >= 250)
    return out


if __name__ == '__main__':
    sys.exit(main())
