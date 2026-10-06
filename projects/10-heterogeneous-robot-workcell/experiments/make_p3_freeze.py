"""P3-FREEZE-04: pin the P3 chain's inputs and thresholds, and fail when one drifts.

WHY A MANIFEST AND NOT A COMMENT
--------------------------------
D036/D043 recorded the thing this file exists to stop: a number written down by hand cannot
notice that it has expired. A threshold repeated in a doc, a report and a launch script drifts
the moment one of them is edited, and the drift is silent -- every artefact still looks right.

So nothing here is typed. Every entry is READ from the file that owns it, and `--check`
re-reads it. If a tolerance is changed in `config/nav2_p3_nav.yaml`, or a wall moves because
`merge_world` changed, or the map no longer matches the world it is a map of, the freeze goes
red and names the file. Regenerating is `--write`, which is a deliberate act.

TWO SECTIONS, BECAUSE THEY FAIL DIFFERENTLY
-------------------------------------------
  * `artefacts` -- files the chain consumes. Byte hashes. A change here means the evidence on
    disk no longer describes what the code would produce.
  * `code` -- the judging and generating scripts themselves. A threshold can live inside a
    judge (`evaluate_n_nav.py --arrival-limit-m`), and an artefact hash cannot see that, so the
    scripts are hashed too. D049 recorded the same reasoning for the two command gates: the
    authority for a decision has to be findable, and there were two of them.

WHAT IT DELIBERATELY DOES NOT FREEZE is listed in the file itself, under `not_frozen`, so that
the boundary of the claim is inside the artefact rather than in a conversation.

A MANIFEST CANNOT NOTICE WHAT IS MISSING FROM IT
-------------------------------------------------
`compare` can prove every listed file is unchanged; it can never prove every file that SHOULD be
listed is listed. Measured cost: `experiments/probe_p3_vision.py` -- the sole judge for
`P3-VISION-03` -- was absent from `CODE` while the paragraph above claimed the judging scripts were
hashed, and `--check` stayed green straight through a full rewrite of that probe. So the expected
set is DERIVED from `docs/TASK_BOARD.md`: every `experiments/*.py` named in a `P3-*` row must be in
`CODE` or in `COVERAGE_EXEMPT` with a written reason. Only rows that START with `P3-` are read, so
P1's scripts cannot leak in, and an exemption that stops being named goes red as stale.
"""
import argparse
import hashlib
import collections
import json
import pathlib
import re
import sys
import xml.etree.ElementTree as ET

ROOT = pathlib.Path(__file__).resolve().parents[1]
OUT = ROOT / 'config' / 'p3_freeze.json'
SCHEMA = 1

ARTEFACTS = (
    'assets/world_p5_candidate_v7_hinged_retainer.xml',
    'assets/world_p5_candidate_v6_retainer.xml',
    'config/nav2_joint_world_v5_retained.yaml',
    'config/joint_world_v5_retained.profile.json',
    'config/joint_world_v5_centered.profile.json',
    'config/nav2_joint_world_v5_centered.yaml',
    'config/nav2_joint_world_v5_loaded.yaml',
    'config/joint_world_v5_loaded.profile.json',
    'assets/maps/joint_world_v5.pgm',
    'assets/maps/joint_world_v5.yaml',
    'config/nav2_joint_world_v5.yaml',
    'config/joint_world_v5.yaml',
    'config/joint_world_v5.profile.json',
    'assets/maps/joint_world_v7.pgm',
    'assets/maps/joint_world_v7.yaml',
    'config/nav2_joint_world_v7.yaml',
    'config/joint_world_v7.yaml',
    'config/joint_world_v7.profile.json',
    'config/joint_world_v7_loaded.profile.json',
    'config/nav2_joint_world_v7_retained.yaml',
    'config/joint_world_v7_retained.profile.json',
    'config/nav2_joint_world_v7_turn.yaml',
    'config/joint_world_v7_turn.profile.json',
    'assets/world_p5_candidate_v5.xml',
    'assets/world_p5_candidate_v4.xml',
    'assets/world_p5_candidate_v3.xml',
    'assets/world_w2_support_v1.xml',
    'assets/world_w2_support_v2.xml',
    # Independent, user-approved integration candidate. Never replaces old world hashes.
    'assets/world_p5_candidate.xml',
    'assets/world_p5_candidate_v2.xml',
    'assets/world_p3_cell.xml',
    'assets/world_p4_cell.xml',
    'assets/world_w2_logistic.xml',
    #: candidate B (H1/H2's world) and the H3 integration world. Both were cited by H-chain
    #: evidence while frozen nowhere; a world a report rests on has to be hashable.
    'assets/world_w5_h085.xml',
    'assets/world_w5_h085_loop.xml',
    'assets/worlds/world_p3_nav.xml',
    'assets/worlds/world_p3_nav_loaded.xml',
    'assets/worlds/world_p3_nav.layout.json',
    'assets/worlds/world_n_probe.xml',
    'assets/maps/p3_nav.yaml',
    'assets/maps/p3_nav.pgm',
    'config/n_probe.yaml',
    'config/p3_nav.yaml',
    'config/p3_nav_loaded.yaml',
    'config/nav2_n_probe.yaml',
    'config/nav2_p3_nav.yaml',
    'config/docking_contract.json',
)

CODE = (
    'experiments/build_hinged_retainer_candidate.py',
    'experiments/probe_tray_retainer.py',
    'experiments/build_tray_retainer_candidate.py',
    'experiments/deck_support_forensics.py',
    'experiments/contact_model_candidate.py',
    'experiments/inspect_contact_flags.py',
    'experiments/inspect_retained_nav.py',
    'experiments/probe_cargo_rest.py',
    'experiments/make_retained_motion_profile.py',
    'experiments/state_publish_schedule.py',
    'experiments/transport_posture_gate.py',
    'experiments/evaluate_continuous_nav.py',
    'experiments/continuous_nav_session.py',
    'experiments/vehicle_rgbd_reader.py',
    'experiments/probe_vehicle_rgbd.py',
    'experiments/probe_vehicle_rgbd_motion.py',
    'experiments/load_motion_gate.py',
    'experiments/probe_vehicle_nav2.py',
    'experiments/probe_loaded_clearance.py',
    'experiments/make_centered_load_profile.py',
    'experiments/loaded_nav_envelope.py',
    'experiments/make_loaded_nav_profile.py',
    'experiments/wheel_authority.py',
    'experiments/evaluate_joint_nav.py',
    'experiments/nav_geometry_inventory.py',
    'experiments/make_joint_nav_profile.py',
    'experiments/make_joint_nav_profile_v7.py',
    'experiments/make_retained_motion_profile_v7.py',
    'experiments/probe_g7a_source_xfer.py',
    'experiments/probe_g6_deckyaw_diag.py',
    'experiments/probe_g7d_failures.py',
    'experiments/probe_g7c_transit_dial.py',
    'experiments/probe_g7c_transit_slow.py',
    'experiments/probe_g7_pair.py',
    'experiments/joint_nav_worker.py',
    'experiments/probe_joint_world_nav2.py',
    'experiments/humanoid_posture_permit.py',
    'experiments/dock_measurements.py',
    'experiments/probe_humanoid_post_supply_hold.py',
    'experiments/inspect_loading_snapshot.py',
    'experiments/probe_wheel_calibration_loading.py',
    'experiments/joint_world_nav_io.py',
    'experiments/joint_world_ros_bridge.py',
    'experiments/ros_command_lease.py',
    'experiments/probe_joint_world_nav_io.py',
    'experiments/probe_vehicle_heading.py',
    'experiments/vehicle_heading.py',
    'experiments/evaluate_supply_rejection.py',
    'experiments/cargo_footprint_judge.py',
    'experiments/humanoid_supply.py',
    'experiments/source_staging.py',
    'experiments/pcl_plane_reader.py',
    'experiments/receiver_rgbd_reader.py',
    'experiments/rgbd_geometry.py',
    'experiments/probe_recorded_receiver.py',
    'experiments/probe_pcl_receiver.py',
    'experiments/pcl_receiver_plane.cpp',
    'experiments/world_owner.py',
    'experiments/probe_world_owner.py',
    'experiments/tray_rgbd_reader.py',
    'experiments/probe_candidate_arm_loading.py',
    'experiments/probe_candidate_multi_loading.py',
    'experiments/build_support_candidate.py',
    'experiments/probe_stop_kinematics.py',
    'experiments/build_p5_candidate_world.py',
    'experiments/probe_p5_candidate_settle.py',
    'experiments/merge_world.py',
    'experiments/build_p3_world.py',
    'experiments/evaluate_p3_world.py',
    'experiments/build_p3_nav_world.py',
    'experiments/make_map_n.py',
    'experiments/make_nav2_params.py',
    'experiments/probe_n_sim.py',
    'experiments/probe_n_ros.py',
    'experiments/probe_n_nav.py',
    'experiments/check_n_nav.py',
    'experiments/evaluate_n_nav.py',
    'experiments/evaluate_p3_nav.py',
    'experiments/probe_p3_vision.py',
    'experiments/build_w2_logistic_world.py',
    'experiments/probe_w2_baseline.py',
    'experiments/dock_observability.py',
    'experiments/probe_w3_dock_mechanism.py',
    'experiments/rejudge_w3.py',
    'experiments/probe_w4_skills.py',
    'experiments/w4_plant.py',
    'experiments/w4_fake_plant.py',
    'experiments/probe_w5_loop.py',
    'experiments/probe_h_selfclear.py',
    'experiments/evaluate_n_arrival_dist.py',
    'experiments/probe_p4_reach.py',
    'experiments/build_p4_handover_world.py',
    'experiments/probe_p4_crouch.py',
    'experiments/probe_p4_balance.py',
    'experiments/probe_p4_clearance.py',
    'experiments/probe_p4_crouch2.py',
    'experiments/probe_p4_ff.py',
    'experiments/probe_p4_holddepth.py',
    'experiments/probe_p4_ff2.py',
    'experiments/probe_p4_stance.py',
    'experiments/probe_p4_armik.py',
    'experiments/_p4_arm_ik.py',
    'experiments/probe_h.py',
    'experiments/build_w5_h085_world.py',
    # --- src/ (added 2026-09-27): the H chain's own code. The freeze hashed only
    # `experiments/*.py`, so the runtime that `reports/p1-h-seq-10` fingerprints could be edited
    # with no frozen record anywhere. Same hole family as `D058`/`D060`/`D096`, new location.
    'src/humanoid007/runtime.py',
    'experiments/probe_h_w5.py',
    'experiments/probe_h2_w5.py',
    'experiments/w5_h085_plan.py',
    'experiments/probe_h3_w5.py',
    #: the scope checker `tests/test_h3_integration.py` runs, and which found the
    #: `XFER` defect that cost a whole run (D114)
    'experiments/check_globals.py',
    #: P4-BELT-03's judge (added 2026-09-30). It drives the 11-skill chain to reach both dock
    #: poses, then judges three arcs on the resulting plant: conveyance to a hard stop, receive
    #: and unload, and the deck-slip criterion `tray_does_not_slide_on_the_deck` that G-2 demoted
    #: from an assertion. Two of its rows are RED BY DESIGN (the ARC 3 finding and the ARC 2
    #: repeated-unload runaway); both are the FAILURE evidence `MASTER_PLAN` line 65 requires.
    'experiments/probe_p4_belt.py',
    # P4's completed skill probes and their execution/fixture dependencies must
    # be pinned too; naming only the primary arm judge left a live coverage hole.
    'experiments/probe_p4_arm.py',
    'experiments/probe_p4_count.py',
    'experiments/probe_p4_interlock.py',
    'experiments/probe_p4_vision.py',
    'experiments/build_p4_cell_world.py',
    'experiments/p4_fixture.py',
    'experiments/arm_bridge.py',
    'experiments/arm_rig.py',
    'scripts/run_bounded.py',
    'experiments/evaluate_belt_checkpoint.py',
    'experiments/probe_loaded_retention.py',
    'experiments/evaluate_receiver_envelope.py',
    'experiments/probe_transfer_recovery.py',
    'experiments/probe_stop_dynamics.py',
)

# The plant is only half the execution path. Pin the actual contract/adapter
# modules, without importing them into the physics control path.
CODE += tuple(sorted(str(p.relative_to(ROOT))
                     for p in (ROOT / 'src' / 'workcell').rglob('*.py')))

#: The row prefixes whose task ids this contract reads. DECLARED, because the literal
#: `'P3-'` lived inside a regex and a new work package would have had to edit a regex to
#: become visible -- which is the same class of hole this contract exists to find.
COVERED_PREFIXES = ('P3-', 'W2-', 'W3-', 'W4-', 'W5-', 'P4-')

#: TASK ID -> the script that judges it. The mapping is typed; its COMPLETENESS is derived,
#: because completeness is the only part of a manifest that can go wrong silently. Two holes were
#: found this way: `probe_p3_vision.py` (`D058`), and then `evaluate_p3_nav.py`, the judge for
#: `P3-NAV-02`, which the prose scan could not see because that board row names its data files and
#: its sniffer and never its judge. Every `P3-*` id in the board must appear here, and every script
#: named here must be frozen or exempt -- so a task row cannot be added while its judge stays
#: invisible.
TASK_JUDGES = {
    'P3-WORLD-01': 'experiments/evaluate_p3_world.py',
    'P3-NAV-02': 'experiments/evaluate_p3_nav.py',
    'P3-VISION-03': 'experiments/probe_p3_vision.py',
    'P3-FREEZE-04': 'experiments/make_p3_freeze.py',
    # The acceptance-repair task: its evidence is the re-judge that `evaluate_p3_nav.py` produces.
    # The repair is also covered by `tests/test_p3_nav_identity.py` (19 tests), which this contract
    # deliberately does not count -- it reads only `experiments/*.py` out of P3 rows, so a test file
    # cannot be pulled into the count by a row that mentions it.
    'P3-ACCEPT-05': 'experiments/evaluate_p3_nav.py',
    # --- the integration work packages (2026-09-26) ----------------------------------
    # W2: one integrated world whose receiver is at its own station, audited and driven.
    'W2-LOGISTIC-01': 'experiments/probe_w2_baseline.py',
    # W3: the docking contract, its observability gate, and the passive mechanism's own
    # measured catch envelope and residual.
    'W3-DOCK-01': 'experiments/probe_w3_dock_mechanism.py',
    # W4: the P2 core commanding the plant, with the gate as the only authority. Its judge
    # also reports the protocol rows that live in tests/test_w4_skills.py, and says out loud
    # which of them are NOT physical claims.
    'W4-SKILL-01': 'experiments/probe_w4_skills.py',
    # W5: the first real logistics closed loop, judged by a probe that drives the whole
    # sub-scenario in one continuous run and checks identity, teleport, weld, support and the
    # two P2 transfer transactions from the physical evidence.
    'W5-LOOP-01': 'experiments/probe_w5_loop.py',
    # P4 begins, with the physics prerequisite the W5 round left BLOCKED. The judge was wired here
    # BEFORE the handover existed, against `probe_h_selfclear.py`; a row whose judge nothing hashes
    # is the exact hole `D058`/`D060` kept finding, and it was cheaper to wire it then than to find
    # it again.
    #
    # ★ IT NOW POINTS AT THE H3 PROBE, because the task's subject is the CROSS-ENTITY HANDOVER and
    # that is what H3 judges. The chain's earlier links stay hashed in `CODE` --
    # `probe_h_selfclear.py` (the self-contact prerequisite, still the judge for that clause),
    # `probe_h_w5.py` (H1) and `probe_h2_w5.py` (H2) -- and the H3 probe IMPORTS H1's instruments
    # and H2's stage driver rather than copying them, so a change to either still changes what H3
    # measures. Repointing a judge is a change to the contract and is recorded in `D113`.
    'P4-HUMAN-01': 'experiments/probe_h3_w5.py',
    # P4-ARM-02: the arm's visual pick-and-place, judged by the two probes that own each half.
    # `probe_p4_arm.py` is the grasping half and `probe_p4_count.py` the counting half; the
    # contract takes the FIRST as the declared judge (one id, one judge) and the second is hashed
    # via `CODE` by the round that added it.
    'P4-ARM-02': 'experiments/probe_p4_arm.py',
    # P4-BELT-03: conveyance to a hard stop, receive/unload, and the G-2 deck-slip criterion.
    # Two of its rows are RED BY DESIGN and are the arc's FAILURE evidence, so a future round that
    # "fixes" them must change this probe -- which is exactly what hashing it here enforces.
    'P4-BELT-03': 'experiments/probe_p4_belt.py',
}

#: Scripts the task board names in a P3 row that are deliberately NOT frozen, with the reason.
#: Without this the coverage check below cannot distinguish a hole from a decision -- and a hole
#: is exactly what it was written to find: `probe_p3_vision.py` was missing from CODE while the
#: docstring above claimed the judging scripts were all hashed, and `--check` stayed green through
#: a full rewrite of that probe. A manifest cannot notice what is missing from it.
#: Task ids that are on the board but genuinely have NO judge yet, each with the reason. This
#: is not a loophole: an id in here is removed from the judge requirement AND is checked the
#: other way round -- if such a task stops appearing on the board, the entry goes stale and
#: the contract goes red. Without it, a not-started task has only two options, and both are
#: wrong: write a fake judge, or leave it off the board where nothing can see it.
NOT_STARTED_TASKS = {
    # Having a judge is not having passed it: p4-belt-06 is FAIL, not DONE.
    # P4-BELT-03 has started and owns a real judge, so it belongs in TASK_JUDGES,
    # not here. The task board alone carries its current completion status.
}

COVERAGE_EXEMPT = {
    'experiments/make_p3_freeze.py': 'the manifest itself; hashing it here would be circular',
    'experiments/sniff_n_state.py': 'a diagnostic instrument (a bare UDP receiver), not a judge or '
                                    'a generator: it writes no artefact and owns no threshold',
}

NOT_FROZEN = (
    'H3 as a PRODUCTION capability -- it runs the H chain and the W5 chain in one world, one '
    'timeline and on one tray entity, but the humanoid SUPPLIES by placing the tray on the source '
    'band and releasing it; the custody transfer is the chain\'s own TRANSFER transaction. It '
    'covers no picking, no order, no inventory, and no visual perception (scope DIAGNOSTIC_ONLY)',
    'the humanoid station as an INDUSTRIAL layout -- x 4.13 is H2\'s station, retained after a '
    'runtime-driven contact test in `assets/world_w5_h085_loop.xml`; it is a declared fixed work '
    'position and NOT the output of any layout optimisation, and the humanoid does not walk there',
    'the CROUCH depth as a capability -- `D_MAX` is None after `reports/p4-armik-05` (every depth '
    'fails the actual-state `tray_held` row), so the H chain works at the STANDING posture only',
    'W5-LOOP-01 as a PRODUCTION capability -- it runs one sub-scenario, in one world, with '
    'declared stand-ins: a rig-mounted source station, a rig-mounted receiver five metres '
    'away, and a retention pusher taken from C own rig. It is NOT an order (no picking, no '
    'BOM, no inventory), and it covers neither the humanoid nor the arm',
    'the docking mechanism as an INDUSTRIAL design -- it is a first-pass geometry whose catch envelope and residual are measured by experiments/probe_w3_dock_mechanism.py',
    'the second chassis as a commissioned transporter -- in the OLD P3/W2/H3 worlds it has no deck and is standby; the independent P5 candidate adds a complete c2 deck, but its transport/order/ROS command path is NOT_RUN',
    'the AMR model -- not selected yet (P1-ENV-01); the chassis is a declared stand-in',
    "C's mounting interface -- the deck sits at a stand-in offset (body_pos [1.92, 0, 0.445] "
    "ahead of the chassis origin); the nav arena therefore excludes it",
    'the real-time factor under a load -- measured only for the loading gate (0.21 with the '
    'hold law), not for a navigating vehicle',
    'SLAM -- the map is an operator prior generated from the world, not mapping output',
    'docs/HANDOFF.md -- generated by experiments/make_handoff.py, which has its own '
    '--check; it is a document, not part of the P3 chain',
    'docs/history/INTEGRATION_BASELINE_*.md -- generated by experiments/make_baseline.py, which has its own --check; a document snapshot, and it deliberately contains LIVE fingerprints so it goes red the moment anything it watches moves',
)


def _parse(path):
    """Comment-stripping parse: `merge_world` writes a header comment containing `--`."""
    text = path.read_text(encoding='utf-8')
    text = re.sub(r'<!--.*?-->', '', text, flags=re.S)
    return ET.fromstring(text)


def _sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _yaml(path):
    import yaml
    return yaml.safe_load(path.read_text(encoding='utf-8'))


def _flatten(node, prefix=''):
    """Every (dotted path, value) in a nested mapping, so keys are found, not asserted."""
    out = []
    if isinstance(node, dict):
        for k, v in node.items():
            out.extend(_flatten(v, f'{prefix}.{k}' if prefix else str(k)))
    elif isinstance(node, list):
        for i, v in enumerate(node):
            out.extend(_flatten(v, f'{prefix}[{i}]'))
    else:
        out.append((prefix, node))
    return out


def _find(tree, key):
    """All dotted paths whose last segment is exactly `key`."""
    return [(p, v) for p, v in _flatten(tree) if p.split('.')[-1] == key]


def code_coverage():
    """Which P3 scripts should be frozen but are not -- from TWO sources, because one was not enough.

    `compare` can prove every listed file is unchanged; it can never prove every file that should
    be listed IS listed. So the expected set comes from elsewhere, and there are two elsewheres
    with different blind spots:

      * the board's TASK IDS under `COVERED_PREFIXES`, each of which must declare a judge --
        this catches a judge that no prose row happens to mention (`evaluate_p3_nav.py` was
        one);
      * the board's ROWS under those prefixes read as prose for `experiments/*.py` -- this
        catches any OTHER script a row points at (`sniff_n_state.py` is one).

    Only the declared prefixes are read, so P1's scripts cannot leak into the count. The
    prefix list is declared rather than buried in a regex so that a new work package's
    scripts cannot stay invisible by the accident of which pattern was typed.
    """

    # ★ The list checks two properties of ITSELF. Neither is derivable from the task board, so
    # before this neither was checked at all: a duplicated entry inflated the published script count
    # (the count is a typed constant, coverage is the criterion), and an entry for a file that no
    # longer exists would have been a stale anchor that `--check` called green.
    _dupes = sorted(n for n, c in collections.Counter(CODE).items() if c > 1)
    _absent = sorted(n for n in CODE if not (ROOT / n).exists())

    board = (ROOT / 'docs' / 'TASK_BOARD.md').read_text(encoding='utf-8')
    pattern = '|'.join(re.escape(p) for p in COVERED_PREFIXES)
    rows = [ln for ln in board.splitlines() if re.match(rf'^\|\s*({pattern})', ln)]
    task_ids = sorted({m for m in re.findall(
        rf'^\|\s*((?:{pattern})[A-Z]+-\d+)\s*\|', board, re.M)})
    named = sorted({f'experiments/{m}'
                    for ln in rows
                    for m in re.findall(r'experiments/([A-Za-z0-9_]+\.py)', ln)})
    listed = set(CODE)
    missing, stale = [], []

    for task in task_ids:
        if task in NOT_STARTED_TASKS:
            continue                     # declared not started; checked for staleness below
        judge = TASK_JUDGES.get(task)
        if judge is None:
            missing.append(f'{task} has no judge declared in TASK_JUDGES')
        elif judge not in listed and judge not in COVERAGE_EXEMPT:
            missing.append(f'{task} is judged by {judge}, which the freeze does not hash')
    for task, judge in sorted(TASK_JUDGES.items()):
        if task not in task_ids:
            stale.append(f'TASK_JUDGES names {task}, which no board row has any more')
        elif not (ROOT / judge).is_file():
            missing.append(f'{task}: its declared judge {judge} does not exist')

    for path in named:
        if path not in listed and path not in COVERAGE_EXEMPT:
            missing.append(f'a P3 row names {path}, which the freeze does not hash')
    for path in sorted(COVERAGE_EXEMPT):
        if path not in named and path not in TASK_JUDGES.values():
            stale.append(path)
    for task in sorted(NOT_STARTED_TASKS):
        if task not in task_ids:
            stale.append(f'{task} is declared NOT STARTED but no board row has it any more')

    return {'rows_read': len(rows), 'task_ids': task_ids, 'named_by_board': named,
            'judges': dict(sorted(TASK_JUDGES.items())), 'exempt': sorted(COVERAGE_EXEMPT),
            'not_started': dict(sorted(NOT_STARTED_TASKS.items())),
            'duplicated_code_entries': _dupes, 'code_entries_absent': _absent,
            'missing_from_code': missing, 'exempt_but_no_longer_named': stale}


def _w2_thresholds():
    """The logistics world's and the docking mechanism's own declared numbers.

    Read from the builder's module constants and from the COMPILED world, because a
    threshold copied out of the file that owns it is one copy too many. The world is
    loaded from disk rather than rebuilt: this contract's job is to notice drift, and if
    the world is stale `build_w2_logistic_world.py --check` says so in its own voice.
    """
    sys.path.insert(0, str(ROOT / 'experiments'))
    import build_w2_logistic_world as wb
    world = ROOT / 'assets' / 'world_w2_logistic.xml'
    out = []
    for name, value, unit, source in (
            ('w2_logistic.receiver_shift_m', wb.RECEIVER_SHIFT_M, 'm', 'builder_constant'),
            ('w2_logistic.standby_offset_y_m', wb.STANDBY_OFFSET_Y_M, 'm',
             'builder_constant'),
            ('w3_dock.chamfer_catch_m', wb.CATCH_M, 'm', 'builder_constant'),
            ('w3_dock.chamfer_angle_deg', wb.CHAMFER_ANGLE_DEG, 'deg', 'builder_constant'),
            ('w3_dock.channel_length_m', wb.CHANNEL_LENGTH_M, 'm', 'builder_constant'),
            ('w3_dock.residual_lateral_m', wb.RESIDUAL_LATERAL_M, 'm', 'builder_constant'),
            ('w3_dock.datum_offset_m', wb.DATUM_OFFSET_M, 'm', 'builder_constant'),
            ('w3_dock.deck_lateral_range_m', wb.DECK_LATERAL_RANGE_M, 'm',
             'builder_constant'),
            ('w3_dock.deck_yaw_range_rad', wb.DECK_YAW_RANGE_RAD, 'rad',
             'builder_constant')):
        out.append({'name': name, 'value': float(value), 'unit': unit,
                    'source': source, 'kind': 'declared'})
    if world.is_file():
        import mujoco
        model = mujoco.MjModel.from_xml_path(str(world))
        data = mujoco.MjData(model)
        mujoco.mj_resetDataKeyframe(model, data, 0)
        mujoco.mj_forward(model, data)

        def row_x(prefix):
            obj = mujoco.mjtObj.mjOBJ_GEOM
            return sorted(float(data.geom_xpos[g][0]) for g in range(model.ngeom)
                          if (mujoco.mj_id2name(model, obj, g) or '').startswith(prefix))

        recv = row_x('c_recv_roller')
        source = row_x('c_fixed_roller')
        chassis = float(data.xpos[model.body('n_base_link').id][0])
        out.append({'name': 'w2_logistic.transport_leg_m',
                    'value': float((recv[0] - source[-1])),
                    'unit': 'm', 'source': 'assets/world_w2_logistic.xml',
                    'kind': 'derived'})
        out.append({'name': 'w2_logistic.chassis_x_at_home_m', 'value': chassis, 'unit': 'm',
                    'source': 'assets/world_w2_logistic.xml', 'kind': 'derived'})
    return out


def derive():
    thresholds = []

    def add(name, value, unit, source, kind):
        thresholds.append({'name': name, 'value': value, 'unit': unit,
                           'source': source, 'kind': kind})

    # ---- the scenario the P3 chain runs: every number the simulator obeys ----
    sim = _yaml(ROOT / 'config' / 'p3_nav.yaml')
    for key, unit in (('v_max_mps', 'm/s'), ('w_max_radps', 'rad/s'), ('ttl_s', 's'),
                      ('silence_s', 's'), ('send_rate_hz', 'Hz')):
        add(f'command.{key}', sim['command'][key], unit, 'config/p3_nav.yaml', 'declared')
    for key, unit in (('timestep_s', 's'), ('realtime_factor', '-'),
                      ('wall_deadline_s', 's'), ('wheel_kp', '-'),
                      ('wheel_torque_limit_nm', 'N.m'), ('duration_s', 's')):
        add(f'sim.{key}', sim['sim'][key], unit, 'config/p3_nav.yaml', 'declared')
    for key, unit in (('rays', 'count'), ('range_min_m', 'm'), ('range_max_m', 'm')):
        add(f'scan.{key}', sim['scan'][key], unit, 'config/p3_nav.yaml', 'declared')
    for key, unit in (('wheel_radius_m', 'm'), ('track_m', 'm')):
        add(f'odom.{key}', sim['odom'][key], unit, 'config/p3_nav.yaml', 'declared')
    add('ros.domain_id', sim['ros']['domain_id'], '-', 'config/p3_nav.yaml', 'declared')

    # ---- nav2: found by KEY NAME so a moved block cannot hide a changed tolerance ----
    nav2 = _yaml(ROOT / 'config' / 'nav2_p3_nav.yaml')
    for key, unit in (('robot_radius', 'm'), ('inflation_radius', 'm'),
                      ('cost_scaling_factor', '-'), ('xy_goal_tolerance', 'm'),
                      ('yaw_goal_tolerance', 'rad'), ('initial_pose', '-'),
                      ('max_vel_theta', 'rad/s'), ('max_vel_x', 'm/s')):
        hits = _find(nav2, key)
        if not hits:
            continue
        for path, value in hits:
            add(f'nav2.{path}', value, unit, 'config/nav2_p3_nav.yaml', 'declared')

    # ---- the room, from the COMPILED model ----
    # NOT from the XML attributes: `cell_wall_*` carries no `friction` attribute at all, it
    # inherits one, so reading the element would have frozen `None` and a real change to the
    # default class would not have been visible. The compiled model is what the solver uses.
    import mujoco
    cell = ROOT / 'assets' / 'world_p3_cell.xml'
    m = mujoco.MjModel.from_xml_path(str(cell))

    def gid(name):
        i = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, name)
        assert i >= 0, f'{name} is not a geom in {cell.name}'
        return i

    gi, wi = gid('cell_ground'), gid('cell_wall_west')
    gx, gy = float(m.geom_pos[gi][0]), float(m.geom_pos[gi][1])
    ghx, ghy = float(m.geom_size[gi][0]), float(m.geom_size[gi][1])
    for name, value in (('room.x_min', gx - ghx), ('room.x_max', gx + ghx),
                        ('room.y_min', gy - ghy), ('room.y_max', gy + ghy)):
        add(name, round(value, 6), 'm', 'assets/world_p3_cell.xml', 'derived')
    add('room.area_m2', round(4.0 * ghx * ghy, 6), 'm^2', 'assets/world_p3_cell.xml', 'derived')
    add('room.wall_thickness_m', round(2.0 * float(m.geom_size[wi][0]), 6), 'm',
        'assets/world_p3_cell.xml', 'derived')
    add('room.wall_height_m', round(2.0 * float(m.geom_size[wi][2]), 6), 'm',
        'assets/world_p3_cell.xml', 'derived')
    add('room.ground_friction', round(float(m.geom_friction[gi][0]), 6), '-',
        'assets/world_p3_cell.xml', 'derived')
    add('room.wall_friction', round(float(m.geom_friction[wi][0]), 6), '-',
        'assets/world_p3_cell.xml', 'derived')
    add('room.wall_geoms', int(sum(1 for i in range(m.ngeom)
                                   if (mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_GEOM, i) or '')
                                   .startswith('cell_wall_'))), 'count',
        'assets/world_p3_cell.xml', 'derived')

    # ---- the nav arena: what the round was actually run against ----
    doc = json.loads((ROOT / 'assets' / 'worlds' / 'world_p3_nav.layout.json')
                     .read_text(encoding='utf-8'))
    lay, ld = doc['bare'], doc['loaded']
    src = 'assets/worlds/world_p3_nav.layout.json'
    add('arena.start_x', lay['start_x'], 'm', src, 'derived')
    add('arena.goal_x', lay['goal_x'], 'm', src, 'derived')
    add('arena.drive_m', round(lay['goal_x'] - lay['start_x'], 6), 'm', src, 'derived')
    add('arena.pose_clearance_m', lay['pose_clearance_m'], 'm', src, 'declared')
    # The loaded variant must differ from the bare one by the load and nothing else, so the two
    # derived poses are recorded as equal -- if a load ever moved them, this row goes red.
    add('arena.loaded.same_start_x', ld['start_x'] == lay['start_x'], '-', src, 'derived')
    add('arena.loaded.same_goal_x', ld['goal_x'] == lay['goal_x'], '-', src, 'derived')
    if ld.get('load'):
        add('arena.loaded.mount_z_m', ld['load']['mount_z_m'], 'm', src, 'derived')
        add('arena.loaded.load_mass_kg', ld['load']['load_mass_kg'], 'kg', src, 'derived')
        add('arena.loaded.chassis_top_m', ld['load']['chassis_top_m'], 'm', src, 'derived')
        add('arena.loaded.tray_lowest_local_m', ld['load']['tray_lowest_local_m'], 'm', src,
            'derived')
    for o in lay['obstacles']:
        add(f"arena.obstacle.{o['name']}.half", o['half'], 'm',
            'assets/worlds/world_p3_nav.layout.json', 'derived')
        add(f"arena.obstacle.{o['name']}.centre", o['centre'], 'm',
            'assets/worlds/world_p3_nav.layout.json', 'derived')

    # ---- the adjudication thresholds, which live in the judges ----
    for script, flags in (('experiments/evaluate_n_nav.py',
                           ('--arrival-limit-m', '--arrival-yaw-limit-rad',
                            '--clearance-margin-m', '--map-scan-tolerance-m')),
                          ('experiments/make_map_n.py',
                           ('--resolution', '--half-extent-m', '--scan-height-m')),
                          ('experiments/build_p3_nav_world.py',
                           ('--check',))):
        text = (ROOT / script).read_text(encoding='utf-8')
        for flag in flags:
            m = re.search(re.escape(flag) + r"'[^\n]*?default=([0-9.]+)", text)
            if m:
                add(f'{pathlib.Path(script).stem}{flag}', float(m.group(1)), '-', script,
                    'declared')

    # ---- the vision chain's declared tolerances, read out of the probe itself ----
    # A tolerance can live in a module constant rather than a flag, and an artefact hash cannot see
    # its value. So the constants are read by NAME and frozen, exactly like the nav flags above.
    vtext = (ROOT / 'experiments' / 'probe_p3_vision.py').read_text(encoding='utf-8')
    for name in ('PART_CLEARANCE', 'CAM_BACK_OFF', 'CAM_HEIGHT', 'ON_BENCH_TOL', 'CHROMA_RATIO',
                 'MIN_PART_PIXELS', 'LOC_TOL'):
        m = re.search(rf'^{name} = ([0-9.]+)', vtext, re.M)
        if m:
            add(f'p3_vision.{name}', float(m.group(1)), '-', 'experiments/probe_p3_vision.py',
                'declared')
    m = re.search(r'^W, H = (\d+), (\d+)', vtext, re.M)
    if m:
        add('p3_vision.resolution_px', f'{m.group(1)}x{m.group(2)}', '-',
            'experiments/probe_p3_vision.py', 'declared')
    m = re.search(r'cam\.fovy = ([0-9.]+)', vtext)
    if m:
        add('p3_vision.fovy_deg', float(m.group(1)), 'deg', 'experiments/probe_p3_vision.py',
            'declared')

    # ---- the P4 crouch/grip gates, read out of the probe by NAME -----------------------------
    # `HAND_GAP_MAX_M` is the review's 2 mm grip gate and `TRAY_SHIFT_MAX_M` is the row that makes
    # it mean anything; both are module constants, so an artefact hash cannot see their value.
    atext = (ROOT / 'experiments' / 'probe_p4_armik.py').read_text(encoding='utf-8')
    for name in ('HAND_GAP_MAX_M', 'TRAY_SHIFT_MAX_M'):
        m = re.search(rf'^{name} = ([0-9.]+)', atext, re.M)
        if m:
            add(f'p4_armik.{name}', float(m.group(1)), 'm', 'experiments/probe_p4_armik.py',
                'declared')

    # ---- the H chain's declared thresholds, read out of the probes by NAME -------------------
    # `tray_task.THRESHOLDS` is the FROZEN six-stage contract; `probe_h2_w5.H2_THRESHOLDS` and
    # `probe_h3_w5.H3_THRESHOLDS` are DECLARED-and-submitted-for-review. All three are module
    # constants, so an artefact hash cannot see that a threshold moved -- which is precisely the
    # thing this contract exists to notice.
    sys.path.insert(0, str(ROOT / 'src'))
    from humanoid007 import tray_task as _tt
    for name, value in _tt.THRESHOLDS.items():
        add(f'tray_task.{name}', float(value), '-', 'src/humanoid007/tray_task.py', 'frozen')
    import probe_h2_w5 as _h2
    for name, value in _h2.H2_THRESHOLDS.items():
        add(f'h2.{name}', float(value), '-', 'experiments/probe_h2_w5.py', 'declared')
    import probe_h3_w5 as _h3
    for name, value in _h3.H3_THRESHOLDS.items():
        add(f'h3.{name}', float(value), '-', 'experiments/probe_h3_w5.py', 'declared')
    add('h3.band_clearance_budget_m', float(_h3.BAND_CLEARANCE_BUDGET_M), 'm',
        'experiments/probe_h3_w5.py', 'declared')
    add('h3.humanoid_station_x_m', float(_h3.H2.DEFAULT_STATION_X), 'm',
        'experiments/probe_h3_w5.py', 'declared')

    thresholds.extend(_w2_thresholds())
    return {
        'schema_version': SCHEMA,
        'artefacts': [{'path': p, 'sha256': _sha(ROOT / p), 'bytes': (ROOT / p).stat().st_size}
                      for p in ARTEFACTS],
        'code': [{'path': p, 'sha256': _sha(ROOT / p)} for p in CODE],
        'code_coverage': code_coverage(),
        'thresholds': thresholds,
        'not_frozen': list(NOT_FROZEN),
    }


def compare(current, frozen):
    """The drift list. Empty means the freeze holds."""
    drift = []
    cf = {a['path']: a['sha256'] for a in frozen.get('artefacts', [])}
    cc = {a['path']: a['sha256'] for a in frozen.get('code', [])}
    for a in current['artefacts']:
        if a['path'] not in cf:
            drift.append(f"artefact {a['path']} is NEW and not in the freeze")
        elif cf[a['path']] != a['sha256']:
            drift.append(f"artefact {a['path']} changed: {cf[a['path']][:12]} -> "
                         f"{a['sha256'][:12]}")
    for p in cf:
        if p not in {a['path'] for a in current['artefacts']}:
            drift.append(f'artefact {p} is in the freeze but no longer exists')
    for a in current['code']:
        if a['path'] not in cc:
            drift.append(f"code {a['path']} is NEW and not in the freeze")
        elif cc[a['path']] != a['sha256']:
            drift.append(f"code {a['path']} changed: {cc[a['path']][:12]} -> {a['sha256'][:12]}")
    old = {t['name']: t for t in frozen.get('thresholds', [])}
    for t in current['thresholds']:
        if t['name'] not in old:
            drift.append(f"threshold {t['name']} is NEW and not in the freeze")
        elif old[t['name']]['value'] != t['value']:
            drift.append(f"threshold {t['name']} changed: {old[t['name']]['value']} -> "
                         f"{t['value']} (source {t['source']})")
    for n in old:
        if n not in {t['name'] for t in current['thresholds']}:
            drift.append(f'threshold {n} is in the freeze but is no longer produced')
    coverage = current.get('code_coverage', {})
    for problem in coverage.get('duplicated_code_entries', []):
        drift.append(f'coverage: CODE lists {problem} more than once')
    for problem in coverage.get('code_entries_absent', []):
        drift.append(f'coverage: CODE names {problem}, which does not exist on disk')
    for problem in coverage.get('missing_from_code', []):
        drift.append(f'coverage: {problem}')
    for problem in coverage.get('exempt_but_no_longer_named', []):
        drift.append(f'coverage: {problem}')
    return drift


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--check', action='store_true',
                    help='re-derive and compare; exit 1 on any drift')
    args = ap.parse_args()

    current = derive()
    if args.check:
        if not OUT.is_file():
            print(f'[FAIL] no freeze at {OUT.relative_to(ROOT)}; run without --check to write it')
            return 1
        frozen = json.loads(OUT.read_text(encoding='utf-8'))
        drift = compare(current, frozen)
        if drift:
            print(f'[FAIL] {len(drift)} drift(s):')
            for d in drift:
                print(f'  - {d}')
            return 1
        print(f'[OK] freeze holds: {len(current["artefacts"])} artefacts, '
              f'{len(current["code"])} scripts, {len(current["thresholds"])} thresholds; '
              f'coverage: all {len(current["code_coverage"]["named_by_board"])} P3 scripts the '
              f'board names are frozen or exempt')
        return 0

    OUT.write_text(json.dumps(current, indent=2, sort_keys=False) + '\n', encoding='utf-8')
    print(f'wrote {OUT.relative_to(ROOT)}')
    print(f'  artefacts  {len(current["artefacts"])}')
    print(f'  code       {len(current["code"])}')
    print(f'  thresholds {len(current["thresholds"])}')
    cv = current['code_coverage']
    print(f'  coverage   {len(cv["named_by_board"])} scripts named in {cv["rows_read"]} P3 rows; '
          f'missing from CODE: {cv["missing_from_code"] or "none"}; '
          f'exempt: {cv["exempt"]}')
    print('  not frozen: ' + '; '.join(NOT_FROZEN))
    return 0


if __name__ == '__main__':
    sys.exit(main())
