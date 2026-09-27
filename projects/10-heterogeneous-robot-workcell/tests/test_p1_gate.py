"""P1-GATE-07: the common-world loading gate.

Two kinds of test here, and the distinction is the point of the gate:

  * PROPERTY tests on the artifact -- the committed world matches its sources, the prefixes are
    complete, the free joints keep the model's own reference pose, the stations do not overlap;
  * INSTRUMENT tests -- the judge must be shown able to FAIL. A gate whose checks have never
    been seen to fail is not a gate, so one test points the judge at a deliberately broken world
    and requires a non-zero exit, and another requires every falsifiability probe to declare the
    status it expects.

Nothing here re-tests the physics of the four roles; that is what their own reports are for.
"""
import ast
import json
import pathlib
import shutil
import subprocess
import sys

import pytest

ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'experiments'))
import merge_world as mw  # noqa: E402

import mujoco  # noqa: E402
import numpy as np  # noqa: E402

MERGED = ROOT / 'assets' / 'world_p1_cell.xml'
MERGE_SRC = (ROOT / 'experiments' / 'merge_world.py').read_text(encoding='utf-8')
GATE_SRC = (ROOT / 'experiments' / 'evaluate_gate.py').read_text(encoding='utf-8')


def merged_model():
    return mujoco.MjModel.from_xml_path(str(MERGED))


# ---------------------------------------------------------------------------------------------
# the artifact
# ---------------------------------------------------------------------------------------------
def test_the_committed_world_matches_its_sources():
    """`--check` rebuilds in memory and diffs; a stale world would silently mislead the gate."""
    result = subprocess.run([str(ROOT / '.venv' / 'bin' / 'python'),
                             str(ROOT / 'experiments' / 'merge_world.py'), '--check'],
                            capture_output=True, text=True, cwd=str(ROOT), timeout=600)
    assert result.returncode == 0, result.stdout + result.stderr
    assert 'matches its sources' in result.stdout


def test_the_merge_never_addresses_state_by_position():
    """No constant positional slice of qpos/qvel/ctrl anywhere in the merge.

    This is the exact shape of the defect that cost the A branch a round: a positional
    `[:, :7]` Jacobian slice silently drove the payload's free joint instead of the arm, and
    merging four models is the operation that reshuffles every index. Addresses must be derived
    (`qpos[adr:adr + width]`) or looked up by name.
    """
    tree = ast.parse(MERGE_SRC)
    offenders = []
    for node in ast.walk(tree):
        if not isinstance(node, ast.Subscript):
            continue
        base = ast.unparse(node.value)
        if not base.endswith(('qpos', 'qvel', 'ctrl', 'key_qpos', 'key_ctrl')):
            continue
        sl = node.slice
        if isinstance(sl, ast.Slice):
            # a slice with a constant bound is a positional address: `qpos[:7]`, `ctrl[0:2]`
            for part in (sl.lower, sl.upper, sl.step):
                if isinstance(part, ast.Constant):
                    offenders.append(f'line {node.lineno}: {ast.unparse(node)}')
        elif isinstance(sl, ast.Constant) and not base.endswith(('key_qpos', 'key_ctrl')):
            # a constant index into the 1-D state is positional too. `key_qpos[0]` is allowed:
            # that 0 selects the KEYFRAME, not a degree of freedom.
            offenders.append(f'line {node.lineno}: {ast.unparse(node)}')
    assert not offenders, (
        f'the merge indexes model state with a constant bound: {offenders}. Derived addresses '
        f'only -- a positional slice here is how a merged model silently drives the wrong body')


def test_the_prefix_reaches_joint_names_not_only_body_names():
    """`attach` prefixes joint names too, and assuming otherwise made the home lookup empty."""
    m = merged_model()
    assert m.joint('a_joint1').id >= 0
    assert mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, 'joint1') < 0, (
        'an unprefixed joint name must not exist in the merged world')
    # and the key strips it back off, or the declarations never match
    j = m.joint('a_joint1').id
    assert mw.joint_key(m, j, strip='a_')[1] == 'joint1'


def test_a_free_joint_keeps_the_model_reference_pose():
    """A keyframe may be a keyframe of the mechanism only; the model's own pose wins for objects.

    Measured reason: A's `home` keyframe sets `payload_free` to the origin, while `qpos0` puts
    the part on the table. Obeying the keyframe buried the part in the ground for the whole run.
    """
    m = merged_model()
    assert m.nkey >= 1
    key = np.array(m.key_qpos[0], dtype=float)
    free = [j for j in range(m.njnt) if int(m.jnt_type[j]) == int(mujoco.mjtJoint.mjJNT_FREE)]
    assert free, 'the merged world must contain the four roles free bodies'
    for j in free:
        adr = int(m.jnt_qposadr[j])
        width = 7
        assert np.allclose(key[adr:adr + width], np.array(m.qpos0[adr:adr + width]), atol=1e-9), (
            f'free joint {mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_JOINT, j)} has a keyframe pose '
            f'that differs from the model reference; a free joint qpos is an absolute world pose '
            f'and must come from the model, not from a mechanism keyframe')


def test_free_joint_quaternions_are_unit():
    m = merged_model()
    for j in range(m.njnt):
        if int(m.jnt_type[j]) != int(mujoco.mjtJoint.mjJNT_FREE):
            continue
        adr = int(m.jnt_qposadr[j])
        q = np.array(m.key_qpos[0][adr + 3:adr + 7], dtype=float)
        assert abs(float(np.linalg.norm(q)) - 1.0) < 1e-6, (
            f'free joint {j} has a non-unit quaternion {q}')


def test_the_declared_free_pose_disagreement_is_reported():
    """The build must say out loud that it ignored a declaration, not swallow it."""
    model = mujoco.MjModel.from_xml_path(str(MERGED))
    spec, layout = mw.build_spec()
    compiled = spec.compile()
    qpos, ctrl, applied, notes = mw.merged_home(compiled, layout)
    text = ' '.join(f'{k} {n}' for k, n, _c in notes)
    assert 'DECLARED FREE POSE IGNORED' in text, (
        'the A role home keyframe disagrees with the model on the payload, so the build must '
        'report it; notes were: ' + text[:400])
    assert 'payload_free' in text
    assert applied > 0, 'the home assembly applied no joint overrides at all'
    del model


def test_the_stations_do_not_overlap():
    """Four roles that all sit near the origin in their own worlds must be given room."""
    layout = mw.station_layout()
    spans = {}
    for role in mw.ROLES:
        lo, hi = mw.role_extent(role)
        x, y = layout[role.key]
        spans[role.key] = (x + float(lo[0]), x + float(hi[0]))
    keys = sorted(spans)
    for i, a in enumerate(keys):
        for b in keys[i + 1:]:
            lo_a, hi_a = spans[a]
            lo_b, hi_b = spans[b]
            assert hi_a <= lo_b or hi_b <= lo_a, (
                f'stations {a} {spans[a]} and {b} {spans[b]} overlap in x; the roles would start '
                f'in contact, which the gate would then report as a merge failure')


def test_the_furniture_each_role_brought_is_dropped():
    """Four floors and an 8x8 m arena are single-role fixtures, not cell furniture."""
    m = merged_model()
    for name in ('floor', 'h_floor', 'a_floor', 'c_floor', 'n_floor',
                 'n_pillar_a', 'n_wall_north', 'n_wall_east'):
        assert mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, name) < 0, (
            f'{name} leaked into the merged world')
    assert mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, 'cell_ground') >= 0
    assert mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, 'cell_wall_north') >= 0


def test_the_shared_ground_is_the_lowest_of_the_four_sources():
    """The choice is declared, and this pins it to the measurement that justifies it."""
    sources = mw.ground_friction_sources()
    values = {v[0] for v in sources.values()}
    assert len(values) > 1, (
        f'the four sources now agree on the ground friction ({values}), so the comment in '
        f'merge_world.py explaining why one shared ground cannot satisfy all four is stale')
    assert mw.SHARED_GROUND_FRICTION == min(values), (
        f'the shared ground is {mw.SHARED_GROUND_FRICTION}; it is documented as the lowest of the '
        f'sources ({min(values)}), so nothing settles here by virtue of extra grip')


def test_every_source_agrees_on_the_time_base():
    sources = mw.sources_agree_on_stepping()
    assert len(set(sources.values())) == 1, (
        f'the four sources no longer share a time base: {sources}. A single world cannot step at '
        f'four rates, so this would have to become a declared gate decision')


def test_the_merged_model_carries_every_role_actuator():
    m = merged_model()
    assert m.nu == 116, f'{m.nu} actuators'
    for key in ('h', 'a', 'c', 'n'):
        got = sum(1 for a in range(m.nu)
                  if (mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_ACTUATOR, a) or '')
                  .startswith(key + '_'))
        assert got > 0, f'role {key} contributed no actuators'
    assert m.nbody == 140 and m.njnt == 125 and m.ngeom == 310


# ---------------------------------------------------------------------------------------------
# the instrument: the judge must be shown able to fail
# ---------------------------------------------------------------------------------------------
def _run_gate(run_id, world=None):
    cmd = [str(ROOT / '.venv' / 'bin' / 'python'),
           str(ROOT / 'experiments' / 'evaluate_gate.py'), '--run-id', run_id]
    if world is not None:
        cmd += ['--world', str(world)]
    return subprocess.run(cmd, capture_output=True, text=True, cwd=str(ROOT), timeout=900)


def test_the_gate_passes_on_the_committed_world():
    out = ROOT / 'reports' / 'p1-gate-selftest'
    shutil.rmtree(out, ignore_errors=True)
    try:
        result = _run_gate('p1-gate-selftest')
        assert result.returncode == 0, result.stdout + result.stderr
        payload = json.loads((out / 'gate.json').read_text(encoding='utf-8'))
        assert payload['verdict'] == 'PASS'
        assert payload['counts'] == {'PASS': len(payload['checks']), 'FAIL': 0, 'NOT_RUN': 0}
        assert (out / 'acceptance.md').is_file()
        # ---------------------------------------------------------------------------------
        # THE SCOPE MUST TRAVEL WITH THE VERDICT -- asserted as a PROPERTY, not as a phrase.
        #
        # This line used to read `assert 'deck mounted on N' in payload['not_established']`.
        # That string was pinning a NEGATION that the V1 assembly made false: once the deck
        # really was mounted, "the deck is not mounted on N" stopped being a true thing to
        # disclaim, and the correct fix was to delete the clause -- at which point the test
        # failed for having been right about a sentence instead of about the world.
        #
        # This is hard lesson 8 in the project's own list: a test that guarantees a property by
        # asserting a literal string exists turns that property into a typed constant. So the
        # assertions below are about the CONTENT that has to be there no matter how the scope is
        # worded -- and the ones that can be checked against the world are.
        # ---------------------------------------------------------------------------------
        scope = payload['scope_claim']
        not_est = payload['not_established']
        assert 'ground' in scope and 'ground' in not_est

        # Whatever the wording, the artifact must still disclaim C's transfer in the assembled
        # world: that is the load-bearing caveat the assembly created, because bolting the deck
        # to the chassis turned C's spacing constant into a docking tolerance.
        assert 'assembled world' in not_est, not_est
        assert 'docking tolerance' in not_est, not_est

        # And it must ASSERT the assembly rather than disclaim it, because the world now has it.
        # Compared against the DERIVED name (`merge_world.CHASSIS_MOUNT_BODY` under n's prefix),
        # not the literal 'base_link': the merged world prefixes every body, and typing the bare
        # source name here would make the assertion wrong rather than make the world wrong.
        want_parent = 'n_' + mw.CHASSIS_MOUNT_BODY
        assert 'mounted on' in scope, scope
        assert payload['assembly']['mounted'] is True, payload['assembly']
        assert payload['assembly']['mount_parent'] == want_parent, payload['assembly']
        assert payload['assembly']['deck_bodies'] > 1, payload['assembly']
        # And the headline number: the chassis is parked where the standalone rig had the deck.
        assert abs(payload['assembly']['dock_offset_m']) < 1e-4, payload['assembly']

        # A STALE NEGATION MUST BE UNABLE TO COME BACK. If someone reverts the assembly, the
        # disclaimer would have to be restored -- and this catches the reverse: a world that has
        # the deck mounted while the scope still says it is not.
        assert not any('not mounted' in s or 'no mounting interface' in s
                       for s in (scope, not_est)), (scope, not_est)
    finally:
        shutil.rmtree(out, ignore_errors=True)


def test_the_gate_fails_on_a_broken_world():
    """The decisive test: point the judge at a world with one cross-role mis-wiring."""
    scratch = ROOT / 'assets' / '_gate_broken_selftest.xml'
    spec = mujoco.MjSpec.from_file(str(MERGED))
    victim = [a for a in spec.actuators if a.name == 'n_wheel_left_motor'][0]
    victim.target = 'h_J00_HIP_PITCH_L'
    scratch.write_text(spec.to_xml(), encoding='utf-8')
    out = ROOT / 'reports' / 'p1-gate-broken-selftest'
    shutil.rmtree(out, ignore_errors=True)
    try:
        result = _run_gate('p1-gate-broken-selftest', world=scratch)
        assert result.returncode != 0, (
            'the judge passed a world whose AMR wheel motor drives a humanoid hip joint:\n'
            + result.stdout + result.stderr)
        payload = json.loads((out / 'gate.json').read_text(encoding='utf-8'))
        assert payload['verdict'] == 'FAIL'
        failed = [c['name'] for c in payload['checks'] if c['status'] == 'FAIL']
        assert 'every actuator drives a joint of its own role' in failed, failed
    finally:
        shutil.rmtree(out, ignore_errors=True)
        if scratch.exists():
            scratch.unlink()


def test_every_falsifiability_probe_declares_the_status_it_expects():
    payload_names = [n for n in ('falsifiability',) if n in GATE_SRC]
    assert payload_names, 'the gate no longer emits a falsifiability section'
    assert "'expected': e" in GATE_SRC or '"expected": e' in GATE_SRC, (
        'the falsifiability entries no longer carry the status they expect, so a probe that '
        'silently stopped probing would read as a pass')
    probes = GATE_SRC.count("out.append((")
    assert probes >= 5, f'only {probes} falsifiability probes are declared'
    for check in ('every name follows the role it is built into',
                  'every actuator drives a joint of its own role',
                  'no role touches another',
                  'moving one role moves no other role',
                  'the joint scene settles',
                  'the shared floor does not bind any contact',
                  'every joint in the merged world is addressable by name'):
        assert check in GATE_SRC, f'the probe for {check!r} is gone'


def test_the_base_free_joint_is_named_and_addressable():
    """The humanoid's base free joint arrives unnamed; the merge must name it.

    It used to be reachable only through its parent body, and that indirection hid a real bug:
    the home assembly could not match an unnamed joint, so it skipped it, and the humanoid
    started at the origin with a ZERO quaternion -- measured as its feet 1.0165 m inside the
    ground, which then looked like role A's arm servo failing.
    """
    m = merged_model()
    unnamed = [j for j in range(m.njnt)
               if mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_JOINT, j) is None]
    assert unnamed == [], f'{len(unnamed)} unnamed joint(s) remain: {unnamed}'
    jid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, 'h_base_free')
    assert jid >= 0, 'the humanoid base free joint is not addressable by name'
    assert int(m.jnt_type[jid]) == int(mujoco.mjtJoint.mjJNT_FREE)
    body = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, int(m.jnt_bodyid[jid]))
    assert body == 'h_LINK_BASE', body
    if m.nkey:
        adr = int(m.jnt_qposadr[jid])
        quat = np.array(m.key_qpos[0][adr + 3:adr + 7], dtype=float)
        assert abs(float(np.linalg.norm(quat)) - 1.0) < 1e-9, (
            f'the home quaternion for the base is not unit: |q| = {np.linalg.norm(quat)}')
        assert float(m.key_qpos[0][adr + 2]) > 0.5, (
            'the humanoid base is not up at home, so the declared start pose is not a standing one')
    assert 'name_base_free_joint' in MERGE_SRC, (
        'the merge no longer names the base joint, so the parent-body workaround can come back')
    assert 'BASE_FREE_JOINT' in MERGE_SRC and 'HUMANOID_BASE_BODY' in MERGE_SRC


def test_the_floor_check_derives_its_bound_from_the_contacts_it_measures():
    """The floor's own friction is harmless only while it stays below what touches it.

    That is a property of the CONTACT SET, so it has to be measured. A constant comparison
    ("the floor is 0.25, which is fine") would go stale the moment a role gained a
    lower-friction contact geom -- and it would go stale silently, which is this project's
    recurring failure mode.
    """
    assert 'the shared floor does not bind any contact' in GATE_SRC
    assert 'lowest = min(seen.values())' in GATE_SRC, (
        'the floor check no longer derives its bound from the geoms that actually made contact')
    assert 'if own > lowest:' in GATE_SRC, (
        'the floor check can no longer fail: without a comparison against the measured bound it '
        'could only ever report PASS')
    assert "'cell_ground'" in GATE_SRC, 'the floor check no longer targets the shared floor geom'
    # The value has to be READ from the geom. A literal would go stale silently when the merge
    # changes -- and a text search cannot tell code from the docstring that quotes the current
    # number, which is exactly how the first version of this assertion failed.
    assert 'own = float(m.geom_friction[floor][0])' in GATE_SRC, (
        "the floor check no longer reads the floor geom's own friction")
    assert 'SHARED_GROUND_FRICTION' in MERGE_SRC
