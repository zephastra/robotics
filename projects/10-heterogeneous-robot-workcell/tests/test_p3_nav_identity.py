"""Tests for the two defects W1 fixed in `evaluate_p3_nav.py`.

Every test here exists because the old code passed it. Specifically:

  * the identity tests name their run `run-001` and `p3-nav-99-loaded`, which is the point: the old
    judge decided the physics from `'loaded' in args.run_id`, so a loaded run whose id lacked the
    word was judged as bare and an unloaded run whose id contained it was judged as loaded. Both
    tests would have failed against the old code, in opposite directions.
  * the verdict tests cover INCOMPLETE, which the old `Results.verdict()` could not return: it built
    its denominator by filtering NOT_RUN out, so a required check that never ran could not stop a
    PASS. `test_a_required_check_that_did_not_run_cannot_pass` is the one that would have caught it.
"""
import json
import shutil
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'experiments'))

import evaluate_p3_nav as judge  # noqa: E402

LAYOUT = json.loads((ROOT / 'assets' / 'worlds' / 'world_p3_nav.layout.json').read_text())
BARE = ROOT / 'assets' / 'worlds' / 'world_p3_nav.xml'
LOADED = ROOT / 'assets' / 'worlds' / 'world_p3_nav_loaded.xml'
REQUIRED_ALL = judge.PROFILE['required']
NA_ROW = next(iter(judge.PROFILE['not_applicable']))


def sha(path):
    import hashlib
    return hashlib.sha256(path.read_bytes()).hexdigest()


def record(world_path, world_sha, config='config/p3_nav.yaml', config_sha=None):
    config_sha = config_sha or sha(ROOT / config)
    return {'world': str(world_path), 'world_sha256': world_sha,
            'config_path': config, 'config_sha256': config_sha}


def plan_for(config, world_is_registered=True):
    return {'sim_config': config, 'sim_config_sha256': sha(ROOT / config)}


# ---------------------------------------------------------------- identity by CONTENT ----

def test_a_loaded_run_named_run_001_is_still_loaded():
    """The guidance's first minimum test, and the one the old code fails."""
    ident = judge.resolve_identity(
        'run-001', plan_for('config/p3_nav_loaded.yaml'),
        record(LOADED, sha(LOADED), config='config/p3_nav_loaded.yaml'), LAYOUT)
    assert ident['variant'] == 'loaded', ident['detail']
    assert ident['status'] == 'PASS'
    assert 'STAND-IN RIGID MOUNT' in ident['detail']


def test_a_bare_run_whose_id_says_loaded_is_still_bare():
    """The same test in the other direction: the id must not be able to lie either way."""
    ident = judge.resolve_identity(
        'p3-nav-99-loaded', plan_for('config/p3_nav.yaml'),
        record(BARE, sha(BARE)), LAYOUT)
    assert ident['variant'] == 'bare', ident['detail']
    assert 'no load is modelled' in ident['detail']


def test_identity_is_by_content_not_by_path(tmp_path):
    """A copy of the loaded world at another path is still the loaded variant: a string comparison
    on the path is not an identity, and the guidance asks explicitly for that distinction."""
    copy = tmp_path / 'somewhere_else.xml'
    copy.write_bytes(LOADED.read_bytes())
    assert sha(copy) == sha(LOADED)
    ident = judge.resolve_identity(
        'whatever', plan_for('config/p3_nav_loaded.yaml'),
        record(copy, sha(copy), config='config/p3_nav_loaded.yaml'), LAYOUT)
    assert ident['variant'] == 'loaded', ident['detail']
    # and the differing filename is REPORTED rather than refused: identity is the content hash
    assert 'somewhere_else.xml' in ident['detail']
    assert 'recorded rather than refused' in ident['detail']


def test_a_path_that_matches_but_content_that_does_not_is_refused():
    """`world_path` exists and is a real file, but the recorded hash belongs to another world."""
    ident = judge.resolve_identity(
        'x', plan_for('config/p3_nav.yaml'), record(BARE, '0' * 64), LAYOUT)
    assert ident['status'] == 'FAIL'
    assert ident['code'] == 'EVIDENCE_STALE'
    assert ident['arena'] is None


def test_a_world_the_layout_does_not_know_is_refused(tmp_path):
    """The file exists and its hash checks out -- against ITSELF. The layout has never heard of it,
    and an unregistered variant must be refused rather than judged against a default."""
    stranger = tmp_path / 'world_from_elsewhere.xml'
    stranger.write_bytes(b'<mujoco><worldbody><body name="x"/></worldbody></mujoco>')
    ident = judge.resolve_identity(
        'x', plan_for('config/p3_nav.yaml'), record(stranger, sha(stranger)), LAYOUT)
    assert ident['status'] == 'FAIL'
    assert ident['code'] == 'EVIDENCE_UNREGISTERED', ident['detail']
    assert ident['arena'] is None


def test_missing_identity_does_not_default_to_bare():
    """It must refuse, not fall back: a default is the same guess as reading the id."""
    empty = {'config_path': 'config/p3_nav.yaml', 'config_sha256': 'a' * 64}
    ident = judge.resolve_identity('x', None, empty, LAYOUT)
    assert ident['status'] == 'FAIL'
    assert ident['code'] == 'EVIDENCE_MISSING'
    assert ident['arena'] is None


def test_a_config_disagreement_is_a_conflict_not_a_choice():
    """nav_plan says bare, the simulator says loaded. Picking the favourable one is the defect."""
    ident = judge.resolve_identity(
        'x', plan_for('config/p3_nav.yaml'),
        record(LOADED, sha(LOADED), config='config/p3_nav_loaded.yaml'), LAYOUT)
    assert ident['code'] == 'EVIDENCE_CONFLICT', ident['detail']
    assert ident['arena'] is None


# ---------------------------------------------------------------- the aggregation ----

def rows(*names, status='PASS'):
    return [{'name': name, 'status': status, 'detail': ''} for name in names]


def test_a_required_check_that_did_not_run_cannot_pass():
    """The verdict the old aggregator could not express."""
    made = rows(*REQUIRED_ALL)
    made[0]['status'] = 'NOT_RUN'
    verdict, reasons = judge.verdict_of(made, judge.PROFILE)
    assert verdict == 'INCOMPLETE', verdict
    assert REQUIRED_ALL[0] in reasons


def test_a_required_check_that_failed_fails():
    made = rows(*REQUIRED_ALL)
    made[2]['status'] = 'FAIL'
    verdict, reasons = judge.verdict_of(made, judge.PROFILE)
    assert verdict == 'FAIL'
    assert REQUIRED_ALL[2] in reasons


def test_a_not_applicable_row_is_not_evidence():
    """The yaw row is declared in advance. It is reported, and it contributes nothing to PASS."""
    made = rows(*REQUIRED_ALL) + [{'name': NA_ROW, 'status': 'NOT_APPLICABLE', 'detail': ''}]
    verdict, _reasons = judge.verdict_of(made, judge.PROFILE)
    assert verdict == 'PASS'
    assert NA_ROW not in judge.PROFILE['required']
    assert NA_ROW in judge.PROFILE['not_applicable']


def test_everything_not_applicable_is_incomplete_not_a_pass():
    """Each required row is present but carries no result. The honest answer is INCOMPLETE: the
    profile asked for eight checks and got nothing, so PASS would mean nothing."""
    made = [{'name': name, 'status': 'NOT_APPLICABLE', 'detail': ''} for name in REQUIRED_ALL]
    verdict, reasons = judge.verdict_of(made, judge.PROFILE)
    assert verdict == 'INCOMPLETE', verdict
    assert sorted(reasons) == sorted(REQUIRED_ALL)


def test_a_duplicate_check_id_is_a_config_error():
    made = rows(*REQUIRED_ALL) + [{'name': REQUIRED_ALL[0], 'status': 'PASS', 'detail': ''}]
    verdict, reasons = judge.verdict_of(made, judge.PROFILE)
    assert verdict == 'CONFIG_ERROR'
    assert 'duplicate' in ' '.join(reasons)


def test_an_unknown_status_is_a_config_error():
    made = rows(*REQUIRED_ALL)
    made[1]['status'] = 'PROBABLY_FINE'
    verdict, reasons = judge.verdict_of(made, judge.PROFILE)
    assert verdict == 'CONFIG_ERROR'
    assert 'does not define' in ' '.join(reasons)


def test_an_undeclared_row_is_a_config_error():
    made = rows(*REQUIRED_ALL) + [{'name': 'a brand new check', 'status': 'PASS', 'detail': ''}]
    verdict, reasons = judge.verdict_of(made, judge.PROFILE)
    assert verdict == 'CONFIG_ERROR'
    assert 'does not declare' in ' '.join(reasons)


def test_an_empty_required_set_is_a_config_error():
    made = rows(*REQUIRED_ALL)
    verdict, reasons = judge.verdict_of(made, {'required': (), 'not_applicable': {}})
    assert verdict == 'CONFIG_ERROR'
    assert 'no required checks' in ' '.join(reasons)


# ---------------------------------------------------------------- end to end on copies ----

def run_judge(source, name, tmp_path):
    dest = tmp_path / name
    shutil.copytree(ROOT / 'reports' / source, dest)
    completed = subprocess.run(
        [sys.executable, str(ROOT / 'experiments' / 'evaluate_p3_nav.py'),
         '--run-id', name, '--reports-dir', str(tmp_path)],
        capture_output=True, text=True)
    gate = json.loads((dest / 'p3_nav_gate.json').read_text())
    return completed.returncode, gate


def test_the_real_loaded_run_keeps_its_identity_under_a_renamed_directory(tmp_path):
    _code, gate = run_judge('p3-nav-06', 'run-001', tmp_path)
    assert gate['variant'] == 'loaded', gate['identity']['detail']
    assert gate['verdict'] == 'PASS'
    assert gate['identity']['code'] is None


def test_the_real_bare_run_keeps_its_identity_under_a_misleading_name(tmp_path):
    _code, gate = run_judge('p3-nav-05', 'zz-loadedd', tmp_path)
    assert gate['variant'] == 'bare', gate['identity']['detail']
    assert gate['verdict'] == 'PASS'


def test_the_gate_records_the_profile_and_the_reasons(tmp_path):
    _code, gate = run_judge('p3-nav-06', 'profiled', tmp_path)
    assert gate['profile']['id'] == 'P3-NAV-02'
    assert gate['profile']['version'] >= 2
    assert set(gate['profile']['required']) == set(REQUIRED_ALL)
    assert gate['counts']['not_run'] == 0
    assert gate['counts']['not_applicable'] == 1
    assert gate['verdict_reasons'] == []
    names = [r['name'] for r in gate['checks']]
    assert len(names) == len(set(names))


def test_the_falsifiability_row_names_the_checks_without_a_probe(tmp_path):
    """It used to claim "every check has been shown able to fail" while covering 5 of 9 rows."""
    _code, gate = run_judge('p3-nav-06', 'probes', tmp_path)
    row = next(r for r in gate['checks'] if r['name'].startswith('each falsifiability probe'))
    assert row['status'] == 'PASS'
    assert 'without one' in row['detail']
    assert 'every check has been shown able to fail' not in row['name']
    covered = {name for probe in gate['falsifiability'] for name in probe['covers']}
    assert REQUIRED_ALL[0] in covered          # the identity row now has a probe
    assert len(covered) == len(REQUIRED_ALL) - 1   # only the probe row itself cannot probe itself
