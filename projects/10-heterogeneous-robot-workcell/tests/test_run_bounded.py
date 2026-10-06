"""Short tests for run identity, deadline enforcement and evidence preservation."""
import importlib.util
import json
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('run_bounded', ROOT / 'scripts/run_bounded.py')
runner = importlib.util.module_from_spec(spec)
spec.loader.exec_module(runner)


@pytest.fixture
def project(tmp_path, monkeypatch):
    (tmp_path / 'experiments').mkdir()
    monkeypatch.setattr(runner, 'ROOT', tmp_path)
    return tmp_path


@pytest.mark.parametrize('run_id', ['../outside', '', '/tmp/run', 'a/b'])
def test_invalid_identity_creates_nothing(project, run_id):
    with pytest.raises(SystemExit):
        runner.main(['--run-id', run_id, '--timeout', '1', 'experiments/probe.py'])
    assert not (project / 'runtime').exists()


def test_exit_code_and_hash_are_preserved(project):
    script = project / 'experiments/probe.py'
    script.write_text('print("measured failure")\nraise SystemExit(2)\n')
    code = runner.main(['--run-id', 'known-failure', '--timeout', '5', 'experiments/probe.py'])
    manifest = json.loads((project / 'reports/known-failure/run_manifest.json').read_text())
    assert code == manifest['exit_code'] == 2
    assert manifest['files_sha256']['experiments/probe.py'] == runner.fingerprint(script)
    assert manifest['child_pid'] > 0
    assert 'measured failure' in (project / 'runtime/bounded_runs/known-failure/stdout.log').read_text()
    with pytest.raises(SystemExit):
        runner.main(['--run-id', 'known-failure', '--timeout', '5', 'experiments/probe.py'])


def test_deadline_keeps_failure_evidence(project):
    (project / 'experiments/probe.py').write_text('import time\ntime.sleep(30)\n')
    code = runner.main(['--run-id', 'timeout-case', '--timeout', '0.1', 'experiments/probe.py'])
    manifest = json.loads((project / 'reports/timeout-case/run_manifest.json').read_text())
    assert code == manifest['exit_code'] == 124
    assert manifest['status'] == 'WALL_TIMEOUT'


@pytest.mark.parametrize('budget', ['nan', 'inf', '0', '-1'])
def test_invalid_deadline_creates_nothing(project, budget):
    with pytest.raises(SystemExit):
        runner.main(['--run-id', 'run', '--timeout', budget, 'experiments/probe.py'])
    assert not (project / 'runtime').exists()
