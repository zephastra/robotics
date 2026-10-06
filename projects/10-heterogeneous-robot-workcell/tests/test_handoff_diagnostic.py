"""Handoff reads explicit results without turning missing evidence into PASS."""
import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'experiments'))
import make_handoff as handoff


def test_independent_diagnostic_result_is_visible(tmp_path, monkeypatch):
    monkeypatch.setattr(handoff, 'ROOT', tmp_path)
    directory = tmp_path / 'reports' / 'limited'
    directory.mkdir(parents=True)
    (directory / 'acceptance.json').write_text(json.dumps({
        'diagnostic_result': 'INCOMPLETE_OR_FAIL', 'p4_exit_gate': 'INCOMPLETE'}))
    result = handoff.run_evidence('reports/limited', 'acceptance.json')
    assert result['state'] == 'OK'
    assert result['verdict'] == 'INCOMPLETE_OR_FAIL'


def test_h3_verdict_has_priority_over_process_completion(tmp_path, monkeypatch):
    monkeypatch.setattr(handoff, 'ROOT', tmp_path)
    directory = tmp_path / 'reports' / 'h3'
    directory.mkdir(parents=True)
    (directory / 'report.json').write_text(json.dumps({
        'h3_overall': 'FAIL: one required row failed', 'status': 'DIAGNOSTIC_COMPLETED'}))
    result = handoff.run_evidence('reports/h3', 'report.json')
    assert result['verdict'].startswith('FAIL')


def test_missing_judgment_remains_missing(tmp_path, monkeypatch):
    monkeypatch.setattr(handoff, 'ROOT', tmp_path)
    directory = tmp_path / 'reports' / 'unjudged'
    directory.mkdir(parents=True)
    (directory / 'report.json').write_text(json.dumps({'samples': []}))
    assert handoff.run_evidence('reports/unjudged', 'report.json')['state'] == 'NO_VERDICT'
