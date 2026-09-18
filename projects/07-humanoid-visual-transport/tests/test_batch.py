import importlib.util
from pathlib import Path

spec = importlib.util.spec_from_file_location('batch007', Path(__file__).resolve().parents[1] / 'scripts/batch.py')
batch = importlib.util.module_from_spec(spec)
spec.loader.exec_module(batch)


def test_reproducible_bounded_cases():
    suite = batch.cases(7)
    assert suite == batch.cases(7)
    assert len(suite) == len({c['id'] for c in suite}) == 30
    assert suite[:21] == batch.cases(8)[:21]
    assert suite[21:] != batch.cases(8)[21:]
    for c in suite:
        assert abs(c['box'][0]) <= .02 and abs(c['box'][1]) <= .01
        assert abs(c['target'][0]) <= .05 and abs(c['target'][1]) <= .03
        assert abs(c['yaw']) <= 5


def test_success_requires_report_and_exit():
    assert batch.classify(0, {'status':'COMPLETED'}) == 'COMPLETED'
    assert batch.classify(0, None) == 'MISSING_REPORT'
    assert batch.classify(0, {'status':'FAILED'}) == 'INCONSISTENT_EXIT'
    assert batch.classify(1, {'status':'COMPLETED'}) == 'INCONSISTENT_EXIT'
    assert batch.classify(-15, {'status':'COMPLETED'}, True) == 'WALL_TIMEOUT'


def test_report_path_confined():
    import pytest
    assert batch.report_path('no report') is None
    with pytest.raises(ValueError):
        batch.report_path('Report: /tmp/outside')


def test_acceptance_rejects_completed_tasks_with_contacts_or_missing_evidence():
    import copy
    good=dict(cases=[dict(id='one')],inputs_unchanged=True,
              results=[dict(case=dict(id='one'),status='COMPLETED',
                            station_contact_diagnostics=dict(contact_free=True,pairs={}))])
    assert batch.acceptance(good)['passed']
    for mode in ('contact','missing','incomplete','changed','failed','duplicate'):
        sample=copy.deepcopy(good)
        if mode=='contact':sample['results'][0]['station_contact_diagnostics']=dict(contact_free=False,pairs={'hip':{}})
        elif mode=='missing':sample['results'][0].pop('station_contact_diagnostics')
        elif mode=='incomplete':sample['cases'].append(dict(id='two'))
        elif mode=='changed':sample['inputs_unchanged']=False
        elif mode=='failed':sample['results'][0]['status']='FAILED'
        else:sample['results'].append(copy.deepcopy(sample['results'][0]))
        assert not batch.acceptance(sample)['passed'],mode
