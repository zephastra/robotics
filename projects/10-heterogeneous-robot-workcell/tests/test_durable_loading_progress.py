import sys
import json
from pathlib import Path
from types import SimpleNamespace
import pytest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'experiments'))
from probe_candidate_multi_loading import write_progress


def test_checkpoints_are_unique_and_never_claim_full_order(tmp_path):
    report=dict(scope='test',initial_count={'status':'UNKNOWN'})
    write_progress(tmp_path,report,SimpleNamespace(time=1.),'unknown')
    first=(tmp_path/'checkpoints/000.json').read_bytes()
    write_progress(tmp_path,report,SimpleNamespace(time=2.),'next')
    assert (tmp_path/'checkpoints/000.json').read_bytes()==first
    saved=json.loads((tmp_path/'checkpoints/001.json').read_text())
    assert saved['full_order']=='NOT_RUN' and not saved['v1_complete']
    assert saved['initial_count']['status']=='UNKNOWN'


def test_reused_checkpoint_refuses_overwrite(tmp_path):
    report=dict(scope='test')
    write_progress(tmp_path,report,SimpleNamespace(time=1.),'one')
    report['checkpoint_count']=0
    with pytest.raises(RuntimeError,match='overwrite'):
        write_progress(tmp_path,report,SimpleNamespace(time=2.),'two')
