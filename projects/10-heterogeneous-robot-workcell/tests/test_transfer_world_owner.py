"""Exercise ACTUAL transfer code, not just the plant's wheel tick入口."""
from types import SimpleNamespace
import sys
from pathlib import Path

import pytest

sys.path.insert(0,str(Path(__file__).resolve().parent))
from test_belt_terminal import Instrument
import w4_plant as wp


def test_transfer_first_write_cannot_bypass_owner(monkeypatch):
    instrument=Instrument(['source_band'])
    calls=[]
    def deny(produce,*,advance=False):
        calls.append(advance)
        raise RuntimeError('owner closed')
    instrument.step_owner=SimpleNamespace(commit=deny)
    monkeypatch.setattr(wp.mujoco,'mj_name2id',lambda *a:-1)
    monkeypatch.setattr(wp.mujoco,'mj_step',lambda *a:pytest.fail('bypassed final writer'))
    with pytest.raises(RuntimeError,match='owner closed'):
        instrument.transfer(direction='onto_deck',timeout_s=.002)
    assert calls==[True]
    assert instrument.data.time==0 and instrument.ticks==0


def test_terminal_transfer_commit_also_uses_owner(monkeypatch):
    instrument=Instrument(['receiver_band'])
    commits=[]
    def commit(produce,*,advance=False):
        commits.append(advance)
        instrument.data.ctrl[:]=produce()
        if advance:instrument.data.time+=.002
    instrument.step_owner=SimpleNamespace(commit=commit)
    monkeypatch.setattr(wp.mujoco,'mj_name2id',lambda *a:-1)
    monkeypatch.setattr(wp.mujoco,'mj_step',lambda *a:pytest.fail('bypassed final writer'))
    result=instrument.transfer(direction='onto_receiver',timeout_s=.002)
    assert result['state']=='RECEIVED' and commits==[False]
    assert instrument.data.time==0


def test_driving_and_final_hold_both_use_owner(monkeypatch):
    instrument=Instrument(['source_band'])
    commits=[]
    def commit(produce,*,advance=False):
        commits.append(advance)
        instrument.data.ctrl[:]=produce()
        if advance:instrument.data.time+=.002
    instrument.step_owner=SimpleNamespace(commit=commit)
    monkeypatch.setattr(wp.mujoco,'mj_name2id',lambda *a:-1)
    monkeypatch.setattr(wp.mujoco,'mj_step',lambda *a:pytest.fail('bypassed final writer'))
    result=instrument.transfer(direction='onto_deck',timeout_s=.002)
    assert result['state']=='TRANSFERRING'
    assert commits==[True,False] and instrument.data.time==.002
