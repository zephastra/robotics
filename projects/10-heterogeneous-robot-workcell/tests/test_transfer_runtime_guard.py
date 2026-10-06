from types import SimpleNamespace
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from test_belt_terminal import Instrument
import w4_plant as wp


def setup(monkeypatch, allowed_ticks):
    instrument = Instrument(['source_band'])
    checks = [0]

    def check():
        checks[0] += 1
        return dict(allowed=checks[0] <= allowed_ticks, reason_code='RESOURCE_UNKNOWN')

    instrument.motion_guard = SimpleNamespace(check=check,
        stop=lambda reason, detail: dict(allowed=False, reason_code=reason))
    instrument._stop_transfer = lambda: dict(stopped_confirmed=True)
    monkeypatch.setattr(wp.mujoco, 'mj_name2id', lambda *a: -1)

    def step(model, data):
        instrument.ticks += 1
        data.time += model.opt.timestep

    monkeypatch.setattr(wp.mujoco, 'mj_step', step)
    return instrument


def test_intrusion_prevents_next_driven_tick(monkeypatch):
    instrument = setup(monkeypatch, 2)
    result = instrument.transfer(direction='onto_deck', timeout_s=1.)
    assert result['state'] == 'STOPPED'
    assert result['interrupted'] is True
    assert instrument.ticks == 2
    assert result['reason_code'] == 'RESOURCE_UNKNOWN'


def test_unknown_permission_does_not_start_band(monkeypatch):
    instrument = setup(monkeypatch, 0)
    result = instrument.transfer(direction='onto_deck', timeout_s=1.)
    assert instrument.ticks == 0
    assert result['state'] == 'STOPPED'
    assert instrument.deployments == []


def test_driving_timeout_latches_and_brakes_not_seats(monkeypatch):
    instrument = setup(monkeypatch, 999)
    result = instrument.transfer(direction='onto_deck', timeout_s=.002)
    assert instrument.ticks == 1
    assert result['reason_code'] == 'TRANSFER_TIMEOUT'
    assert not result['pusher_seated']


def test_stop_ack_without_measured_stop_is_unknown(monkeypatch):
    instrument = setup(monkeypatch, 0)
    instrument._stop_transfer = lambda: dict(stopped_confirmed=False)
    result = instrument.transfer(direction='onto_deck', timeout_s=1.)
    assert result['state'] == 'UNKNOWN'
