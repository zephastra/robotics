"""Protocol instruments for real transfer code; not physical acceptance."""
import sys
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'experiments'))
import w4_plant as wp


class Instrument:
    transfer = wp.LogisticsPlant.transfer

    def __init__(self, support):
        self.support = support
        self.pitch = 0.08
        self.tray_envelope_radius_m = 0.2
        self.on_crowns = True
        self.x = 1.0
        self.trailing = 1.0
        self.model = SimpleNamespace(nu=0, opt=SimpleNamespace(timestep=0.002))
        self.data = SimpleNamespace(time=0.0, ctrl=np.zeros(0))
        self.pusher_limits = {'lift': [0.0, 0.001], 'slide': [0.0, 0.01]}
        self.pusher_command = {'lift': 0.0, 'slide': 0.0}
        self.ticks = 0
        self.deployments = []

    def tray_support(self):
        return list(self.support)

    def tray_state(self):
        return {'supported_by': self.tray_support(), 'x_trailing_m': self.trailing,
                'x_leading_m': 1.1,
                'x_m': self.x, 'on_crowns': self.on_crowns,
                'end_s': self.data.time}

    def _hold(self):
        return np.zeros(0)

    def row_window(self, row):
        return (0.0, 2.0)

    def deploy_pusher(self, **command):
        self.deployments.append(command)
        self.pusher_command.update(command)

    def chassis_speed(self):
        return 0.0

    def pusher_position(self):
        return dict(self.pusher_command)


@pytest.fixture
def instrument(monkeypatch):
    current = Instrument(['receiver_band'])
    monkeypatch.setattr(wp.mujoco, 'mj_name2id', lambda *a: -1)

    def step(model, data):
        current.ticks += 1
        data.time += model.opt.timestep

    monkeypatch.setattr(wp.mujoco, 'mj_step', step)
    return current


def test_already_received_transfer_does_not_tick_or_deploy(instrument):
    result = instrument.transfer(direction='onto_receiver', timeout_s=4.0)
    assert result['state'] == 'RECEIVED'
    assert instrument.ticks == 0
    assert instrument.deployments == []


def test_unknown_direction_is_rejected_before_any_motion(instrument):
    with pytest.raises(ValueError):
        instrument.transfer(direction='typo', timeout_s=4.0)
    assert instrument.ticks == 0
    assert instrument.deployments == []


def test_no_support_is_unknown_not_delivered(instrument):
    instrument.support = []
    result = instrument.transfer(direction='onto_receiver', timeout_s=4.0)
    assert result['state'] == 'UNKNOWN'
    assert instrument.ticks == 0
    assert instrument.deployments == []


def test_loading_terminal_is_also_motion_free(instrument):
    instrument.support = ['deck']
    result = instrument.transfer(direction='onto_deck', timeout_s=4.0)
    assert result['state'] == 'RECEIVED'
    assert instrument.ticks == 0
    assert instrument.deployments == []


def test_tilted_receiver_contact_is_not_delivery(instrument):
    instrument.on_crowns = False
    result = instrument.transfer(direction='onto_receiver', timeout_s=4.0)
    assert result['state'] == 'UNKNOWN'
    assert instrument.ticks == 0


def test_centre_arrived_but_trailing_edge_outside_is_not_delivery(instrument):
    instrument.trailing = -0.01
    result = instrument.transfer(direction='onto_receiver', timeout_s=4.0)
    assert result['state'] == 'UNKNOWN'
    assert instrument.ticks == 0


def test_receiver_entrance_contact_is_not_delivery(instrument):
    instrument.x = 0.01
    result = instrument.transfer(direction='onto_receiver', timeout_s=4.0)
    assert result['state'] == 'UNKNOWN'
    assert instrument.ticks == 0


def test_current_yaw_clear_but_future_rotation_not_clear_is_not_delivery(instrument):
    instrument.x = 0.15
    instrument.trailing = 0.03
    result = instrument.transfer(direction='onto_receiver', timeout_s=4.0)
    assert result['state'] == 'UNKNOWN'
    assert instrument.ticks == 0


def test_unload_arrival_has_no_seating_run_on(instrument, monkeypatch):
    instrument.support = ['deck']

    def arrive(model, data):
        instrument.ticks += 1
        data.time += model.opt.timestep
        instrument.support = ['receiver_band']

    monkeypatch.setattr(wp.mujoco, 'mj_step', arrive)
    result = instrument.transfer(direction='onto_receiver', timeout_s=4.0)
    assert result['state'] == 'RECEIVED'
    assert instrument.ticks == 1


def test_budget_exhaustion_has_no_extra_seating_ticks(instrument):
    instrument.support = ['source_band']
    result = instrument.transfer(direction='onto_deck', timeout_s=0.002)
    assert result['state'] == 'TRANSFERRING'
    assert instrument.ticks == 1


@pytest.mark.parametrize('duration', [0.0, -1.0, float('nan'), float('inf')])
def test_invalid_budget_is_rejected_before_motion(instrument, duration):
    with pytest.raises(ValueError):
        instrument.transfer(direction='onto_receiver', timeout_s=duration)
    assert instrument.ticks == 0
