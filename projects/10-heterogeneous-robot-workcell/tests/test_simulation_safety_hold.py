from types import SimpleNamespace
import sys
from pathlib import Path
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent))
from test_belt_terminal import Instrument
import w4_plant as wp


def frozen():
    instrument = Instrument(['source_band'])
    instrument.safety_frozen = True
    instrument._frozen_brake = dict(stopped_confirmed=False, simulation_frozen=True,
                                   hold_mode='SIM_FROZEN_NO_PHYSICAL_STOP_PROOF')
    instrument.chassis_x = lambda: 1.
    return instrument


def test_frozen_transfer_does_not_tick_or_claim_stop():
    instrument = frozen()
    result = instrument.transfer(direction='onto_deck', timeout_s=5.)
    assert result['state'] == 'UNKNOWN'
    assert result['brake']['stopped_confirmed'] is False
    assert instrument.ticks == 0
    assert instrument.deployments == []


def test_frozen_vehicle_cannot_drive_after_new_request():
    result = wp.LogisticsPlant.drive_to(frozen(), 2., speed=.3, timeout_s=30.)
    assert result['travelled_m'] == 0.
    assert result['stopped_confirmed'] is False
    assert result['simulation_frozen'] is True


def test_reset_cannot_clear_unresolved_cargo_hold():
    with pytest.raises(RuntimeError):
        wp.LogisticsPlant.reset(frozen())


def test_direct_tick_cannot_bypass_freeze():
    with pytest.raises(RuntimeError):
        wp.LogisticsPlant._tick(frozen(), lambda side: 0.)
