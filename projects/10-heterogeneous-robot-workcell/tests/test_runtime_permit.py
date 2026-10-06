import sys
from pathlib import Path
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src'))
from workcell.runtime_permit import RuntimePermit


def setup():
    now = [10.]
    sample = dict(owner='transfer_1', epoch=1, generation=1, sequence=0,
                  observed_wall_s=10., cancel_requested=False, zone_clear=True,
                  stop_chain_healthy=True)
    guard = RuntimePermit(lambda: dict(sample), owner='transfer_1', epoch=1,
                          generation=1, max_age_s=.5, clock=lambda: now[0])
    return guard, now, sample


def test_clear_then_intrusion_is_latched_even_after_clear_returns():
    guard, now, sample = setup()
    assert guard.check()['allowed']
    sample['zone_clear'] = False
    assert not guard.check()['allowed']
    sample['zone_clear'] = True
    assert not guard.check()['allowed']


@pytest.mark.parametrize('field,value', [('zone_clear', None), ('zone_clear', 1),
    ('stop_chain_healthy', None), ('cancel_requested', None), ('generation', 0),
    ('epoch', True), ('owner', 'other'), ('observed_wall_s', float('nan')),
    ('observed_wall_s', 11.), ('sequence', -1)])
def test_unknown_stale_and_contradictory_evidence_is_refused(field, value):
    guard, now, sample = setup()
    sample[field] = value
    assert not guard.check()['allowed']


def test_wall_expiry_works_without_sim_time_advancing():
    guard, now, sample = setup()
    assert guard.check()['allowed']
    now[0] = 10.6
    assert guard.check()['reason_code'] == 'STALE_OBSERVATION'


def test_recovery_requires_stop_new_generation_and_fresh_evidence():
    guard, now, sample = setup()
    sample['cancel_requested'] = True
    assert not guard.check()['allowed']
    sample['cancel_requested'] = False
    with pytest.raises(ValueError):
        guard.rearm(generation=2, stopped_confirmed=False)
    with pytest.raises(ValueError):
        guard.rearm(generation=1, stopped_confirmed=True)
    with pytest.raises(ValueError):
        guard.rearm(generation=2, stopped_confirmed=True)
    assert not guard.check()['allowed']
    sample['generation'] = 2
    assert guard.rearm(generation=2, stopped_confirmed=True)['allowed']


def test_regressed_wall_clock_is_not_fresh():
    guard, now, sample = setup()
    assert guard.check()['allowed']
    now[0] = sample['observed_wall_s'] = 9.
    assert guard.check()['reason_code'] == 'CLOCK_RESET'


def test_sensor_exception_cannot_open_permission():
    def observe():
        raise RuntimeError('missing sensor')
    guard = RuntimePermit(observe, owner='t', epoch=1, generation=1, max_age_s=.5)
    assert not guard.check()['allowed']


def test_sensor_stamp_during_collection_is_not_a_future_stamp():
    guard, now, sample = setup()
    def observe():
        now[0] += .001
        sample['observed_wall_s'] = now[0]
        return dict(sample)
    guard.observe = observe
    assert guard.check()['allowed']
