"""L1 §12: sim time vs wall time, pause behaviour, clock reset.

Maps to TEST_AND_ACCEPTANCE L1 item 12 and cases I04/I05:
  * pausing the simulator must not stop the wall watchdog
  * a clock reset must end the current run rather than reuse old state

These are the tests that stop a frozen simulator from looking like a healthy
fleet. If sim and wall time were the same variable, a paused world would freeze
every deadline and the fleet would wait forever.
"""

from __future__ import annotations

import pytest

from fleet_core import ReasonCode
from fleet_core.clock import FakeClock, SimClock


# --------------------------------------------------------------------------- #
# two timelines
# --------------------------------------------------------------------------- #


def test_normal_operation_advances_both_timelines(clock):
    clock.advance_sim(5.0)
    assert clock.sim_now() == pytest.approx(5.0)
    assert clock.wall_now() == pytest.approx(5.0)


def test_pause_freezes_sim_but_not_wall(clock):
    clock.advance_sim(3.0)
    clock.pause()
    clock.advance_wall(100.0)
    assert clock.sim_now() == pytest.approx(3.0), "a paused world makes no sim progress"
    assert clock.wall_now() == pytest.approx(103.0), "wall time keeps running regardless"
    assert clock.paused() is True


def test_cannot_advance_sim_while_paused(clock):
    clock.pause()
    with pytest.raises(RuntimeError):
        clock.advance_sim(1.0)


def test_resume_continues_from_the_frozen_sim_time(clock):
    clock.advance_sim(3.0)
    clock.pause()
    clock.advance_wall(50.0)
    clock.resume()
    clock.advance_sim(1.0)
    assert clock.sim_now() == pytest.approx(4.0)
    assert clock.wall_now() == pytest.approx(54.0)


def test_negative_advance_is_rejected(clock):
    with pytest.raises(ValueError):
        clock.advance_sim(-1.0)
    with pytest.raises(ValueError):
        clock.advance_wall(-1.0)


# --------------------------------------------------------------------------- #
# I04: a permit must expire on wall time even while the sim is frozen
# --------------------------------------------------------------------------- #


def test_permit_ttl_expires_on_wall_time_while_sim_is_paused(book, cfg, clock):
    book.permit_ttl_s = 10.0
    res = book.acquire(
        task_id="task-A", robot_id="r01", resources=(cfg.corridor,),
        now=clock.sim_now(), wall_now=clock.wall_now(),
        boot_id="b1", revision=0, generation=1, epoch=clock.epoch,
    )
    assert res.granted is True

    clock.pause()
    clock.advance_wall(11.0)
    assert clock.sim_now() == 0.0, "sim never moved"

    expired = book.expire_due(wall_now=clock.wall_now())
    assert len(expired) == 1, "the wall watchdog must fire even with sim frozen"
    assert book.is_free(cfg.corridor) is False, "expiry is not clearance"


def test_permit_does_not_expire_while_both_clocks_are_within_ttl(book, cfg, clock):
    book.permit_ttl_s = 10.0
    book.acquire(
        task_id="task-A", robot_id="r01", resources=(cfg.corridor,),
        now=clock.sim_now(), wall_now=clock.wall_now(),
        boot_id="b1", revision=0, generation=1, epoch=clock.epoch,
    )
    clock.advance_sim(5.0)
    assert book.expire_due(wall_now=clock.wall_now()) == []
    assert book.owner_of(cfg.corridor) == "task-A"


# --------------------------------------------------------------------------- #
# I05: reset invalidates the old epoch
# --------------------------------------------------------------------------- #


def test_reset_bumps_the_epoch(clock):
    start = clock.epoch
    clock.reset(sim=0.0)
    assert clock.epoch == start + 1


def test_reset_zeroes_sim_and_keeps_wall(clock):
    clock.advance_sim(20.0)
    wall_before = clock.wall_now()
    clock.reset(sim=0.0)
    assert clock.sim_now() == 0.0
    assert clock.wall_now() >= wall_before, "wall clock is monotonic across a reset"


def test_repeated_resets_keep_counting(clock):
    for _ in range(5):
        clock.reset()
    assert clock.epoch == 5


def test_reset_can_move_sim_time_arbitrarily(clock):
    clock.advance_sim(10.0)
    clock.reset(sim=1234.0)
    assert clock.sim_now() == 1234.0


# --------------------------------------------------------------------------- #
# SimClock: conservative when the feed is missing
# --------------------------------------------------------------------------- #


def test_simclock_starts_paused_when_no_clock_feed_arrives():
    sc = SimClock(stale_after_s=0.0)
    # stale_after_s=0 means "anything is stale", i.e. no feed yet -> paused.
    assert sc.paused() is True


def test_simclock_reports_not_paused_after_a_fresh_update():
    sc = SimClock(stale_after_s=100.0)
    sc.update_sim(4.2)
    assert sc.sim_now() == pytest.approx(4.2)
    assert sc.paused() is False


def test_simclock_reset_bumps_epoch_and_zeroes_sim():
    sc = SimClock()
    sc.update_sim(9.0)
    e0 = sc.epoch
    sc.reset()
    assert sc.sim_now() == 0.0
    assert sc.epoch == e0 + 1


def test_simclock_wall_is_monotonic():
    sc = SimClock()
    a = sc.wall_now()
    b = sc.wall_now()
    assert b >= a
