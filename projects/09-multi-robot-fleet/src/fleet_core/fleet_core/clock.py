"""Clocks for 009.

Two independent timelines, on purpose:

  * **sim** time drives physics, battery drain and "stopped for N sim seconds".
  * **wall** time drives watchdog deadlines and permit TTL.

They must not be conflated. CONTRACTS requires that pausing the simulator stops
sim time but keeps the wall watchdog armed (TEST_AND_ACCEPTANCE I04), so a permit
that expires on wall time must still expire when sim is frozen.

`epoch` increments on `reset()`. Anything carrying an epoch from before the reset
is stale and must be rejected rather than reused (I05).
"""

from __future__ import annotations

import time
from dataclasses import dataclass, field
from typing import Protocol


class Clock(Protocol):
    @property
    def epoch(self) -> int: ...

    def sim_now(self) -> float: ...

    def wall_now(self) -> float: ...

    def paused(self) -> bool: ...


@dataclass
class FakeClock:
    """Deterministic clock for tests. Never sleeps; tests advance it."""

    _sim: float = 0.0
    _wall: float = 0.0
    _epoch: int = 0
    _paused: bool = False

    @property
    def epoch(self) -> int:
        return self._epoch

    def sim_now(self) -> float:
        return self._sim

    def wall_now(self) -> float:
        return self._wall

    def paused(self) -> bool:
        return self._paused

    # -- test controls ----------------------------------------------------- #

    def advance_sim(self, dt: float) -> None:
        """Advance both timelines. Normal operation."""
        if dt < 0:
            raise ValueError("cannot advance backwards")
        if self._paused:
            raise RuntimeError("sim is paused; use advance_wall or resume()")
        self._sim += dt
        self._wall += dt

    def advance_wall(self, dt: float) -> None:
        """Advance wall time only -- what a watchdog sees while sim is frozen."""
        if dt < 0:
            raise ValueError("cannot advance backwards")
        self._wall += dt

    def pause(self) -> None:
        self._paused = True

    def resume(self) -> None:
        self._paused = False

    def reset(self, *, sim: float = 0.0, wall: float | None = None) -> None:
        """A simulator restart. Invalidates every prior epoch."""
        self._sim = sim
        self._wall = self._wall if wall is None else wall
        self._epoch += 1


@dataclass
class SimClock:
    """Real clock fed by an external /clock source (used from P2 on).

    Wall time is monotonic process time. Sim time is whatever the caller last
    published; until then it stays at 0 and `paused()` reports True, which makes
    the watchdog conservative rather than optimistic.
    """

    _sim: float = 0.0
    _wall: float = field(default_factory=time.monotonic)
    _epoch: int = 0
    stale_after_s: float = 1.0
    _last_sim_update_wall: float = field(default_factory=time.monotonic)

    @property
    def epoch(self) -> int:
        return self._epoch

    def sim_now(self) -> float:
        return self._sim

    def wall_now(self) -> float:
        return time.monotonic()

    def paused(self) -> bool:
        return (time.monotonic() - self._last_sim_update_wall) > self.stale_after_s

    def update_sim(self, sim_time: float) -> None:
        self._sim = sim_time
        self._last_sim_update_wall = time.monotonic()

    def reset(self) -> None:
        self._sim = 0.0
        self._epoch += 1
        self._last_sim_update_wall = time.monotonic()
