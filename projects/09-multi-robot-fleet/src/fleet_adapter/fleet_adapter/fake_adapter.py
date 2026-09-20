"""Backend-neutral adapter contract.

The whole point of this file is that `fake` and `gazebo_nav2` are the *same*
interface with different backends, and the backend name travels with every
result. A report that cannot say which backend produced it is not evidence
(TEST_AND_ACCEPTANCE §1), so `backend` is part of the contract, not an afterthought.

Motion lifecycle rules the contract encodes:

  * One live command generation at a time. Starting a new command supersedes the
    old one; callbacks carrying the old generation are dropped.
  * `cancel()` acknowledges a request. It does NOT claim motion stopped.
    `stopped_confirmed` is the only thing that may be treated as stopped.
  * Losing the adapter is not stopping. A robot whose adapter died keeps whatever
    velocity it last had until something downstream intervenes, so the fleet must
    treat it as "unknown", never as "safe".
"""

from __future__ import annotations

from dataclasses import dataclass

from .adapter_base import Adapter, AdapterPhase, AdapterStatus, MotionCommand


@dataclass
class FakeAdapter:
    """Deterministic stand-in backend. **NOT a simulator and NOT a verification.**

    It moves a point at a fixed speed. It produces no physics, no collisions, no
    contacts and no sensor data, so nothing it returns may ever be reported as a
    physical or safety pass. The `backend` string is deliberately loud so that a
    report built on it cannot be mistaken for gazebo_nav2 evidence.
    """

    robot_id: str
    x: float = 0.0
    y: float = 0.0
    speed_mps: float = 0.35
    backend: str = "fake"
    boot_id: str = "fake-boot"
    epoch: int = 0
    _phase: AdapterPhase = AdapterPhase.IDLE
    _cmd: MotionCommand | None = None
    _generation: int = 0
    _cancel_generation: int | None = None
    _distance_travelled_m: float = 0.0
    _fault: str = ""
    _message: str = ""

    # -- Adapter ------------------------------------------------------- #

    def start(self, cmd: MotionCommand) -> None:
        if self._fault:
            self._message = f"refusing command while faulted: {self._fault}"
            return
        self._generation = cmd.generation
        self._cmd = cmd
        self._cancel_generation = None
        self._phase = AdapterPhase.MOVING
        self._message = "moving"

    def tick(self, dt_s: float) -> AdapterStatus:
        if dt_s < 0:
            raise ValueError("dt_s must be non-negative")

        if self._phase is AdapterPhase.MOVING and self._cmd is not None:
            tx, ty = self._cmd.target_x, self._cmd.target_y
            dx, dy = tx - self.x, ty - self.y
            dist = (dx * dx + dy * dy) ** 0.5
            step = self.speed_mps * dt_s
            if dist <= step or dist == 0.0:
                self.x, self.y = tx, ty
                self._distance_travelled_m += dist
                self._phase = AdapterPhase.ARRIVED
                self._message = "arrived"
            else:
                self.x += dx / dist * step
                self.y += dy / dist * step
                self._distance_travelled_m += step

        elif self._phase is AdapterPhase.STOPPING:
            # A fake backend can stop instantly; a real one cannot, which is why
            # the contract has a distinct STOPPING phase and a separate
            # `stopped_confirmed` flag rather than a bare `stopped` bool.
            self._phase = AdapterPhase.STOPPED
            self._message = "stopped"

        return self.status()

    def cancel(self, *, generation: int) -> bool:
        """Acknowledge a cancel. Returns False if the generation is not live."""
        if self._cmd is None or generation != self._generation:
            return False
        self._cancel_generation = generation
        self._phase = AdapterPhase.STOPPING
        self._message = "stopping after cancel"
        return True

    def status(self) -> AdapterStatus:
        speed = self.speed_mps if self._phase is AdapterPhase.MOVING else 0.0
        return AdapterStatus(
            robot_id=self.robot_id,
            backend=self.backend,
            phase=self._phase,
            generation=self._generation,
            x=self.x,
            y=self.y,
            speed_mps=speed,
            message=self._message,
            stopped_confirmed=self._phase
            in (AdapterPhase.STOPPED, AdapterPhase.FAULTED),
            arrived=self._phase is AdapterPhase.ARRIVED,
            boot_id=self.boot_id,
            epoch=self.epoch,
            # The fake has exact knowledge of its own pose, so its localization
            # is trivially valid. This is a statement about the stand-in, NOT a
            # claim that a real localizer is healthy -- a gazebo_nav2 adapter
            # must derive this from pose age and covariance.
            localization_valid=True,
        )

    def shutdown(self) -> None:
        self._phase = AdapterPhase.FAULTED
        self._message = "adapter shut down; last velocity NOT retracted"

    # -- fault injection (test-only helpers) --------------------------- #

    def inject_fault(self, reason: str) -> None:
        self._fault = reason
        self._phase = AdapterPhase.FAULTED
        self._message = f"fault: {reason}"

    @property
    def distance_travelled_m(self) -> float:
        return self._distance_travelled_m
