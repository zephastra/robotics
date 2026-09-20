"""Backend-neutral adapter contract (canonical definitions live here).

Imported by both the fake backend (P1) and the Nav2 backend (P2+). Nothing here
touches ROS: the Nav2 adapter is the only place that will.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from typing import Protocol


class AdapterPhase(str, Enum):
    IDLE = "IDLE"
    MOVING = "MOVING"
    ARRIVED = "ARRIVED"
    STOPPING = "STOPPING"
    STOPPED = "STOPPED"
    FAULTED = "FAULTED"


@dataclass
class MotionCommand:
    task_id: str
    robot_id: str
    generation: int
    target_x: float
    target_y: float
    target_yaw: float = 0.0


@dataclass
class AdapterStatus:
    robot_id: str
    backend: str
    phase: AdapterPhase
    generation: int
    x: float = 0.0
    y: float = 0.0
    speed_mps: float = 0.0
    message: str = ""
    stopped_confirmed: bool = False
    arrived: bool = False
    # Identity of the process that produced this status. A permit issued under a
    # different boot is unverifiable and must not authorise motion.
    boot_id: str = ""
    epoch: int = 0
    # --- inputs the P4 geofence needs ------------------------------------ #
    # A protected-crossing test is a *geometric* claim: it needs the whole
    # footprint (hence yaw) and the current turn rate (hence angular_radps),
    # otherwise a robot rotating in place next to the boundary looks still.
    yaw: float = 0.0
    angular_radps: float = 0.0
    # Defaults to False, i.e. fail closed (docs/DECISIONS.md D-P4-07). An
    # adapter that has not said its localization is healthy does not get to
    # enter a protected region on the strength of a missing field.
    localization_valid: bool = False


class Adapter(Protocol):
    """The only surface the fleet core may use to move a robot.

    Rules the contract encodes:

      * One live command generation at a time. A new command supersedes the old;
        callbacks carrying the superseded generation are dropped.
      * `cancel()` acknowledges a request. It does NOT claim motion stopped.
        Only `status().stopped_confirmed` may be treated as stopped.
      * Losing the adapter is not stopping. A robot whose adapter died keeps its
        last velocity until something downstream intervenes, so the fleet must
        treat it as UNKNOWN, never as safe.
    """

    backend: str

    def start(self, cmd: MotionCommand) -> None: ...

    def tick(self, dt_s: float) -> AdapterStatus: ...

    def cancel(self, *, generation: int) -> bool: ...

    def status(self) -> AdapterStatus: ...

    def shutdown(self) -> None: ...
