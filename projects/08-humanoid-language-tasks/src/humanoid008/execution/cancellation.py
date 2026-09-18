"""Stop and cancel handling (sections 13 and 14).

STOP always outranks ordinary queued work, and any in-flight model response.

In V1 both STOP and CANCEL resolve to an explicitly-labelled **simulation
freeze**: the simulation is frozen, the skill stops advancing, and the report is
saved. This is not a controlled hardware emergency stop -- there is no controlled
deceleration and no load handling here, and the report must not imply otherwise.

The token is a plain object with no locking: in V1 stop requests are set from the
same thread that runs the control loop, and the guard only needs to *read* it.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum


class StopKind(str, Enum):
    NONE = "none"
    STOP = "stop"
    CANCEL = "cancel"


@dataclass
class ControlToken:
    _kind: StopKind = StopKind.NONE
    _reason: str = ""
    _requested_at_s: float | None = None
    _handled_at_s: float | None = None

    # ------------------------------------------------------------------ #
    # requests
    # ------------------------------------------------------------------ #

    def request_stop(self, reason: str = "stop requested", now_s: float | None = None) -> None:
        """A user stop. Never downgraded by a later cancel."""
        if self._kind is StopKind.STOP:
            return
        self._kind = StopKind.STOP
        self._reason = reason
        self._requested_at_s = now_s
        self._handled_at_s = None

    def request_cancel(self, reason: str = "cancel requested", now_s: float | None = None) -> None:
        """Ordinary cancellation. Ignored if a stop is already pending."""
        if self._kind is not StopKind.NONE:
            return
        self._kind = StopKind.CANCEL
        self._reason = reason
        self._requested_at_s = now_s

    def mark_handled(self, now_s: float) -> None:
        """Record when the control loop actually noticed. Used for STOP latency."""
        if self._requested_at_s is not None and self._handled_at_s is None:
            self._handled_at_s = now_s

    # ------------------------------------------------------------------ #
    # reads
    # ------------------------------------------------------------------ #

    @property
    def stop_requested(self) -> bool:
        return self._kind is not StopKind.NONE

    @property
    def kind(self) -> StopKind:
        return self._kind

    @property
    def reason(self) -> str:
        return self._reason

    @property
    def requested_at_s(self) -> float | None:
        return self._requested_at_s

    @property
    def handled_at_s(self) -> float | None:
        return self._handled_at_s

    @property
    def latency_s(self) -> float | None:
        """Wall-clock seconds between request and handling, if both are known."""
        if self._requested_at_s is None or self._handled_at_s is None:
            return None
        return self._handled_at_s - self._requested_at_s
