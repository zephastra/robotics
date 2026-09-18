"""Layer 3 supervision: the run-time guard around an executing skill (section 13).

Checked on every tick, so it stays cheap:

- a stop or cancel request always wins, immediately
- an actuator command that has gone stale (older than the declared TTL) must not
  be applied
- a body/base fault (excessive tilt) stops the skill

``control`` is the token from :mod:`humanoid008.execution.cancellation`. It is
duck-typed rather than imported, to avoid a supervision <-> execution import
cycle.

This is a runtime check. It is **not** an ex-ante safety guarantee, and the
report must never describe it as one.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any, Mapping

import yaml

from ..contracts import ReasonCode
from .validator import OK, Verdict


@dataclass(frozen=True)
class RuntimePolicy:
    command_ttl_s: float = 0.5
    max_tilt_rad: float = 1.0

    @classmethod
    def from_mapping(cls, data: Mapping[str, Any]) -> "RuntimePolicy":
        guard = data.get("runtime_guard") or {}
        return cls(
            command_ttl_s=float(guard.get("command_ttl_s", 0.5)),
            max_tilt_rad=float(guard.get("max_tilt_rad", 1.0)),
        )

    @classmethod
    def load(cls, path: str | Path) -> "RuntimePolicy":
        raw = yaml.safe_load(Path(path).read_text(encoding="utf-8")) or {}
        return cls.from_mapping(raw)


class RuntimeGuard:
    def __init__(self, policy: RuntimePolicy, control) -> None:
        self._policy = policy
        self._control = control

    @property
    def policy(self) -> RuntimePolicy:
        return self._policy

    def check(
        self,
        *,
        now_s: float,
        command_issued_at_s: float | None = None,
        tilt_rad: float | None = None,
    ) -> Verdict:
        if getattr(self._control, "stop_requested", False):
            return Verdict(False, ReasonCode.CANCELLED, str(self._control.reason))

        if (
            command_issued_at_s is not None
            and now_s - command_issued_at_s > self._policy.command_ttl_s
        ):
            return Verdict(
                False,
                ReasonCode.PRECONDITION_FAILED,
                f"actuator command expired after "
                f"{now_s - command_issued_at_s:.3f}s (limit {self._policy.command_ttl_s:.3f}s)",
            )

        if tilt_rad is not None and abs(tilt_rad) > self._policy.max_tilt_rad:
            return Verdict(
                False,
                ReasonCode.PRECONDITION_FAILED,
                f"body tilt {tilt_rad:.3f} rad exceeds {self._policy.max_tilt_rad:.3f} rad",
            )

        return OK
