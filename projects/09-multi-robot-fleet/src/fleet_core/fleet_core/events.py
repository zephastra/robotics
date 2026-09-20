"""Append-only event log.

Every state change the fleet makes goes through here, so a report can be
reconstructed from events alone and the evaluator never has to trust a status
field. `seq` is monotonic per log; `epoch` rides along so a post-restart reader
can tell pre-reset events from post-reset ones.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Iterator


@dataclass(frozen=True)
class Event:
    seq: int
    sim_t: float
    wall_t: float
    epoch: int
    kind: str
    subject: str
    data: dict[str, Any] = field(default_factory=dict)

    def as_dict(self) -> dict[str, Any]:
        return {
            "seq": self.seq,
            "sim_t": self.sim_t,
            "wall_t": self.wall_t,
            "epoch": self.epoch,
            "kind": self.kind,
            "subject": self.subject,
            **self.data,
        }


class EventLog:
    """In-memory event log. Persistence lives in the ledger."""

    def __init__(self) -> None:
        self._events: list[Event] = []

    def emit(
        self,
        *,
        sim_t: float,
        wall_t: float,
        epoch: int,
        kind: str,
        subject: str,
        **data: Any,
    ) -> Event:
        ev = Event(
            seq=len(self._events) + 1,
            sim_t=sim_t,
            wall_t=wall_t,
            epoch=epoch,
            kind=kind,
            subject=subject,
            data=dict(data),
        )
        self._events.append(ev)
        return ev

    def __len__(self) -> int:
        return len(self._events)

    def __iter__(self) -> Iterator[Event]:
        return iter(self._events)

    def of_kind(self, kind: str) -> list[Event]:
        return [e for e in self._events if e.kind == kind]

    def for_subject(self, subject: str) -> list[Event]:
        return [e for e in self._events if e.subject == subject]

    def counts(self) -> dict[str, int]:
        out: dict[str, int] = {}
        for e in self._events:
            out[e.kind] = out.get(e.kind, 0) + 1
        return out
