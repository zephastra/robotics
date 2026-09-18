"""Target-station occupancy rules.

The rule this module exists to enforce: **UNKNOWN is not FREE.** A station whose
occupancy has not actually been observed may not receive an object, and it must
block execution exactly like OCCUPIED does.

Section 11: until a real depth/ROI occupancy algorithm exists, only the fake
backend may set occupancy -- hence :func:`from_fake_evidence`, which is
deliberately named so it can never be mistaken for a real detector.
"""

from __future__ import annotations

from ..contracts import StationState, StationStatus


def is_available_target(station: StationState | None) -> bool:
    """True only when a station is *known* to be free right now."""
    return station is not None and station.status is StationStatus.FREE


def describe(station: StationState | None) -> str:
    if station is None:
        return "no observation for that station"
    return f"{station.station_id} is {station.status.value}"


def from_fake_evidence(occupied: bool) -> StationStatus:
    """Fake-backend helper. The only place allowed to invent occupancy."""
    return StationStatus.OCCUPIED if occupied else StationStatus.FREE
