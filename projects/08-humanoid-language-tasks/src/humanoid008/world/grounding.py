"""Binding a language reference to exactly one entity or station.

House rule (sections 11 and 25.2): **we never guess.**

- zero matches  -> ``UNKNOWN_ENTITY``
- more than one -> ``AMBIGUOUS_REFERENCE`` (must become a clarification question)
- not visible   -> ``TARGET_NOT_VISIBLE``

The caller supplies candidate ids (the rule parser derives them from an alias
table); this module is what decides whether the candidate set is usable.
"""

from __future__ import annotations

from typing import Iterable

from ..contracts import ReasonCode, WorldState


class GroundingError(Exception):
    """A reference could not be bound to exactly one usable entity."""

    def __init__(self, reason_code: ReasonCode, message: str) -> None:
        super().__init__(message)
        self.reason_code = reason_code


def _unique(candidates: Iterable[str]) -> list[str]:
    seen: list[str] = []
    for item in candidates:
        if item and item not in seen:
            seen.append(item)
    return seen


def resolve_object_id(
    candidates: Iterable[str],
    world: WorldState,
    *,
    require_visible: bool = True,
) -> str:
    matches = _unique(candidates)
    if not matches:
        raise GroundingError(
            ReasonCode.UNKNOWN_ENTITY, "no known object matches that reference"
        )
    if len(matches) > 1:
        raise GroundingError(
            ReasonCode.AMBIGUOUS_REFERENCE,
            f"that reference matches several objects: {sorted(matches)}",
        )
    entity_id = matches[0]
    entity = world.entity(entity_id)
    if entity is None:
        raise GroundingError(
            ReasonCode.UNKNOWN_ENTITY, f"'{entity_id}' is not present in the world"
        )
    if require_visible and not entity.visible:
        raise GroundingError(
            ReasonCode.TARGET_NOT_VISIBLE, f"'{entity_id}' is not currently visible"
        )
    return entity_id


def resolve_station_id(candidates: Iterable[str], world: WorldState) -> str:
    matches = _unique(candidates)
    if not matches:
        raise GroundingError(
            ReasonCode.UNKNOWN_ENTITY, "no known station matches that reference"
        )
    if len(matches) > 1:
        raise GroundingError(
            ReasonCode.AMBIGUOUS_REFERENCE,
            f"that reference matches several stations: {sorted(matches)}",
        )
    station_id = matches[0]
    if world.station(station_id) is None:
        raise GroundingError(
            ReasonCode.UNKNOWN_ENTITY, f"'{station_id}' is not present in the world"
        )
    return station_id
