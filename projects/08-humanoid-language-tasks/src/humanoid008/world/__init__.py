"""World state, entity grounding and station occupancy."""

from .grounding import GroundingError, resolve_object_id, resolve_station_id
from .occupancy import describe, from_fake_evidence, is_available_target
from .store import WorldStore

__all__ = [
    "GroundingError",
    "WorldStore",
    "describe",
    "from_fake_evidence",
    "is_available_target",
    "resolve_object_id",
    "resolve_station_id",
]
