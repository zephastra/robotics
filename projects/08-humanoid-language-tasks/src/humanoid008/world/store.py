"""World state storage with semantic revisioning.

``world_revision`` is **not** a frame counter (section 25.3). It advances only
when a decision-relevant fact changes: an entity moves or disappears, a station's
occupancy changes, or the robot's holding state changes. Sampling alone only
advances ``snapshot_seq``.

If ``world_revision`` advanced on every tick, every async planner response would
be permanently stale.
"""

from __future__ import annotations

from typing import Mapping

from ..contracts import (
    CapabilityState,
    EntityObservation,
    RobotState,
    StationState,
    WorldState,
)


class WorldStore:
    """Mutable world state plus revision bookkeeping."""

    def __init__(self, *, sim_time_s: float = 0.0) -> None:
        self._revision = 0
        self._seq = 0
        self._sim_time = float(sim_time_s)
        self._robot = RobotState(
            holding_object_id=None, current_skill=None, hand_contact_n=0.0
        )
        self._entities: dict[str, EntityObservation] = {}
        self._stations: dict[str, StationState] = {}

    # ------------------------------------------------------------------ #
    # readers
    # ------------------------------------------------------------------ #

    @property
    def world_revision(self) -> int:
        return self._revision

    @property
    def snapshot_seq(self) -> int:
        return self._seq

    def parts(
        self,
    ) -> tuple[
        RobotState,
        Mapping[str, EntityObservation],
        Mapping[str, StationState],
    ]:
        """Raw parts, so a capability provider can inspect them before snapshot()."""
        return self._robot, dict(self._entities), dict(self._stations)

    # ------------------------------------------------------------------ #
    # mutation
    # ------------------------------------------------------------------ #

    def _touch(self, semantic: bool) -> None:
        if semantic:
            self._revision += 1

    def observe_entity(self, observation: EntityObservation, *, semantic: bool = True) -> None:
        previous = self._entities.get(observation.entity_id)
        if previous is not None and not _entity_changed(previous, observation):
            semantic = False
        self._entities[observation.entity_id] = observation
        self._touch(semantic)

    def remove_entity(self, entity_id: str, *, semantic: bool = True) -> None:
        if self._entities.pop(entity_id, None) is not None:
            self._touch(semantic)

    def set_station(self, station: StationState, *, semantic: bool = True) -> None:
        previous = self._stations.get(station.station_id)
        if previous is not None and previous.status is station.status:
            semantic = False
        self._stations[station.station_id] = station
        self._touch(semantic)

    def set_robot(self, robot: RobotState, *, semantic: bool = True) -> None:
        if (
            self._robot.holding_object_id == robot.holding_object_id
            and self._robot.current_skill is robot.current_skill
        ):
            semantic = False
        self._robot = robot
        self._touch(semantic)

    def advance_time(self, seconds: float) -> None:
        """Time passing is never a semantic change."""
        self._sim_time += float(seconds)

    # ------------------------------------------------------------------ #
    # snapshot
    # ------------------------------------------------------------------ #

    def view(self) -> WorldState:
        """A WorldState for evaluating preconditions.

        Capabilities are *placeholders* here (everything provisionally available)
        because they are what we are about to compute. This does not advance
        ``snapshot_seq``, because it is not a sample.
        """
        from ..contracts import CapabilityState, SkillName

        placeholder = CapabilityState(available=tuple(SkillName), blocked={})
        return self._build(placeholder)

    def snapshot(self, capabilities: CapabilityState) -> WorldState:
        """Produce a validated, revisioned snapshot. Advances snapshot_seq."""
        self._seq += 1
        return self._build(capabilities)

    def _build(self, capabilities: CapabilityState) -> WorldState:
        return WorldState.from_dict(
            {
                "schema_version": "1.0",
                "world_revision": self._revision,
                "snapshot_seq": self._seq,
                "sim_time_s": round(self._sim_time, 6),
                "robot": self._robot.to_dict(),
                "entities": {k: e.to_dict() for k, e in self._entities.items()},
                "stations": {k: s.to_dict() for k, s in self._stations.items()},
                "capabilities": capabilities.to_dict(),
            }
        )


def _entity_changed(before: EntityObservation, after: EntityObservation) -> bool:
    """A change that matters: visibility, or a meaningful position shift."""
    if before.visible != after.visible:
        return True
    if (before.x is None) != (after.x is None):
        return True
    if (before.y is None) != (after.y is None):
        return True
    if before.x is not None and after.x is not None:
        if abs(before.x - after.x) > 0.01 or abs(before.y - after.y) > 0.01:
            return True
    return False
