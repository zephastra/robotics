"""World store, semantic revisioning, occupancy and grounding tests."""

from __future__ import annotations

import pytest

from humanoid008.contracts import (
    CapabilityState,
    EntityObservation,
    EvidenceSource,
    ReasonCode,
    StationState,
    StationStatus,
    WorldState,
)
from humanoid008.world import (
    GroundingError,
    WorldStore,
    is_available_target,
    resolve_object_id,
    resolve_station_id,
)

NO_CAPS = CapabilityState(available=(), blocked={})


def entity(
    entity_id: str,
    *,
    category: str = "box",
    x: float = 0.0,
    y: float = 0.0,
    visible: bool = True,
    observed_at_s: float = 0.0,
) -> EntityObservation:
    return EntityObservation(
        entity_id=entity_id,
        category=category,
        x=x,
        y=y,
        visible=visible,
        observed_at_s=observed_at_s,
        source=EvidenceSource.FAKE_WORLD,
    )


def station(
    station_id: str,
    *,
    status: StationStatus = StationStatus.FREE,
) -> StationState:
    return StationState(
        station_id=station_id,
        status=status,
        source=EvidenceSource.FAKE_WORLD,
        observed_at_s=0.0,
    )


def make_world(*, entities=(), stations=()) -> WorldState:
    store = WorldStore()
    for item in entities:
        store.observe_entity(item)
    for item in stations:
        store.set_station(item)
    return store.snapshot(NO_CAPS)


# --------------------------------------------------------------------------- #
# revisioning (section 25.3)
# --------------------------------------------------------------------------- #


def test_time_and_sampling_do_not_advance_world_revision():
    store = WorldStore()
    store.observe_entity(entity("box_01"))
    baseline = store.world_revision

    store.advance_time(1.0)
    store.snapshot(NO_CAPS)
    store.snapshot(NO_CAPS)

    assert store.world_revision == baseline


def test_repeated_identical_observation_does_not_advance_revision():
    store = WorldStore()
    store.observe_entity(entity("box_01", x=1.0))
    baseline = store.world_revision

    store.observe_entity(entity("box_01", x=1.0))

    assert store.world_revision == baseline


def test_real_movement_advances_revision():
    store = WorldStore()
    store.observe_entity(entity("box_01", x=1.0))
    baseline = store.world_revision

    store.observe_entity(entity("box_01", x=2.0))

    assert store.world_revision == baseline + 1


def test_visibility_change_advances_revision():
    store = WorldStore()
    store.observe_entity(entity("box_01", visible=True))
    baseline = store.world_revision

    store.observe_entity(entity("box_01", visible=False))

    assert store.world_revision == baseline + 1


def test_snapshot_seq_always_advances():
    store = WorldStore()
    store.snapshot(NO_CAPS)
    store.snapshot(NO_CAPS)
    assert store.snapshot_seq == 2


# --------------------------------------------------------------------------- #
# occupancy (section 11)
# --------------------------------------------------------------------------- #


def test_missing_station_is_not_available():
    assert not is_available_target(None)


def test_unknown_station_is_not_available():
    """UNKNOWN must block, exactly like OCCUPIED."""
    assert not is_available_target(station("station_b", status=StationStatus.UNKNOWN))


def test_occupied_station_is_not_available():
    assert not is_available_target(station("station_b", status=StationStatus.OCCUPIED))


def test_free_station_is_available():
    assert is_available_target(station("station_b", status=StationStatus.FREE))


# --------------------------------------------------------------------------- #
# grounding (sections 11 and 25.2)
# --------------------------------------------------------------------------- #


def test_grounding_binds_a_unique_visible_object():
    world = make_world(entities=[entity("box_01")])
    assert resolve_object_id(["box_01"], world) == "box_01"


def test_grounding_reports_unknown_entity():
    world = make_world()
    with pytest.raises(GroundingError) as excinfo:
        resolve_object_id(["box_99"], world)
    assert excinfo.value.reason_code is ReasonCode.UNKNOWN_ENTITY


def test_grounding_reports_ambiguity_instead_of_guessing():
    world = make_world(entities=[entity("box_01"), entity("box_02")])
    with pytest.raises(GroundingError) as excinfo:
        resolve_object_id(["box_01", "box_02"], world)
    assert excinfo.value.reason_code is ReasonCode.AMBIGUOUS_REFERENCE


def test_grounding_refuses_an_invisible_object():
    world = make_world(entities=[entity("box_01", visible=False)])
    with pytest.raises(GroundingError) as excinfo:
        resolve_object_id(["box_01"], world)
    assert excinfo.value.reason_code is ReasonCode.TARGET_NOT_VISIBLE


def test_grounding_can_skip_the_visibility_requirement():
    world = make_world(entities=[entity("box_01", visible=False)])
    assert resolve_object_id(["box_01"], world, require_visible=False) == "box_01"


def test_grounding_binds_a_unique_station():
    world = make_world(stations=[station("station_b")])
    assert resolve_station_id(["station_b"], world) == "station_b"


def test_grounding_ignores_empty_candidates():
    world = make_world(entities=[entity("box_01")])
    assert resolve_object_id(["", None, "box_01"], world) == "box_01"  # type: ignore[list-item]
