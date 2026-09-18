"""Fake skills and their in-memory runtime.

**This is a fake backend.** There is no physics, no renderer and no MuJoCo here.
It exists so the offline loop can be exercised deterministically without any
simulation stack.

Two rules are enforced by construction:

1. ``scene.yaml`` coordinates are *initialisation data only*. Entities and
   stations enter the world **unobserved**; only an OBSERVE_* skill turns them
   into observations (section 11, 25.5).
2. Anything produced through this runtime must be reported as
   ``execution_backend = fake`` and ``physical_outcome = NOT_EVALUATED``
   (section 25.4). Fake success is never evidence about a robot.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any, Callable, Iterable, Mapping

import yaml

from ..contracts import (
    EntityObservation,
    EvidenceSource,
    GoalSpec,
    ReasonCode,
    RobotState,
    SkillName,
    SkillStatus,
    StationState,
    StationStatus,
    WorldState,
)
from ..world import occupancy
from ..world.store import WorldStore
from .base import PreconditionError, Skill
from .registry import SkillRegistry, load_skill_declarations

FAKE_DURATION_S = 0.2


# --------------------------------------------------------------------------- #
# scene
# --------------------------------------------------------------------------- #


@dataclass(frozen=True)
class FakeScene:
    scene_id: str
    objects: Mapping[str, Mapping[str, Any]]
    stations: Mapping[str, bool]

    @classmethod
    def load(cls, path: str | Path) -> "FakeScene":
        raw = yaml.safe_load(Path(path).read_text(encoding="utf-8")) or {}
        objects = {
            str(key): dict(value) for key, value in (raw.get("entities") or {}).items()
        }
        stations = {
            str(key): bool((value or {}).get("occupied", False))
            for key, value in (raw.get("stations") or {}).items()
        }
        if not objects:
            raise ValueError("fake scene must declare at least one entity")
        return cls(scene_id=str(raw.get("scene_id", "unnamed")), objects=objects, stations=stations)


# --------------------------------------------------------------------------- #
# runtime
# --------------------------------------------------------------------------- #


class FakeRuntime:
    """In-memory fake robot. Deterministic and side-effect free outside itself."""

    def __init__(self, scene: FakeScene, *, faults: Iterable[SkillName] = ()) -> None:
        self.scene = scene
        self._faults = set(faults)
        self._store = WorldStore()
        self._sim_time = 0.0

        # Initialisation data. Never read by control code directly -- only the
        # OBSERVE_* effects below may copy it into the world as observations.
        self._truth = {key: dict(value) for key, value in scene.objects.items()}
        self._station_truth = dict(scene.stations)

        self._visible: dict[str, bool] = {key: False for key in scene.objects}
        self._observed_at: dict[str, float] = {key: 0.0 for key in scene.objects}
        self._station_observed_at: dict[str, float] = {key: 0.0 for key in scene.stations}

        self._object_state: dict[str, str] = {key: "on_source" for key in scene.objects}
        self._held: str | None = None
        self._placed_target: str | None = None
        self._carried_to: str | None = None
        self._contact_n = 0.0
        self._current_skill: SkillName | None = None

        self._publish()

    # ------------------------------------------------------------------ #
    # world access
    # ------------------------------------------------------------------ #

    def view(self) -> WorldState:
        return self._store.view()

    def snapshot(self, capabilities) -> WorldState:
        return self._store.snapshot(capabilities)

    def advance(self, dt_s: float) -> None:
        self._sim_time += dt_s
        self._store.advance_time(dt_s)

    def faults(self, skill: SkillName) -> bool:
        return skill in self._faults

    @property
    def sim_time_s(self) -> float:
        return self._sim_time

    # ------------------------------------------------------------------ #
    # effects
    # ------------------------------------------------------------------ #

    def note_skill(self, skill: SkillName | None) -> None:
        self._current_skill = skill
        self._publish()

    def observe_object(self, entity_id: str) -> None:
        self._visible[entity_id] = True
        self._observed_at[entity_id] = self._sim_time
        self._publish()

    def observe_target(self, station_id: str) -> None:
        self._station_observed_at[station_id] = self._sim_time
        self._publish()

    def grasp(self, entity_id: str) -> None:
        self._held = entity_id
        self._contact_n = 1.5
        self._object_state[entity_id] = "held"
        self._publish()

    def lift(self, entity_id: str) -> None:
        self._object_state[entity_id] = "held"
        self._publish()

    def carry(self, target_id: str) -> None:
        self._carried_to = target_id
        self._publish()

    def place(self, entity_id: str, target_id: str) -> None:
        self._held = None
        self._contact_n = 0.0
        self._object_state[entity_id] = "placed"
        self._placed_target = target_id
        self._station_truth[target_id] = True
        self._visible[entity_id] = True
        self._observed_at[entity_id] = self._sim_time
        self._station_observed_at[target_id] = self._sim_time
        self._publish()

    # ------------------------------------------------------------------ #
    # facts the skills use for preconditions
    # ------------------------------------------------------------------ #

    @property
    def held(self) -> str | None:
        return self._held

    @property
    def placed_target(self) -> str | None:
        return self._placed_target

    @property
    def carried_to(self) -> str | None:
        return self._carried_to

    # ------------------------------------------------------------------ #
    # projection into the world
    # ------------------------------------------------------------------ #

    def _publish(self) -> None:
        for entity_id, spec in self._truth.items():
            state = self._object_state[entity_id]
            visible = self._visible[entity_id]
            x = y = None
            if visible and state == "on_source":
                x = float(spec.get("x", 0.0))
                y = float(spec.get("y", 0.0))
            self._store.observe_entity(
                EntityObservation(
                    entity_id=entity_id,
                    category=str(spec.get("category", "object")),
                    x=x,
                    y=y,
                    visible=visible,
                    observed_at_s=self._observed_at[entity_id],
                    source=EvidenceSource.FAKE_WORLD,
                )
            )

        for station_id, occupied in self._station_truth.items():
            observed_at = self._station_observed_at[station_id]
            status = (
                StationStatus.UNKNOWN
                if observed_at <= 0.0
                else occupancy.from_fake_evidence(occupied)
            )
            self._store.set_station(
                StationState(
                    station_id=station_id,
                    status=status,
                    source=EvidenceSource.FAKE_WORLD,
                    observed_at_s=observed_at,
                )
            )

        self._store.set_robot(
            RobotState(
                holding_object_id=self._held,
                current_skill=self._current_skill,
                hand_contact_n=self._contact_n,
            )
        )


# --------------------------------------------------------------------------- #
# skills
# --------------------------------------------------------------------------- #


class _FakeSkill(Skill):
    """Shared timing / fault skeleton for every fake skill."""

    duration_s = FAKE_DURATION_S
    fault_reason = ReasonCode.PRECONDITION_FAILED

    def __init__(self, runtime: FakeRuntime) -> None:
        super().__init__()
        self._runtime = runtime
        self._args: dict[str, str] = {}
        self._elapsed = 0.0

    # lifecycle -------------------------------------------------------- #

    def start(self, world: WorldState, args: Mapping[str, str]) -> None:
        self._args = dict(args)
        self._elapsed = 0.0
        self._status = SkillStatus.RUNNING
        self._runtime.note_skill(self.name)

    def tick(self, world: WorldState, dt_s: float) -> None:
        if self.is_finished:
            return
        self._elapsed += dt_s
        self._runtime.advance(dt_s)
        if self._elapsed + 1e-9 < self.duration_s:
            return
        if self._runtime.faults(self.name):
            self._finish(SkillStatus.FAILED, self.fault_reason)
        else:
            self._effect(world)
            self._finish(SkillStatus.SUCCEEDED, ReasonCode.OK)
        self._runtime.note_skill(None)

    def request_cancel(self, reason: str) -> None:
        if not self.is_finished:
            self._finish(SkillStatus.CANCELLED, ReasonCode.CANCELLED)
            self._runtime.note_skill(None)

    def world_delta(self) -> Mapping[str, Any]:
        return {}

    def _effect(self, world: WorldState) -> None:  # pragma: no cover - overridden
        raise NotImplementedError


def _require(value: bool, message: str) -> None:
    if not value:
        raise PreconditionError(message)


class ObserveObject(_FakeSkill):
    name = SkillName.OBSERVE_OBJECT

    def check_preconditions(self, world, args):
        entity_id = args.get("object_id", "")
        _require(bool(entity_id), "no object_id was supplied")
        _require(entity_id in world.entities, f"unknown object '{entity_id}'")

    def _effect(self, world):
        self._runtime.observe_object(self._args["object_id"])


class ObserveTarget(_FakeSkill):
    name = SkillName.OBSERVE_TARGET

    def check_preconditions(self, world, args):
        target_id = args.get("target_id", "")
        _require(bool(target_id), "no target_id was supplied")
        _require(world.station(target_id) is not None, f"unknown station '{target_id}'")

    def _effect(self, world):
        self._runtime.observe_target(self._args["target_id"])


class ApproachObject(_FakeSkill):
    name = SkillName.APPROACH_OBJECT

    def check_preconditions(self, world, args):
        entity_id = args.get("object_id", "")
        entity = world.entity(entity_id)
        _require(entity is not None, f"unknown object '{entity_id}'")
        _require(entity.visible, f"'{entity_id}' has not been observed yet")
        _require(self._runtime.held is None, "already holding something")

    def _effect(self, world):
        return None


class GraspObject(_FakeSkill):
    name = SkillName.GRASP_OBJECT
    fault_reason = ReasonCode.GRASP_NOT_CONFIRMED

    def check_preconditions(self, world, args):
        entity_id = args.get("object_id", "")
        entity = world.entity(entity_id)
        _require(entity is not None, f"unknown object '{entity_id}'")
        _require(entity.visible, f"'{entity_id}' has not been observed yet")
        _require(self._runtime.held is None, "already holding something")

    def _effect(self, world):
        self._runtime.grasp(self._args["object_id"])


class LiftObject(_FakeSkill):
    name = SkillName.LIFT_OBJECT

    def check_preconditions(self, world, args):
        entity_id = args.get("object_id", "")
        _require(self._runtime.held == entity_id, f"'{entity_id}' is not held")

    def _effect(self, world):
        self._runtime.lift(self._args["object_id"])


class CarryToTarget(_FakeSkill):
    name = SkillName.CARRY_TO_TARGET

    def check_preconditions(self, world, args):
        target_id = args.get("target_id", "")
        _require(world.station(target_id) is not None, f"unknown station '{target_id}'")
        _require(self._runtime.held is not None, "nothing is held")

    def _effect(self, world):
        self._runtime.carry(self._args["target_id"])


class PlaceObject(_FakeSkill):
    name = SkillName.PLACE_OBJECT
    fault_reason = ReasonCode.PLACEMENT_OUT_OF_TOLERANCE

    def check_preconditions(self, world, args):
        object_id = args.get("object_id", "")
        target_id = args.get("target_id", "")
        _require(self._runtime.held == object_id, f"'{object_id}' is not held")
        station = world.station(target_id)
        _require(station is not None, f"unknown station '{target_id}'")
        _require(
            occupancy.is_available_target(station),
            f"destination {occupancy.describe(station)}",
        )

    def _effect(self, world):
        self._runtime.place(self._args["object_id"], self._args["target_id"])


class VerifyResult(_FakeSkill):
    name = SkillName.VERIFY_RESULT

    def check_preconditions(self, world, args):
        target_id = args.get("target_id", "")
        _require(
            self._runtime.placed_target == target_id,
            f"nothing has been placed at '{target_id}'",
        )

    def _effect(self, world):
        return None


_FAKE_SKILL_TYPES = (
    ObserveObject,
    ObserveTarget,
    ApproachObject,
    GraspObject,
    LiftObject,
    CarryToTarget,
    PlaceObject,
    VerifyResult,
)


def build_fake_registry(
    registry_config: str | Path,
    runtime: FakeRuntime,
) -> SkillRegistry:
    """Build the fake skill set for exactly the skills declared in the config."""
    declared = load_skill_declarations(registry_config)
    by_name = {skill_type.name: skill_type for skill_type in _FAKE_SKILL_TYPES}
    factories: dict[SkillName, Callable[[], Skill]] = {}
    for name in declared:
        skill_type = by_name.get(name)
        if skill_type is None:
            raise ValueError(f"no fake implementation for declared skill {name}")
        factories[name] = (lambda st=skill_type: st(runtime))
    return SkillRegistry(factories, declared)
