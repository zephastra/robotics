"""The closed skill registry (section 10).

The registry is the authority on what may ever be proposed or executed; a
proposal naming a skill outside this set is rejected.

It hands out a **fresh** skill instance per run (skills are stateful and must not
carry state between runs), and it computes the world's capability state so a
planner can see which skills are currently blocked and why.
"""

from __future__ import annotations

from pathlib import Path
from typing import Callable, Mapping

import yaml

from ..contracts import CapabilityState, GoalSpec, SkillName, WorldState
from .base import PreconditionError, Skill, arguments_for

SkillFactory = Callable[[], Skill]


class SkillRegistry:
    def __init__(
        self,
        factories: Mapping[SkillName, SkillFactory],
        arg_names: Mapping[SkillName, tuple[str, ...]],
    ) -> None:
        self._factories = dict(factories)
        self._arg_names = dict(arg_names)

    def names(self) -> tuple[SkillName, ...]:
        return tuple(self._factories)

    def __contains__(self, name: object) -> bool:
        return isinstance(name, SkillName) and name in self._factories

    def new_skill(self, name: SkillName) -> Skill:
        """A fresh, untried instance. Raises KeyError if not registered."""
        factory = self._factories.get(name)
        if factory is None:
            raise KeyError(f"skill {name} is not registered")
        skill = factory()
        skill.arg_names = self._arg_names.get(name, ())
        return skill

    def argument_names(self, name: SkillName) -> tuple[str, ...]:
        if name not in self._factories:
            raise KeyError(f"skill {name} is not registered")
        return self._arg_names.get(name, ())

    def arguments_for(self, name: SkillName, goal: GoalSpec) -> dict[str, str]:
        if name not in self._factories:
            raise KeyError(f"skill {name} is not registered")
        return arguments_for(self._arg_names.get(name, ()), goal)

    def capabilities(self, world: WorldState, goal: GoalSpec) -> CapabilityState:
        """Which skills could start right now, and why the others cannot."""
        available: list[SkillName] = []
        blocked: dict[str, str] = {}
        for name in self._factories:
            skill = self.new_skill(name)
            try:
                skill.check_preconditions(world, arguments_for(self._arg_names.get(name, ()), goal))
            except PreconditionError as exc:
                blocked[name.value] = str(exc)
            else:
                available.append(name)
        return CapabilityState(available=tuple(available), blocked=blocked)


def load_skill_declarations(path: str | Path) -> dict[SkillName, tuple[str, ...]]:
    """Read the declared argument names for every skill from the registry config."""
    raw = yaml.safe_load(Path(path).read_text(encoding="utf-8")) or {}
    entries = raw.get("skills")
    if not isinstance(entries, list) or not entries:
        raise ValueError("skill registry config must declare a non-empty 'skills' list")
    table: dict[SkillName, tuple[str, ...]] = {}
    for entry in entries:
        if not isinstance(entry, Mapping) or "name" not in entry:
            raise ValueError("each skill entry needs a 'name'")
        table[SkillName(str(entry["name"]))] = tuple(
            str(item) for item in entry.get("args", [])
        )
    return table
