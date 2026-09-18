"""Skills: the only components allowed to change the robot's actions."""

from .base import PreconditionError, Skill, arguments_for
from .fake import FakeRuntime, FakeScene, build_fake_registry
from .registry import SkillRegistry, load_skill_declarations

__all__ = [
    "FakeRuntime",
    "FakeScene",
    "PreconditionError",
    "Skill",
    "SkillRegistry",
    "arguments_for",
    "build_fake_registry",
    "load_skill_declarations",
]
