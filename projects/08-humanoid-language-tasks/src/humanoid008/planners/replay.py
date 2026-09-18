"""Recorded-response replay (section 12).

Replays a previously recorded sequence of proposals. Two rules:

- replay output is labelled ``replay`` and is **never** reported as a model
  result;
- the volatile request fields (``request_id``, ``task_id``, ``goal_revision``,
  ``world_revision``) are rewritten at replay time, so the same recording can be
  re-run against a fresh world instead of tripping a revision check.

Replay exists to exercise the *loop* deterministically. It proves nothing about
any model.
"""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any, Iterable, Mapping, Sequence

from ..contracts import ActionProposal, GoalSpec, SkillResult, WorldState, strict_json_loads
from .base import Planner


class ReplayPlanner(Planner):
    name = "replay"

    def __init__(self, proposals: Sequence[Mapping[str, Any]]) -> None:
        self._proposals = [dict(item) for item in proposals]

    @property
    def length(self) -> int:
        return len(self._proposals)

    @classmethod
    def from_jsonl(cls, path: str | Path) -> "ReplayPlanner":
        text = Path(path).read_text(encoding="utf-8")
        rows = [strict_json_loads(line) for line in text.splitlines() if line.strip()]
        return cls(rows)

    def propose(
        self,
        *,
        goal: GoalSpec,
        world: WorldState,
        history: Sequence[SkillResult],
        request_id: str,
    ) -> ActionProposal | None:
        # The step index is the number of skills already finished. A rejected
        # proposal ends the task rather than advancing, so this stays monotonic.
        index = len(history)
        if index >= len(self._proposals):
            return None
        template = dict(self._proposals[index])
        template.update(
            {
                "request_id": request_id,
                "task_id": goal.task_id,
                "goal_revision": goal.goal_revision,
                "world_revision": world.world_revision,
            }
        )
        return ActionProposal.from_dict(template)


def record_proposals(rows: Iterable[Mapping[str, Any]], path: str | Path) -> Path:
    """Write a proposal recording as JSONL. Refuses to clobber an existing file."""
    target = Path(path)
    if target.exists():
        raise FileExistsError(f"refusing to overwrite existing recording: {target}")
    target.parent.mkdir(parents=True, exist_ok=True)
    with target.open("w", encoding="utf-8") as handle:
        for row in rows:
            handle.write(json.dumps(dict(row), ensure_ascii=False) + "\n")
    return target
