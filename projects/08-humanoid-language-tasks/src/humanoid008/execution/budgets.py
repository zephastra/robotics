"""Budgets that replanning cannot reset (section 13).

The model has no authority over these numbers, and starting a new internal task
does not reset them. There is deliberately **no reset method** on
:class:`BudgetLedger`: the only way to get fresh budget is to start a genuinely
new task with a genuinely new ledger.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Mapping

import yaml

#: Budget counters the supervisor is allowed to spend.
BUDGET_KEYS = (
    "max_steps_per_task",
    "model_calls_per_task",
    "replans_per_task",
    "retries_per_skill",
    "stance_realign_per_task",
)


class BudgetExhausted(RuntimeError):
    """A budget was exceeded. The task must stop and say so."""


@dataclass
class BudgetLedger:
    limits: Mapping[str, int]
    used: dict[str, int] = field(default_factory=dict)

    def __post_init__(self) -> None:
        self.limits = dict(self.limits)
        self.used = {key: 0 for key in self.limits}

    def can_spend(self, key: str, amount: int = 1) -> bool:
        if key not in self.limits:
            raise KeyError(f"unknown budget {key!r}")
        return self.used[key] + amount <= self.limits[key]

    def spend(self, key: str, amount: int = 1) -> None:
        if not self.can_spend(key, amount):
            raise BudgetExhausted(
                f"{key} exhausted: {self.used[key]}/{self.limits[key]} used"
            )
        self.used[key] += amount

    def remaining(self, key: str) -> int:
        if key not in self.limits:
            raise KeyError(f"unknown budget {key!r}")
        return self.limits[key] - self.used[key]

    def snapshot(self) -> dict[str, Any]:
        return {"limits": dict(self.limits), "used": dict(self.used)}


def load_budget_limits(path) -> dict[str, int]:
    """Read the declared budget limits from the supervision config."""
    raw = yaml.safe_load(open(path, encoding="utf-8")) or {}
    budgets = raw.get("budgets") or {}
    limits = {key: int(budgets[key]) for key in BUDGET_KEYS if key in budgets}
    missing = [key for key in BUDGET_KEYS if key not in limits]
    if missing:
        raise ValueError(f"supervision config is missing budgets: {missing}")
    return limits
