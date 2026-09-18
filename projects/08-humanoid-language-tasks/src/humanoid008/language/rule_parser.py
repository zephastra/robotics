"""Declared-coverage rule parser: text -> goal draft, clarification, or refusal.

Section 12: the rule planner's vocabulary is finite and **must be declared**.
Anything outside it is rejected or clarified -- never guessed.

This parser does **not** touch the world. It only produces *candidate* entity ids
from the alias table. The trusted binding -- exactly one match, must be visible,
and any alternative target must trace back to the user's own words -- happens
afterwards in the task manager (section 25.1).

Known and accepted limitation: this parser recognises declared vocabulary and
ignores everything else, so a sentence that embeds a valid instruction inside
unrelated text will still parse as that instruction. Prompt-injection resistance
is explicitly *not* the parser's job (section 12); the executor validates every
proposal before acting.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
from pathlib import Path
from typing import Any, Mapping, Sequence

import yaml

from ..contracts import Intent, ReasonCode
from .normalizer import normalize


class ParseKind(str, Enum):
    GOAL = "goal"
    CLARIFY = "clarify"
    REJECT = "reject"


@dataclass(frozen=True)
class GoalDraft:
    """An untrusted candidate set. Never executed directly."""

    intent: Intent
    object_candidates: tuple[str, ...] = ()
    source_candidates: tuple[str, ...] = ()
    target_candidates: tuple[str, ...] = ()
    alternative_target_candidates: tuple[str, ...] = ()
    allow_retry: bool = True
    allow_retarget: bool = False


@dataclass(frozen=True)
class ParseOutcome:
    kind: ParseKind
    normalized: str
    draft: GoalDraft | None = None
    intent_hint: Intent | None = None
    question: str | None = None
    pending_fields: tuple[str, ...] = ()
    reason_code: ReasonCode = ReasonCode.OK
    detail: str = ""


# --------------------------------------------------------------------------- #
# declared vocabulary
# --------------------------------------------------------------------------- #


@dataclass(frozen=True)
class RuleVocabulary:
    transport_verbs: tuple[str, ...]
    inspect_verbs: tuple[str, ...]
    negation_markers: tuple[str, ...]
    alternative_markers: tuple[str, ...]
    occupied_markers: tuple[str, ...]
    object_aliases: Mapping[str, tuple[str, ...]] = field(default_factory=dict)
    station_aliases: Mapping[str, tuple[str, ...]] = field(default_factory=dict)

    @classmethod
    def from_mapping(cls, data: Mapping[str, Any]) -> "RuleVocabulary":
        def terms(key: str) -> tuple[str, ...]:
            return tuple(normalize(str(item)) for item in data.get(key, []))

        def alias_table(key: str) -> dict[str, tuple[str, ...]]:
            table = data.get(key) or {}
            if not isinstance(table, Mapping):
                raise ValueError(f"rule vocabulary '{key}' must be a mapping")
            return {
                str(entity_id): tuple(normalize(str(name)) for name in names)
                for entity_id, names in table.items()
            }

        return cls(
            transport_verbs=terms("transport_verbs"),
            inspect_verbs=terms("inspect_verbs"),
            negation_markers=terms("negation_markers"),
            alternative_markers=terms("alternative_markers"),
            occupied_markers=terms("occupied_markers"),
            object_aliases=alias_table("object_aliases"),
            station_aliases=alias_table("station_aliases"),
        )

    @classmethod
    def load(cls, path: str | Path) -> "RuleVocabulary":
        raw = yaml.safe_load(Path(path).read_text(encoding="utf-8")) or {}
        if not isinstance(raw, Mapping):
            raise ValueError("rule planner config must be a mapping")
        return cls.from_mapping(raw)


# --------------------------------------------------------------------------- #
# parsing
# --------------------------------------------------------------------------- #


def _hits(text: str, needles: Sequence[str]) -> list[str]:
    return [needle for needle in needles if needle and needle in text]


def _candidates(text: str, aliases: Mapping[str, Sequence[str]]) -> list[str]:
    """Entity ids whose alias appears in the text, ordered by first appearance."""
    found: list[tuple[int, str]] = []
    for entity_id, names in aliases.items():
        best: int | None = None
        for name in names:
            index = text.find(name)
            if index >= 0 and (best is None or index < best):
                best = index
        if best is not None:
            found.append((best, entity_id))
    found.sort(key=lambda item: item[0])
    return [entity_id for _, entity_id in found]


def parse(text: str, vocabulary: RuleVocabulary) -> ParseOutcome:
    """Parse one utterance. Deterministic; no world access, no side effects."""
    normalized = normalize(text)
    if not normalized:
        return ParseOutcome(
            kind=ParseKind.REJECT,
            normalized=normalized,
            reason_code=ReasonCode.UNSUPPORTED_INTENT,
            detail="the instruction is empty",
        )

    inspect_hit = _hits(normalized, vocabulary.inspect_verbs)
    transport_hit = _hits(normalized, vocabulary.transport_verbs)
    negated = bool(_hits(normalized, vocabulary.negation_markers))

    if not inspect_hit and not transport_hit:
        return ParseOutcome(
            kind=ParseKind.REJECT,
            normalized=normalized,
            reason_code=ReasonCode.UNSUPPORTED_INTENT,
            detail="no supported action verb was recognised",
        )

    if transport_hit and negated and not inspect_hit:
        return ParseOutcome(
            kind=ParseKind.REJECT,
            normalized=normalized,
            reason_code=ReasonCode.UNSUPPORTED_INTENT,
            detail="moving was explicitly forbidden and no other goal was given",
        )

    intent = Intent.INSPECT if inspect_hit else Intent.TRANSPORT

    object_candidates = _candidates(normalized, vocabulary.object_aliases)
    if not object_candidates:
        return ParseOutcome(
            kind=ParseKind.CLARIFY,
            normalized=normalized,
            intent_hint=intent,
            question="哪一个物体？我没有识别到明确的物件。",
            pending_fields=("object_id",),
            reason_code=ReasonCode.UNKNOWN_ENTITY,
            detail="the instruction did not name a known object",
        )

    if intent is Intent.INSPECT:
        # inspect needs no destination (section 25.2)
        return ParseOutcome(
            kind=ParseKind.GOAL,
            normalized=normalized,
            draft=GoalDraft(intent=intent, object_candidates=tuple(object_candidates)),
        )

    station_candidates = _candidates(normalized, vocabulary.station_aliases)
    alternative_requested = bool(
        _hits(normalized, vocabulary.alternative_markers)
    ) and bool(_hits(normalized, vocabulary.occupied_markers))

    if not station_candidates:
        return ParseOutcome(
            kind=ParseKind.CLARIFY,
            normalized=normalized,
            question="要放到哪个台位？",
            pending_fields=("target_id",),
            reason_code=ReasonCode.UNKNOWN_ENTITY,
            detail="the instruction did not name a known destination",
        )

    if alternative_requested and len(station_candidates) >= 2:
        target = (station_candidates[0],)
        alternatives = tuple(station_candidates[1:])
        source: tuple[str, ...] = ()
    elif len(station_candidates) >= 2:
        source = (station_candidates[0],)
        target = (station_candidates[-1],)
        alternatives = ()
    else:
        source = ()
        target = (station_candidates[0],)
        alternatives = ()

    return ParseOutcome(
        kind=ParseKind.GOAL,
        normalized=normalized,
        draft=GoalDraft(
            intent=intent,
            object_candidates=tuple(object_candidates),
            source_candidates=source,
            target_candidates=target,
            alternative_target_candidates=alternatives,
        ),
    )
