"""Every reviewable language sample in tests/fixtures/instructions.jsonl.

Section 17.2: at least 60 fixed samples, each with a declared expected outcome.
Rule parsing is only ever accepted for its **declared coverage** -- the point is
auditability, not a claim of general language ability.
"""

from __future__ import annotations

import json
from pathlib import Path

import pytest

from humanoid008.language import ParseKind, RuleVocabulary, parse

PROJECT_ROOT = Path(__file__).resolve().parents[2]
FIXTURES = PROJECT_ROOT / "tests" / "fixtures" / "instructions.jsonl"
RULE_CONFIG = PROJECT_ROOT / "config" / "planners" / "rule.yaml"

SAMPLES = [
    json.loads(line)
    for line in FIXTURES.read_text(encoding="utf-8").splitlines()
    if line.strip()
]


def test_fixture_set_is_large_enough_and_covers_every_group():
    groups = {sample["group"] for sample in SAMPLES}
    assert groups == {
        "explicit",
        "ambiguous",
        "unsupported",
        "injection",
        "repeat_update",
    }
    assert len(SAMPLES) >= 60


def test_every_sample_id_is_unique():
    ids = [sample["id"] for sample in SAMPLES]
    assert len(ids) == len(set(ids))


@pytest.fixture(scope="module")
def vocab() -> RuleVocabulary:
    return RuleVocabulary.load(RULE_CONFIG)


@pytest.mark.parametrize("sample", SAMPLES, ids=[sample["id"] for sample in SAMPLES])
def test_sample_matches_its_declared_expectation(vocab, sample):
    outcome = parse(sample["text"], vocab)
    expected = sample["expect"]

    if expected == "reject":
        assert outcome.kind is ParseKind.REJECT, (
            f"{sample['id']} expected reject, got {outcome.kind}"
        )
        return

    if expected == "clarify":
        assert outcome.kind is ParseKind.CLARIFY, (
            f"{sample['id']} expected clarify, got {outcome.kind}"
        )
        assert sample["pending"] in outcome.pending_fields
        return

    assert outcome.kind is ParseKind.GOAL, (
        f"{sample['id']} expected goal, got {outcome.kind} ({outcome.detail})"
    )
    draft = outcome.draft
    assert draft.intent.value == sample["intent"], sample["id"]
    assert draft.object_candidates == (sample["object_id"],), sample["id"]
    if "target_id" in sample:
        assert draft.target_candidates == (sample["target_id"],), sample["id"]
    if "source_id" in sample:
        assert draft.source_candidates == (sample["source_id"],), sample["id"]
    if "alternative_target_id" in sample:
        assert draft.alternative_target_candidates == (
            sample["alternative_target_id"],
        ), sample["id"]
