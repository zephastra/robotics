"""The batch case list must be checkable against the contract, and the check must be able to fail.

TEST_AND_ACCEPTANCE section 5 freezes 30 cases; section 5's strategy adds eight seeded repeats.
`scripts/check_batch_manifest.py` is what keeps `config/scenarios/regression_v1.yaml` honest,
and this file is what keeps the checker honest: every rule below is exercised by breaking the
manifest on purpose. A guard that cannot fail is not a guard, and this project has shipped one
of those before (`check_undefined_names.py` had half its checks switched off in the shell the
project actually used, and printed success).

The mutations are applied to a *copy* of the shipped manifest with the real contract, the real
limitations register and the real fleet config behind them, so a test cannot pass by testing a
fixture that has drifted from the documents.
"""

from __future__ import annotations

import sys
from pathlib import Path

import pytest
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))

from check_batch_manifest import contract_ids, load_manifest, planned_runs  # noqa: E402

MANIFEST = ROOT / "config" / "scenarios" / "regression_v1.yaml"


def _mutate(tmp_path: Path, change) -> Path:
    """Write a manifest that differs from the shipped one in exactly one way."""
    body = yaml.safe_load(MANIFEST.read_text(encoding="utf-8"))
    change(body)
    out = tmp_path / "regression_v1.yaml"
    out.write_text(yaml.safe_dump(body, sort_keys=False), encoding="utf-8")
    return out


def _refused(tmp_path: Path, change) -> str:
    """Run the guard on a mutated manifest and return what it complained about.

    The findings are printed, not raised: `SystemExit` carries only a count, which is why the
    first version of this helper asserted against "13 finding(s)" and every test failed while
    the guard was behaving perfectly. Capture the output, not the exception.
    """
    import contextlib
    import io

    buffer = io.StringIO()
    path = _mutate(tmp_path, change)
    with contextlib.redirect_stdout(buffer), pytest.raises(SystemExit):
        load_manifest(path, ROOT)
    return buffer.getvalue()


# --------------------------------------------------------------------------- #
# the shipped list
# --------------------------------------------------------------------------- #


def test_the_shipped_manifest_covers_the_contract_exactly():
    manifest = load_manifest(MANIFEST, ROOT)          # must not raise
    ids = set(manifest["cases"])
    assert ids == set(contract_ids(ROOT / "docs" / "TEST_AND_ACCEPTANCE.md"))
    assert len(ids) == 30


def test_planned_runs_expand_only_the_frozen_repeats():
    manifest = load_manifest(MANIFEST, ROOT)
    runs = planned_runs(manifest)
    assert len(runs) == 38, "30 cases plus two extra seeds for the four the contract names"
    by_case: "dict[str, list[int]]" = {}
    for run in runs:
        by_case.setdefault(run["case"], []).append(run["seed"])
    assert by_case["N04"] == [1, 2, 3]
    assert by_case["N05"] == [1, 2, 3]
    assert by_case["N07"] == [1, 2, 3]
    assert by_case["N08"] == [1, 2, 3]
    assert by_case["N02"] == [1], "a case the contract does not repeat must not be repeated"


def test_every_blocked_case_names_a_registered_prerequisite():
    manifest = load_manifest(MANIFEST, ROOT)
    for cid, body in manifest["cases"].items():
        for key in body.get("requires") or []:
            assert key in manifest["prerequisites"], f"{cid}: {key} is undeclared"


# --------------------------------------------------------------------------- #
# and the guard can fail
# --------------------------------------------------------------------------- #


def test_a_dropped_case_is_refused(tmp_path):
    message = _refused(tmp_path, lambda b: b["cases"].pop("N07"))
    assert "missing" in message and "N07" in message


def test_an_invented_case_is_refused(tmp_path):
    message = _refused(tmp_path, lambda b: b["cases"].update(
        {"X01": {"title": "not in the contract"}}))
    assert "does not" in message and "X01" in message


def test_a_case_with_neither_a_plan_nor_a_prerequisite_is_refused(tmp_path):
    def change(body):
        body["cases"]["N02"] = {"title": "silently empty"}
    message = _refused(tmp_path, change)
    assert "neither" in message


def test_a_case_that_is_both_blocked_and_expected_is_refused(tmp_path):
    def change(body):
        body["cases"]["N02"]["requires"] = ["cross_corridor_task_routing"]
    message = _refused(tmp_path, change)
    assert "both" in message


def test_an_unknown_prerequisite_is_refused(tmp_path):
    def change(body):
        body["cases"]["N02"]["requires"] = ["something_nobody_declared"]
        body["cases"]["N02"].pop("submit")
        body["cases"]["N02"].pop("expect")
    assert "not declared" in _refused(tmp_path, change)


def test_a_prerequisite_missing_from_the_register_is_refused(tmp_path):
    """The coupling that stops a NOT_RUN case being parked without a written reason."""
    def change(body):
        body["prerequisites"]["probe_key_absent_from_the_register"] = {
            "summary": "declared here, but nowhere a reader looks"}
        body["cases"]["N02"] = {"title": "probe",
                                "requires": ["probe_key_absent_from_the_register"]}
    message = _refused(tmp_path, change)
    assert "LIMITATIONS.md" in message


def test_a_safety_pass_without_truth_is_refused(tmp_path):
    """Section 1: a safety verdict with no recording behind it must be UNKNOWN.

    The case has to LOSE its truth source for this to test anything. Once the manifest started
    declaring `truth: fleet_recorder`, setting `expect.safety: PASS` on N02 was a consistent
    manifest and the guard was right not to refuse -- so this test stopped exercising its own
    rule while still passing, which is the failure mode this whole file is about.
    """
    def change(body):
        body["cases"]["N02"]["expect"]["safety"] = "PASS"
        body["cases"]["N02"].pop("truth", None)
    message = _refused(tmp_path, change)
    assert "truth" in message


def test_a_scenario_that_does_not_exist_is_refused(tmp_path):
    def change(body):
        body["cases"]["N02"]["scenario"] = "no_such_scenario"
    message = _refused(tmp_path, change)
    assert "no_such_scenario" in message


def test_a_station_the_fleet_config_does_not_define_is_refused(tmp_path):
    def change(body):
        body["cases"]["N02"]["submit"][0]["pick"] = "S_nowhere"
    message = _refused(tmp_path, change)
    assert "S_nowhere" in message


def test_a_runner_that_is_not_in_the_fleet_is_refused(tmp_path):
    def change(body):
        body["cases"]["N02"]["runners"] = ["r99"]
    message = _refused(tmp_path, change)
    assert "r99" in message


def test_a_seed_set_that_is_not_eight_extra_runs_is_refused(tmp_path):
    def change(body):
        body["seed_repeats"]["seeds"] = [2]
        del body["cases"]["N05"]           # and drop one repeatable case for good measure
    message = _refused(tmp_path, change)
    assert "eight" in message or "missing" in message


def test_a_missing_outcome_field_is_refused(tmp_path):
    def change(body):
        del body["cases"]["N02"]["expect"]["behavior"]
    assert "behavior" in _refused(tmp_path, change)


def test_an_outcome_outside_the_contract_vocabulary_is_refused(tmp_path):
    def change(body):
        body["cases"]["N02"]["expect"]["task"] = "OK"
    message = _refused(tmp_path, change)
    assert "expect.task" in message
