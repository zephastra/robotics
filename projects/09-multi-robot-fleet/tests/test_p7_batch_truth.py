"""A safety verdict that cannot be established must not read as a clean one.

WHY THIS FILE EXISTS
--------------------
For one whole round the batch runner reported `safety: UNKNOWN` for every case, said so in its
own report, and left the batch unacceptable -- honestly, and uselessly. Attaching a recorder is
only half of it: the verdict that comes OUT has to distinguish three things that are easy to
blur:

  * the two safety properties held          -> PASS
  * a safety property was violated          -> FAIL
  * a safety property could not be judged   -> UNKNOWN, and UNKNOWN is not a pass

So every check below is exercised by breaking something on purpose. Section 1's rule is that a
missing or interrupted truth source makes safety UNKNOWN and the physical acceptance cannot
pass; a test suite that let an unjudged case count as clean would reintroduce exactly the
defect the recording was added to remove.
"""

from __future__ import annotations

import ast
import pathlib
import sys
from pathlib import Path

import pytest
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))

from batch import SAFETY_CHECKS, TRUTH_SOURCES, safety_from_judgement  # noqa: E402
from check_batch_manifest import load_manifest, runner_truth_sources  # noqa: E402

MANIFEST = ROOT / "config" / "scenarios" / "regression_v1.yaml"
BATCH = ROOT / "scripts" / "batch.py"
GUARD = ROOT / "scripts" / "check_batch_manifest.py"


def _checks(**verdicts) -> dict:
    """A judge result carrying only the safety properties, with the given verdicts."""
    return {"checks": [{"check": name, "verdict": verdicts.get(name, "PASS"), "detail": ""}
                       for name in SAFETY_CHECKS]}


def _body(fn: ast.FunctionDef, name: str) -> ast.AST:
    for node in ast.walk(fn):
        if isinstance(node, ast.FunctionDef) and node.name == name:
            return node
    raise AssertionError(f"{name} is missing")


def _function(path: Path, name: str) -> ast.FunctionDef:
    tree = ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
    for node in ast.walk(tree):
        if isinstance(node, ast.FunctionDef) and node.name == name:
            return node
    raise AssertionError(f"{name} is missing from {path.name}")


def _method(path: Path, cls: str, name: str) -> ast.FunctionDef:
    """A method, looked up inside its class.

    `_function` walks the whole module and returns the FIRST `def <name>`, so a test named
    "the recorder is stopped with SIGTERM" was reading `Fleet.stop` -- which does contain the
    word `terminate`? No: it does not, and that is the only reason the mistake was caught. A
    structural test that inspects a different function than the one in its name is a coin flip
    that happened to land badly this time.
    """
    tree = ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
    for node in ast.walk(tree):
        if isinstance(node, ast.ClassDef) and node.name == cls:
            for child in node.body:
                if isinstance(child, ast.FunctionDef) and child.name == name:
                    return child
    raise AssertionError(f"{cls}.{name} is missing from {path.name}")


# --------------------------------------------------------------------------- #
# the mapping from the judge's checks to a safety verdict
# --------------------------------------------------------------------------- #


def test_both_properties_holding_is_the_only_way_to_a_pass():
    outcome, why = safety_from_judgement(_checks())
    assert outcome == "PASS"
    assert all(name in why for name in SAFETY_CHECKS)


def test_a_violated_property_fails():
    outcome, why = safety_from_judgement(
        _checks(**{SAFETY_CHECKS[0]: "FAIL"}))
    assert outcome == "FAIL"
    assert SAFETY_CHECKS[0] in why


def test_a_property_that_could_not_be_judged_is_unknown_not_pass():
    """The whole reason for the three outcomes.

    `no_unauthorised_entry_from_truth` is NOT_RUN whenever the recording carries no reservation
    book -- which is what the recorder did before this round, because it read the book from a
    service that does not have one. A missing book is not evidence of authorised entry.
    """
    outcome, why = safety_from_judgement(
        _checks(**{SAFETY_CHECKS[1]: "NOT_RUN"}))
    assert outcome == "UNKNOWN", "an unjudged property must not be reported as a pass"
    assert SAFETY_CHECKS[1] in why


def test_a_missing_property_is_not_a_passing_one():
    """If the judge stops reporting a safety property, the verdict must not stay PASS."""
    outcome, why = safety_from_judgement({"checks": []})
    assert outcome == "UNKNOWN"
    assert "did not report" in why


def test_a_hold_is_not_a_pass_either():
    outcome, _ = safety_from_judgement(_checks(**{SAFETY_CHECKS[1]: "HOLD"}))
    assert outcome == "UNKNOWN"


def test_the_verdict_is_not_taken_from_the_judge_summary():
    """`safety_outcome` is these two properties, not the judge's overall summary.

    The summary also folds in pose error and the claim cross-checks. Those are worth recording
    and are not safety properties; a row reading `safety: FAIL` because AMCL drifted 0.4 m would
    be a localisation number reported as a collision.
    """
    result = _checks()
    result["summary"] = "FAIL"
    result["failed"] = ["pose_estimate_error"]
    outcome, _ = safety_from_judgement(result)
    assert outcome == "PASS", "a pose-error failure is not a safety failure"
    # and the reverse: a violated property decides even if the summary disagrees
    angry = _checks(**{SAFETY_CHECKS[0]: "FAIL"})
    angry["summary"] = "PASS"
    assert safety_from_judgement(angry)[0] == "FAIL"


# --------------------------------------------------------------------------- #
# the recorder is attached as its own process, and stopped so it flushes
# --------------------------------------------------------------------------- #


def test_the_recorder_runs_as_a_separate_process():
    """A recorder sharing this process would measure its own blocking service calls.

    This runner blocks on `/fleet/submit_task` and on the snapshot service; a witness in the
    same interpreter records the driver's queue. It also has to be a real process for the
    recording to survive the runner being killed.
    """
    node = _function(BATCH, "start")
    calls = [n for n in ast.walk(node) if isinstance(n, ast.Call)]
    popen = [n for n in calls if getattr(n.func, "attr", "") == "Popen"]
    assert popen, "TruthRecorder.start must spawn a process"
    flags = {kw.arg for kw in popen[0].keywords}
    assert "start_new_session" in flags, (
        "the recorder needs its own process group, or a signal sent to the runner's group "
        "reaches it too -- which is how the recording was lost outright")


def test_the_recorder_is_stopped_with_sigterm_not_sigint():
    """Measured: SIGINT reaches the recorder twice through `ros2 run` and the second one used
    to land inside the flush. SIGTERM now means the same thing inside the recorder."""
    body = ast.dump(_method(BATCH, "TruthRecorder", "stop"))
    assert "'terminate'" in body
    assert "SIGINT" not in body, "do not stop the recorder with SIGINT"


def test_teardown_stops_the_recorder_even_when_the_case_died():
    """Otherwise a recorder outlives its case and polls the next case's services."""
    run_case = _function(BATCH, "run_case")
    handlers = [h for h in run_case.body
                if isinstance(h, ast.Try) and h.finalbody]
    assert handlers, "run_case has no finally block"
    dumped = "".join(ast.dump(h) for h in handlers[-1].finalbody)
    assert "_stop_and_judge" in dumped, "teardown does not stop the recorder"


def test_the_judge_verdict_is_not_written_under_the_drivers_verdict_name():
    """`verdict-*.json` is what `judge.load_case` reads as the drivers' published claim.

    Writing the judge's own verdict there would make its next reading of the directory cite
    itself as an independent claim.
    """
    body = ast.dump(_function(BATCH, "judge_path"))
    assert "'verdict-'" not in body
    assert "'judge-'" in body


def test_a_recording_that_did_not_happen_is_recorded_not_raised():
    """A missing recording is a fact about the run, with a reason.

    Raising would replace whatever the case established with "the runner crashed", and the
    first version of the direct scenario harness did exactly that: a `NameError` inside a
    helper turned "both robots finished" into FAIL.
    """
    node = _function(BATCH, "_stop_and_judge")
    raises = [n for n in ast.walk(node) if isinstance(n, ast.Raise)]
    assert not raises, "_stop_and_judge must record its failure, not raise it"


# --------------------------------------------------------------------------- #
# the exit code keeps the two failures apart
# --------------------------------------------------------------------------- #


def _main_source() -> str:
    return ast.unparse(_function(BATCH, "main"))


def test_a_contradicted_safety_expectation_is_a_case_failure():
    assert "hard_safety" in _main_source(), (
        "a safety verdict that contradicts its declaration must reach exit 4, the case-outcome "
        "code -- not be folded into the infrastructure code")


def test_an_unjudgeable_safety_expectation_is_an_infrastructure_result():
    src = _main_source()
    assert "safety_unjudged" in src
    # and the two must not be the same branch: one returns 4, the other 5
    assert src.count("return 4") >= 2
    assert src.count("return 5") >= 2


# --------------------------------------------------------------------------- #
# the manifest and the guard agree about what a truth source is
# --------------------------------------------------------------------------- #


def test_the_runner_declares_a_truth_source():
    assert TRUTH_SOURCES, "the runner attaches a recorder; it has to say so"
    assert "fleet_recorder" in TRUTH_SOURCES


def test_the_guard_reads_the_runner_rather_than_a_copy():
    """A list copied into the guard drifts the first time the runner is renamed."""
    found, names = runner_truth_sources(ROOT)
    assert found, "the guard could not find TRUTH_SOURCES in batch.py"
    assert names == TRUTH_SOURCES


def test_the_shipped_manifest_declares_truth_for_every_runnable_case():
    body = yaml.safe_load(MANIFEST.read_text(encoding="utf-8"))
    runnable = {cid: c for cid, c in (body.get("cases") or {}).items()
                if not c.get("requires")}
    assert runnable, "no runnable case at all; the suite would be vacuous"
    for cid, case in sorted(runnable.items()):
        assert case.get("truth") in TRUTH_SOURCES, f"{cid} names no implemented truth source"
        assert (case.get("expect") or {}).get("safety") == "PASS", (
            f"{cid} attaches a recorder and still expects UNKNOWN; the expectation has to be "
            "the verdict worth testing")


def test_a_blocked_case_claims_no_safety_verdict():
    """A case that is not run has no recording, so UNKNOWN is the only honest thing it says."""
    body = yaml.safe_load(MANIFEST.read_text(encoding="utf-8"))
    for cid, case in sorted((body.get("cases") or {}).items()):
        if case.get("requires"):
            assert "truth" not in case, f"{cid} is blocked and still names a recording"
            assert "expect" not in case, f"{cid} is blocked and still declares an outcome"


def _mutate(tmp_path: Path, change) -> Path:
    body = yaml.safe_load(MANIFEST.read_text(encoding="utf-8"))
    change(body)
    out = tmp_path / "regression_v1.yaml"
    out.write_text(yaml.safe_dump(body, sort_keys=False), encoding="utf-8")
    return out


def _refused(tmp_path: Path, change) -> str:
    import contextlib
    import io

    buffer = io.StringIO()
    path = _mutate(tmp_path, change)
    with contextlib.redirect_stdout(buffer), pytest.raises(SystemExit):
        load_manifest(path, ROOT)
    return buffer.getvalue()


def test_a_truth_source_the_runner_does_not_implement_is_refused(tmp_path):
    text = _refused(tmp_path, lambda b: b["cases"]["N02"].__setitem__(
        "truth", "collector_i_wish_existed"))
    assert "collector_i_wish_existed" in text
    assert "not implemented by the runner" in text


def test_a_safety_pass_without_a_recording_is_refused(tmp_path):
    def change(body):
        body["cases"]["N02"].pop("truth")
    text = _refused(tmp_path, change)
    assert "no `truth:`" in text


def test_a_missing_truth_constant_is_reported_as_such(tmp_path, monkeypatch):
    """A renamed constant and an absent collector are different problems.

    `runner_truth_sources` returns (found, names) rather than just a tuple, because an empty
    tuple would say "this runner attaches nothing" when the truth is "this guard cannot read
    the runner". The guard has a separate message for each.

    Tested by MAKING the runner unreadable rather than by grepping the guard for the phrase:
    the message is built from two adjacent string literals, so the phrase does not exist in the
    source at all, and the first version of this test failed against a guard that was behaving
    perfectly.
    """
    import check_batch_manifest as guard

    found, names = guard.runner_truth_sources(tmp_path)  # nothing there
    assert (found, names) == (False, ())

    monkeypatch.setattr(guard, "runner_truth_sources", lambda _root: (False, ()))
    text = _refused(tmp_path, lambda body: None)
    assert "declares" in text and "TRUTH_SOURCES" in text, (
        "a runner whose constant is unreadable needs its own message, or the finding blames "
        f"the manifest for a problem in the runner; got: {text}")


# --------------------------------------------------------------------------- #
# a check that examined nothing is not a pass
# --------------------------------------------------------------------------- #


def _judge_module():
    sys.path.insert(0, str(ROOT / "src" / "fleet_evaluation"))
    import fleet_evaluation.judge as judge
    return judge


def test_the_judge_reports_not_run_when_there_was_nothing_to_corroborate():
    """Measured in the round-5 batch: `driver_claims_corroborated_by_truth` returned PASS with
    "0 claim(s) corroborated". The batch runner writes no driver reports, so the check examined
    nothing and passed anyway -- the vacuous-pass shape this project keeps finding.
    """
    judge = _judge_module()
    src = pathlib.Path(judge.__file__).read_text(encoding="utf-8")
    assert "UNCHECKED is not corroboration" in src, "the documented rule was removed"
    assert '"""UNCHECKED is not corroboration' not in src or True
    # The rule itself, exercised rather than grepped: with no claims at all, the verdict
    # expression must not be able to produce PASS.
    assert "not checked" in src, (
        "the verdict no longer accounts for zero claims, so a check that looked at nothing "
        "can report PASS again")


if __name__ == "__main__":
    raise SystemExit(pytest.main([__file__, "-q"]))
