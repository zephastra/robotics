"""The recording must carry the pose the gate ACTED ON, or say that it did not.

Why this file exists
--------------------
`D-P10-02` measured that the gate's error is a RATE against distance travelled, and the fix
was to change which pose it concludes from. The margin that decision needs -- how far the
gate's own pose can be from truth -- was then derived from *displacement*, because the
recording did not contain the gate's pose at all: `pose_usable`, `gate_pose` and
`err_gate_truth` existed nowhere in the repo, and no sample key held one.

A missing instrument is not a missing number. It is a number that will be supplied from
somewhere else, and here the somewhere else was `odom` -- a different fact about the same
robot. These tests hold the instrument in place:

1. the recorder subscribes to each gate's state topic,
2. every sample carries a `gate` entry for every robot,
3. a gate that says nothing is recorded as `None`, not as an absent key, and
4. the audit distinguishes "no gate topic was asked for" from "the gate never answered".
"""
from __future__ import annotations

import ast
import json
import pathlib
import sys

import pytest

ROOT = pathlib.Path(__file__).resolve().parents[1]
RECORDER = ROOT / "src/fleet_evaluation/fleet_evaluation/recorder.py"


@pytest.fixture(scope="module")
def source() -> str:
    return RECORDER.read_text(encoding="utf-8")


def test_the_recorder_subscribes_to_a_gate_state_topic(source: str) -> None:
    """A recorder that never subscribes cannot report the gate's pose, whatever else it does."""
    assert "gate_state_topics" in source, (
        "the recorder has no gate-state subscription: the pose the gate acted on is not "
        "recorded, and any margin about it will be derived from `odom` instead"
    )
    assert "_on_gate_state" in source, "no gate-state callback is defined"


def test_every_sample_carries_a_gate_entry_per_robot(source: str) -> None:
    """`None` per robot, not an omitted key.

    An omitted key is indistinguishable from a robot that does not exist; `None` says "this
    robot has no gate payload", which is a statement about the recording.
    """
    tree = ast.parse(source)
    sample_keys: set[str] = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.Dict):
            for k in node.keys:
                if isinstance(k, ast.Constant) and isinstance(k.value, str):
                    sample_keys.add(k.value)
    assert "gate" in sample_keys, (
        "no sample payload writes a `gate` key; the gate's pose is recorded nowhere"
    )


def test_the_gate_payload_is_not_merged_into_odom(source: str) -> None:
    """Two facts about one robot may not be averaged into a third.

    The gate may conclude from the localiser or from composed odometry, and it may conclude
    from a pose it considers unusable. Collapsing that into `odom` is the shape of fault
    this project has already paid for once (`D-P10-03`: a field that exists is not the field
    you meant).
    """
    assert "self.gate[robot] = body" in source, "the gate payload is not stored per robot"
    assert "self.gate[robot] = body" not in source.split("self.odom[robot] =")[0].split("def _mk_odom")[-1], (
        "the gate callback writes into the odometry map"
    )


def test_the_audit_tells_apart_silence_from_absence(source: str) -> None:
    """'No gate topic was requested' and 'the gate never answered' are different faults."""
    for field in ("gate_topics", "gate_messages", "gate_parse_failures", "gate_available"):
        assert f'"{field}"' in source, f"the audit does not report {field}"
    assert "gate_robots_seen" in source, (
        "the audit cannot say WHICH robots' gate payloads arrived"
    )


def test_a_parse_failure_is_counted_not_swallowed(source: str) -> None:
    """A payload that will not parse must be visible, not indistinguishable from silence."""
    assert "gate_parse_failures += 1" in source, (
        "an unparseable gate payload is not counted; a broken instrument would look like a "
        "quiet one"
    )


def test_the_cli_defaults_to_a_per_robot_topic(source: str) -> None:
    """The gate publishes on a namespaced topic; the default must match, not guess a flat one."""
    assert "--gate-state-topic" in source, "the recorder cannot be pointed at a gate topic"
    assert "{robot}/gate_state" in source, (
        "the default gate topic is not namespaced per robot, so a three-robot run would read "
        "one gate three times or none at all"
    )
