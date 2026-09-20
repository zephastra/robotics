"""The navigation-feedback instrument, exercised against the shape ROS actually sends.

WHY THIS FILE EXISTS
--------------------
The driver's `_on_feedback` is never called by the unit suite: only the ROS executor calls
it, so every existing test passed while the method had the wrong arity AND read the wrong
message level. Four whole cases were run and thrown away before anyone opened a driver
report (round 17, 2026-09-18).

The method cannot simply be imported. `staged_crossing` imports rclpy at module level, and
this suite deliberately runs in a shell that has never sourced ROS -- that is what makes the
"core does not depend on ROS" claim provable. So the test extracts the method's own source
out of the file by AST and executes just that, which has two useful consequences:

  * it runs the REAL source, not a copy that could drift and go on passing;
  * `rclpy.action.client.ActionClient` cannot be imported here either, so this file would
    have caught the arity bug without ROS being present at all.

The message shapes below are the real ones:

    NavigateToPose_FeedbackMessage   <- what the callback receives (the WRAPPER)
      .feedback                      <- NavigateToPose_Feedback
        .distance_remaining          (float32)
        .number_of_recoveries        (int16)
"""

from __future__ import annotations

import ast
import pathlib
import textwrap
from types import SimpleNamespace

import pytest

ROOT = pathlib.Path(__file__).resolve().parents[1]
DRIVER = ROOT / "src" / "fleet_ros" / "fleet_ros" / "staged_crossing.py"


def _method(name: str, cls: str = "StagedCrossing"):
    """Compile one method out of the real file, without importing the module around it."""
    text = DRIVER.read_text(encoding="utf-8")
    tree = ast.parse(text, filename=str(DRIVER))
    for node in tree.body:
        if not (isinstance(node, ast.ClassDef) and node.name == cls):
            continue
        for item in node.body:
            if isinstance(item, ast.FunctionDef) and item.name == name:
                segment = ast.get_source_segment(text, item)
                assert segment is not None
                # `get_source_segment` starts at `def` (no indent) while the body keeps its
                # own, so give the first line the same 4 spaces and let dedent do the rest.
                lines = segment.splitlines()
                lines[0] = "    " + lines[0]
                module = ast.parse(textwrap.dedent("\n".join(lines)))
                namespace: dict = {}
                exec(compile(module, str(DRIVER), "exec"), namespace)  # noqa: S102
                return namespace[name]
    raise AssertionError(f"{cls}.{name} is not in {DRIVER}")


@pytest.fixture()
def recorder():
    """A stand-in for the driver, holding only what the callback touches."""
    return SimpleNamespace(_feedback={})


def _navigating(distance: float = 3.25, recoveries: int = 2) -> SimpleNamespace:
    """One NavigateToPose feedback message, wrapped the way ROS wraps it."""
    return SimpleNamespace(
        feedback=SimpleNamespace(distance_remaining=distance, number_of_recoveries=recoveries),
        goal_id=SimpleNamespace(uuid=[0] * 16),
    )


def test_the_callback_survives_being_called_with_one_argument(recorder):
    """THE regression. ROS passes one argument; two required parameters killed every run."""
    callback = _method("_on_feedback")
    callback(recorder, _navigating())          # must not raise


def test_it_reads_the_payload_from_inside_the_wrapper(recorder):
    """The wrapper carries the payload at `.feedback`; reading it at the top level finds nothing."""
    _method("_on_feedback")(recorder, _navigating(distance=3.25, recoveries=2))
    assert recorder._feedback["distance_remaining_m"] == 3.25
    assert recorder._feedback["recoveries"] == 2
    assert recorder._feedback["feedback_samples"] == 1


def test_the_numbers_are_not_the_getattr_default(recorder):
    """-1.0 is the sentinel for 'never said'. If it came back as data, the instrument is lying."""
    _method("_on_feedback")(recorder, _navigating(distance=0.5, recoveries=0))
    assert recorder._feedback["distance_remaining_m"] == 0.5
    assert recorder._feedback["recoveries"] == 0


def test_repeated_messages_accumulate_a_sample_count(recorder):
    callback = _method("_on_feedback")
    for _ in range(3):
        callback(recorder, _navigating(distance=1.0, recoveries=1))
    assert recorder._feedback["feedback_samples"] == 3
    assert recorder._feedback["distance_remaining_m"] == 1.0


def test_a_message_it_cannot_read_is_counted_apart_from_silence(recorder):
    """A message that arrived and could not be read must not look like a message never sent."""
    _method("_on_feedback")(recorder, SimpleNamespace(header=SimpleNamespace()))
    assert recorder._feedback.get("feedback_samples", 0) == 0
    assert recorder._feedback["unreadable"] == 1


def test_fields_at_the_wrong_level_do_not_masquerade_as_data(recorder):
    """The negative control for the second fault.

    A flat message carrying `distance_remaining` at the TOP level is exactly what the broken
    version expected. It must be recorded as unreadable -- NOT quietly become -1.0 with a
    sample count attached, which is how an instrument reports confident nonsense.
    """
    flat = SimpleNamespace(distance_remaining=3.25, number_of_recoveries=2)
    _method("_on_feedback")(recorder, flat)
    assert recorder._feedback.get("feedback_samples", 0) == 0
    assert recorder._feedback["unreadable"] == 1
    assert "distance_remaining_m" not in recorder._feedback
