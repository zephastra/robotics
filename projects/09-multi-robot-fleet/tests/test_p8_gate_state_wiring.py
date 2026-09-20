"""The gate's verdict has one topic name, and every process that names it must agree.

WHY THIS FILE EXISTS
--------------------
Measured in a live fleet on 2026-09-17:

    /r01/fleet/gate_state   Publisher count: 0    Subscription count: 2
    /r01/gate_state         Publisher count: 1    (safety_gate)   Subscription count: 0

The safety gate published its verdict on a topic nobody read; the leg executor read a topic
nobody wrote. Both were "wired", and the wiring was two different names.

Nothing in this project could see it. DDS does not error on a subscription with no publisher.
`tests/test_p4_gate_wiring.py` checks that the gate's PAYLOAD has the right fields, which it did.
`scripts/check_guards.sh` checks the guard list. `inspect_graph.py` can enumerate publishers and
subscribers, but only while a fleet is up and only if someone asks it that question -- and the
question nobody asked was "do these two names refer to the same topic".

So this test asks the structural version of that question, which needs no fleet: collect EVERY
`gate_state_topic` value in the tree and require them to be equal.

It is deliberately a source-level check rather than a graph check. A graph check would be
stronger but can only run against a live fleet, and this defect survived four phases precisely
because live checks are the ones that get skipped. The two belong together: this one fails in two
seconds, and `scripts/probe_gate_state.py` reports the live publisher count for the case where
the source is right and the process still is not there.
"""

from __future__ import annotations

import pathlib
import re

ROOT = pathlib.Path(__file__).resolve().parents[1]
SRC = ROOT / "src"
SCRIPTS = ROOT / "scripts"
GATE = SRC / "fleet_ros" / "fleet_ros" / "gate_node.py"
ADAPTER = SRC / "fleet_ros" / "fleet_ros" / "nav2_adapter_node.py"

#: The canonical name. Chosen by counting consumers, not by preference: `acceptance_p4.py`,
#: `probe_odom_truth.py`, `check_fleet_isolation.py`'s REQUIRED_TOPICS and
#: `stop_distance_calibrator.py` all already use it, and only the adapter did not.
CANONICAL = "gate_state"

#: Covers both spellings in this tree: `declare_parameter("gate_state_topic", "x")` and
#: `"gate_state_topic": "x"` in a launch parameter dict.
SETTING = re.compile(r'"gate_state_topic"\s*[:,]\s*"([^"]+)"')

#: This file and the probe both record the mistake in prose, and the probe's docstring is where a
#: reader is most likely to look. Prose is not a defect -- the distinction that cost this project
#: a false red once already, when a patch's own assertion searched for a name that only appeared
#: in the comment explaining it.
PROSE_ONLY = {"probe_gate_state.py", pathlib.Path(__file__).name}


def _python_files():
    return sorted(list(SRC.rglob("*.py")) + list(SCRIPTS.glob("*.py")))


def test_the_gate_publishes_on_the_canonical_name():
    """The gate is the anchor: it is the only process that PUBLISHES this topic."""
    text = GATE.read_text(encoding="utf-8")
    assert f'declare_parameter("gate_state_topic", "{CANONICAL}")' in text, (
        "the gate's own default is not the canonical name; every other consumer was chosen to "
        "match it, so changing it here changes the contract")


def test_every_component_that_names_the_topic_names_the_same_one():
    settings: dict[str, list[str]] = {}
    for path in _python_files():
        values = SETTING.findall(path.read_text(encoding="utf-8"))
        if values:
            settings[str(path.relative_to(ROOT))] = values
    assert settings, ("no `gate_state_topic` setting found anywhere; this check has drifted away "
                      "from the tree it guards")
    wrong = {path: values for path, values in settings.items()
             if any(v != CANONICAL for v in values)}
    assert not wrong, (
        f"these files name the gate state topic something other than {CANONICAL!r}: {wrong}. "
        "A subscription to a topic nobody publishes is silent -- no error, no warning, no log "
        "line -- which is how /r01/fleet/gate_state came to have 0 publishers and 2 subscribers "
        "while /r01/gate_state had 1 publisher and 0 subscribers")


def test_the_leg_executor_takes_the_name_from_the_parameter():
    """The adapter must read the parameter, so the launch has one place to set the name."""
    text = ADAPTER.read_text(encoding="utf-8")
    assert 'self.get_parameter("gate_state_topic").value' in text, (
        "the adapter no longer reads the parameter; a hardcoded name would let the launch and the "
        "node disagree again, which is the whole fault this file is about")


def test_no_source_line_names_the_dead_topic_in_code():
    """`fleet/gate_state` was a second name for one topic. It must not come back as code."""
    offenders = []
    for path in _python_files():
        if path.name in PROSE_ONLY:
            continue
        for number, line in enumerate(path.read_text(encoding="utf-8").splitlines(), 1):
            code = line.split("#", 1)[0]
            if "fleet/gate_state" in code:
                offenders.append(f"{path.relative_to(ROOT)}:{number}")
    assert not offenders, (
        f"these lines name the dead topic `fleet/gate_state` in code: {offenders}")
