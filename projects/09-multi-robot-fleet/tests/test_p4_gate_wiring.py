"""P4.7: the gate must keep the guard it builds, and must say what it saw.

One test per way the permit path can break silently at startup or be misread from
outside. The two defects this file pins produced the *same* runtime symptom -- the gate
refusing every motion as ``RESOURCE_UNKNOWN`` with ``holdings none`` -- and neither was
visible in any log line, which is why both cost a simulation run to find.

These tests read the ROS sources rather than importing them: the defects were in the
*order* of statements inside ``__init__`` and in *which* counters get published, and the
only way to pin those is to look.
"""

from __future__ import annotations

from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SRC = ROOT / "src"
GATE = SRC / "fleet_ros" / "fleet_ros" / "gate_node.py"
DRIVER = SRC / "fleet_ros" / "fleet_ros" / "staged_crossing.py"
COORDINATOR = SRC / "fleet_ros" / "fleet_ros" / "coordinator_node.py"
HARNESS = ROOT / "scripts" / "acceptance_p4.py"


def function_body(source: str, name: str) -> str:
    """Slice one method body out, from ``def name(`` to the next top-level ``def``."""
    marker = f"    def {name}("
    start = source.index(marker)
    rest = source[start + len(marker):]
    nxt = rest.find("\n    def ")
    return rest[:nxt] if nxt >= 0 else rest


# --------------------------------------------------------------------------- #
# the guard must survive __init__
# --------------------------------------------------------------------------- #


def test_the_gate_does_not_uninstall_its_own_guard():
    """The bug, exactly.

    ``__init__`` used to build the guard in the geofence block and then re-run an
    annotated ``self._zguard: CrossingZoneGuard | None = None`` further down. The
    annotation is irrelevant; the assignment is not. ``self.gate.zone`` kept the object,
    so the geofence went on judging from its own geometry, and ``_on_permit`` returned at
    ``if self._zguard is None``. The gate therefore refused every motion as
    ``RESOURCE_UNKNOWN`` while its driver held a live, renewed permit.
    """
    init = function_body(GATE.read_text(encoding="utf-8"), "__init__")
    build = init.find("self._zguard = CrossingZoneGuard(")
    reset = init.find("self._zguard: CrossingZoneGuard | None = None")

    assert build >= 0, "the guard is no longer built in __init__; this test needs updating"
    assert reset >= 0, (
        "the annotated default is gone. It must exist and must come BEFORE the guard is "
        "built -- as a declaration, never as a reset")
    assert reset < build, (
        "the `_zguard = None` default is assigned AFTER the guard is built, so it wipes "
        "it. The geofence keeps judging (`self.gate.zone` holds the object) while "
        "`_on_permit` returns early, so the permit mirror stays empty and every motion "
        "is refused as RESOURCE_UNKNOWN. Move the line above the geofence block.")


def test_the_gate_refuses_to_start_when_the_flag_is_on_without_a_guard():
    """A tripwire, because the symptom above is indistinguishable from a traffic bug."""
    init = function_body(GATE.read_text(encoding="utf-8"), "__init__")
    assert "self.traffic_guard and self._zguard is None" in init, (
        "nothing checks that the guard survived __init__. Without the check the failure "
        "mode reappears silently and is diagnosed as a reservation problem.")
    assert "raise RuntimeError" in init, "the boundary check must refuse to start"


def test_the_gate_announces_which_topic_it_mirrors():
    """A mirror on the wrong topic logs identically to a mirror that gets no grants."""
    src = GATE.read_text(encoding="utf-8")
    assert "permit mirror subscribed on" in src
    assert "permit_topic = str(" in src, "the topic must be a named local, not inline"


def test_the_gate_reports_its_own_permit_path():
    """Four counters, because one number cannot separate the two failure shapes."""
    src = GATE.read_text(encoding="utf-8")
    for field in ('"guard_installed"', '"permit_messages_seen"',
                  '"permit_grants_seen"', '"permit_adopted"'):
        assert field in src, f"gate_state does not report {field}"
    handle = function_body(src, "_on_permit")
    assert "self._granted_seen += 1" in handle, (
        "grants must be counted before the early returns, or a gate that receives only "
        "refusals and one that cannot adopt a grant look the same")


# --------------------------------------------------------------------------- #
# the driver must not mistake a transient state for a verdict
# --------------------------------------------------------------------------- #


def test_the_driver_retries_a_transient_refusal():
    """``LOCALIZATION_STALE`` and ``RESOURCE_UNKNOWN`` are states, not verdicts.

    Both were treated as terminal, so one 1.9 s odometry gap on a loaded host -- seen
    after 197 legitimate queue retries, against a 1.5 s freshness limit, with the robot
    parked -- ended the crossing. Asking again causes no motion, so the retry is free.
    """
    acquire = function_body(DRIVER.read_text(encoding="utf-8"), "acquire")
    assert 'if resp.reason_code == "INVALID_INPUT":' in acquire, (
        "acquire must retry everything except a malformed request")
    assert 'if resp.reason_code != "RESOURCE_BUSY":' not in acquire, (
        "the old policy stopped on every refusal that was not RESOURCE_BUSY")


def test_the_driver_still_gives_up_eventually():
    """Patience is not the same as waiting forever."""
    acquire = function_body(DRIVER.read_text(encoding="utf-8"), "acquire")
    assert "time.monotonic() < deadline" in acquire
    assert "still queued after" in acquire


# --------------------------------------------------------------------------- #
# the harness must be able to see a half-broken path
# --------------------------------------------------------------------------- #


def test_the_harness_cross_checks_the_two_ends_of_the_permit_path():
    """A driver holding a permit whose gate saw zero grants is one line of JSON."""
    src = HARNESS.read_text(encoding="utf-8")
    assert "gate_grants_seen" in src, "the harness must read the gate's grant counter"
    assert "PERMIT PATH MISMATCH" in src, (
        "the harness must report the mismatch loudly; it is the only check that spans "
        "both processes")


def test_the_coordinator_and_the_gate_agree_on_the_permit_topic_default():
    """Two nodes, one default. A divergence here is a silent, total loss of the mirror."""
    import re

    def default_of(path: Path) -> str:
        m = re.search(r'declare_parameter\("permit_topic",\s*"([^"]+)"\)',
                      path.read_text(encoding="utf-8"))
        assert m is not None, f"{path.name} no longer declares permit_topic"
        return m.group(1)

    assert default_of(GATE) == default_of(COORDINATOR)
