"""009 robot adapters -- backend-neutral motion interface.

`fake` (P1) and `gazebo_nav2` (P2+) implement the same `Adapter` protocol and
carry their backend name in every status, so a report can never be ambiguous
about which one produced it.

`safety_gate` is deliberately backend-independent: it judges authorisation and
health, and it owns the final velocity. Nothing else may publish it.
"""

from .adapter_base import Adapter, AdapterPhase, AdapterStatus, MotionCommand
from .fake_adapter import FakeAdapter
from .nav2_readiness import ACTIVE as NAV2_LC_ACTIVE
from .nav2_readiness import Nav2Readiness, should_wait_for_recovery
from .safety_gate import (
    GateDecision,
    GateMode,
    GateReason,
    Limits,
    PermitView,
    SafetyGate,
    ZoneGuard,
    ZoneVerdict,
)
from .zone_guard import CrossingZoneGuard, map_reason

__all__ = [
    "Adapter",
    "AdapterPhase",
    "AdapterStatus",
    "FakeAdapter",
    "NAV2_LC_ACTIVE",
    "Nav2Readiness",
    "should_wait_for_recovery",
    "GateDecision",
    "GateMode",
    "GateReason",
    "Limits",
    "MotionCommand",
    "PermitView",
    "SafetyGate",
    "ZoneGuard",
    "ZoneVerdict",
    "CrossingZoneGuard",
    "map_reason",
]
