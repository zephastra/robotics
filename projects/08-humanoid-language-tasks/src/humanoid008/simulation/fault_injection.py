"""Scene-level fault injection (section 17.3 step 5).

Faults change the **simulator** state only; the controller must discover the
change through its sensors. A fault is a recorded test intervention (it goes in
the manifest), never a recovery hook the task can call to fix itself.

This is the P4 scaffolding. The full fault matrix (occlusion / stale
observation / model hang / cancellation / occupancy / illegal proposal) is
exercised in the P6/P7 supervised runs; only scene-geometry faults are wired
here because the unsplit baseline has no planner to hang or propose illegally.
"""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class Fault:
    kind: str
    description: str
    params: dict


def occlude_destination(runtime, offset: tuple[float, float, float] = (0.0, 0.0, 0.02)) -> Fault:
    """Place a small occluder on the destination marker so vision loses it."""
    raise NotImplementedError(
        "destination occluder body is not present in the imported combined.xml; "
        "add one before enabling this fault"
    )


def displace_payload(runtime, delta: tuple[float, float, float]) -> Fault:
    """Nudge the payload off its nominal initial pose (test intervention)."""
    address = runtime.m.joint("payload_free").qposadr[0]
    runtime.d.qpos[address : address + 3] += list(delta)
    return Fault("displace_payload", "payload initial pose displaced", {"delta": list(delta)})


FAULT_HANDLERS = {
    "displace_payload": displace_payload,
    "occlude_destination": occlude_destination,
}


def inject(runtime, kind: str, **params) -> Fault:
    """Apply a named fault. Returns a manifest-ready record of the intervention."""
    handler = FAULT_HANDLERS.get(kind)
    if handler is None:
        raise ValueError(f"unknown fault {kind!r} (available: {sorted(FAULT_HANDLERS)})")
    return handler(runtime, **params)
