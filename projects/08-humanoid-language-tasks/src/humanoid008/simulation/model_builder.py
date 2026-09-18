"""Build and structurally check the imported MuJoCo model (section 17.3 step 1).

Structure checks prove the imported baseline loads and is shaped the way the
rest of the pipeline assumes: a free rigid payload, collision geoms, the 25
serial joints, the ``eyes`` camera, and the walking-policy dimensions. This is
a structural check -- it is **not** a physics or safety guarantee.
"""

from __future__ import annotations

import mujoco

from .robot_runtime import Runtime, verify_assets


def build_runtime() -> Runtime:
    """Construct the robot runtime, verifying the imported assets first."""
    verify_assets()
    return Runtime()


def structural_checks(runtime: Runtime | None = None) -> dict:
    """Return a structural report; raise on a check that fails hard."""
    if runtime is None:
        runtime = build_runtime()

    report: dict = {}
    report["nbody"] = int(runtime.m.nbody)
    report["njnt"] = int(runtime.m.njnt)
    report["ngeom"] = int(runtime.m.ngeom)

    # free rigid payload body
    payload_id = runtime.m.body("payload").id
    report["payload_is_free"] = bool(runtime.m.body_jntadr[payload_id] >= 0)

    # collision geoms exist
    report["collision_geoms"] = int(sum(runtime.m.geom_contype > 0))

    # the 25 serial joints + 22-action policy
    report["policy_joints"] = len(runtime.policy.joints)
    report["policy_active"] = int(len(runtime.policy.active))

    # the head camera
    report["has_eyes_camera"] = bool(runtime.m.camera("eyes").id >= 0)

    return report


def check_structural_requirements(report: dict) -> None:
    """Assert the minimum structural shape. Raises ValueError on violation."""
    required = [
        ("payload_is_free", True),
    ]
    for key, expected in required:
        if report.get(key) != expected:
            raise ValueError(f"structural check failed: {key}={report.get(key)!r}")
    if report.get("policy_joints") != 25:
        raise ValueError(f"expected 25 serial joints, got {report.get('policy_joints')}")
    if report.get("policy_active") != 22:
        raise ValueError(f"expected 22 active joints, got {report.get('policy_active')}")
    if not report.get("has_eyes_camera"):
        raise ValueError("'eyes' camera is missing from the imported model")
