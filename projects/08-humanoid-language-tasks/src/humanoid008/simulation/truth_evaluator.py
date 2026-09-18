"""Independent truth evaluator (section 16).

The evaluator reads the **terminal** simulator state (payload pose, velocity,
contact) and computes the physical acceptance metrics. It is a *read-only*
sink: its output never flows back into the controller to steer a retry. The
task's own completion claim is checked against this, and a contradiction means
the run is FAILED.
"""

from __future__ import annotations

import numpy as np

import mujoco

from .tactile import sense


def evaluate_transport(runtime, target_id="station_b") -> dict:
    """Compute the physical acceptance metrics for a transport episode.

    ``target_id`` selects which destination fixture the payload is judged
    against (``station_b`` green marker, ``station_c`` blue marker).
    """
    truth = runtime.d.body("payload").xpos.copy()
    table_geom = "destination_c_table" if target_id == "station_c" else "destination_table"
    target = runtime.m.geom(table_geom).pos.copy() + np.array([0.0, 0.0, 0.047])
    velocity = np.zeros(6)
    mujoco.mj_objectVelocity(
        runtime.m, runtime.d, mujoco.mjtObj.mjOBJ_BODY, runtime.m.body("payload").id, velocity, 0
    )
    touch = sense(runtime)
    return dict(
        payload=truth.tolist(),
        target=target.tolist(),
        xy_error=float(np.linalg.norm(truth[:2] - target[:2])),
        height_error=float(abs(truth[2] - target[2])),
        speed=float(np.linalg.norm(velocity[3:])),
        touch=touch,
    )


def transport_accepted(evaluation: dict) -> bool:
    """The frozen acceptance thresholds for transport (section 16)."""
    return bool(
        evaluation["xy_error"] < 0.06
        and evaluation["height_error"] < 0.012
        and evaluation["speed"] < 0.025
        and evaluation["touch"]["destination"]
        and evaluation["touch"]["left"] < 0.02
        and evaluation["touch"]["right"] < 0.02
    )
