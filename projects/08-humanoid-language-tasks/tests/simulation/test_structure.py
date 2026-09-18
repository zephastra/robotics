"""P4 structure check as a pytest suite (section 17.3 step 1).

Skipped automatically when the simulation stack is not installed. The stand
test is the minimal proof that the imported model + walking policy run in 008's
own venv.
"""

from __future__ import annotations

import numpy as np
import pytest

pytest.importorskip("mujoco", reason="simulation profile not installed (bash scripts/setup.sh --profile sim)")
pytest.importorskip("MNN", reason="simulation profile not installed")

from humanoid008.simulation.model_builder import (  # noqa: E402
    build_runtime,
    check_structural_requirements,
    structural_checks,
)


def test_imported_model_satisfies_structure_requirements():
    runtime = build_runtime()
    report = structural_checks(runtime)
    assert report["payload_is_free"] is True
    assert report["policy_joints"] == 25
    assert report["policy_active"] == 22
    assert report["has_eyes_camera"] is True
    assert report["collision_geoms"] > 0
    check_structural_requirements(report)  # raises on hard violation


def test_robot_stands_under_the_walking_policy():
    runtime = build_runtime()
    # 250 physics steps * control_decimation 5 = 50 policy calls = 0.5 s sim
    for _ in range(250):
        runtime.step(np.zeros(3), stationary=False)
    assert runtime.d.qpos[2] > 0.5, "robot collapsed"
    assert runtime.feet_loaded(), "feet lost contact"
