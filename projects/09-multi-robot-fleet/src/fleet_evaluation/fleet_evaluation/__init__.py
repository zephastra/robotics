"""Independent evaluation of a recorded 009 run.

Read-only, pure Python, no ROS. It exists so that verdicts do not come from the same
process that drives the robots, and so that safety properties are decided by the world's
own pose stream rather than by the estimate the controller acts on.

    fleet_eval --run-dir reports/p4_acceptance_20260915T170337Z

`judge.py` is the whole of it today. Recording is a separate concern and lives in
`fleet_evaluation.recorder`, which needs ROS and must never be in the control path.
"""

from fleet_evaluation.judge import (  # noqa: F401
    FAIL,
    HOLD,
    NOT_RUN,
    PASS,
    attribute_truth,
    containment_from_odom,
    containment_from_truth,
    judge_case,
    load_case,
    load_samples,
    load_spawns,
    pose_error,
)

__all__ = [
    "FAIL",
    "HOLD",
    "NOT_RUN",
    "PASS",
    "attribute_truth",
    "containment_from_odom",
    "containment_from_truth",
    "judge_case",
    "load_case",
    "load_samples",
    "load_spawns",
    "pose_error",
]
