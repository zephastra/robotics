"""Mission configuration models and validation."""

from dataclasses import dataclass
from enum import Enum
from pathlib import Path
from typing import Any

import yaml


VALID_FAILURE_POLICIES = {"skip", "abort"}
VALID_ACTIONS = {"wait", "inspect", "pickup", "deliver"}


class MissionState(str, Enum):
    IDLE = "IDLE"
    WAITING_FOR_NAV2 = "WAITING_FOR_NAV2"
    NAVIGATING = "NAVIGATING"
    EXECUTING_TASK = "EXECUTING_TASK"
    PAUSED = "PAUSED"
    RETRYING = "RETRYING"
    BLOCKED = "BLOCKED"
    WAITING_FOR_CLEARANCE = "WAITING_FOR_CLEARANCE"
    RECOVERING = "RECOVERING"
    LOW_BATTERY = "LOW_BATTERY"
    RETURNING_HOME = "RETURNING_HOME"
    COMPLETED = "COMPLETED"
    COMPLETED_WITH_WARNINGS = "COMPLETED_WITH_WARNINGS"
    CANCELED = "CANCELED"
    FAILED = "FAILED"


@dataclass(frozen=True)
class MissionPose:
    name: str
    x: float
    y: float
    yaw: float


@dataclass(frozen=True)
class MissionTask:
    task_id: str
    name: str
    pose: MissionPose
    action: str
    stay_seconds: float
    retries: int
    timeout_seconds: float
    failure_policy: str


@dataclass(frozen=True)
class MissionDefinition:
    mission_id: str
    frame_id: str
    failure_policy: str
    return_home: bool
    return_home_on_cancel: bool
    low_battery_threshold: float
    blocked_timeout_seconds: float
    minimum_progress_distance: float
    recovery_wait_seconds: float
    max_blocked_recoveries: int
    home: MissionPose
    tasks: tuple[MissionTask, ...]


def _mapping(value: Any, field_name: str) -> dict[str, Any]:
    if not isinstance(value, dict):
        raise ValueError(f"{field_name} must be a mapping")
    return value


def _required_text(data: dict[str, Any], key: str, field_name: str) -> str:
    value = data.get(key)
    if not isinstance(value, str) or not value.strip():
        raise ValueError(f"{field_name}.{key} must be a non-empty string")
    return value.strip()


def _number(
    data: dict[str, Any], key: str, field_name: str, default: float | None = None
) -> float:
    value = data.get(key, default)
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{field_name}.{key} must be a number")
    return float(value)


def _pose(data: dict[str, Any], field_name: str, default_name: str) -> MissionPose:
    return MissionPose(
        name=str(data.get("name", default_name)),
        x=_number(data, "x", field_name),
        y=_number(data, "y", field_name),
        yaw=_number(data, "yaw", field_name, 0.0),
    )


def load_mission(path: str | Path) -> MissionDefinition:
    """Load and validate a mission YAML file."""
    mission_path = Path(path).expanduser().resolve()
    if not mission_path.is_file():
        raise ValueError(f"Mission file does not exist: {mission_path}")

    with mission_path.open(encoding="utf-8") as stream:
        document = yaml.safe_load(stream)

    root = _mapping(document, "document")
    mission_data = _mapping(root.get("mission"), "mission")
    mission_id = _required_text(mission_data, "id", "mission")
    frame_id = str(mission_data.get("frame_id", "map")).strip() or "map"
    failure_policy = str(mission_data.get("failure_policy", "skip"))
    if failure_policy not in VALID_FAILURE_POLICIES:
        raise ValueError("mission.failure_policy must be 'skip' or 'abort'")

    threshold = _number(
        mission_data, "low_battery_threshold", "mission", default=20.0
    )
    if not 0.0 <= threshold <= 100.0:
        raise ValueError("mission.low_battery_threshold must be between 0 and 100")

    blocked_timeout = _number(
        mission_data, "blocked_timeout_seconds", "mission", default=15.0
    )
    minimum_progress = _number(
        mission_data, "minimum_progress_distance", "mission", default=0.15
    )
    recovery_wait = _number(
        mission_data, "recovery_wait_seconds", "mission", default=6.0
    )
    max_recoveries = mission_data.get("max_blocked_recoveries", 3)
    if isinstance(max_recoveries, bool) or not isinstance(max_recoveries, int):
        raise ValueError("mission.max_blocked_recoveries must be an integer")
    if (
        blocked_timeout <= 0.0
        or minimum_progress <= 0.0
        or recovery_wait < 0.0
        or max_recoveries < 0
    ):
        raise ValueError(
            "mission recovery settings require blocked_timeout_seconds > 0, "
            "minimum_progress_distance > 0, recovery_wait_seconds >= 0, "
            "and max_blocked_recoveries >= 0"
        )

    home_data = _mapping(mission_data.get("home"), "mission.home")
    home = _pose(home_data, "mission.home", "home")

    raw_tasks = mission_data.get("tasks")
    if not isinstance(raw_tasks, list) or not raw_tasks:
        raise ValueError("mission.tasks must be a non-empty list")

    tasks: list[MissionTask] = []
    seen_ids: set[str] = set()
    for index, raw_task in enumerate(raw_tasks):
        field_name = f"mission.tasks[{index}]"
        task_data = _mapping(raw_task, field_name)
        task_id = _required_text(task_data, "id", field_name)
        if task_id in seen_ids:
            raise ValueError(f"Duplicate task id: {task_id}")
        seen_ids.add(task_id)

        action = str(task_data.get("action", "wait"))
        if action not in VALID_ACTIONS:
            raise ValueError(
                f"{field_name}.action must be one of {sorted(VALID_ACTIONS)}"
            )

        stay_seconds = _number(task_data, "stay_seconds", field_name, 0.0)
        timeout_seconds = _number(task_data, "timeout_seconds", field_name, 90.0)
        retries_value = task_data.get("retries", 1)
        if isinstance(retries_value, bool) or not isinstance(retries_value, int):
            raise ValueError(f"{field_name}.retries must be an integer")
        if stay_seconds < 0.0 or timeout_seconds <= 0.0 or retries_value < 0:
            raise ValueError(
                f"{field_name} requires stay_seconds >= 0, timeout_seconds > 0, "
                "and retries >= 0"
            )

        task_policy = str(task_data.get("failure_policy", failure_policy))
        if task_policy not in VALID_FAILURE_POLICIES:
            raise ValueError(f"{field_name}.failure_policy must be 'skip' or 'abort'")

        tasks.append(
            MissionTask(
                task_id=task_id,
                name=str(task_data.get("name", task_id)),
                pose=_pose(task_data, field_name, task_id),
                action=action,
                stay_seconds=stay_seconds,
                retries=retries_value,
                timeout_seconds=timeout_seconds,
                failure_policy=task_policy,
            )
        )

    return MissionDefinition(
        mission_id=mission_id,
        frame_id=frame_id,
        failure_policy=failure_policy,
        return_home=bool(mission_data.get("return_home", True)),
        return_home_on_cancel=bool(mission_data.get("return_home_on_cancel", True)),
        low_battery_threshold=threshold,
        blocked_timeout_seconds=blocked_timeout,
        minimum_progress_distance=minimum_progress,
        recovery_wait_seconds=recovery_wait,
        max_blocked_recoveries=max_recoveries,
        home=home,
        tasks=tuple(tasks),
    )
