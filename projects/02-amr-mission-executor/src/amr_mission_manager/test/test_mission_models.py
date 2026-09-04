from pathlib import Path

import pytest

from amr_mission_manager.mission_models import load_mission


VALID_MISSION = """
mission:
  id: test_mission
  frame_id: map
  failure_policy: skip
  low_battery_threshold: 20
  home: {name: home, x: 0, y: 0, yaw: 0}
  tasks:
    - id: station_a
      x: 1.0
      y: 2.0
      action: inspect
"""


def _write(tmp_path: Path, content: str) -> Path:
    mission_file = tmp_path / "mission.yaml"
    mission_file.write_text(content, encoding="utf-8")
    return mission_file


def test_load_valid_mission(tmp_path: Path) -> None:
    mission = load_mission(_write(tmp_path, VALID_MISSION))

    assert mission.mission_id == "test_mission"
    assert mission.home.x == 0.0
    assert mission.tasks[0].task_id == "station_a"
    assert mission.tasks[0].retries == 1
    assert mission.tasks[0].timeout_seconds == 90.0
    assert mission.blocked_timeout_seconds == 15.0
    assert mission.minimum_progress_distance == 0.15
    assert mission.recovery_wait_seconds == 6.0
    assert mission.max_blocked_recoveries == 3


def test_duplicate_task_ids_are_rejected(tmp_path: Path) -> None:
    duplicate = VALID_MISSION.replace(
        "      action: inspect\n",
        "      action: inspect\n"
        "    - id: station_a\n"
        "      x: 2.0\n"
        "      y: 3.0\n",
    )

    with pytest.raises(ValueError, match="Duplicate task id"):
        load_mission(_write(tmp_path, duplicate))


def test_invalid_failure_policy_is_rejected(tmp_path: Path) -> None:
    invalid = VALID_MISSION.replace("failure_policy: skip", "failure_policy: retry_forever")

    with pytest.raises(ValueError, match="failure_policy"):
        load_mission(_write(tmp_path, invalid))


def test_invalid_battery_threshold_is_rejected(tmp_path: Path) -> None:
    invalid = VALID_MISSION.replace("low_battery_threshold: 20", "low_battery_threshold: 120")

    with pytest.raises(ValueError, match="low_battery_threshold"):
        load_mission(_write(tmp_path, invalid))


def test_invalid_recovery_settings_are_rejected(tmp_path: Path) -> None:
    invalid = VALID_MISSION.replace(
        "low_battery_threshold: 20",
        "low_battery_threshold: 20\n  blocked_timeout_seconds: 0",
    )

    with pytest.raises(ValueError, match="recovery settings"):
        load_mission(_write(tmp_path, invalid))
