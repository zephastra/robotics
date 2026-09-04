import rclpy

from amr_mission_manager.mission_manager import MissionManager
import amr_mission_manager.mission_manager as mission_manager_module


class FakeManager:
    def __init__(self, completion_results):
        self.completion_results = iter(completion_results)
        self.calls = []
        self.events = []

    def cancelTask(self):
        self.calls.append("cancel")

    def isTaskComplete(self, running_task):
        self.calls.append(("complete", running_task))
        return next(self.completion_results)

    def _record_event(self, event_type, **details):
        self.events.append((event_type, details))


def test_cancel_waits_for_nav2_result(monkeypatch):
    manager = FakeManager([False, True])
    spin_calls = []
    monkeypatch.setattr(
        rclpy,
        "spin_once",
        lambda _node, timeout_sec: spin_calls.append(timeout_sec),
    )

    result = MissionManager._cancel_navigation_and_wait(
        manager, "navigate_to_pose", "low_battery"
    )

    assert result is True
    assert manager.calls == [
        "cancel",
        ("complete", "navigate_to_pose"),
        ("complete", "navigate_to_pose"),
    ]
    assert spin_calls == [0.1]
    assert manager.events[-1][0] == "navigation_cancel_completed"
    assert manager.events[-1][1]["reason"] == "low_battery"


def test_cancel_timeout_is_reported(monkeypatch):
    manager = FakeManager([False])
    monotonic_values = iter([0.0, 0.0, 2.0])
    monkeypatch.setattr(
        mission_manager_module.time,
        "monotonic",
        lambda: next(monotonic_values),
    )
    monkeypatch.setattr(rclpy, "spin_once", lambda _node, timeout_sec: None)

    result = MissionManager._cancel_navigation_and_wait(
        manager,
        "navigate_to_pose",
        "low_battery",
        timeout_seconds=1.0,
    )

    assert result is False
    assert manager.events[-1][0] == "navigation_cancel_timeout"
