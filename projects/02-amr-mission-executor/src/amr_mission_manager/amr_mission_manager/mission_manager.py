"""Warehouse mission manager built on top of Nav2 Simple Commander."""

import json
import math
import sys
import time
from datetime import datetime, timezone
from pathlib import Path

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float32, String
from std_srvs.srv import Trigger
from visualization_msgs.msg import Marker, MarkerArray

from .mission_models import (
    MissionDefinition,
    MissionPose,
    MissionState,
    MissionTask,
    load_mission,
)
from .navigation_watchdog import NavigationProgressWatchdog


class MissionManager(BasicNavigator):
    """Execute a validated mission and coordinate recovery and return-home logic."""

    def __init__(self) -> None:
        super().__init__(node_name="mission_manager")
        self.declare_parameter("mission_file", "")
        self.declare_parameter("report_directory", "/tmp/amr_mission_reports")
        self.declare_parameter("battery_topic", "/mission/battery_percentage")
        self.declare_parameter("status_period", 1.0)

        mission_file = str(self.get_parameter("mission_file").value)
        if not mission_file:
            raise ValueError("The mission_file parameter is required")

        self.mission: MissionDefinition = load_mission(mission_file)
        self.report_directory = Path(
            str(self.get_parameter("report_directory").value)
        ).expanduser()
        self.status_period = max(
            0.2, float(self.get_parameter("status_period").value)
        )

        transient_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.status_publisher = self.create_publisher(
            String, "/mission/status", transient_qos
        )
        self.event_publisher = self.create_publisher(
            String, "/mission/events", transient_qos
        )
        self.marker_publisher = self.create_publisher(
            MarkerArray, "/mission/waypoints", transient_qos
        )
        self.create_subscription(
            Float32,
            str(self.get_parameter("battery_topic").value),
            self._battery_callback,
            transient_qos,
        )
        self.create_service(Trigger, "/mission/pause", self._pause_callback)
        self.create_service(Trigger, "/mission/resume", self._resume_callback)
        self.create_service(Trigger, "/mission/cancel", self._cancel_callback)

        self.state = MissionState.IDLE
        self.current_task: MissionTask | None = None
        self.current_task_index = -1
        self.battery_percentage: float | None = None
        self.pause_requested = False
        self.cancel_requested = False
        self.events: list[dict[str, object]] = []
        self.completed_tasks: list[str] = []
        self.failed_tasks: list[str] = []
        self.skipped_tasks: list[str] = []
        self.total_retries = 0
        self.total_blocked_recoveries = 0
        self.total_blocked_wait_seconds = 0.0
        self.started_at: str | None = None
        self.finished_at: str | None = None
        self._last_status_time = 0.0
        self._task_markers = {
            task.task_id: "PENDING" for task in self.mission.tasks
        }

        self._record_event(
            "mission_loaded",
            mission_file=str(Path(mission_file).resolve()),
            task_count=len(self.mission.tasks),
        )
        self._publish_markers()
        self._publish_status("Mission configuration loaded")

    @staticmethod
    def _utc_now() -> str:
        return datetime.now(timezone.utc).isoformat()

    def _battery_callback(self, message: Float32) -> None:
        self.battery_percentage = float(message.data)

    def _pause_callback(self, _request: Trigger.Request, response: Trigger.Response):
        if self.state in {
            MissionState.COMPLETED,
            MissionState.COMPLETED_WITH_WARNINGS,
            MissionState.CANCELED,
            MissionState.FAILED,
        }:
            response.success = False
            response.message = f"Mission is already {self.state.value}"
            return response
        self.pause_requested = True
        self._record_event("pause_requested")
        response.success = True
        response.message = "Mission pause requested"
        return response

    def _resume_callback(self, _request: Trigger.Request, response: Trigger.Response):
        if not self.pause_requested:
            response.success = False
            response.message = "Mission is not paused"
            return response
        self.pause_requested = False
        self._record_event("resume_requested")
        response.success = True
        response.message = "Mission resumed"
        return response

    def _cancel_callback(self, _request: Trigger.Request, response: Trigger.Response):
        self.cancel_requested = True
        self.pause_requested = False
        self._record_event("cancel_requested")
        response.success = True
        response.message = "Mission cancellation requested"
        return response

    def _record_event(self, event_type: str, **details: object) -> None:
        event: dict[str, object] = {
            "timestamp": self._utc_now(),
            "event": event_type,
            **details,
        }
        self.events.append(event)
        message = String()
        message.data = json.dumps(event, ensure_ascii=False)
        self.event_publisher.publish(message)

    def _set_state(self, state: MissionState, message: str = "") -> None:
        if state != self.state:
            self.state = state
            self._record_event(
                "state_changed",
                state=state.value,
                task_id=self.current_task.task_id if self.current_task else None,
                message=message,
            )
        self._publish_status(message, force=True)

    def _publish_status(self, message_text: str = "", force: bool = False) -> None:
        now = time.monotonic()
        if not force and now - self._last_status_time < self.status_period:
            return
        self._last_status_time = now
        status = {
            "mission_id": self.mission.mission_id,
            "state": self.state.value,
            "current_task": self.current_task.task_id if self.current_task else None,
            "current_task_index": self.current_task_index,
            "task_count": len(self.mission.tasks),
            "battery_percentage": self.battery_percentage,
            "completed": len(self.completed_tasks),
            "failed": len(self.failed_tasks),
            "skipped": len(self.skipped_tasks),
            "retries": self.total_retries,
            "blocked_recoveries": self.total_blocked_recoveries,
            "blocked_wait_seconds": round(self.total_blocked_wait_seconds, 2),
            "message": message_text,
            "timestamp": self._utc_now(),
        }
        message = String()
        message.data = json.dumps(status, ensure_ascii=False)
        self.status_publisher.publish(message)

    def _pose_stamped(self, pose: MissionPose) -> PoseStamped:
        message = PoseStamped()
        message.header.frame_id = self.mission.frame_id
        message.header.stamp = self.get_clock().now().to_msg()
        message.pose.position.x = pose.x
        message.pose.position.y = pose.y
        message.pose.orientation.z = math.sin(pose.yaw / 2.0)
        message.pose.orientation.w = math.cos(pose.yaw / 2.0)
        return message

    @staticmethod
    def _marker_color(status: str) -> tuple[float, float, float]:
        colors = {
            "PENDING": (0.10, 0.45, 1.00),
            "ACTIVE": (1.00, 0.75, 0.00),
            "SUCCEEDED": (0.10, 0.85, 0.20),
            "FAILED": (1.00, 0.10, 0.10),
            "SKIPPED": (0.45, 0.45, 0.45),
            "HOME": (0.70, 0.20, 1.00),
        }
        return colors.get(status, colors["PENDING"])

    def _append_marker_pair(
        self,
        markers: MarkerArray,
        marker_id: int,
        pose: MissionPose,
        status: str,
        label: str,
    ) -> None:
        red, green, blue = self._marker_color(status)
        stamp = self.get_clock().now().to_msg()

        point = Marker()
        point.header.frame_id = self.mission.frame_id
        point.header.stamp = stamp
        point.ns = "mission_waypoints"
        point.id = marker_id
        point.type = Marker.CYLINDER
        point.action = Marker.ADD
        point.pose.position.x = pose.x
        point.pose.position.y = pose.y
        point.pose.position.z = 0.12
        point.pose.orientation.w = 1.0
        point.scale.x = 0.35
        point.scale.y = 0.35
        point.scale.z = 0.24
        point.color.r = red
        point.color.g = green
        point.color.b = blue
        point.color.a = 0.95
        markers.markers.append(point)

        text = Marker()
        text.header.frame_id = self.mission.frame_id
        text.header.stamp = stamp
        text.ns = "mission_labels"
        text.id = 1000 + marker_id
        text.type = Marker.TEXT_VIEW_FACING
        text.action = Marker.ADD
        text.pose.position.x = pose.x
        text.pose.position.y = pose.y
        text.pose.position.z = 0.55
        text.pose.orientation.w = 1.0
        text.scale.z = 0.28
        text.color.r = red
        text.color.g = green
        text.color.b = blue
        text.color.a = 1.0
        text.text = f"{label} [{status}]"
        markers.markers.append(text)

    def _publish_markers(self) -> None:
        markers = MarkerArray()
        self._append_marker_pair(
            markers, 0, self.mission.home, "HOME", self.mission.home.name
        )
        for index, task in enumerate(self.mission.tasks, start=1):
            self._append_marker_pair(
                markers,
                index,
                task.pose,
                self._task_markers[task.task_id],
                task.name,
            )
        self.marker_publisher.publish(markers)

    def _set_task_marker(self, task: MissionTask, status: str) -> None:
        self._task_markers[task.task_id] = status
        self._publish_markers()

    def _is_low_battery(self) -> bool:
        return (
            self.battery_percentage is not None
            and self.battery_percentage <= self.mission.low_battery_threshold
        )

    def _wait_while_paused(self) -> str:
        self._set_state(MissionState.PAUSED, "Mission paused")
        while self.pause_requested and not self.cancel_requested:
            rclpy.spin_once(self, timeout_sec=0.2)
            self._publish_status("Waiting for resume")
        return "canceled" if self.cancel_requested else "resumed"

    def _cancel_navigation_and_wait(
        self,
        running_task,
        reason: str,
        timeout_seconds: float = 15.0,
    ) -> bool:
        """Cancel a Nav2 goal and wait until its result future is settled."""
        self._record_event("navigation_cancel_requested", reason=reason)
        self.cancelTask()
        deadline = time.monotonic() + timeout_seconds
        while time.monotonic() < deadline:
            if self.isTaskComplete(running_task):
                self._record_event("navigation_cancel_completed", reason=reason)
                return True
            rclpy.spin_once(self, timeout_sec=0.1)

        self._record_event(
            "navigation_cancel_timeout",
            reason=reason,
            timeout_seconds=timeout_seconds,
        )
        return False

    def _navigate_to_pose(
        self,
        pose: MissionPose,
        timeout_seconds: float,
        state: MissionState,
        ignore_battery: bool = False,
    ) -> str:
        self._set_state(state, f"Navigating to {pose.name}")
        running_task = self.goToPose(self._pose_stamped(pose))
        if running_task is None:
            return "failed"

        # Navigation deadlines and stall detection must follow ROS time. When Gazebo
        # runs slower than real time, a wall-clock deadline would cancel healthy
        # navigation before the configured amount of simulated time has elapsed.
        navigation_started = self.get_clock().now().nanoseconds / 1_000_000_000.0
        deadline = navigation_started + timeout_seconds
        watchdog = NavigationProgressWatchdog(
            timeout_seconds=self.mission.blocked_timeout_seconds,
            minimum_progress_distance=self.mission.minimum_progress_distance,
            last_progress_at=navigation_started,
        )
        while not self.isTaskComplete(running_task):
            if self.cancel_requested:
                self._cancel_navigation_and_wait(running_task, "mission_cancel")
                return "canceled"
            if self.pause_requested:
                self._cancel_navigation_and_wait(running_task, "mission_pause")
                return "paused"
            if not ignore_battery and self._is_low_battery():
                self._cancel_navigation_and_wait(running_task, "low_battery")
                return "low_battery"
            navigation_now = (
                self.get_clock().now().nanoseconds / 1_000_000_000.0
            )
            if navigation_now >= deadline:
                self._cancel_navigation_and_wait(running_task, "navigation_timeout")
                self._record_event("navigation_timeout", target=pose.name)
                return "failed"

            feedback = self.getFeedback(running_task)
            distance = getattr(feedback, "distance_remaining", None) if feedback else None
            text = f"Navigating to {pose.name}"
            if distance is not None:
                text += f", {distance:.2f} m remaining"
                current_pose = getattr(feedback, "current_pose", None)
                pose_value = getattr(current_pose, "pose", None)
                position = getattr(pose_value, "position", None)
                current_x = getattr(position, "x", None)
                current_y = getattr(position, "y", None)
                if watchdog.update(
                    float(distance),
                    navigation_now,
                    float(current_x) if current_x is not None else None,
                    float(current_y) if current_y is not None else None,
                ):
                    self._record_event(
                        "navigation_blocked",
                        target=pose.name,
                        distance_remaining=round(float(distance), 3),
                        blocked_timeout_seconds=self.mission.blocked_timeout_seconds,
                    )
                    self._cancel_navigation_and_wait(
                        running_task, "navigation_blocked"
                    )
                    return "blocked"
            self._publish_status(text)

        result = self.getResult()
        if result == TaskResult.SUCCEEDED:
            return "succeeded"

        error_code, error_message = self.getTaskError()
        self._record_event(
            "navigation_failed",
            target=pose.name,
            result=str(result),
            error_code=error_code,
            error_message=error_message,
        )
        if result == TaskResult.FAILED:
            self._record_event(
                "navigation_failure_classified_as_blocked",
                target=pose.name,
                error_code=error_code,
            )
            return "blocked"
        return "failed"

    def _wait_for_clearance(
        self,
        task: MissionTask | None,
        recovery_attempt: int,
        ignore_battery: bool = False,
    ) -> str:
        target_name = task.name if task is not None else self.mission.home.name
        task_id = task.task_id if task is not None else None
        wait_started = time.monotonic()
        self._set_state(MissionState.BLOCKED, f"Path blocked near {target_name}")
        self._record_event(
            "clearance_wait_started",
            task_id=task_id,
            target=target_name,
            recovery_attempt=recovery_attempt,
            wait_seconds=self.mission.recovery_wait_seconds,
        )
        self._set_state(
            MissionState.WAITING_FOR_CLEARANCE,
            f"Waiting for clearance near {target_name}",
        )
        deadline = wait_started + self.mission.recovery_wait_seconds
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.2)
            if self.cancel_requested:
                return "canceled"
            if not ignore_battery and self._is_low_battery():
                return "low_battery"
            remaining = max(0.0, deadline - time.monotonic())
            self._publish_status(
                f"Path blocked; retrying {target_name} in {remaining:.1f}s"
            )

        waited_seconds = time.monotonic() - wait_started
        self.total_blocked_wait_seconds += waited_seconds
        self._record_event(
            "clearance_wait_completed",
            task_id=task_id,
            target=target_name,
            recovery_attempt=recovery_attempt,
            waited_seconds=round(waited_seconds, 3),
        )
        self._set_state(
            MissionState.RECOVERING,
            f"Retrying navigation to {target_name}",
        )
        return "retry"

    def _execute_station_action(self, task: MissionTask) -> str:
        self._set_state(
            MissionState.EXECUTING_TASK,
            f"Executing {task.action} at {task.name}",
        )
        self._record_event(
            "station_action_started",
            task_id=task.task_id,
            action=task.action,
            duration=task.stay_seconds,
        )
        remaining = task.stay_seconds
        previous = time.monotonic()
        while remaining > 0.0:
            rclpy.spin_once(self, timeout_sec=min(0.2, remaining))
            now = time.monotonic()
            if self.cancel_requested:
                return "canceled"
            if self._is_low_battery():
                return "low_battery"
            if self.pause_requested:
                pause_result = self._wait_while_paused()
                if pause_result == "canceled":
                    return "canceled"
                previous = time.monotonic()
                self._set_state(
                    MissionState.EXECUTING_TASK,
                    f"Resumed {task.action} at {task.name}",
                )
                continue
            remaining -= now - previous
            previous = now
            self._publish_status(
                f"Executing {task.action}, {max(0.0, remaining):.1f}s remaining"
            )

        self._record_event(
            "station_action_completed", task_id=task.task_id, action=task.action
        )
        return "succeeded"

    def _execute_task(self, task: MissionTask) -> str:
        attempt = 0
        blocked_recoveries = 0
        while attempt <= task.retries:
            if self.cancel_requested:
                return "canceled"
            if self._is_low_battery():
                return "low_battery"
            if self.pause_requested:
                if self._wait_while_paused() == "canceled":
                    return "canceled"

            self._set_task_marker(task, "ACTIVE")
            self._record_event(
                "navigation_started",
                task_id=task.task_id,
                attempt=attempt + 1,
                x=task.pose.x,
                y=task.pose.y,
            )
            outcome = self._navigate_to_pose(
                task.pose, task.timeout_seconds, MissionState.NAVIGATING
            )

            if outcome == "paused":
                if self._wait_while_paused() == "canceled":
                    return "canceled"
                continue
            if outcome in {"canceled", "low_battery"}:
                return outcome
            if outcome == "succeeded":
                return self._execute_station_action(task)
            if outcome == "blocked":
                blocked_recoveries += 1
                if blocked_recoveries > self.mission.max_blocked_recoveries:
                    self._record_event(
                        "blocked_recovery_exhausted",
                        task_id=task.task_id,
                        recovery_attempts=blocked_recoveries - 1,
                    )
                    return "failed"
                self.total_blocked_recoveries += 1
                self._record_event(
                    "blocked_recovery_started",
                    task_id=task.task_id,
                    recovery_attempt=blocked_recoveries,
                )
                wait_result = self._wait_for_clearance(task, blocked_recoveries)
                if wait_result in {"canceled", "low_battery"}:
                    return wait_result
                continue

            attempt += 1
            if attempt <= task.retries:
                self.total_retries += 1
                self._set_state(
                    MissionState.RETRYING,
                    f"Retrying {task.name} ({attempt}/{task.retries})",
                )
                self._record_event(
                    "task_retry", task_id=task.task_id, retry=attempt
                )
                retry_deadline = time.monotonic() + 3.0
                while time.monotonic() < retry_deadline:
                    rclpy.spin_once(self, timeout_sec=0.2)

        return "failed"

    def _return_home(self) -> bool:
        self.current_task = None
        self.current_task_index = len(self.mission.tasks)
        self.cancel_requested = False
        self.pause_requested = False
        self._record_event("return_home_started")
        for attempt in range(1, 4):
            outcome = self._navigate_to_pose(
                self.mission.home,
                timeout_seconds=120.0,
                state=MissionState.RETURNING_HOME,
                ignore_battery=True,
            )
            if outcome == "succeeded":
                self._record_event("return_home_completed", attempt=attempt)
                return True
            if outcome == "blocked":
                self.total_blocked_recoveries += 1
                wait_result = self._wait_for_clearance(
                    None,
                    attempt,
                    ignore_battery=True,
                )
                if wait_result == "canceled":
                    self.cancel_requested = False
                continue
            if outcome == "paused":
                if self._wait_while_paused() == "canceled":
                    self.cancel_requested = False
                continue
            self._record_event("return_home_retry", attempt=attempt)
            if attempt < 3:
                retry_deadline = time.monotonic() + 2.0
                while time.monotonic() < retry_deadline:
                    rclpy.spin_once(self, timeout_sec=0.2)
        self._record_event("return_home_failed")
        return False

    def _write_report(self) -> Path:
        self.report_directory.mkdir(parents=True, exist_ok=True)
        safe_timestamp = datetime.now().strftime("%Y%m%d-%H%M%S")
        report_path = self.report_directory / (
            f"{self.mission.mission_id}-{safe_timestamp}.json"
        )
        report = {
            "mission_id": self.mission.mission_id,
            "started_at": self.started_at,
            "finished_at": self.finished_at,
            "final_state": self.state.value,
            "completed_tasks": self.completed_tasks,
            "failed_tasks": self.failed_tasks,
            "skipped_tasks": self.skipped_tasks,
            "total_retries": self.total_retries,
            "total_blocked_recoveries": self.total_blocked_recoveries,
            "total_blocked_wait_seconds": round(
                self.total_blocked_wait_seconds, 3
            ),
            "final_battery_percentage": self.battery_percentage,
            "events": self.events,
        }
        report_path.write_text(
            json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8"
        )
        self.get_logger().info(f"Mission report written to {report_path}")
        return report_path

    def run_mission(self) -> bool:
        self.started_at = self._utc_now()
        self._set_state(MissionState.WAITING_FOR_NAV2, "Waiting for Nav2")
        self.setInitialPose(self._pose_stamped(self.mission.home))
        self._record_event(
            "initial_pose_set",
            x=self.mission.home.x,
            y=self.mission.home.y,
            yaw=self.mission.home.yaw,
        )
        self.waitUntilNav2Active()
        self._record_event("nav2_ready")

        finish_reason = "completed"
        aborted = False
        for index, task in enumerate(self.mission.tasks):
            self.current_task_index = index
            self.current_task = task
            outcome = self._execute_task(task)

            if outcome == "succeeded":
                self.completed_tasks.append(task.task_id)
                self._set_task_marker(task, "SUCCEEDED")
                self._record_event("task_completed", task_id=task.task_id)
                continue

            if outcome == "low_battery":
                finish_reason = "low_battery"
                self._set_state(
                    MissionState.LOW_BATTERY,
                    f"Battery at {self.battery_percentage:.1f}%; returning home",
                )
                self._set_task_marker(task, "PENDING")
                break

            if outcome == "canceled":
                finish_reason = "canceled"
                self._set_task_marker(task, "PENDING")
                break

            self.failed_tasks.append(task.task_id)
            self._set_task_marker(task, "FAILED")
            self._record_event(
                "task_failed",
                task_id=task.task_id,
                failure_policy=task.failure_policy,
            )
            if task.failure_policy == "abort":
                finish_reason = "failed"
                aborted = True
                break
            self.skipped_tasks.append(task.task_id)
            self._set_task_marker(task, "SKIPPED")

        should_return = (
            self.mission.return_home
            or finish_reason == "low_battery"
            or (finish_reason == "canceled" and self.mission.return_home_on_cancel)
        )
        home_succeeded = self._return_home() if should_return else True

        if not home_succeeded:
            self._set_state(MissionState.FAILED, "Unable to return home")
        elif finish_reason == "canceled":
            self._set_state(MissionState.CANCELED, "Mission canceled; robot returned home")
        elif finish_reason == "low_battery":
            self._set_state(
                MissionState.COMPLETED_WITH_WARNINGS,
                "Low battery; unfinished mission returned home",
            )
        elif aborted:
            self._set_state(MissionState.FAILED, "Mission aborted by failure policy")
        elif self.failed_tasks:
            self._set_state(
                MissionState.COMPLETED_WITH_WARNINGS,
                "Mission completed with skipped tasks",
            )
        else:
            self._set_state(MissionState.COMPLETED, "Mission completed successfully")

        self.finished_at = self._utc_now()
        self._record_event("mission_finished", final_state=self.state.value)
        self._write_report()
        return self.state in {
            MissionState.COMPLETED,
            MissionState.COMPLETED_WITH_WARNINGS,
            MissionState.CANCELED,
        }


def main(args=None) -> None:
    rclpy.init(args=args)
    manager: MissionManager | None = None
    exit_code = 1
    try:
        manager = MissionManager()
        exit_code = 0 if manager.run_mission() else 1
    except KeyboardInterrupt:
        pass
    except Exception as error:  # noqa: BLE001 - report ROS launch-time failures clearly
        if manager is not None:
            manager.get_logger().error(f"Mission manager failed: {error}")
        else:
            print(f"Mission manager failed: {error}", file=sys.stderr)
    finally:
        if manager is not None and rclpy.ok():
            manager.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    raise SystemExit(exit_code)


if __name__ == "__main__":
    main()
