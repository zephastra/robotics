"""Pure navigation progress monitoring used by the mission manager."""

from dataclasses import dataclass
from math import hypot


@dataclass
class NavigationProgressWatchdog:
    timeout_seconds: float
    minimum_progress_distance: float
    last_progress_at: float
    reference_distance: float | None = None
    reference_x: float | None = None
    reference_y: float | None = None
    last_motion_at: float | None = None

    def update(
        self,
        distance: float,
        now: float,
        current_x: float | None = None,
        current_y: float | None = None,
    ) -> bool:
        """Return True when neither the robot nor its goal distance has progressed."""
        if self.reference_distance is None:
            self.reference_distance = distance
            self.reference_x = current_x
            self.reference_y = current_y
            self.last_progress_at = now
            self.last_motion_at = now
            return False

        improvement = self.reference_distance - distance
        position_progress = False
        if (
            current_x is not None
            and current_y is not None
            and self.reference_x is not None
            and self.reference_y is not None
        ):
            position_progress = (
                hypot(current_x - self.reference_x, current_y - self.reference_y)
                >= self.minimum_progress_distance
            )

        if improvement >= self.minimum_progress_distance:
            self.reference_distance = distance
            self.last_progress_at = now

        if position_progress:
            self.reference_x = current_x
            self.reference_y = current_y
            self.last_motion_at = now

        if current_x is None or current_y is None:
            return now - self.last_progress_at >= self.timeout_seconds

        last_motion_at = self.last_motion_at
        if last_motion_at is None:
            last_motion_at = self.last_progress_at

        stopped_too_long = now - last_motion_at >= self.timeout_seconds
        no_goal_progress_too_long = (
            now - self.last_progress_at >= self.timeout_seconds * 3.0
        )
        return stopped_too_long or no_goal_progress_too_long
