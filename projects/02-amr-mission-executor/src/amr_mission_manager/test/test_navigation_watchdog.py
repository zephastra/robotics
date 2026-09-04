from amr_mission_manager.navigation_watchdog import NavigationProgressWatchdog


def _watchdog() -> NavigationProgressWatchdog:
    return NavigationProgressWatchdog(
        timeout_seconds=5.0,
        minimum_progress_distance=0.2,
        last_progress_at=0.0,
    )


def test_watchdog_detects_no_progress() -> None:
    watchdog = _watchdog()

    assert watchdog.update(8.0, 0.0) is False
    assert watchdog.update(7.95, 4.0) is False
    assert watchdog.update(7.94, 5.0) is True


def test_watchdog_resets_after_meaningful_progress() -> None:
    watchdog = _watchdog()

    assert watchdog.update(8.0, 0.0) is False
    assert watchdog.update(7.7, 4.0) is False
    assert watchdog.update(7.65, 8.0) is False
    assert watchdog.update(7.64, 9.0) is True


def test_replanning_distance_increase_does_not_hide_a_stationary_robot() -> None:
    watchdog = _watchdog()

    assert watchdog.update(4.0, 0.0, 1.0, 2.0) is False
    assert watchdog.update(5.2, 4.5, 1.0, 2.0) is False
    assert watchdog.update(5.15, 5.0, 1.0, 2.0) is True


def test_robot_position_progress_resets_after_replanning() -> None:
    watchdog = _watchdog()

    assert watchdog.update(4.0, 0.0, 1.0, 2.0) is False
    assert watchdog.update(5.2, 4.5, 1.25, 2.0) is False
    assert watchdog.update(5.1, 8.0, 1.30, 2.0) is False
    assert watchdog.update(5.05, 9.4, 1.31, 2.0) is False
    assert watchdog.update(5.04, 9.5, 1.31, 2.0) is True


def test_motion_without_goal_progress_eventually_counts_as_blocked() -> None:
    watchdog = _watchdog()

    assert watchdog.update(4.0, 0.0, 1.0, 2.0) is False
    assert watchdog.update(5.2, 4.5, 1.25, 2.0) is False
    assert watchdog.update(5.1, 9.0, 1.50, 2.0) is False
    assert watchdog.update(5.0, 14.0, 1.75, 2.0) is False
    assert watchdog.update(4.9, 15.0, 2.00, 2.0) is True
