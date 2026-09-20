"""P6 display tests. No ROS, no simulator.

AI_EXECUTION_PROMPTS item 6 asks for display tests covering log disconnect, data
expiry and unknown resources, and adds the sentence that makes them matter: the page
"must not keep showing normal after the link is gone".

So the tests below are mostly negative. It is easy to write a dashboard that looks
right while the fleet is healthy; the failure that matters is the one where it keeps
looking right after the fleet has stopped answering.
"""

from __future__ import annotations

from fleet_tools.display import BAD, OK, UNKNOWN, WARN, liveness, summarise


def live_snapshot() -> dict:
    return {
        "backend": "gazebo_nav2",
        "payload_mode": "logical: no mechanical handling in v1",
        "epoch": 7,
        "robots": {
            "r01": {
                "robot_id": "r01", "x": -6.0, "y": -2.0, "battery_fraction": 0.94,
                "band": "OK", "operating_state": "EXECUTING", "task_id": "t1",
                "fresh": True, "stopped_confirmed": False, "gate_reason": "",
                "fault_code": "", "parked": "", "localization_valid": True,
            },
            "r02": {
                "robot_id": "r02", "x": 6.0, "y": -2.0, "battery_fraction": 0.12,
                "band": "LOW", "operating_state": "IDLE", "task_id": "",
                "fresh": True, "stopped_confirmed": True, "gate_reason": "",
                "fault_code": "", "parked": "", "localization_valid": True,
            },
        },
        "tasks": [{"task_id": "t1", "state": "EXECUTING", "kind": "station_transfer",
                   "robot_id": "r01", "leg": "to_pick-0", "attempts": 0, "detail": ""}],
        "payloads": [{"payload_id": "cargo_1", "state": "HELD", "holder": "r01"}],
        "chargers": {"grants": 0},
        "refusals": {"NO_CAPABLE_ROBOT": 3},
        "reconcile": {"not_resumed": []},
    }


def live_traffic() -> dict:
    return {
        "age_s": 1.2,
        "resources": {
            "mid": {"state": "FREE", "owner": "", "generation": 0, "queue": []},
            "mid_left": {"state": "OCCUPIED", "owner": "r01", "generation": 4, "queue": []},
            "mid_right": {"state": "UNKNOWN", "owner": "", "generation": 0,
                          "queue": ["r02"]},
            "mid_blocked": {"state": "BLOCKED", "owner": "", "generation": 0, "queue": []},
        },
    }


# --------------------------------------------------------------------------- #
# liveness
# --------------------------------------------------------------------------- #


def test_no_reading_is_not_age_zero():
    """The whole point of the distinction.

    A node that never answered and a node that answered a microsecond ago are the two
    ends of the scale. Collapsing them would let a dead fleet render as perfectly
    current, which is the failure the P6 instructions single out.
    """
    never = liveness(None)
    assert never.state == "DISCONNECTED"
    assert never.may_show_ok is False
    assert not never.live
    fresh = liveness(0.0)
    assert fresh.state == "LIVE" and fresh.may_show_ok is True


def test_age_bands():
    assert liveness(1.0, fresh_s=3.0, stale_s=12.0).state == "LIVE"
    assert liveness(3.0, fresh_s=3.0, stale_s=12.0).state == "LIVE"
    assert liveness(5.0, fresh_s=3.0, stale_s=12.0).state == "AGING"
    assert liveness(30.0, fresh_s=3.0, stale_s=12.0).state == "STALE"


def test_only_live_may_show_ok():
    for age in (None, 4.0, 60.0):
        assert liveness(age).may_show_ok is False, age


# --------------------------------------------------------------------------- #
# summarise: the page must not keep looking normal
# --------------------------------------------------------------------------- #


def test_a_stale_snapshot_produces_no_ok_state_anywhere():
    """The negative test that this whole module exists for.

    Feed it a snapshot that would be entirely green if it were current, aged past the
    stale threshold, and assert that not one display state comes back as `ok`.
    """
    page = summarise(live_snapshot(), live_traffic(), 45.0, traffic_age_s=45.0)
    assert page["liveness"]["state"] == "STALE"
    assert page["liveness"]["may_show_ok"] is False
    for row in page["robots"]:
        assert row["display_state"] != OK, row
        assert row["display_state"] == UNKNOWN, row
    assert all("stale" in " ".join(r["notes"]) for r in page["robots"])


def test_a_stale_snapshot_withholds_the_values_it_cannot_vouch_for():
    """Stale numbers are worse than absent ones: they get quoted.

    Battery, pose and the task list are all withheld once the reading is not live,
    because the alternative is a page showing 94% on a robot that may have been flat
    for a minute.
    """
    page = summarise(live_snapshot(), live_traffic(), 45.0, traffic_age_s=45.0)
    for row in page["robots"]:
        assert row["battery_fraction"] is None
        assert row["position"] is None
        assert row["battery_band"] == "UNKNOWN"
    assert page["tasks"] == []
    assert page["payloads"] == []
    assert page["backend"] == "UNKNOWN"
    assert page["payload_mode"] == "UNKNOWN"


def test_no_reading_at_all():
    page = summarise(None, None, None)
    assert page["liveness"]["state"] == "DISCONNECTED"
    assert page["robots"] == []
    assert page["resources"] == []
    assert page["backend"] == "UNKNOWN"
    assert "no reading" in page["liveness"]["banner"]


def test_a_live_reading_does_report_the_fleet():
    page = summarise(live_snapshot(), live_traffic(), 0.5, traffic_age_s=1.2)
    assert page["liveness"]["may_show_ok"] is True
    states = {r["robot_id"]: r["display_state"] for r in page["robots"]}
    assert states == {"r01": OK, "r02": WARN}, "a LOW battery is a warning, not an OK"
    assert page["backend"] == "gazebo_nav2"
    assert page["tasks"] and page["tasks"][0]["task_id"] == "t1"


def test_the_task_service_freshness_flag_outranks_the_pages_own_age():
    """Two freshness tests exist and the stricter one wins.

    The page's age is about the last successful poll; the task service's `fresh` is
    about whether the robot spoke recently. A robot that went quiet a while ago can
    still be inside a perfectly current snapshot, and the page must not colour it OK.
    """
    snap = live_snapshot()
    snap["robots"]["r01"]["fresh"] = False
    page = summarise(snap, live_traffic(), 0.5, traffic_age_s=1.2)
    row = next(r for r in page["robots"] if r["robot_id"] == "r01")
    assert row["display_state"] == UNKNOWN
    assert any("not fresh" in n for n in row["notes"])
    assert row["position"] is None


def test_faults_and_parks_show_as_bad_not_as_ok():
    snap = live_snapshot()
    snap["robots"]["r01"]["fault_code"] = "injected_fault"
    page = summarise(snap, live_traffic(), 0.5, traffic_age_s=1.2)
    assert next(r for r in page["robots"] if r["robot_id"] == "r01")["display_state"] == BAD


# --------------------------------------------------------------------------- #
# resources: unknown is never available
# --------------------------------------------------------------------------- #


def test_unknown_and_blocked_resources_are_never_displayed_as_free():
    """The same rule the gate enforces, applied to the picture.

    A page that painted an UNKNOWN region green would be advising a human that a
    corridor is open while the gate is holding it shut.
    """
    page = summarise(live_snapshot(), live_traffic(), 0.5, traffic_age_s=1.2)
    rows = {r["name"]: r for r in page["resources"]}
    assert rows["mid"]["display_state"] == OK
    assert rows["mid_left"]["display_state"] == WARN
    assert rows["mid_right"]["state"] == "UNKNOWN"
    assert rows["mid_right"]["display_state"] == BAD
    assert rows["mid_blocked"]["display_state"] == BAD
    assert rows["mid_right"]["queue"] == ["r02"]
    assert rows["mid_left"]["generation"] == 4


def test_the_coordinator_reading_is_labelled_with_its_own_age():
    """Two sources, two ages. Folding them together would let a live task service
    vouch for a coordinator that stopped answering."""
    page = summarise(live_snapshot(), live_traffic(), 0.5, traffic_age_s=1.2)
    assert page["traffic_age_s"] == 1.2
    page = summarise(live_snapshot(), None, 0.5)
    assert page["resources"] == []
    assert page["traffic_age_s"] is None


def test_the_page_says_it_is_read_only():
    page = summarise(live_snapshot(), live_traffic(), 0.5, traffic_age_s=1.2)
    assert "no control endpoints" in page["read_only"]
    assert "fleet_cli" in page["read_only"]


# --------------------------------------------------------------------------- #
# the corridor is a second source and gets its own age
# --------------------------------------------------------------------------- #


def test_the_corridor_view_is_withheld_when_its_own_reading_is_old():
    """The corridor comes from the coordinator, the tasks from the task service.

    Ageing them together would show a stale corridor on a page that still said LIVE, and
    quote the owner/generation a crossing decision turns on as if it were current. The
    task half here is genuinely fine, which is the point: one source being old must not
    erase the other, and must not be hidden by it either.
    """
    page = summarise(live_snapshot(), live_traffic(), 0.5, traffic_age_s=45.0)
    assert page["liveness"]["may_show_ok"] is True, "the task half really is current"
    assert page["traffic_liveness"]["may_show_ok"] is False
    assert page["traffic_liveness"]["state"] == "STALE"
    rows = {r["name"]: r for r in page["resources"]}
    assert rows, "the resources are still listed: which ones exist is not the stale part"
    for row in rows.values():
        assert row["display_state"] != OK, row
        assert row["owner"] == "" and row["queue"] == [] and row["generation"] == 0, row
        assert any("corridor" in note for note in row["notes"]), row
    assert page["tasks"], "the task list is not affected by the corridor's age"
    assert page["backend"] == "gazebo_nav2"


def test_a_fresh_corridor_reading_is_shown_with_its_owner():
    page = summarise(live_snapshot(), live_traffic(), 0.5, traffic_age_s=0.4)
    rows = {r["name"]: r for r in page["resources"]}
    assert rows["mid"]["display_state"] == OK
    assert page["traffic_liveness"]["may_show_ok"] is True
    assert page["traffic_liveness"]["age_s"] == 0.4


# --------------------------------------------------------------------------- #
# the rows must be identifiable, from a payload shaped the way PRODUCTION shapes it
# --------------------------------------------------------------------------- #


def production_snapshot() -> dict:
    """A payload built to `task_service_node.snapshot()`'s contract.

    The id is the key of the robots dict and there is NO `robot_id` field inside the entry.
    `live_snapshot()` above adds the field, which is why it could not catch the defect that
    made every row read "?": a fixture that is kinder than production hides exactly the bugs
    that production causes.
    """
    return {
        "backend": "gazebo_nav2",
        "payload_mode": "logical: no mechanical handling in v1",
        "epoch": 7,
        "robots": {
            "r01": {"x": -6.0, "y": -2.0, "yaw": 0.0, "battery_fraction": 0.94,
                    "battery_wh": 94.0, "band": "OK", "operating_state": "EXECUTING",
                    "task_id": "t1", "fresh": True, "stopped_confirmed": False,
                    "gate_reason": "", "fault_code": "", "parked": "",
                    "localization_valid": True},
            "r02": {"x": 6.0, "y": -2.0, "yaw": 3.14, "battery_fraction": 0.21,
                    "battery_wh": 21.0, "band": "LOW", "operating_state": "IDLE",
                    "task_id": "", "fresh": True, "stopped_confirmed": True,
                    "gate_reason": "", "fault_code": "", "parked": "",
                    "localization_valid": True},
            "r03": {"x": -6.0, "y": 0.6, "yaw": 0.0, "battery_fraction": 0.80,
                    "battery_wh": 80.0, "band": "OK", "operating_state": "IDLE",
                    "task_id": "", "fresh": True, "stopped_confirmed": True,
                    "gate_reason": "", "fault_code": "", "parked": "",
                    "localization_valid": True},
        },
        "tasks": [],
        "payloads": [],
    }


def test_robot_rows_are_identifiable_from_a_production_shaped_payload():
    page = summarise(production_snapshot(), live_traffic(), 0.5, traffic_age_s=0.4)
    ids = [r["robot_id"] for r in page["robots"]]
    assert ids == ["r01", "r02", "r03"], (
        "every row read '?' when the id was only available as the robots dict key: "
        f"got {ids}")
    assert "?" not in ids, ids
    # The values still have to come through, or "identifiable" is the only thing that works.
    by_id = {r["robot_id"]: r for r in page["robots"]}
    assert by_id["r01"]["display_state"] == OK and by_id["r01"]["task_id"] == "t1"
    assert by_id["r02"]["display_state"] == WARN, "a LOW battery is a warning"
    assert by_id["r03"]["position"] == (-6.0, 0.6)


def test_a_dead_robot_can_be_told_apart_from_the_live_ones():
    """The case the live check exercised: kill one adapter, keep the fleet up."""
    snap = production_snapshot()
    snap["robots"]["r03"]["fresh"] = False
    page = summarise(snap, live_traffic(), 0.5, traffic_age_s=0.4)
    dead = [r for r in page["robots"] if r["robot_id"] == "r03"]
    alive = [r for r in page["robots"] if r["robot_id"] != "r03"]
    assert len(dead) == 1 and len(alive) == 2
    assert dead[0]["display_state"] == UNKNOWN
    assert dead[0]["position"] is None and dead[0]["battery_fraction"] is None
    assert any("not fresh" in n for n in dead[0]["notes"])
    assert all(r["display_state"] == OK and r["position"] for r in alive if r["robot_id"] == "r01")



def test_without_a_corridor_age_the_corridor_is_not_vouched_for():
    """No age given means no age known.

    It must NOT fall back to the task service's age: that fallback is exactly the
    conflation this parameter exists to prevent, and it is how `traffic.get("age_s")`
    came to be read from a payload that never carried it.
    """
    page = summarise(live_snapshot(), live_traffic(), 0.5)
    assert page["traffic_liveness"]["state"] == "DISCONNECTED"
    assert page["traffic_age_s"] is None
    assert all(r["display_state"] == UNKNOWN for r in page["resources"])
