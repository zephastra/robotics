"""L1 §1-2: config and input validation must fail closed.

Maps to TEST_AND_ACCEPTANCE L1 item 1 ("配置字段、单位、非法值、未知站点、重复 ID").
"""

from __future__ import annotations

import copy

import pytest

from fleet_core import ConfigError, FleetConfig, validate_fleet_config, validate_task_spec
from fleet_core.domain import Capability, TaskSpec

from conftest import base_config_raw


# --------------------------------------------------------------------------- #
# structural failures
# --------------------------------------------------------------------------- #


def test_valid_config_parses():
    cfg = validate_fleet_config(base_config_raw())
    assert isinstance(cfg, FleetConfig)
    assert set(cfg.robots) == {"r01", "r02", "r03"}
    assert cfg.permit_ttl_s == 30.0


@pytest.mark.parametrize("bad_root", [None, [], "nope", 3])
def test_non_mapping_root_rejected(bad_root):
    with pytest.raises(ConfigError):
        validate_fleet_config(bad_root)  # type: ignore[arg-type]


def test_missing_robots_rejected():
    raw = base_config_raw()
    del raw["robots"]
    with pytest.raises(ConfigError, match="robots"):
        validate_fleet_config(raw)


def test_empty_robots_rejected():
    raw = base_config_raw()
    raw["robots"] = {}
    with pytest.raises(ConfigError):
        validate_fleet_config(raw)


def test_empty_stations_rejected():
    raw = base_config_raw()
    raw["stations"] = {}
    with pytest.raises(ConfigError):
        validate_fleet_config(raw)


# --------------------------------------------------------------------------- #
# per-field failures -- units are part of the name, so a wrong field is caught
# --------------------------------------------------------------------------- #


@pytest.mark.parametrize(
    "field",
    [
        "battery_capacity_wh",
        "discharge_wh_per_m",
        "discharge_wh_per_s",
        "max_speed_mps",
        "footprint_length_m",
        "footprint_width_m",
    ],
)
def test_missing_required_robot_field_rejected(field):
    raw = base_config_raw()
    del raw["robots"]["r01"][field]
    with pytest.raises(ConfigError, match=field):
        validate_fleet_config(raw)


@pytest.mark.parametrize("value", [0, -1, -0.5])
def test_non_positive_physical_values_rejected(value):
    raw = base_config_raw()
    raw["robots"]["r01"]["battery_capacity_wh"] = value
    with pytest.raises(ConfigError, match="battery_capacity_wh"):
        validate_fleet_config(raw)


@pytest.mark.parametrize("value", ["fast", None, [1]])
def test_non_numeric_value_rejected(value):
    raw = base_config_raw()
    raw["robots"]["r01"]["max_speed_mps"] = value
    with pytest.raises(ConfigError, match="max_speed_mps"):
        validate_fleet_config(raw)


def test_nan_and_inf_station_coordinates_rejected():
    for bad in (float("nan"), float("inf")):
        raw = base_config_raw()
        raw["stations"]["S_left"]["x"] = bad
        with pytest.raises(ConfigError, match="S_left.x"):
            validate_fleet_config(raw)


def test_unknown_capability_rejected():
    raw = base_config_raw()
    raw["robots"]["r01"]["capabilities"] = ["FLY"]
    with pytest.raises(ConfigError, match="unknown capability"):
        validate_fleet_config(raw)


def test_empty_capabilities_rejected():
    raw = base_config_raw()
    raw["robots"]["r01"]["capabilities"] = []
    with pytest.raises(ConfigError):
        validate_fleet_config(raw)


def test_robot_spec_rejects_non_positive_battery_directly():
    """The dataclass guards itself too, not only the parser."""
    from fleet_core.domain import RobotSpec

    with pytest.raises(ValueError):
        RobotSpec(
            robot_id="bad",
            capabilities=frozenset({Capability.CARRY}),
            battery_capacity_wh=0.0,
            discharge_wh_per_m=0.5,
            discharge_wh_per_s=0.01,
            max_speed_mps=0.35,
            footprint_length_m=0.6,
            footprint_width_m=0.45,
        )


# --------------------------------------------------------------------------- #
# corridor must be reservable as a bundle
# --------------------------------------------------------------------------- #


def test_corridor_without_exit_buffer_rejected():
    """A corridor that cannot express an atomic bundle must not load."""
    raw = base_config_raw()
    raw["corridor"] = {"name": "mid", "exit_buffers": {}}
    with pytest.raises(ConfigError, match="exit_buffers"):
        validate_fleet_config(raw)


def test_corridor_absent_is_allowed():
    raw = base_config_raw()
    del raw["corridor"]
    cfg = validate_fleet_config(raw)
    assert cfg.corridor is None


def test_low_battery_must_exceed_reserve():
    raw = base_config_raw()
    raw["safety"]["low_battery_wh"] = 3.0
    raw["safety"]["battery_reserve_wh"] = 5.0
    with pytest.raises(ConfigError, match="low_battery_wh"):
        validate_fleet_config(raw)


# --------------------------------------------------------------------------- #
# task specs
# --------------------------------------------------------------------------- #


def test_task_unknown_station_rejected(cfg):
    bad = TaskSpec(request_id="r", pick_station="S_left", drop_station="NOWHERE")
    with pytest.raises(ConfigError, match="UNKNOWN_STATION"):
        validate_task_spec(bad, cfg)


def test_task_unknown_pick_station_rejected(cfg):
    bad = TaskSpec(request_id="r", pick_station="NOWHERE", drop_station="S_left")
    with pytest.raises(ConfigError, match="pick_station"):
        validate_task_spec(bad, cfg)


def test_task_same_pick_and_drop_rejected(cfg):
    bad = TaskSpec(request_id="r", pick_station="S_left", drop_station="S_left")
    with pytest.raises(ConfigError):
        validate_task_spec(bad, cfg)


def test_task_empty_request_id_rejected(cfg):
    bad = TaskSpec(request_id="", pick_station="S_left", drop_station="S_right")
    with pytest.raises(ConfigError, match="request_id"):
        validate_task_spec(bad, cfg)


def test_identical_station_names_are_not_confused(cfg):
    """Unknown must be rejected rather than fuzzy-matched to a near miss."""
    bad = TaskSpec(request_id="r", pick_station="S_LEFT", drop_station="S_right")
    with pytest.raises(ConfigError):
        validate_task_spec(bad, cfg)


def test_config_parsing_does_not_mutate_input():
    raw = base_config_raw()
    snapshot = copy.deepcopy(raw)
    validate_fleet_config(raw)
    assert raw == snapshot
