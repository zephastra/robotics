"""Shared fixtures for the P1 core tests.

Every fixture here is deliberately tiny and explicit: if a test needs a fleet of
three robots or a corridor with two exit buffers, it says so in its own body
rather than inheriting a large opaque fixture.
"""

from __future__ import annotations

import sys
from pathlib import Path

import pytest

SRC = Path(__file__).resolve().parents[1] / "src"
# ament_python layout is src/<pkg>/<pkg>/... so the *inner* directory is what
# must be on sys.path. Adding the outer one makes `fleet_core` resolve to a
# namespace package with no __init__.py.
for _pkg in ("fleet_core", "fleet_adapter", "fleet_tools", "fleet_evaluation"):
    _p = SRC / _pkg
    if str(_p) not in sys.path:
        sys.path.insert(0, str(_p))

from fleet_core import (  # noqa: E402
    Clock,
    FakeClock,
    Ledger,
    ResourceBook,
    TaskSpec,
    TaskMachine,
    validate_fleet_config,
)

# --------------------------------------------------------------------------- #
# config
# --------------------------------------------------------------------------- #


def base_config_raw() -> dict:
    return {
        "robots": {
            "r01": {
                "capabilities": ["CARRY"],
                "battery_capacity_wh": 100.0,
                "discharge_wh_per_m": 0.5,
                "discharge_wh_per_s": 0.01,
                "max_speed_mps": 0.35,
                "footprint_length_m": 0.60,
                "footprint_width_m": 0.45,
            },
            "r02": {
                "capabilities": ["CARRY"],
                "battery_capacity_wh": 100.0,
                "discharge_wh_per_m": 0.5,
                "discharge_wh_per_s": 0.01,
                "max_speed_mps": 0.35,
                "footprint_length_m": 0.60,
                "footprint_width_m": 0.45,
            },
            "r03": {
                "capabilities": ["INSPECT"],
                "battery_capacity_wh": 100.0,
                "discharge_wh_per_m": 0.5,
                "discharge_wh_per_s": 0.01,
                "max_speed_mps": 0.35,
                "footprint_length_m": 0.60,
                "footprint_width_m": 0.45,
            },
        },
        "stations": {
            "S_left": {"x": -5.0, "y": 2.5},
            "S_right": {"x": 5.0, "y": 2.5},
            "S_left2": {"x": -5.0, "y": -2.5},
        },
        "corridor": {
            "name": "mid",
            "exit_buffers": {"left": "mid_left", "right": "mid_right"},
        },
        "chargers": {"C1": {}, "C2": {}},
        "charger_capacity": 1,
        "safety": {
            "permit_ttl_s": 30.0,
            "battery_reserve_wh": 5.0,
            "low_battery_wh": 15.0,
        },
    }


@pytest.fixture
def cfg():
    return validate_fleet_config(base_config_raw())


@pytest.fixture
def clock() -> FakeClock:
    return FakeClock()


@pytest.fixture
def ledger(tmp_path) -> Ledger:
    led = Ledger(tmp_path / "fleet.sqlite3")
    yield led
    led.close()


@pytest.fixture
def machine(ledger) -> TaskMachine:
    return TaskMachine(ledger, epoch=0)


@pytest.fixture
def book(cfg) -> ResourceBook:
    b = ResourceBook(permit_ttl_s=cfg.permit_ttl_s)
    b.capacity[cfg.corridor] = 1
    for rid in cfg.exit_buffers.values():
        b.capacity[rid] = 1
    for rid in cfg.chargers.values():
        b.capacity[rid] = cfg.charger_capacity
    return b


@pytest.fixture
def spec_factory():
    def make(request_id: str = "req-1", **kw) -> TaskSpec:
        base = dict(
            request_id=request_id,
            pick_station="S_left",
            drop_station="S_right",
        )
        base.update(kw)
        return TaskSpec(**base)

    return make
