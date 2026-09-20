"""Config validation. Fail closed: an unknown or ill-formed field is an error.

Rules that matter more than they look:

  * unknown station in a task spec -> rejected, never "closest match"
  * duplicate ids -> rejected, not silently overwritten
  * units are explicit in every name (`_m`, `_mps`, `_wh`, `_s`); a bare number
    with a wrong unit is a silent safety bug, so no field is unit-less
  * a corridor without an exit buffer is rejected -- CONTRACTS requires the two
    to be reserved atomically, so a config that cannot express that must not load
"""

from __future__ import annotations

from typing import Any

from .domain import (
    BatteryConfig,
    Capability,
    DirectionSpec,
    FleetConfig,
    NodePose,
    PayloadMode,
    ReasonCode,
    ResourceId,
    ResourceKind,
    ResourceSpec,
    RobotSpec,
    StationSpec,
    TaskKind,
    TaskSpec,
    TrafficConfig,
)
from .geometry import Rect, StopModel


class ConfigError(ValueError):
    """Raised for any config that cannot be trusted."""


_REQUIRED_ROBOT_FIELDS = (
    "battery_capacity_wh",
    "discharge_wh_per_m",
    "discharge_wh_per_s",
    "max_speed_mps",
    "footprint_length_m",
    "footprint_width_m",
)


def _need(mapping: dict[str, Any], key: str, where: str) -> Any:
    if key not in mapping:
        raise ConfigError(f"{where}: missing required field '{key}'")
    return mapping[key]


def _positive(value: Any, where: str) -> float:
    try:
        f = float(value)
    except (TypeError, ValueError) as exc:
        raise ConfigError(f"{where}: expected a number, got {value!r}") from exc
    if not (f > 0):
        raise ConfigError(f"{where}: must be > 0, got {f}")
    return f


def _finite(value: Any, where: str) -> float:
    try:
        f = float(value)
    except (TypeError, ValueError) as exc:
        raise ConfigError(f"{where}: expected a number, got {value!r}") from exc
    if f != f or f in (float("inf"), float("-inf")):
        raise ConfigError(f"{where}: must be finite, got {f}")
    return f


def parse_robot(robot_id: str, raw: dict[str, Any]) -> RobotSpec:
    where = f"robots.{robot_id}"
    if not isinstance(raw, dict):
        raise ConfigError(f"{where}: expected a mapping")
    for f in _REQUIRED_ROBOT_FIELDS:
        _need(raw, f, where)

    caps_raw = raw.get("capabilities", ["CARRY"])
    if not isinstance(caps_raw, (list, tuple)) or not caps_raw:
        raise ConfigError(f"{where}.capabilities: expected a non-empty list")
    caps: set[Capability] = set()
    for c in caps_raw:
        try:
            caps.add(Capability(str(c)))
        except ValueError as exc:
            raise ConfigError(f"{where}.capabilities: unknown capability {c!r}") from exc

    return RobotSpec(
        robot_id=robot_id,
        capabilities=frozenset(caps),
        battery_capacity_wh=_positive(raw["battery_capacity_wh"], f"{where}.battery_capacity_wh"),
        discharge_wh_per_m=_positive(raw["discharge_wh_per_m"], f"{where}.discharge_wh_per_m"),
        discharge_wh_per_s=_positive(raw["discharge_wh_per_s"], f"{where}.discharge_wh_per_s"),
        max_speed_mps=_positive(raw["max_speed_mps"], f"{where}.max_speed_mps"),
        footprint_length_m=_positive(raw["footprint_length_m"], f"{where}.footprint_length_m"),
        footprint_width_m=_positive(raw["footprint_width_m"], f"{where}.footprint_width_m"),
        turn_wh_per_rad=_non_negative(
            raw.get("turn_wh_per_rad", 0.0), f"{where}.turn_wh_per_rad"
        ),
    )


def parse_station(station_id: str, raw: dict[str, Any]) -> StationSpec:
    where = f"stations.{station_id}"
    if not isinstance(raw, dict):
        raise ConfigError(f"{where}: expected a mapping")
    return StationSpec(
        station_id=station_id,
        x=_finite(_need(raw, "x", where), f"{where}.x"),
        y=_finite(_need(raw, "y", where), f"{where}.y"),
        yaw=_finite(raw.get("yaw", 0.0), f"{where}.yaw"),
    )


def validate_fleet_config(raw: dict[str, Any]) -> FleetConfig:
    """Turn a raw mapping into a validated FleetConfig, or raise ConfigError."""
    if not isinstance(raw, dict):
        raise ConfigError("config root must be a mapping")

    robots_raw = _need(raw, "robots", "config")
    stations_raw = _need(raw, "stations", "config")
    if not isinstance(robots_raw, dict) or not robots_raw:
        raise ConfigError("config.robots must be a non-empty mapping")
    if not isinstance(stations_raw, dict) or not stations_raw:
        raise ConfigError("config.stations must be a non-empty mapping")

    robots = {rid: parse_robot(rid, r) for rid, r in robots_raw.items()}
    if len(robots) != len(robots_raw):
        raise ConfigError("duplicate robot ids in config")

    stations = {sid: parse_station(sid, s) for sid, s in stations_raw.items()}
    if len(stations) != len(stations_raw):
        raise ConfigError("duplicate station ids in config")

    # corridor + exit buffers: the pair is reserved atomically, so both halves
    # must be expressible or the config is unusable.
    corridor: ResourceId | None = None
    exit_buffers: dict[str, ResourceId] = {}
    corridor_raw = raw.get("corridor")
    if corridor_raw is not None:
        name = _need(corridor_raw, "name", "corridor")
        corridor = ResourceId(ResourceKind.CORRIDOR, str(name))
        for side, spec in (corridor_raw.get("exit_buffers") or {}).items():
            exit_buffers[str(side)] = ResourceId(ResourceKind.EXIT_BUFFER, str(spec))
        if not exit_buffers:
            raise ConfigError(
                "config.corridor has no exit_buffers: the corridor and its exit "
                "buffer must be reservable together (CONTRACTS §traffic)"
            )

    chargers: dict[str, ResourceId] = {}
    for cid in (raw.get("chargers") or {}):
        chargers[str(cid)] = ResourceId(ResourceKind.CHARGER, str(cid))

    safety = raw.get("safety") or {}
    permit_ttl_s = _positive(safety.get("permit_ttl_s", 30.0), "safety.permit_ttl_s")
    battery_reserve_wh = _finite(safety.get("battery_reserve_wh", 5.0), "safety.battery_reserve_wh")
    low_battery_wh = _finite(safety.get("low_battery_wh", 15.0), "safety.low_battery_wh")
    if low_battery_wh <= battery_reserve_wh:
        raise ConfigError(
            "safety.low_battery_wh must exceed safety.battery_reserve_wh, "
            f"got {low_battery_wh} <= {battery_reserve_wh}"
        )

    return FleetConfig(
        robots=robots,
        stations=stations,
        corridor=corridor,
        exit_buffers=exit_buffers,
        chargers=chargers,
        charger_capacity=int(raw.get("charger_capacity", 1) or 1),
        permit_ttl_s=permit_ttl_s,
        battery_reserve_wh=battery_reserve_wh,
        low_battery_wh=low_battery_wh,
        battery=parse_battery(raw),
        raw=raw,
    )


def validate_task_spec(spec: TaskSpec, cfg: FleetConfig) -> None:
    """Reject a task that names something the fleet does not have."""
    if not spec.request_id:
        raise ConfigError("task.request_id must be non-empty")
    if spec.kind is TaskKind.VISIT_STATION:
        # Declared by CONTRACTS section 2, deliberately not implemented. Saying so
        # here beats letting it fall through to the station checks, where it would
        # be reported as UNKNOWN_STATION for a station that exists -- a confusing
        # message about the wrong field, which is worse than no message.
        raise ConfigError(
            "task.kind=visit_station is declared in CONTRACTS section 2 but is not "
            "implemented in v1; see docs/LIMITATIONS.md"
        )
    if spec.kind is TaskKind.RETURN_TO_CHARGE:
        # A return-to-charge has no source. The origin is wherever the robot is,
        # which the dispatcher knows and the task has no business restating --
        # inventing a station the robot is not at would make the battery
        # pre-check compute its budget from the wrong place.
        if spec.pick_station:
            raise ConfigError(
                "task.kind=return_to_charge takes no pick_station, got "
                f"{spec.pick_station!r}"
            )
        if spec.drop_station not in cfg.chargers:
            raise ConfigError(
                f"task.drop_station={spec.drop_station!r}: "
                f"{ReasonCode.UNKNOWN_STATION.value} charger "
                f"(known: {sorted(cfg.chargers)})"
            )
        return

    for role, sid in (("pick_station", spec.pick_station), ("drop_station", spec.drop_station)):
        if sid not in cfg.stations:
            raise ConfigError(
                f"task.{role}={sid!r}: {ReasonCode.UNKNOWN_STATION.value} "
                f"(known: {sorted(cfg.stations)})"
            )
    if spec.pick_station == spec.drop_station:
        raise ConfigError("task pick_station and drop_station must differ")


def parse_battery(raw: dict[str, Any]) -> BatteryConfig:
    """Read the `battery:` section. A missing section means the contract defaults."""
    body = raw.get("battery") or {}
    if not isinstance(body, dict):
        raise ConfigError("config.battery must be a mapping")
    try:
        return BatteryConfig(
            charge_wh_per_s=_positive(
                body.get("charge_wh_per_s", 5.0), "battery.charge_wh_per_s"
            ),
            low_fraction=_finite(body.get("low_fraction", 0.20), "battery.low_fraction"),
            critical_fraction=_finite(
                body.get("critical_fraction", 0.08), "battery.critical_fraction"
            ),
            resume_fraction=_finite(
                body.get("resume_fraction", 0.80), "battery.resume_fraction"
            ),
        )
    except ValueError as exc:
        # BatteryConfig raises plain ValueError for a bad ordering; callers of
        # this module catch ConfigError, so translate rather than leak a type
        # nobody expects.
        raise ConfigError(f"config.battery: {exc}") from exc


def parse_task_request(raw: dict[str, Any], cfg: FleetConfig) -> TaskSpec:
    """Turn one submission (CLI, YAML or ROS service) into a validated TaskSpec.

    The enum lookups are strict on purpose. CONTRACTS section 2 warns that the
    internal enums may not be substituted by synonyms, so an unknown `kind` is an
    error and never a fallback to station_transfer.
    """
    if not isinstance(raw, dict):
        raise ConfigError("task request must be a mapping")
    request_id = str(raw.get("request_id") or "").strip()
    if not request_id:
        raise ConfigError("task.request_id must be non-empty")

    kind_raw = raw.get("kind", TaskKind.STATION_TRANSFER.value)
    try:
        kind = TaskKind(str(kind_raw))
    except ValueError as exc:
        raise ConfigError(
            f"task.kind={kind_raw!r}: unknown kind "
            f"(known: {sorted(k.value for k in TaskKind)})"
        ) from exc

    mode_raw = raw.get("payload_mode", PayloadMode.LOGICAL.value)
    try:
        mode = PayloadMode(str(mode_raw))
    except ValueError as exc:
        raise ConfigError(
            f"task.payload_mode={mode_raw!r}: unknown mode "
            f"(known: {sorted(m.value for m in PayloadMode)})"
        ) from exc

    cap_raw = raw.get("required_capability", Capability.CARRY.value)
    try:
        capability = Capability(str(cap_raw))
    except ValueError as exc:
        raise ConfigError(
            f"task.required_capability={cap_raw!r}: unknown capability "
            f"(known: {sorted(c.value for c in Capability)})"
        ) from exc

    spec = TaskSpec(
        request_id=request_id,
        pick_station=str(raw.get("pick_station") or ""),
        drop_station=str(
            raw.get("drop_station") or raw.get("destination_station") or ""
        ),
        required_capability=capability,
        payload_id=(str(raw["payload_id"]) if raw.get("payload_id") else None),
        kind=kind,
        payload_mode=mode,
        priority=int(raw.get("priority", 10)),
        service_duration_s=_non_negative(
            raw.get("service_duration_s", 0.0), "task.service_duration_s"
        ),
        max_attempts=int(raw.get("max_attempts", 2)),
        timeout_sim_s=_positive(raw.get("timeout_sim_s", 180.0), "task.timeout_sim_s"),
    )
    validate_task_spec(spec, cfg)
    return spec


# --------------------------------------------------------------------------- #
# P4 traffic config
# --------------------------------------------------------------------------- #

_TRAFFIC_KEYS = ("resources", "bundles", "nodes", "directions", "traffic")


def _non_negative(value: Any, where: str) -> float:
    try:
        f = float(value)
    except (TypeError, ValueError) as exc:
        raise ConfigError(f"{where}: expected a number, got {value!r}") from exc
    if f != f or f < 0.0:
        raise ConfigError(f"{where}: must be finite and >= 0, got {f}")
    return f


def _mapping(value: Any, where: str) -> dict:
    if not isinstance(value, dict):
        raise ConfigError(f"{where}: expected a mapping")
    return value


def validate_traffic_config(raw: dict[str, Any]) -> TrafficConfig:
    """Parse `config/resources.yaml`. Fail closed on anything unexpected.

    Geometric sanity -- "is the waiting point actually outside the region?" -- is
    deliberately NOT done here. It needs a robot footprint, which lives in
    fleet.yaml, and mixing the two files' concerns is how a config passes
    validation and still deadlocks. `scripts/validate_traffic_geometry.py` does
    that cross-check with both files in hand.
    """
    root = _mapping(raw, "resources config root")
    if root.get("schema_version") != 1:
        raise ConfigError(f"resources.schema_version must be 1, got {root.get('schema_version')!r}")
    for key in _TRAFFIC_KEYS:
        if key not in root:
            raise ConfigError(f"resources config: missing '{key}'")

    resources: dict[str, ResourceSpec] = {}
    for name, body in _mapping(root["resources"], "resources").items():
        where = f"resources.{name}"
        body = _mapping(body, where)
        kind_raw = _need(body, "kind", where)
        try:
            kind = ResourceKind(str(kind_raw))
        except ValueError as exc:
            raise ConfigError(f"{where}.kind: unknown resource kind {kind_raw!r}") from exc
        rects_raw = body.get("rects")
        if not isinstance(rects_raw, (list, tuple)) or not rects_raw:
            raise ConfigError(f"{where}.rects: expected a non-empty list")
        rects: list[Rect] = []
        for i, r in enumerate(rects_raw):
            try:
                rects.append(Rect.from_mapping(_mapping(r, f"{where}.rects[{i}]")))
            except ValueError as exc:
                raise ConfigError(f"{where}.rects[{i}]: {exc}") from exc
        resources[str(name)] = ResourceSpec(
            name=str(name),
            kind=kind,
            capacity=int(body.get("capacity", 1) or 1),
            rects=tuple(rects),
        )

    bundles: dict[str, tuple[str, ...]] = {}
    for direction, members in _mapping(root["bundles"], "bundles").items():
        if not isinstance(members, (list, tuple)) or not members:
            raise ConfigError(f"bundles.{direction}: expected a non-empty list")
        if len(set(members)) != len(members):
            raise ConfigError(f"bundles.{direction}: duplicate resource in bundle")
        bundles[str(direction)] = tuple(str(m) for m in members)

    nodes: dict[str, NodePose] = {}
    for name, body in _mapping(root["nodes"], "nodes").items():
        where = f"nodes.{name}"
        body = _mapping(body, where)
        nodes[str(name)] = NodePose(
            name=str(name),
            x=_finite(_need(body, "x", where), f"{where}.x"),
            y=_finite(_need(body, "y", where), f"{where}.y"),
            yaw=_finite(body.get("yaw", 0.0), f"{where}.yaw"),
        )

    directions: dict[str, DirectionSpec] = {}
    for direction, body in _mapping(root["directions"], "directions").items():
        where = f"directions.{direction}"
        body = _mapping(body, where)
        directions[str(direction)] = DirectionSpec(
            direction=str(direction),
            wait_node=str(_need(body, "wait_node", where)),
            align_node=str(_need(body, "align_node", where)),
            exit_node=str(_need(body, "exit_node", where)),
            release_node=str(_need(body, "release_node", where)),
            entry_rect=str(_need(body, "entry_rect", where)),
            approach_axis=str(body.get("approach_axis", "x")),
            approach_sign=float(body.get("approach_sign", 1.0)),
        )

    traffic = _mapping(root["traffic"], "traffic")
    stop_raw = _mapping(traffic.get("stop_model") or {}, "traffic.stop_model")

    # The fastest speed the stop distances were measured at. One consumer, two facts: it is
    # the speed at which a clearance is monotone, and it is a term in the release node's
    # clearance budget. Deriving it from the table means the two cannot drift apart, and
    # `tests/test_p10_one_boundary.py` fails if the table's largest key moves.
    measured = _mapping(stop_raw.get("reported_stop_distance_m") or {},
                        "traffic.stop_model.reported_stop_distance_m")
    measured_speeds: list[float] = []
    for key in measured:
        try:
            measured_speeds.append(float(key))
        except (TypeError, ValueError) as exc:
            raise ConfigError(
                "traffic.stop_model.reported_stop_distance_m: "
                f"{key!r} is not a speed") from exc

    stop = StopModel(
        max_measured_speed_mps=max(measured_speeds) if measured_speeds else 0.0,
        latency_s=_non_negative(_need(stop_raw, "latency_s", "traffic.stop_model"), "traffic.stop_model.latency_s"),
        a_stop_mps2=_positive(_need(stop_raw, "a_stop_mps2", "traffic.stop_model"), "traffic.stop_model.a_stop_mps2"),
        localization_margin_m=_non_negative(
            _need(stop_raw, "localization_margin_m", "traffic.stop_model"),
            "traffic.stop_model.localization_margin_m",
        ),
        turning_sweep_m=_non_negative(
            stop_raw.get("turning_sweep_m", 0.0), "traffic.stop_model.turning_sweep_m"
        ),
    )

    ttl = _positive(_need(traffic, "permit_ttl_s", "traffic"), "traffic.permit_ttl_s")
    renew_before = _non_negative(traffic.get("renew_before_s", 0.0), "traffic.renew_before_s")
    if renew_before >= ttl:
        raise ConfigError(
            f"traffic.renew_before_s ({renew_before}) must be smaller than "
            f"traffic.permit_ttl_s ({ttl}), otherwise a robot is always already late"
        )

    cfg = TrafficConfig(
        resources=resources,
        bundles=bundles,
        nodes=nodes,
        directions=directions,
        footprint_margin_m=_non_negative(
            _need(traffic, "footprint_margin_m", "traffic"), "traffic.footprint_margin_m"
        ),
        node_reach_tolerance_m=_positive(
            _need(traffic, "node_reach_tolerance_m", "traffic"), "traffic.node_reach_tolerance_m"
        ),
        permit_ttl_s=ttl,
        renew_before_s=renew_before,
        stop=stop,
        raw=root,
    )

    # A bundle must be all-or-nothing in shape too: if the corridor and its exit
    # buffer are listed, every direction that uses the corridor must name an exit
    # buffer, or the "atomic pair" is a fiction.
    for direction, members in cfg.bundles.items():
        kinds = {cfg.resources[m].kind for m in members}
        if ResourceKind.CORRIDOR in kinds and ResourceKind.EXIT_BUFFER not in kinds:
            raise ConfigError(
                f"bundles.{direction}: a corridor bundle must include an exit buffer "
                "(MASTER_PLAN section 7.2), got " + ", ".join(sorted(k.value for k in kinds))
            )

    return cfg


def load_traffic_config(path: str | "Path") -> TrafficConfig:
    """Convenience loader. YAML is read with safe_load only -- never eval."""
    import yaml  # local import: fleet_core stays importable without PyYAML

    from pathlib import Path as _Path

    with _Path(path).open("r", encoding="utf-8") as fh:
        raw = yaml.safe_load(fh)
    return validate_traffic_config(raw)
