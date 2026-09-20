"""P6 display logic: what the read-only page is allowed to claim.

MASTER_PLAN section 9 (P6) and AI_EXECUTION_PROMPTS item 6 put three hard rules on
the display, and all three are about NOT lying:

  * it must not play a preset animation, and must not use evaluator ground truth to
    drive anything -- so every value here comes from the coordinator's and the task
    service's own reported state, never from the truth stream;
  * **it must not keep showing green after the link is gone.** This is the reason the
    module exists. A dashboard that renders its last known snapshot in the normal
    colours is worse than no dashboard: it asserts "all fine" on evidence that is
    minutes old, and the operator has no way to tell;
  * an unknown resource is UNKNOWN, not free. Same rule the gate enforces, applied to
    the picture.

The logic lives here, with no ROS import, so it can be tested without a simulator --
the same reason the rest of the fleet's rules live in fleet_core. `dashboard.py` only
renders what this module decides.
"""

from __future__ import annotations

from dataclasses import dataclass, field

#: Below this, a reading is current.
FRESH_S = 3.0
#: Above this, the page stops claiming to be live at all.
STALE_S = 12.0

#: Colour tokens the page understands. Kept as words rather than CSS so the tests can
#: assert on meaning instead of on a hex string.
OK = "ok"
WARN = "warn"
BAD = "bad"
UNKNOWN = "unknown"


@dataclass(frozen=True)
class Liveness:
    state: str
    age_s: float | None
    banner: str
    may_show_ok: bool

    @property
    def live(self) -> bool:
        return self.state == "LIVE"


def liveness(age_s: float | None, *, fresh_s: float = FRESH_S, stale_s: float = STALE_S) -> Liveness:
    """How much this reading may be trusted.

    ``None`` means no reading at all -- not "age zero". Collapsing the two would make a
    node that never answered look perfectly current, which is exactly the failure mode
    the P6 instructions single out.
    """
    if age_s is None:
        return Liveness(
            state="DISCONNECTED",
            age_s=None,
            banner="DISCONNECTED — no reading has ever arrived. Nothing on this page "
                   "is evidence of anything.",
            may_show_ok=False,
        )
    if age_s <= fresh_s:
        return Liveness("LIVE", age_s, f"live, {age_s:.1f}s old", True)
    if age_s <= stale_s:
        return Liveness(
            state="AGING",
            age_s=age_s,
            banner=f"AGING — last reading is {age_s:.1f}s old; treat every value below "
                   "as approximate.",
            may_show_ok=False,
        )
    return Liveness(
        state="STALE",
        age_s=age_s,
        banner=f"NOT LIVE — last reading is {age_s:.1f}s old. The fleet may have "
               "stopped, crashed or been unreachable; this page cannot tell you which, "
               "and must not be read as 'all fine'.",
        may_show_ok=False,
    )


@dataclass
class RobotRow:
    robot_id: str
    display_state: str
    battery_fraction: float | None
    battery_band: str
    task_id: str
    position: tuple[float, float] | None
    notes: list[str] = field(default_factory=list)


@dataclass
class ResourceRow:
    name: str
    state: str
    display_state: str
    owner: str
    generation: int
    queue: list[str] = field(default_factory=list)
    notes: list[str] = field(default_factory=list)


def _robot_display(robot: dict, live: Liveness, robot_id: str | None = None) -> RobotRow:
    """One robot row. `robot_id` is authoritative when given.

    The task service publishes `"robots": {rid: {...}}` -- the id is the dict key and there
    is no `robot_id` field inside the entry -- while this module's own test fixture put the
    field in as well. Reading only the field therefore produced a table of rows all labelled
    "?", on a page whose purpose is to say which robot is doing what. The key is passed in
    from `summarise`; the field is kept as a fallback so a caller that has only the field
    still works.
    """
    notes: list[str] = []
    state = UNKNOWN
    if not live.may_show_ok:
        notes.append(f"reading {live.state.lower()}")
    if robot.get("fresh") is False:
        # The task service has its own freshness test on incoming state, and it is
        # narrower than the page's. When it says stale, the page says UNKNOWN even if
        # the page's own age would allow OK.
        notes.append("state not fresh at the task service")
        state = UNKNOWN
    elif robot.get("fault_code"):
        notes.append(f"fault {robot['fault_code']}")
        state = BAD
    elif robot.get("parked"):
        notes.append(f"PARKED: {robot['parked']}")
        state = BAD
    elif not robot.get("localization_valid", True):
        notes.append("localization invalid")
        state = BAD
    elif live.may_show_ok:
        band = robot.get("band", "OK")
        state = WARN if band in ("LOW", "CRITICAL") else OK
    if robot.get("gate_reason"):
        notes.append(f"gate: {robot['gate_reason']}")
    if robot.get("stopped_confirmed"):
        notes.append("stopped")
    # A reading the task service has already called not-fresh is not trustworthy even
    # when this page's own polling is current, so its numbers are withheld for the same
    # reason they are withheld when the page itself is stale: a stale battery figure or
    # pose gets quoted as if it were now. The test that found this asserted only the
    # pose; the battery is withheld too because the two are the same claim.
    trustworthy = live.may_show_ok and bool(robot.get("fresh", True))
    pos = None
    if trustworthy and robot.get("localization_valid", True):
        pos = (robot.get("x"), robot.get("y"))
    return RobotRow(
        robot_id=robot_id or robot.get("robot_id") or "?",
        display_state=state,
        battery_fraction=robot.get("battery_fraction") if trustworthy else None,
        battery_band=robot.get("band", "UNKNOWN") if trustworthy else "UNKNOWN",
        task_id=robot.get("task_id", ""),
        position=pos,
        notes=notes,
    )


def _resource_display(name: str, entry: dict, live: Liveness) -> ResourceRow:
    """One corridor resource. `live` is the CORRIDOR's own liveness, not the page's.

    The two sources age independently: the task service owns tasks and robots, the
    coordinator owns the corridor. Passing the page's liveness here would mean a live
    task service could make a stale corridor look current.
    """
    state = str(entry.get("state", "UNKNOWN"))
    notes: list[str] = []
    # UNKNOWN/BLOCKED must never render as available. The gate refuses entry on
    # exactly these two values; a page that painted them green would be advising a
    # robot into a region the gate is holding shut.
    display = {OK: OK, "FREE": OK}.get(state.upper(), UNKNOWN)
    if state.upper() in ("OCCUPIED", "RESERVED", "CLEARING"):
        display = WARN
    if state.upper() in ("UNKNOWN", "BLOCKED"):
        display = BAD
    if not live.may_show_ok:
        # Same rule as the robots: a reading the page cannot vouch for is not quoted as
        # current. Owner, generation and queue are the crossing decision, so they go --
        # the resource is still named, because which regions exist is not the stale part.
        notes.append(f"corridor reading {live.state.lower()}")
        display = UNKNOWN
        return ResourceRow(name=name, state="UNKNOWN", display_state=display,
                           owner="", generation=0, queue=[], notes=notes)
    return ResourceRow(
        name=name,
        state=state,
        display_state=display,
        owner=str(entry.get("owner", "") or ""),
        generation=int(entry.get("generation", 0) or 0),
        queue=list(entry.get("queue", []) or []),
        notes=notes,
    )


def summarise(
    snapshot: dict | None,
    traffic: dict | None,
    age_s: float | None,
    *,
    traffic_age_s: float | None = None,
    fresh_s: float = FRESH_S,
    stale_s: float = STALE_S,
) -> dict:
    """The whole page payload, decided here so the page can stay dumb.

    Anything the page could get wrong by being clever -- whether to show green, whether
    a number is current, whether a resource is available -- is decided once, here, and
    tested.
    """
    live = liveness(age_s, fresh_s=fresh_s, stale_s=stale_s)
    out: dict = {
        "liveness": {
            "state": live.state,
            "age_s": live.age_s,
            "banner": live.banner,
            "may_show_ok": live.may_show_ok,
        },
        "read_only": (
            "This page has no control endpoints. Submission, cancellation and fault "
            "injection are done with fleet_cli."
        ),
        "robots": [],
        "tasks": [],
        "payloads": [],
        "resources": [],
        "chargers": {},
        "backend": "UNKNOWN",
        "payload_mode": "UNKNOWN",
        "refusals": {},
        "reconcile": {},
        "traffic_age_s": None,
    }
    if snapshot is None:
        return out

    if live.may_show_ok:
        out["backend"] = snapshot.get("backend", "UNKNOWN")
        out["payload_mode"] = snapshot.get("payload_mode", "UNKNOWN")
        out["tasks"] = list(snapshot.get("tasks", []))
        out["payloads"] = list(snapshot.get("payloads", []))
        out["chargers"] = dict(snapshot.get("chargers", {}))
        out["refusals"] = dict(snapshot.get("refusals", {}))
        out["reconcile"] = dict(snapshot.get("reconcile", {}))
    # `.items()`, not `.values()`: the robot id IS the dict key, and sorting on a field
    # that does not exist made the row order arbitrary.
    out["robots"] = [
        _robot_display(row, live, robot_id=rid).__dict__
        for rid, row in sorted(snapshot.get("robots", {}).items())
    ]

    # The corridor is a SECOND, independent source, so it is judged on its OWN age rather
    # than borrowing the task service's. It used to be labelled from `traffic.get("age_s")`
    # -- a key the coordinator's status_report() does not publish -- so the label was
    # permanently None while the rows below were still shown. Freezing the coordinator
    # would then have left a stale owner/generation/queue on a page still saying LIVE.
    corridor = liveness(traffic_age_s, fresh_s=fresh_s, stale_s=stale_s)
    out["traffic_liveness"] = {
        "state": corridor.state,
        "age_s": corridor.age_s,
        "may_show_ok": corridor.may_show_ok,
    }
    out["traffic_age_s"] = corridor.age_s
    if traffic:
        resources = traffic.get("resources") or {}
        out["resources"] = [
            _resource_display(name, entry, corridor).__dict__
            for name, entry in sorted(resources.items())
        ]
    return out
