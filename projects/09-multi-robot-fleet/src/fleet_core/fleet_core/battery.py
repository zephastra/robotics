"""Battery model for 009 -- SIMULATED, not measured.

CONTRACTS.md section 5 prescribes a linear consumption model:

    E = idle_rate*dt + distance_rate*distance + turn_rate*abs(delta_yaw)

and says in as many words that the thresholds are demo rules, not lithium
protection. This module therefore refuses to let a caller forget that: every
result carries ``simulated=True``, and the thresholds live in config so a report
can quote the numbers that were actually in force.

Three mistakes this module is shaped to prevent:

  * **Unclamped charge.** A negative ``charge_wh`` is not "very empty", it is a
    modelling error, and letting it flow on makes the allocator's
    finish-ability test agree with a number that cannot exist. ``simulate``
    clamps and *reports* that it clamped.
  * **Charging as negative discharge.** Regen braking is not modelled. Charging
    is its own rate from config, and nothing here infers energy recovery from
    deceleration.
  * **Distance-only reachability.** A robot that arrives at the charger having
    turned 3 rad has spent energy doing it, so ``can_reach_charger`` takes a turn
    budget as well as a distance.

Nothing in this file imports ROS, so it is testable with a fake clock and no
simulator -- the same rule the rest of fleet_core follows.
"""

from __future__ import annotations

from dataclasses import dataclass

from .domain import Band, FleetConfig


@dataclass(frozen=True)
class BatteryStep:
    """One integration step. ``clamped_empty`` is the honesty flag."""

    robot_id: str
    charge_wh: float
    consumed_wh: float
    clamped_empty: bool
    simulated: bool = True


@dataclass(frozen=True)
class Reach:
    """Whether an energy budget closes, and by how much."""

    ok: bool
    charge_after_wh: float
    margin_wh: float
    needed_wh: float


class BatteryModel:
    """Linear, config-driven, explicitly simulated energy accounting."""

    def __init__(self, cfg: FleetConfig) -> None:
        self.cfg = cfg
        self.battery = cfg.battery

    # ------------------------------------------------------------------ #
    # per-robot parameters
    # ------------------------------------------------------------------ #

    def capacity_wh(self, robot_id: str) -> float:
        return self.cfg.robots[robot_id].battery_capacity_wh

    def fraction(self, robot_id: str, charge_wh: float) -> float:
        cap = self.capacity_wh(robot_id)
        return max(0.0, min(1.0, charge_wh / cap))

    def band(self, robot_id: str, charge_wh: float) -> Band:
        """Which band the charge is in. Read left to right: CRITICAL wins."""
        f = self.fraction(robot_id, charge_wh)
        b = self.battery
        if f <= b.critical_fraction:
            return Band.CRITICAL
        if f <= b.low_fraction:
            return Band.LOW
        return Band.OK

    def needs_charge(self, robot_id: str, charge_wh: float) -> bool:
        """LOW or worse: the robot should be queued for a charger."""
        return self.band(robot_id, charge_wh) is not Band.OK

    def may_resume_service(self, robot_id: str, charge_wh: float) -> bool:
        """Resume only above the recovery threshold.

        A robot that starts taking work the instant it clears the low threshold
        oscillates between charger and task; the hysteresis is the point.
        """
        return self.fraction(robot_id, charge_wh) >= self.battery.resume_fraction

    # ------------------------------------------------------------------ #
    # integration
    # ------------------------------------------------------------------ #

    def consumed_for(
        self,
        robot_id: str,
        *,
        dt_s: float = 0.0,
        distance_m: float = 0.0,
        delta_yaw_rad: float = 0.0,
    ) -> float:
        spec = self.cfg.robots[robot_id]
        return (
            spec.discharge_wh_per_s * max(0.0, dt_s)
            + spec.discharge_wh_per_m * max(0.0, distance_m)
            + spec.turn_wh_per_rad * abs(delta_yaw_rad)
        )

    def simulate(
        self,
        robot_id: str,
        *,
        charge_wh: float,
        dt_s: float,
        distance_m: float = 0.0,
        delta_yaw_rad: float = 0.0,
    ) -> BatteryStep:
        """Discharge one step. ``dt_s`` covers idle, service and waiting alike."""
        used = self.consumed_for(
            robot_id, dt_s=dt_s, distance_m=distance_m, delta_yaw_rad=delta_yaw_rad
        )
        raw = charge_wh - used
        if raw < 0.0:
            # Report the *modelled* demand, not the energy that happened to be
            # available. Reporting `charge_wh - 0.0` here would say a step that
            # wanted 5.6 Wh spent 0.5 Wh, and the shortfall -- the whole reason
            # this branch exists -- would vanish from the record.
            return BatteryStep(robot_id, 0.0, used, True)
        return BatteryStep(robot_id, min(raw, self.capacity_wh(robot_id)), used, False)

    def charge(self, robot_id: str, *, charge_wh: float, dt_s: float) -> BatteryStep:
        """Simulated charging at a pad. Not electrical contact, not measured."""
        cap = self.capacity_wh(robot_id)
        gain = self.battery.charge_wh_per_s * max(0.0, dt_s)
        raw = charge_wh + gain
        if raw > cap:
            return BatteryStep(robot_id, cap, -(cap - charge_wh), False)
        return BatteryStep(robot_id, raw, -gain, False)

    # ------------------------------------------------------------------ #
    # planning
    # ------------------------------------------------------------------ #

    def can_finish(
        self,
        robot_id: str,
        *,
        charge_wh: float,
        distance_m: float,
        delta_yaw_rad: float = 0.0,
        dt_s: float = 0.0,
    ) -> Reach:
        """Can this robot complete the trip and still hold the reserve?"""
        needed = self.consumed_for(
            robot_id, dt_s=dt_s, distance_m=distance_m, delta_yaw_rad=delta_yaw_rad
        )
        after = charge_wh - needed
        return Reach(
            ok=after >= self.cfg.battery_reserve_wh,
            charge_after_wh=after,
            margin_wh=after - self.cfg.battery_reserve_wh,
            needed_wh=needed,
        )

    def can_reach_charger(
        self,
        robot_id: str,
        *,
        charge_wh: float,
        distance_m: float,
        turnaround_rad: float = 0.0,
    ) -> Reach:
        """Reachability for a low-battery return.

        Uses the *reserve* as the floor, not zero: arriving with nothing left is
        not arrival, it is a robot that stopped on the way. ``turnaround_rad``
        budgets the heading change at both ends of the trip.
        """
        needed = self.consumed_for(
            robot_id, distance_m=distance_m, delta_yaw_rad=turnaround_rad
        )
        after = charge_wh - needed
        return Reach(
            ok=after >= self.cfg.battery_reserve_wh,
            charge_after_wh=after,
            margin_wh=after - self.cfg.battery_reserve_wh,
            needed_wh=needed,
        )

    # ------------------------------------------------------------------ #

    def snapshot(self) -> dict:
        b = self.battery
        return {
            "simulated": b.simulated,
            "charge_wh_per_s": b.charge_wh_per_s,
            "low_fraction": b.low_fraction,
            "critical_fraction": b.critical_fraction,
            "resume_fraction": b.resume_fraction,
            "note": "simulated linear model; not measured battery data",
        }
