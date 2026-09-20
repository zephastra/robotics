"""Geometry for the protected-crossing decision.

One question, asked at gate frequency: *is this robot's footprint, plus margin,
about to be inside a region it has no permit for?* It has to be answered the same
way every time, so the shapes are deliberately primitive:

  * ``Rect`` -- axis-aligned, the only primitive. Resource polygons in
    ``config/resources.yaml`` are declared as rectangles.
  * a rotated robot footprint. It is **never** decomposed into a point. Testing
    the centre point alone would refuse too little -- CONTRACTS §7: "the resource
    polygon is not drawn only on the robot's centre line". Testing the axis
    aligned bounding box alone would refuse too much: at 45 deg a 0.60 x 0.45
    robot presents a 0.74 m box, and the gate would block legal motion.

    So: AABB is used as a cheap *reject* filter only. A hit is re-tested with a
    separating-axis test, which is exact for convex boxes. A miss on the AABB is
    a real miss (a rotated box always fits inside its AABB).

Everything here is pure Python and pure geometry: no ROS, no clock, no state.
That makes it unit-testable without a simulator, which is the point -- the gate
has no room for untested arithmetic.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Iterable, Sequence

Point = tuple[float, float]


# --------------------------------------------------------------------------- #
# rectangles
# --------------------------------------------------------------------------- #


@dataclass(frozen=True)
class Rect:
    """An axis-aligned rectangle in map coordinates. Inclusive edges."""

    name: str
    x_min: float
    y_min: float
    x_max: float
    y_max: float

    def __post_init__(self) -> None:
        if not (self.x_max >= self.x_min and self.y_max >= self.y_min):
            raise ValueError(f"rect {self.name!r}: min must not exceed max")

    @property
    def width(self) -> float:
        return self.x_max - self.x_min

    @property
    def height(self) -> float:
        return self.y_max - self.y_min

    @property
    def centre(self) -> Point:
        return ((self.x_min + self.x_max) / 2.0, (self.y_min + self.y_max) / 2.0)

    def contains_point(self, p: Point) -> bool:
        return self.x_min <= p[0] <= self.x_max and self.y_min <= p[1] <= self.y_max

    def expanded(self, m: float) -> "Rect":
        """Grow on all sides by ``m``. Used for the localization margin."""
        if m < 0:
            raise ValueError("expansion must not be negative")
        return Rect(self.name, self.x_min - m, self.y_min - m, self.x_max + m, self.y_max + m)

    def overlaps_aabb(self, lo: Point, hi: Point) -> bool:
        """Cheap reject filter against another axis-aligned box."""
        return not (hi[0] < self.x_min or lo[0] > self.x_max or hi[1] < self.y_min or lo[1] > self.y_max)

    def as_dict(self) -> dict[str, float | str]:
        return {
            "name": self.name,
            "x_min": self.x_min,
            "y_min": self.y_min,
            "x_max": self.x_max,
            "y_max": self.y_max,
        }

    @staticmethod
    def from_mapping(raw: dict) -> "Rect":
        try:
            return Rect(
                name=str(raw["name"]),
                x_min=float(raw["x_min"]),
                y_min=float(raw["y_min"]),
                x_max=float(raw["x_max"]),
                y_max=float(raw["y_max"]),
            )
        except KeyError as exc:
            raise ValueError(f"rect {raw!r}: missing field {exc}") from exc


@dataclass
class Region:
    """A named union of rectangles -- the area a permit authorises.

    A union of axis-aligned rectangles rather than one polygon on purpose: the
    containment predicate stays closed-form and a unit test can falsify a single
    rectangle without reasoning about winding order.
    """

    name: str
    rects: tuple[Rect, ...] = ()
    margin_m: float = 0.0

    def __post_init__(self) -> None:
        if not self.rects:
            raise ValueError(f"region {self.name!r}: needs at least one rectangle")

    def contains_point(self, p: Point) -> bool:
        return any(r.contains_point(p) for r in self.rects)

    def aabb(self) -> Rect:
        return Rect(
            f"{self.name}:aabb",
            min(r.x_min for r in self.rects),
            min(r.y_min for r in self.rects),
            max(r.x_max for r in self.rects),
            max(r.y_max for r in self.rects),
        )

    def rect_at(self, p: Point) -> Rect | None:
        for r in self.rects:
            if r.contains_point(p):
                return r
        return None

    def box_overlap(self, corners: Sequence[Point]) -> tuple[Rect, ...]:
        """Which rectangles this oriented box touches, margin included."""
        lo, hi = aabb_of(corners)
        hits: list[Rect] = []
        for r in self.rects:
            grown = r.expanded(self.margin_m) if self.margin_m else r
            if not grown.overlaps_aabb(lo, hi):
                continue
            if _convex_overlap(corners, grown):
                hits.append(r)
        return tuple(hits)

    def fully_outside(self, corners: Sequence[Point]) -> bool:
        """True only when the *whole* footprint (plus margin) clears the region.

        This is the predicate behind ConfirmClear: being parked next to the
        corridor is not evidence of having left it.
        """
        return not self.box_overlap(corners)

    def as_dict(self) -> dict:
        return {
            "name": self.name,
            "margin_m": self.margin_m,
            "rects": [r.as_dict() for r in self.rects],
        }


# --------------------------------------------------------------------------- #
# oriented footprint
# --------------------------------------------------------------------------- #


def footprint_corners(
    x: float, y: float, yaw: float, length_m: float, width_m: float
) -> tuple[Point, Point, Point, Point]:
    """The four corners of the robot body, in map coordinates.

    ``length_m`` is along the robot's forward axis (its +x in body frame) and
    ``width_m`` across it, matching the footprint declared in config/fleet.yaml.
    Wheel overhang is included in those numbers, not added here.
    """
    if length_m <= 0 or width_m <= 0:
        raise ValueError("footprint must be positive")
    c, s = math.cos(yaw), math.sin(yaw)
    hl, hw = length_m / 2.0, width_m / 2.0
    local = ((hl, hw), (hl, -hw), (-hl, -hw), (-hl, hw))
    return tuple((x + lx * c - ly * s, y + lx * s + ly * c) for lx, ly in local)  # type: ignore[return-value]


def aabb_of(points: Sequence[Point]) -> tuple[Point, Point]:
    xs = [p[0] for p in points]
    ys = [p[1] for p in points]
    return (min(xs), min(ys)), (max(xs), max(ys))


def footprint_radius(length_m: float, width_m: float) -> float:
    """Circumscribed radius. The conservative shape for 'roughly here' checks."""
    return 0.5 * math.hypot(length_m, width_m)


def _project(points: Sequence[Point], axis: Point) -> tuple[float, float]:
    dots = [p[0] * axis[0] + p[1] * axis[1] for p in points]
    return min(dots), max(dots)


def _convex_overlap(box: Sequence[Point], rect: Rect) -> bool:
    """Separating-axis test between an oriented box and an axis-aligned rect."""
    rect_corners = (
        (rect.x_min, rect.y_min),
        (rect.x_max, rect.y_min),
        (rect.x_max, rect.y_max),
        (rect.x_min, rect.y_max),
    )
    axes: list[Point] = [(1.0, 0.0), (0.0, 1.0)]
    for i in range(len(box)):
        ax, ay = box[i]
        bx, by = box[(i + 1) % len(box)]
        edge = (bx - ax, by - ay)
        length = math.hypot(edge[0], edge[1])
        if length > 0:
            axes.append((-edge[1] / length, edge[0] / length))
    for axis in axes:
        lo_a, hi_a = _project(box, axis)
        lo_b, hi_b = _project(rect_corners, axis)
        if hi_a < lo_b or hi_b < lo_a:
            return False  # a gap on this axis proves no overlap
    return True


# --------------------------------------------------------------------------- #
# stopping maths
# --------------------------------------------------------------------------- #


@dataclass(frozen=True)
class StopModel:
    """Conservatively how far past a command the robot keeps moving.

    ``a_stop_mps2`` must come from the measured stop distances, and it must be
    the *conservative* end of them. P2 measured 0.1381 m at 0.35 m/s (worst of
    three passes, ground truth, through the safety gate). A straight-line start
    is assumed here rather than a curve fit, so the number is pessimistic on
    purpose -- CONTRACTS §8: "a_stop must come from the P2 stop measurement,
    conservatively".
    """

    latency_s: float          # gate-to-wheel delay, wall clock
    a_stop_mps2: float        # deceleration magnitude
    localization_margin_m: float
    turning_sweep_m: float = 0.0
    #: The fastest speed the stop distances this model was fitted to were measured at.
    #: Derived from `reported_stop_distance_m` by the validator rather than typed in a second
    #: time. It exists because `envelope_m` grows with speed, so a clearance proved at the
    #: speed a robot happens to have is falsified by accelerating -- which is exactly how the
    #: passage driver came to report `cleared` for a pose the gate refuses.
    #: 0.0 means "no measurement supplied", in which case the clearance boundary falls back to
    #: the footprint margin alone AND SAYS SO in the outcome's `detail`.
    max_measured_speed_mps: float = 0.0

    def envelope_m(self, speed_mps: float) -> float:
        """Distance from the *current* position within which motion is committed."""
        v = max(0.0, speed_mps)
        return v * self.latency_s + (v * v) / (2.0 * self.a_stop_mps2)

    def worst_case_envelope_m(self) -> float:
        """The envelope at the fastest speed this model was MEASURED at.

        Read from the measurements, not chosen: the clearance proof has to hold for every speed
        the gate may authorise, and the largest speed anyone has data for is the only bound this
        file is entitled to assume.
        """
        return self.envelope_m(self.max_measured_speed_mps)

    def reserved_radius_m(self, speed_mps: float, footprint_radius_m: float) -> float:
        return self.envelope_m(speed_mps) + footprint_radius_m + self.localization_margin_m + self.turning_sweep_m


def dominant_speed(twist: tuple[float, float, float], footprint_radius_m: float) -> float:
    """Reduce a planar twist to one conservative linear speed.

    Yaw rate turns into an equivalent tip speed, so spinning in place is not
    mistaken for standing still -- a robot rotating next to the boundary still
    sweeps space.
    """
    vx, vy, wz = twist
    return math.hypot(vx, vy) + abs(wz) * footprint_radius_m


# --------------------------------------------------------------------------- #
# 2D frame composition
# --------------------------------------------------------------------------- #


def wrap_angle(rad: float) -> float:
    """Fold an angle into (-pi, pi]."""
    while rad <= -math.pi:
        rad += 2.0 * math.pi
    while rad > math.pi:
        rad -= 2.0 * math.pi
    return rad


def compose_2d(origin, local):
    """``origin (+) local``: a pose given in a child frame, into the parent frame.

    Needed because gz's DiffDrive publishes odometry whose origin is the spawn
    pose, while the traffic rectangles are in the map frame. Per this stack's
    measurement, ``map -> odom`` is nearly constant (0.077 m of change over
    3.400 m of travel), so composing with the spawn pose reproduces the map pose
    without depending on a TF lookup that AMCL may not be refreshing while a robot
    stands still.

    This is an approximation with a measured size, not an identity. If the
    localiser is ever improved, this is the first thing to revisit.
    """
    ox, oy = origin[0], origin[1]
    oyaw = origin[2] if len(origin) > 2 else 0.0
    c, s = math.cos(oyaw), math.sin(oyaw)
    return (
        ox + local[0] * c - local[1] * s,
        oy + local[0] * s + local[1] * c,
        wrap_angle(oyaw + (local[2] if len(local) > 2 else 0.0)),
    )
