"""Which pose may a component conclude from, and when is a localised pose unusable?

THE MEASUREMENT THIS IS BUILT ON
--------------------------------
One instrumented N01, with the gate's own published pose, the raw odometry, AMCL's belief and the
simulator's truth sampled together in one process (`_p009/_round10/pose_analysis.txt`):

    the gate's pose IS spawn (+) odom      mean 0.12 mm, max 17.50 mm -- not a frame bug
    its error against truth                err = 0.0809 x distance travelled, R^2 = 0.923
                                           p95 1.725 m while moving
    AMCL's error against truth             p50 0.205 m, p95 0.348 m while moving
    AMCL's age while moving                p50 0.351 s, p95 0.853 s, max 5.296 s
    AMCL's age AT REST                     p95 242.7 s, max 422.8 s

Then, live, with both poses measured against truth in the same run
(`_p009/_round11/measured/pose_track.jsonl`, 2873 samples, N01):

    the pose the gate REPORTED (spawn (+) odom)   median 1.5516 m   p95 2.6825 m
    the pose the gate ACTED ON (localiser)        median 0.1866 m   p95 0.3305 m

So the localiser is 2-5x closer to truth than the composed pose, and it publishes at roughly 1-3 Hz
while the robot moves.

WHY THE FRESHNESS RULE HAS TWO CLAUSES
--------------------------------------
A single "refuse when older than T" rule produces a freeze in one direction and a wrong answer in
the other, and both are measured, not hypothetical:

  * **Too strict.** The localiser stops publishing while the robot is stationary -- age p95 242.7 s
    at rest. A gate that refuses any command on a stale pose therefore refuses at every stop, which
    is the latch this project has spent three rounds removing, re-created inside the fix.
  * **Too loose.** Accepting an eight-second-old pose at 0.35 m/s authorises motion using a position
    2.8 m out of date -- worse than the 1.86 m of drift being removed.

The property that actually matters is not "how old is the message" but **"does the estimate still
describe where the robot is"**. So a localised pose is usable when EITHER

    age <= max_age_s                          (the ordinary case while moving), OR
    the robot has moved <= max_moved_m since the estimate arrived
                                              (which is what makes a stale estimate still true)

and unusable otherwise, with the refusal naming both numbers. The displacement is measured from
odometry in the same frame, so it does not depend on speed and cannot be fooled by a robot that is
stationary with a stopped localiser.

WHY THE ESCAPE HATCH IS FILLED LAZILY (D-P11-02)
------------------------------------------------
The second clause needs the odometry pose from the instant the estimate arrived. Recording
`self._odom_raw` at that instant records `None` if no odometry has been seen yet -- and live it was:
AMCL publishes its initial pose the moment it activates, and the gate's first odom sample came
after it. So the displacement stayed unknown for ever and the hatch never opened. Worse, it could
not recover by itself, because `amcl_pose` is MOTION-GATED (`nav2_params.yaml` sets
`update_min_d: 0.10` since D-P17-06 -- 0.25 when this was measured -- and `update_min_a: 0.2`):
refusing motion stops the localiser, which keeps the
refusal live. Measured: `localiser_messages: 1` over a 640 s run, 0.000 m of travel.

`note_odom` therefore fills a missed arrival snapshot on the first odometry sample after an
estimate is present. The window left unmeasured is one odometry period -- 20 ms at the recorder's
50 Hz, 7 mm at the 0.35 m/s limit.

WHY THIS IS ONE CLASS AND NOT TWO IMPLEMENTATIONS
-------------------------------------------------
The safety gate and the fleet coordinator must agree about where a robot is: the gate refuses
motion on that answer and the coordinator decides `at_node`, the start-up occupancy sweep, the
un-bricking re-verify and `confirm_clear` on it. Measured on 2026-09-17, the coordinator's composed
pose was 1.5516 m (median) from truth while its `at_node` tolerance is 0.80 m, so its precondition
could not be satisfied once odometry had accumulated. Two components that must agree about one fact
may not each keep their own copy of it -- the same argument as `D-P10-01`, which found the refusal
and the clearance proof computing two different boundaries.

WHAT THIS DOES NOT ESTABLISH
----------------------------
The localiser's p95 error is 0.348 m while moving, which is **larger than the 0.25 m of margins the
traffic geometry applies** (`footprint_margin_m` 0.15 + `stop.localization_margin_m` 0.10). Switching
the pose source reduces the error by a factor of five and does not, by itself, make the corridor
boundary sound: `localization_margin_m` is documented as "covers the localizer being a few
centimetres out", and 0.10 m is not a bound on a 0.348 m error. That is a separate, derived decision
and it is registered rather than guessed at here.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from .geometry import compose_2d

#: The component composes the spawn pose with raw odometry. Its error is a RATE (8.1% of distance
#: travelled), which is why this is not the right default once a localiser is available.
POSE_SOURCE_SPAWN_ODOM = "spawn_odom"

#: The component uses the localiser's pose. It must then have a localiser: the absence of one is a
#: refusal, never a silent fall back to the composed pose (which would be the 1.86 m this exists
#: to remove, reappearing with no trace).
POSE_SOURCE_LOCALISER = "localiser"

VALID_SOURCES = (POSE_SOURCE_SPAWN_ODOM, POSE_SOURCE_LOCALISER)


@dataclass(frozen=True)
class LocaliserPolicy:
    """The bounds on "the estimate still describes where the robot is".

    `max_age_s` defaults to 2.0 s, which is 2.3x the measured p95 age while moving (0.853 s) and
    holds a fresh pose on 98.3% of moving samples. Chosen from that table, not for convenience: the
    analyser prints the fraction of samples each candidate bound accepts.
    """

    max_age_s: float = 2.0
    max_moved_m: float = 0.10

    def __post_init__(self) -> None:
        if self.max_age_s <= 0.0:
            raise ValueError("max_age_s must be positive: a zero bound refuses every command")
        if self.max_moved_m < 0.0:
            raise ValueError("max_moved_m must be non-negative")

    def describe(self) -> str:
        return (f"usable while age <= {self.max_age_s:.2f} s, or while the robot has moved "
                f"<= {self.max_moved_m:.3f} m since the estimate arrived")


@dataclass(frozen=True)
class LocaliserVerdict:
    """`usable` is never inferred from a missing reason, and the reason always carries numbers."""

    usable: bool
    reason: str
    age_s: float | None = None
    moved_m: float | None = None


def judge_localiser(
    *,
    age_s: float | None,
    moved_since_estimate_m: float | None,
    policy: LocaliserPolicy,
) -> LocaliserVerdict:
    """May the gate conclude from this localised pose? Never raises; always says why."""
    moved = None if moved_since_estimate_m is None else round(moved_since_estimate_m, 4)
    if age_s is None:
        return LocaliserVerdict(
            usable=False,
            reason="no localised pose has arrived, so there is no position to bound motion with",
        )
    if age_s <= policy.max_age_s:
        return LocaliserVerdict(
            usable=True,
            reason=f"localised pose is fresh (age {age_s:.3f} s <= {policy.max_age_s:.2f} s)",
            age_s=age_s,
            moved_m=moved,
        )
    if moved_since_estimate_m is not None and moved_since_estimate_m <= policy.max_moved_m:
        return LocaliserVerdict(
            usable=True,
            reason=(f"localised pose is {age_s:.3f} s old but the robot has moved only "
                    f"{moved_since_estimate_m:.3f} m since it arrived (<= "
                    f"{policy.max_moved_m:.3f} m), so it still describes where the robot is"),
            age_s=age_s,
            moved_m=moved,
        )
    moved_text = ("the displacement since it arrived is unknown"
                  if moved_since_estimate_m is None
                  else f"the robot has moved {moved_since_estimate_m:.3f} m since it arrived "
                       f"(> {policy.max_moved_m:.3f} m)")
    return LocaliserVerdict(
        usable=False,
        reason=(f"localised pose is {age_s:.3f} s old (> {policy.max_age_s:.2f} s) and "
                f"{moved_text}, so it no longer bounds where the robot is"),
        age_s=age_s,
        moved_m=moved,
    )


@dataclass(frozen=True)
class TrustedPose:
    """One component's answer to "where may I act as if the robot is?".

    `pose` is `None` whenever `usable` is False. That is deliberate: a caller handed a pose
    alongside `usable=False` will eventually use it, and using the composed pose after refusing on
    the localiser is exactly the 1.86 m this whole mechanism exists to remove. A caller that wants
    the composed pose for *reporting* reads `PoseSource.composed` and says so.
    """

    pose: tuple[float, float, float] | None
    usable: bool
    source: str
    reason: str


class PoseSource:
    """The pose one component may conclude from, kept in one place.

    Fed by two callbacks (`note_odom`, `note_localiser`) and asked one question (`trusted`). No ROS
    in here on purpose: both consumers are nodes, and a policy that can only be exercised through a
    running node is a policy that gets exercised by hope.
    """

    def __init__(
        self,
        *,
        source: str,
        policy: LocaliserPolicy,
        spawn: tuple[float, float, float] = (0.0, 0.0, 0.0),
        odom_timeout_s: float = 1.5,
    ) -> None:
        if source not in VALID_SOURCES:
            raise ValueError(
                f"pose source {source!r} is not one of {list(VALID_SOURCES)}. An unknown source is "
                "not a default: a component that silently composes odometry while the operator "
                "believes it is using the localiser is the failure this exists to prevent.")
        self.source = source
        self.policy = policy
        self.spawn = spawn
        self.odom_timeout_s = odom_timeout_s
        self.composed: tuple[float, float, float] | None = None
        self.localiser: tuple[float, float, float] | None = None
        self.localiser_messages = 0
        self._odom_raw: tuple[float, float, float] | None = None
        self._odom_wall: float | None = None
        self._localiser_wall: float | None = None
        self._odom_at_arrival: tuple[float, float, float] | None = None
        self.verdict: LocaliserVerdict | None = None

    # ------------------------------------------------------------------ #
    # input
    # ------------------------------------------------------------------ #
    def note_odom(
        self, *, raw: tuple[float, float, float], wall: float
    ) -> tuple[float, float, float]:
        """Record an odometry sample; returns the composed map-frame pose.

        Also fills a missed arrival snapshot (see the module docstring). Done here, on the first
        sample after an estimate, rather than in `note_localiser`, because `note_localiser` cannot
        know that odometry is a moment away.
        """
        self._odom_raw = raw
        self.composed = compose_2d(self.spawn, raw)
        self._odom_wall = wall
        if self._localiser_wall is not None and self._odom_at_arrival is None:
            self._odom_at_arrival = raw
        return self.composed

    def note_localiser(self, *, pose: tuple[float, float, float], wall: float) -> None:
        self.localiser = pose
        self._localiser_wall = wall
        self._odom_at_arrival = self._odom_raw
        self.localiser_messages += 1

    # ------------------------------------------------------------------ #
    # readings
    # ------------------------------------------------------------------ #
    def odom_age_s(self, now: float) -> float | None:
        return None if self._odom_wall is None else now - self._odom_wall

    def localiser_age_s(self, now: float) -> float | None:
        return None if self._localiser_wall is None else now - self._localiser_wall

    def moved_since_estimate(self) -> float | None:
        """Displacement in the odom frame since the last estimate arrived.

        `None` means *unknown*, which is not the same as zero and must not be reported as such.
        """
        if self._odom_at_arrival is None or self._odom_raw is None:
            return None
        return math.hypot(
            self._odom_raw[0] - self._odom_at_arrival[0],
            self._odom_raw[1] - self._odom_at_arrival[1],
        )

    def disagreement_m(self) -> float | None:
        """|localiser - composed|, for reporting. Never used to decide anything."""
        if self.localiser is None or self.composed is None:
            return None
        return round(math.hypot(self.localiser[0] - self.composed[0],
                                self.localiser[1] - self.composed[1]), 4)

    # ------------------------------------------------------------------ #
    # the question
    # ------------------------------------------------------------------ #
    def trusted(self, *, now: float) -> TrustedPose:
        """The pose this component may act on, and whether it may act at all."""
        if self.source == POSE_SOURCE_SPAWN_ODOM:
            age = self.odom_age_s(now)
            fresh = age is not None and age <= self.odom_timeout_s
            usable = bool(fresh and self.composed is not None)
            return TrustedPose(
                pose=self.composed if usable else None,
                usable=usable,
                source=POSE_SOURCE_SPAWN_ODOM,
                reason=(f"composed pose is fresh (age {age:.3f} s <= {self.odom_timeout_s:.2f} s)"
                        if usable else
                        f"no odometry within {self.odom_timeout_s:.2f} s, so there is no position "
                        "to bound motion with"),
            )

        self.verdict = judge_localiser(
            age_s=self.localiser_age_s(now),
            moved_since_estimate_m=self.moved_since_estimate(),
            policy=self.policy,
        )
        usable = bool(self.verdict.usable and self.localiser is not None)
        return TrustedPose(
            pose=self.localiser if usable else None,
            usable=usable,
            source=POSE_SOURCE_LOCALISER,
            reason=self.verdict.reason,
        )


#: How often the component may ask for a no-motion estimate. AMCL has to run a filter
#: update to answer, so asking at the gate's 20 Hz would be work for nothing; and the
#: answer arrives as an ordinary `amcl_pose`, which `note_localiser` already records.
DEFAULT_REFRESH_INTERVAL_S = 2.0


@dataclass(frozen=True)
class RefreshNeed:
    """Whether the component should ask the localiser for an estimate without motion."""

    needed: bool
    reason: str


def needs_refresh(
    *,
    age_s: float | None,
    policy: LocaliserPolicy,
    since_last_request_s: float | None,
    min_interval_s: float = DEFAULT_REFRESH_INTERVAL_S,
) -> RefreshNeed:
    """Should we ask the localiser for an estimate even though the robot has not moved?

    The loop this breaks, measured: refusing motion stops AMCL (`update_min_d` is a motion
    gate -- 0.25 m when this was measured, 0.10 m since D-P17-06), so the estimate's age grows
    without bound (`localiser_age_s` p50 316 s,
    max 813 s) while the displacement freezes (`localiser_moved_m` p50 0.2583, max 0.3397)
    and can never fall back under the 0.100 m hatch. Both clauses of the freshness rule then
    fail for ever and the refusal sustains itself.

    No threshold is needed here, and that is deliberate: asking cannot make a pose less
    accurate, so there is nothing to bound. What IS bounded is the rate, because each answer
    costs the localiser a filter update.
    """
    if age_s is None:
        return RefreshNeed(
            needed=False,
            reason="no estimate has arrived yet; there is nothing to refresh, and asking "
                   "before the first estimate would race the localiser's own start-up",
        )
    if age_s <= policy.max_age_s:
        return RefreshNeed(False, f"the estimate is fresh (age {age_s:.3f} s)")
    if since_last_request_s is not None and since_last_request_s < min_interval_s:
        return RefreshNeed(
            needed=False,
            reason=(f"asked {since_last_request_s:.3f} s ago; the localiser needs time to "
                    f"answer (interval {min_interval_s:.2f} s)"),
        )
    return RefreshNeed(
        needed=True,
        reason=(f"the estimate is {age_s:.3f} s old (> {policy.max_age_s:.2f} s) and no "
                f"no-motion update has been requested recently, so ask the localiser for one "
                f"instead of waiting for motion that the refusal is preventing"),
    )
