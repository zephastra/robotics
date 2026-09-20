"""When to ask whether a robot's Nav2 executor is up, and what an answer is worth.

WHY THIS IS ITS OWN OBJECT
--------------------------
`navigate_to_pose`'s action server **exists** while its node is still INACTIVE. So
`wait_for_server()` returns true, `send_goal_async()` resolves with `accepted=False`, and
`bt_navigator` logs "Action server is inactive. Rejecting the goal." From the client's side
"not up yet" and "busy" are the *same event* -- and the adapter's retry loop spent its whole
budget on the first of them. Measured: the task service creates a charge run on its first
tick, 13-16 s before Nav2 reaches ACTIVE, and `auto-charge-<robot>-<epoch>-1` ended `FAILED`
with `attempts=2` in two otherwise healthy runs (D-P5-23).

Two rules, and the second one is the important one:

  * **ask the lifecycle, do not infer it from the action client.** The client cannot tell
    the two cases apart; the lifecycle service can.
  * **an answer we do not have is not a reason to block.** `active()` returns ``None`` when
    nothing has been asked yet, when a request is in flight, or when the service is absent
    -- and every caller treats ``None`` as "carry on exactly as before this existed". A
    readiness gate that blocks when it cannot see would be a new failure mode rather than a
    fix, and this is the whole reason the class exists instead of a bare boolean.

No ROS import here. The caller performs the service call and hands the state id in; the
schedule and the interpretation live here, where they can be tested with a fake clock and no
simulator -- the same split `fleet_core` uses for everything else.
"""

from __future__ import annotations

import time
from typing import Callable

#: `lifecycle_msgs/msg/State`.ACTIVE. Named rather than written as a literal so a reader can
#: look it up; pinned by a test, because a wrong id here would make every robot look
#: permanently unready and the gate would block instead of wait.
ACTIVE = 3

#: How long an answer is trusted. Long enough that a 10 Hz state publisher does not turn
#: into a 10 Hz service call, short enough that a lifecycle transition is noticed inside the
#: leg's own patience.
DEFAULT_TTL_S = 0.5


def should_wait_for_recovery(active: "bool | None") -> bool:
    """Is a refusal explained by the executor not being up yet?

    Only an explicit `False` may hold a leg back. `None` means the fleet cannot tell, and
    "cannot tell" has to behave exactly as the code did before this check existed --
    otherwise an unreachable lifecycle service would become a fleet-wide stop.
    """
    return active is False


class Nav2Readiness:
    """Cache, ask, and remember. Never blocks; never calls anything itself."""

    def __init__(
        self,
        *,
        clock: Callable[[], float] = time.monotonic,
        ttl_s: float = DEFAULT_TTL_S,
    ) -> None:
        self._now_fn = clock
        self._ttl_s = float(ttl_s)
        self._at: "float | None" = None
        self._active: "bool | None" = None
        self.asks = 0

    # ------------------------------------------------------------------ #

    def due(self) -> bool:
        """Is a fresh answer worth asking for?"""
        return self._at is None or (self._now_fn() - self._at) >= self._ttl_s

    def record(self, state_id: "int | None") -> None:
        """Record an ANSWER.

        ``None`` means the ask failed or raised. That is recorded as "no longer known"
        rather than as "not active": a failed question is not evidence of a stopped
        executor, and treating it as one would refuse legs for a reason nobody observed.
        """
        self._at = self._now_fn()
        self._active = None if state_id is None else (int(state_id) == ACTIVE)

    def asked_without_an_answer(self) -> None:
        """An ask was issued but cannot be harvested yet, or there is nobody to ask.

        Only the clock moves. The last known answer is kept, because a request in flight
        says nothing about the state and must not erase what was already observed.
        """
        self._at = self._now_fn()

    def active(self) -> "bool | None":
        return self._active

    def describe(self) -> str:
        """One line for a log or a status page: what is known and when it was learned."""
        if self._active is None:
            return f"nav2 readiness unknown (asks={self.asks})"
        age = "n/a" if self._at is None else f"{self._now_fn() - self._at:.2f}s ago"
        return f"nav2 active={self._active} (observed {age}, asks={self.asks})"
