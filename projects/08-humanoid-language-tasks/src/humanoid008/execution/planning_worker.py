"""Off-loop planning (section 13).

The control loop must never block on HTTP. A planner call therefore runs on a
worker thread and the caller *polls*, staying free to service STOP.

Properties enforced here:

- at most one planning request is in flight (bounded queue, size 1);
- a stop abandons the in-flight call, and its late result is discarded **by
  generation** rather than applied to a task that has already stopped;
- the measured stop latency is returned so it gets recorded, instead of being
  quietly assumed to be small.
"""

from __future__ import annotations

import queue
import threading
from dataclasses import dataclass
from typing import Any, Callable

from .cancellation import ControlToken


@dataclass(frozen=True)
class PlanningOutcome:
    proposal: Any | None = None
    error: BaseException | None = None
    stopped: bool = False
    stop_latency_s: float | None = None


class PlanningWorker:
    def __init__(self, planner, *, poll_interval_s: float = 0.002) -> None:
        self._planner = planner
        self._poll = poll_interval_s
        self._results: "queue.Queue[tuple[int, Any, BaseException | None]]" = queue.Queue(
            maxsize=1
        )
        self._generation = 0
        self._busy = False
        self._lock = threading.Lock()

    @property
    def busy(self) -> bool:
        with self._lock:
            return self._busy

    def submit(self, **kwargs: Any) -> None:
        with self._lock:
            if self._busy:
                raise RuntimeError("a planning request is already in flight")
            self._busy = True
            self._generation += 1
            generation = self._generation

        def work() -> None:
            proposal = None
            error: BaseException | None = None
            try:
                proposal = self._planner.propose(**kwargs)
            except BaseException as exc:  # surfaced to the caller, never swallowed
                error = exc
            try:
                self._results.put_nowait((generation, proposal, error))
            except queue.Full:
                # the caller already gave up on this request
                pass

        threading.Thread(target=work, name="h008-planning", daemon=True).start()

    def wait(
        self, *, control: ControlToken, now_s: Callable[[], float]
    ) -> PlanningOutcome:
        """Block until the call returns, or a stop wins. Exactly one outcome."""
        while True:
            if control.stop_requested:
                with self._lock:
                    # invalidate whatever is in flight so a late result cannot be used
                    self._generation += 1
                    self._busy = False
                control.mark_handled(now_s())
                return PlanningOutcome(stopped=True, stop_latency_s=control.latency_s)

            try:
                generation, proposal, error = self._results.get(timeout=self._poll)
            except queue.Empty:
                continue

            with self._lock:
                matched = generation == self._generation
                if matched:
                    self._busy = False
            if not matched:
                continue  # a late result from an abandoned request: never apply it
            return PlanningOutcome(proposal=proposal, error=error)
