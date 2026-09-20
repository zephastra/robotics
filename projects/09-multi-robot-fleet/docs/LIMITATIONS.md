# 009 — Limitations and NOT_RUN register

This file is the honest boundary of the project. Read it before quoting any number from a
report, and before planning work that assumes something here is finished.

The rule this project follows: **anything not executed is `NOT_RUN`, and `NOT_RUN` is not a
pass.** A claim that has not been run is never softened into "should work" or "effectively
verified". If a stage is partial, it says which half is missing.

Last updated: 2026-09-16 (P5 scenarios, evaluation layer, P6 live run).

---

## 0. How to read the status of anything here

Three outcomes are kept separate for every check, and they are never averaged:

| | meaning |
|---|---|
| **task** | did the work the task asked for happen |
| **safety** | did anything unsafe happen, regardless of whether the task succeeded |
| **expected_behavior** | did the system do what the contract says it should |

A run can pass `task` and fail `safety`. A run can pass `safety` and be worthless because it
never exercised anything. The reports say which is which.

**`backend`** is labelled on every result: `fake` or `gazebo_nav2`. A `fake` backend can
never produce a safety PASS — it is `NOT_RUN`, because a software stub cannot fail.
**Ground truth** (Gazebo's `dynamic_pose` stream) is never fed to control. It is only used
to check the estimates afterwards.

---

## 1. Verified with evidence

| what | evidence |
|---|---|
| Corridor crossing, one robot, both directions | 3/3 SUCCEEDED; minimum \|y\| inside the gap 0.015 / 0.075 / 0.084 m against a 0.6 m half-width |
| Two robots, opposing traffic, both request orders | `case1_rc=0 case2_rc=0`; `both_completed=True` in both; `simultaneous=0`, `unauthorised=0` over 637 + 648 samples |
| Permission path end to end | coordinator `grants_seen` 260–297, gate `guard_installed: True`, `mismatch: False` |
| Passage order is arbitrated, not timed | r01 queued 103 times in case 1, r02 queued 115 times in case 2 — swap the request order and the winner swaps |
| Task chain, three robots | `p5-A` submitted → assigned → four legs → `picked cargo_A` → `delivered cargo_A` → `SUCCEEDED`; `p5-B` SUCCEEDED; `p5-D` CANCELED after a confirmed stop |
| Refusals | unknown station, cross-corridor route, `payload_mode=physical`, duplicate `request_id` — all refused with the contract's own codes |
| Static guards | 11 guards run by `scripts/check_guards.sh` before the build (and the runner itself is self-tested: an injected `exit 7` must fail it); 864 core tests; 57 traffic-geometry checks; asset hashes recorded |
| Independent evaluation layer | `fleet_evaluation` exists: a pure-Python judge (it does not import ROS, and a test asserts that) plus a subscribe-only recorder. Ground truth is the judge; the ledger, the gate log and the driver report are only *claims* (CORROBORATED / CONTRADICTED / UNCHECKED) |
| P6 read-only dashboard, live fleet | 20/20 against a running three-robot fleet, and 23/23 for the static HTTP surface. Three real display defects were found by running it, none of which the static checks could see (section 4) |
| Three real interruptions, measured | `SIGSTOP` on the task service, `SIGCONT`, and `SIGKILL` of one robot's adapter: `LIVE -> AGING (3.3-11.7 s) -> STALE (13.8 s)`, recovery `-> LIVE`. A dead robot is named, its numbers withheld and the reason printed; the other two keep their values. Real signals, not an injection hook |
| Battery state injection | `FaultInject` can set a robot's state of charge, gated by its own boolean field. Verified against a live fleet; see D-P5-18 for why it injects a state rather than a decision |

## 2. NOT_RUN — never executed, so no conclusion is available

- **No long-run or repeated-run stress.** "No deadlock" and "no starvation" are therefore
  **not claimed**. The accurate statement is: *not observed in the two opposing-traffic
  cases and the one three-robot task run.*
- **No deliberate attempt to bypass the gate.** There is a positive control (a robot holding
  a valid permit crossed cleanly, with zero unauthorised entries and zero gate
  `RESOURCE_UNKNOWN`), but no adversarial case. "Nav2 cannot bypass the gate" is **not
  claimed.**
- **A permit expiring while a robot is INSIDE the corridor is not implemented.**
  `D-P4-12`: the robot would be stopped with no legal exit, so it needs a *retreat* permit.
  That is a safety-layer change and was deliberately left out rather than half-done. The
  resource is blocked and the task is parked, which is the safe half.
- **Exit-buffer occupancy as a blocking condition is covered by unit tests only.** No live
  run has had a second robot arrive at an occupied exit buffer.
- **`config/scenarios/` now exists AND is consumed** — but only since 2026-09-16. Before
  that the directory was deliberately absent, because a scenario file the launch ignored
  would have been a declared-but-unused artefact. The launch now reads `scenario:=<name>`,
  applies the `budgets:` block, prints every effective value, and refuses an unknown key.
  `scripts/check_scenario_budgets.py` checks both directions in `build.sh`: a key a scenario
  names that the launch does not forward, and a key the launch forwards that its node never
  declares. Seven controls (`_p009/p5b/controls.sh`) prove the guard can fail.
- **`fleet_evaluation` exists, and the first thing it did was contradict a published
  claim.** `judge.py` is pure Python and `recorder.py` only subscribes. Run against the
  P4 acceptance report it found that the ground-truth channel had recorded **one** robot:
  slot 0 was `r02`'s spawn with 10.401 m of travel, slot 1 sat still at (0.200, 0.000) with
  **0.000 m** of travel, and **`r01` was absent entirely**. The earlier `usable_for_cross_check:
  true` had been decided on the *width* of the transform, and a width check is not an identity
  check. The P4 PASS still stands — it was computed from map-frame odometry and the ledger —
  but "cross-checked against ground truth" was never true, and is no longer claimed.
- **The world's truth stream carries one usable robot pose per recording, even with two
  robots running (measured again this round).** The N03 batch case ran r01 and r02; the
  recording's transform width was 16 (two models x 8 links), slot 0 tracked r01, and slot 1 sat
  at (0.200, 0.000) with **0.000 m** of travel for the whole run -- the same non-robot slot the
  P4 analysis found. r02's pose is therefore absent from the channel, the judge refuses to
  attribute slots, and every safety property for that case is `NOT_RUN`. Nothing about the
  robots' behaviour is contradicted by this; what it means is that a two-robot safety verdict
  cannot be produced until the slot-to-robot mapping is understood. N03 is reported as
  `safety: UNKNOWN` and the batch exits 5 because of it.
- **A corridor crossing completed end to end, from a task.** `p7-N01-s1-1` (pick S_left_a,
  drop S_right_a) crossed west_to_east under the dispatcher's control: one `crossing_started`,
  `attempts: 0`, `SUCCEEDED`. The passage driver's own report: wait_west 10.4 s, align_west
  10.1 s, **acquire granted on the FIRST request for 12.00 s**, 8 renewals in flight, exit_east
  23.7 s, release_east 12.6 s, `confirm_clear` released ['mid','mid_right'], `complete: true`.
  This is ONE crossing: no contention, no queue, no cancel and no retry were in the path that
  ran, so it establishes that the wiring works and nothing about robustness.
- **A leg on the robot's OWN side aborted twice with `nav2 status=6`.** After that crossing,
  `to_pick-0` to S_right_b -- a 5 m drive inside the east half, no corridor involved -- was
  aborted by Nav2 on both attempts and the task was parked `NAV_FAILED`. Not a crossing problem
  and not an allocation problem: an ordinary navigation failure with a status code, and the only
  thing failing in the case that crosses. NOT diagnosed.
- **`/fleet/acquire_passage` ANSWERS, and its refusal is a designed precondition.**
  `scripts/probe_acquire.py` asked it directly, with a fleet up and the sweep complete
  (`sweep_done: true`, "start-up sweep: 3 verified clear, 0 occupied, 0 held at t=5.2s"):
  **4 of 4 requests answered**, every one `PRECONDITION_NOT_REACHED`, with the distance named --
  "not at a legal asking point: wait_west 4.880 m, align_west 4.118 m (pose x=-6.000 y=-2.000).
  Tolerance 0.80 m is set from the stack's measured arrival error". A passage may only be asked
  for from a waiting or align node, and the probe deliberately stood still at the spawn. So the
  service is not the problem, and the round-5 report's "no response" was an artefact of that run
  being killed mid-request (41 acquire attempts in 20 s is forty 0.5 s backoffs, not one 10 s
  timeout). **What the crossing run's ~40 refusals actually said is still unknown**, because the
  driver discarded them; that is fixed -- every refusal is logged and carried into the terminal
  detail -- and the next N01 run prints it. The open question is narrower and checkable: is the
  robot within the 0.80 m asking tolerance, *judged from the coordinator's odometry*, when it
  asks? Round 5 measured p95 |claim-truth| = 1.86 m on a different case, so that is a live
  possibility, not a hypothesis to be assumed.
- **Route connectivity is a preference now, not a refusal.** The refusal that made a first-leg
  crossing unreachable was removed: it sat upstream of `_issue_legs`, so a task whose first
  station is across the barrier could never be assigned (measured: 816 `ROUTE_REQUIRES_PERMIT`,
  the task stuck in `ACCEPTED`, `crossed_legs` empty). The rule survives as a preference over the
  candidate SET -- a robot that would need a passage is dropped only if another candidate would
  not -- computed by `fleet_core.split_by_passage`, with both outcomes counted in the status
  snapshot (`route_connectivity`). **Verified live on a first-leg crossing (N01, round 7):
  the dispatcher started a crossing for the plan's FIRST leg** -- `crossing_started
  to_pick-0 direction=west_to_east`, and the driver's report is `complete: true` over
  `wait_west 15.3 s -> align_west 8.0 s -> acquire granted on the 1st request -> exit_east
  18.1 s -> release_east 8.4 s -> confirm_clear released ['mid','mid_right']`, with no refusal
  printed by the driver and `ROUTE_REQUIRES_PERMIT` **absent from the refusals counter**
  (`{'NO_CAPABLE_ROBOT': 1663, 'PAYLOAD_HELD_ELSEWHERE': 1663, 'RESOURCE_BUSY': 133}`). For
  precision, the string does appear once in the log -- `no live permit for r01
  (ROUTE_REQUIRES_PERMIT); publishing granted=false`, the coordinator's permit mirror answering
  a robot that holds nothing -- which is a mirrored refusal to GRANT, not a task-allocation
  refusal; the gate reads it and correctly declines it. What that run did NOT exercise is
  the preference itself: `split_by_passage` is only consulted when the candidate set has more
  than one member, and N01 has one robot, so `route_connectivity` is absent from the snapshot
  rather than non-zero. The preference is unit-tested and structurally pinned; the field
  evidence is that the refusal is gone, not that the preference chose well.
  One consequence to know: with no refusal left that is permanent,
  `_watch_unassignable`'s `PERMANENT_REFUSALS` trigger no longer occurs in practice, so that
  watchdog is now a backstop rather than a live path.
- **A plan whose FIRST station is across the barrier used to be refused outright.** See the
  route-connectivity entry above: that refusal is gone, and the first-leg case is what
  measured it. `_eligible_robots` used to drop every candidate whose first leg needed a
  passage (CONTRACTS section 6), upstream of `_issue_legs`, so the crossing code was
  unreachable for such a task -- 816 `ROUTE_REQUIRES_PERMIT`, the task stuck in `ACCEPTED`,
  `crossed_legs` empty. Kept here rather than deleted because the failure shape is worth
  recognising: a gate placed one step too early does not look like a gate, it looks like a
  task that nothing ever picks up.
- **The judge must be told which frame its samples are in, and cannot guess.** The P4 sampler
  stored odometry already composed into `map`; `fleet_recorder` stores it raw, with the origin
  at the spawn pose. Using the wrong one translates the whole fleet by one spawn pose — 6.325 m
  in the case that found it — and it then reads as a localisation failure rather than as a
  format error. `--odom-frame` overrides it, and an ambiguous sample set is `NOT_RUN`.
- **The P6 dashboard has now been driven against a live fleet (20/20), and the static checks
  it passed beforehand were green while being wrong.** `check_p6_dashboard.sh` had only ever
  run with *no fleet at all*, which is the one state in which the display does not fail: with
  a fleet up it spent 284 s in `DISCONNECTED` and never read a single sample. See section 4
  for the deadlock and for the fixture-shaped defects that hid it.
- **The cancel confirmation budget is 15 s, not the 3 s the contract names.** The adapter's
  own settle window is 10 s (`CANCEL_SETTLE_S`), so 3 s is unachievable without making
  `stopped_confirmed` easier to obtain. Recorded as a conflict in `docs/DECISIONS.md`
  (D-P5-13) and asserted by a test, rather than silently redefined. Before this round the
  cancel had **no** budget at all: `_srv_cancel` recorded the request and transmitted nothing,
  so one cancel took 306 s to confirm.
- **A task parked while its robot still holds cargo has no automatic path back.** The payload
  stays with the holder (`cargo_C HELD holder=r02` in the 08:26Z run) and needs a human. That
  is the contract's stated behaviour, not a shortcut, but it does mean a faulted robot is out
  of service for the rest of the session.
- **`CANCELING` is not modelled as a task state.** The contract names
  `CANCELING -> CANCELED`; this implementation carries the same information in a
  `cancel_requested` column, because adding a state to the frozen `TaskState` enum would
  invalidate the accepted P1 evidence. Registered as a vocabulary difference, not renamed.
- **`turn_wh_per_rad` defaults to 0.0.** No turn-energy measurement exists, so the battery
  model under-counts every turn. Any energy budget that includes turning is optimistic by an
  unmeasured amount.
- **The robot state mirror is a topic subscriber.** It is therefore never authoritative; a
  dropped message is not detected as such. Freshness is re-checked on every decision
  (`STATE_MAX_AGE_S = 3.0`), and a quiet robot is treated as a fault rather than as idle, but
  a single *wrong* sample inside the freshness window is accepted.
- **Charging is a simulated model.** No charger-pad occupancy sensor exists. `C_left` /
  `C_right` capacity is enforced by the allocator, not measured.
- **Rows written before the `kind` migration cannot say what kind they were.**
  `Ledger._migrate()` adds `kind` and `payload_mode` to an existing database with
  defaults, because refusing to open the file would be worse than an imperfect label
  (CONTRACTS section 11: clearing the database is not recovery). Any row created before
  that migration reads back as `station_transfer` — **wrong for a charge row**, and a
  reader comparing an old ledger against a new one must know it. The migration records
  what it added in `meta.migrated_columns` (see D-P5-19).
- **Charger selection ignored the robot's position — FIXED (D-P5-22), with the live run that
  closes it.** `ChargerAllocator.request` with no preference walked `sorted(chargers)`, so an
  east-side robot was routinely granted the west pad; because a charge run is *pinned*, the
  route gate then refused that robot every tick and the task never left `ACCEPTED` while the
  pad stayed granted (`grants=2, releases=0`). Pad selection is now reachability-aware
  (`legs.reachable_chargers`), an unreachable pad is not a candidate, and a pinned run whose
  own robot cannot be offered it is parked with the blocking refusal and the pad released.
  Live evidence, both scenarios re-run after the fix:
  `reports/scen3456/low_battery_20260916T132401Z.txt` (9/9: r01 charges 0.1363 → 0.8062,
  `grants=2 releases=2`, finishing 0.23 m from `C_left`) and
  `reports/scen3456/charge_queue_20260916T132532Z.txt` (11/11: r02 east → `C_right`,
  r03 west → `C_left`, r01 west waiting in the queue with
  `reachable: ['C_left']` recorded, no pad ever holding two robots).

- **A charge run could be created before the robot's Nav2 was active, and Nav2's refusals
  spent the run's attempts — FIXED (D-P5-23), design and tests only so far.** `_watch_battery`
  runs from the task service's first tick, so a low robot got a charge run while
  `bt_navigator` was still becoming ACTIVE (13–16 s); Nav2 correctly rejected the goal
  ("Action server is inactive") and a rejection consumed one of the run's two attempts, so
  the first run could die outright (`auto-charge-r01-<epoch>-1` `FAILED` with `attempts=2`,
  then `-2` succeeded). The adapter now asks `bt_navigator/get_state` and waits for ACTIVE
  before spending an attempt, and the dispatcher treats `NAV2_NOT_ACTIVE` as a bounded wait
  (`wait_budget_per_task`) rather than a failure. **Verified live in both scenarios**:
  `low_battery_20260916T135059Z.txt` now holds **one** charge task (`attempts=0`, `SUCCEEDED`,
  `grants=1 releases=1`) where it previously held a `FAILED` run and a successful one with
  `grants=2`; `charge_queue_20260916T135208Z.txt` has both charge runs at `attempts=0` and no
  `FAILED` one, where it previously had three failures before the run that worked. The
  queue/capacity assertions in that run rest on 2 samples (the instrument stops as soon as a
  robot reaches a pad, which now happens sooner) — see
  `charge_queue_20260916T122243Z.txt` for the 165-sample capacity evidence.

- **The readiness gate cannot see, and says so.** `nav2_readiness` returns `None` when the
  lifecycle service is absent or a request is still in flight, and `None` deliberately does
  NOT hold a leg back. So on a machine where `bt_navigator/get_state` is unreachable, the
  adapter behaves exactly as it did before D-P5-23 — including spending attempts on a
  bring-up race. That is the conservative direction, and it is not verified on such a
  machine: no run here has removed the lifecycle service.
- **A refusal named the wrong cause: `NO_CAPABLE_ROBOT` for a merely busy robot — FIXED
  (D-P5-20).** Observed live while the only INSPECT-capable robot was executing another task.
  `Allocator.allocate` collected `RESOURCE_BUSY` in its rejected list and then reported
  `NO_CAPABLE_ROBOT` regardless. The dispatch decision was correct — the task was not sent —
  but an operator reading it would look for a missing capability that exists. The report now
  goes through a declared precedence (`allocator.REASON_PRECEDENCE`) with
  `NO_CAPABLE_ROBOT` last, the per-robot refusals ride on the `Allocation` and are written to
  the event log once per change, and an offline robot is reported as `RESOURCE_UNKNOWN`
  instead of as incapable. Unit-tested in `tests/test_p1_allocator_reason.py`; the live
  re-observation is **NOT_RUN**, because the scenario that produced it (a capable robot busy
  while an INSPECT task waits) has not been driven again since.
- **The CRITICAL band's `NO_SAFE_CHARGER` path has no live evidence.** `_service_battery`
  proves charger reachability only when the band is CRITICAL. Every live battery scenario is
  written in `(critical_fraction, low_fraction]` deliberately, because a CRITICAL robot takes
  the reachability branch instead, which is a different claim. Unit-tested; not run.
- **The battery override mixes clocks.** The adapter integrates energy against sim time and
  expires an injected charge against wall time. At the measured RTF of about 1.000 the two
  agree, but in a paused or throttled world the `duration_s` restore lands at a different
  point in the energy history than the number suggests. Recorded rather than smoothed over.
- **`_reconcile_at_boot`'s label fix is asserted, not imported from the contract.**
  `INTERRUPTED_BY_RESTART` is an addition under CONTRACTS §12's "至少支持这些 reason_code"
  (see D-P5-16). A reader checking the code list against the contract will not find it there.
- **`VISIT_STATION` is refused by name**, not silently treated as a transfer, because the
  contract declares it and does not specify it.
- **No physical payload handling.** `payload_mode: physical` returns
  `UNSUPPORTED_PAYLOAD_MODE`. Payloads are ledger rows. Nothing is picked up.

## 3. Known-weak instrumentation

- **The acceptance sampler self-throttles.** Sampling sits at roughly 9–10 Hz against a
  ~20 Hz publication rate, so its own self-audit reports `WITHHELD`. Gate verdict counts are
  therefore **indicators, not measurements**. The run-to-run equality of the geometry checks
  is what carries the weight, not the counts.
- **`probe_odom_truth.py` needs wheel angles**, so it requires `bridge_joint_states:=true`.
  The default is off.
- **The world-level ground-truth stream loses frame names and stamps.** Robots must be
  identified by message index plus two independent geometric sources, never by name.
  Measured: 2816 of 2816 `child_frame_id` values empty across 176 messages.

## 4. Traps that have already cost real runs

Kept here because each one produced a *plausible wrong conclusion*, not an error message.

| trap | what it looked like | what it actually was |
|---|---|---|
| A guard whose answer depends on the shell | "the build is green" | the Node-member list was 109 entries with ROS sourced and 23 without; half the check was off in the shell the project uses |
| A guard that reads a field nobody writes | "the freshness condition is checked" | `position_known` was filled, `localization_valid` and `pose_age_s` were not, so two conditions could never fail |
| Timing used as a precondition | "a fault was injected during a task" | the injection was at a wall-clock offset; by then the target robot was three legs into a task, so the scenario never ran as designed |
| A failure label that is a constant | "the task was parked because cancel was not confirmed" | every park wrote `CANCEL_NOT_CONFIRMED`, for navigation timeouts and blocked regions too |
| A hardcoded `UNKNOWN` input | "the fault was in the corridor" | `in_corridor` was always `UNKNOWN`, so *every* fault was classified as a corridor loss and two rows of the recovery table were unreachable in production |
| A frame mismatch | "the robot is at (0, 0) every time" | odometry starts at the spawn pose; comparing it to map-frame rectangles freezes the whole fleet. Fixed in the gate, then the same bug reappeared in the adapter |
| A one-shot answer to a repeatable question | "a single failed crossing locks the corridor for the session" | `UNKNOWN` after expiry was a one-way valve: the boot sweep ran once and `ConfirmClear` refuses on `UNKNOWN` |
| Trusting an acceptance check's own arithmetic | "the vehicle was 0.35 m short" | the tolerance was tighter than the stack's measured arrival error of 0.5051 m |
| A silent `cp` into a directory that does not exist | "rosidl says the file is missing" | the `action/` directory had never been created, so the copy did nothing and the build reported a missing file |
| A guard that resolved executables from the wrong file | "the table and the packages have drifted apart" | `src/fleet_ros/setup.py` is a leftover ament_python scaffold naming four of that package's six executables; the real installation is the package's CMakeLists installing `scripts/*`. A fault in the guard itself, reported as a fault in the project |
| An HTTP client that latches | "the dashboard server stopped answering" | `urllib` succeeded twice and then reset on every subsequent request, while a hand-written raw socket succeeded 11 times out of 11 at the same moment. The server was never the failing half. **Why `urllib` latches is still not explained** — that is written down rather than covered with a retry loop |
| A test double that supplies fields production does not carry | "the page shows every robot" | the fixture put `robot_id` inside each robot entry, while production carries it as the dictionary key; `summarise` iterated `.values()` and lost it, so every robot rendered as `"?"` and the row order was random. A double kinder than production hides exactly the mistakes production makes |
| A scenario budget that was never forwarded | "the run used the scenario's budgets" | the launch forwarded two of the keys and the nodes used their own defaults for the rest, so the report quoted numbers that were never applied. The guard that now prevents this checks the two directions separately, because one direction alone would have passed |
| A field that exists in memory but not in the database | "the robot is charging: the pad is held, the counter is moving, the page looks right" | `tasks` had no `kind` column, so every task read back from the ledger came out as `STATION_TRANSFER`. The dispatcher decides whether the low-battery admission rule applies from that field, so it refused the low robot **its own charge task**, once per tick, while the pad stayed granted and nothing charged. `Ledger.submit` returns the in-memory task, so no unit test that inspects a submit's return value could see it |
| A readiness gate that checks half the precondition | "the fleet is up" | three fresh `RobotState` messages arrive before r03's `bt_navigator` becomes ACTIVE, and a goal sent into that window is *rejected*, not queued. The first leg burned both retries and the task failed with `NAV_FAILED` -- a fleet fault that was an instrument fault |
| A counter that stands in for a fact | "two charge pads are occupied" | `chargers.grants` counts admissions, not arrivals. In the passing run it read `grants=2, releases=0` while all three robots sat at their spawns with no task, because a colliding generated `request_id` made the charge run impossible to create after the pad had already been granted. The scenario now also requires an executed charge task and a robot inside the pad radius |
| An identity that is unique only within one process | "the charge run is created" | `request_id = auto-charge-<rid>-<serial>` with a per-process serial, against a durable ledger: the second session's first charge run reused the first session's key, `Ledger.submit` correctly returned `created=False`, and the caller returned early *after* granting the pad. The resource stayed held all session. Fixed by putting the epoch in the id and by releasing the pad when creation fails |

## 5. What a reader should not conclude from this project

- It is a **simulation study**. No claim here extends to physical hardware.
- Passing acceptance runs mean **the rules held in those runs**, nothing about runs that were
  not performed.
- The vehicle model is a first design: wheels are 0.077 m collision radius against a 0.075 m
  nominal, chosen to break ground tangency. That is a **2.7% rolling-radius fudge**, and the
  effective rolling radius is still not determined.

- **Nav2's lifecycle manager can abort a bring-up, and the case then cannot run at all.** Seen
  once, on a single-robot bring-up during the first batch: `Failed to change state for node:
  planner_server. Exception: planner_server/get_state service client: async_send_request
  failed.` followed by `Failed to bring up all requested nodes. Aborting bringup.` Nav2's own
  manager gives up rather than retrying, so no robot ever becomes ACTIVE and no leg can be
  issued. The batch recorded it as a **startup problem** (§9 exit 3 semantics) rather than as a
  case failure — the distinction matters, because a case verdict that is really an environment
  failure blames the fleet for the launcher. On the retry the same case passed, so this is
  intermittent; it has not been characterised (how often, under what load) and no run here has
  measured it. It is the same shape as D-P5-23 — "not up yet" read as a failure — one layer
  further out, on the launch side.

  `scripts/batch.py` now quotes the launch log's last error lines into the case's `summary.json`,
  so a case that could not start carries its cause instead of a bare timeout.

## The corridor exclusivity claim is not asserted (2026-09-18)

`crossing_exclusive` was written this round and is **not declared by any case**, because as
written it measures the wrong interval and a check that reports correct behaviour as a violation
is worse than no check: it invites a fix to the coordinator for nothing.

It reconstructed an occupancy from `crossing_started` to `crossing_complete`, but
`crossing_started` marks the passage driver's PROCESS LAUNCH -- so the interval includes the queue
for the permit. N04, whose whole subject is waiting, produced:

    r01 [670.9, 801.9] vs r02 [687.9, 856.9]   -- reported as an overlap

and that is what queueing looks like. The interval the claim needs (permit held, or body inside
the corridor) is not recoverable: the events stamp `time.time()`, the driver's stages stamp
`elapsed_s` off `time.monotonic()`, and the report carries no absolute start. Deriving a base was
rejected -- it puts an unknown process-spawn delay into the answer.

What IS asserted is `crossing_order`, which compares two timestamps from one file and needs no
base; N04 measured it holding. The replacement is named in `docs/DECISIONS.md` **D-P16-04**:
`passage_granted` / `passage_released` events from the coordinator, which owns the resource, or
absolute wall stamps on the driver's stages.

The queue itself is recorded: `crossing_acquire_requests` reports 1 request for r01 and 86 for
r02 on N04, which is the direct evidence that the corridor made a robot wait.

## P7 batch prerequisites (2026-09-16)

`config/scenarios/regression_v1.yaml` freezes TEST_AND_ACCEPTANCE section 5's 30 cases plus
the 8 seeded repeats, for 38 runs.

**As of round 17 (2026-09-18, stage 2) all 38 are runnable and none is blocked.**
`scripts/check_batch_manifest.py` reports `runnable today: 38`, `blocked: 0`, and the
manifest carries no `requires:` key at all. The register below is kept rather than deleted
because it is the record of WHICH capability each formerly-blocked case needed and why, and
because a reader coming from an older revision needs to see the change. The 16/22 split this
paragraph used to quote is round 16's state.

Each key below is marked with what it is now. "RETIRED" means a case no longer declares it;
"IMPLEMENTED" means the machinery exists and the case runs, which is **not** the same as the
case passing — several of these have never produced a verdict.

### Correction, 2026-09-20 — "IMPLEMENTED" was over-claiming, and the register said so twice

Two things this register asserted are not true, and a reader has to see that before the
entries below, not after.

**1. "IMPLEMENTED" meant the POLICY, not the ACTUATOR.** `fleet_core.stage2_capabilities` and
`fleet_core.trigger` declare eighteen step actions; `scripts/batch.py:fire_step` dispatches
ten. The other eight — `fail_ledger_writes`, `heal_ledger_writes`, `inject_stale_goal_result`,
`resume_permit_renewal`, `suspend_permit_renewal`, `start_truth_collector`,
`stop_truth_collector`, `terminate_run` — exist only in the declaration layer. A case naming
one of them cannot deliver its fault, and `scripts/check_batch_manifest.py` now refuses such a
case instead of letting it run and report a verdict about a fault that never happened. Five
cases are marked `requires:` for exactly this reason (F10, I03, I06, I07, I08); their content
is preserved under `intended_*`. The entries below that call these keys "IMPLEMENTED and
retired" are describing the policy files, which is not what a reader of a *limitations*
register needs to know.

**2. `drive_to_pose` did not close F03.** The `batch_driver_extended` entry below says
`drive_to_pose` means "a robot can now be driven to the exit buffer". It cannot. A pad is not a
legal arrival point: `config/resources.yaml` records that arriving there is refused
(`RESOURCE_UNKNOWN` on `mid`) while standing there is allowed, so the permit can never be
requested and the driver reports `Failed to make progress` (error_code 105) — the file says in
as many words that this "reads as a navigation fault and is not one". Measured: the
2026-09-19 change moved F03's target from `exit_west` to `exit_east` and **both** timed out.
F03 now declares `pad_occupation_by_crossing` (see the new entry below).

**3. Also stale:** the `obstacle_injection` entry ends "**Not yet run**: neither F01 nor F02
has produced a verdict." Both have now. F01 records `every declared outcome held` after
`despawn_obstacle` was fixed to use the world's `remove` service, and F02 records
`FAILED / FAIL / FAIL` with `no_unauthorised_entry_from_truth: 1949 sample(s)`.

The keys, with what each one actually is:

- `crossing_scenarios` — **IMPLEMENTED (round 16, 2026-09-18); retired from this list.**
  N04, N05, N06 and N07 are runnable, which with the seeded repeats of N04/N05/N07 is 10 of
  the 38 runs. Two pieces were missing and both are now in: the four scenario files
  (`config/scenarios/n0[4-7]_*.yaml`), and the ARRIVAL-ORDER vocabulary this entry's previous
  version named as the remaining gap. A case states the order it is about with `wait_for` on a
  submission -- a PREDICATE over the fleet snapshot, evaluated until it holds
  (`fleet_core.wait_for`), **never a sleep**, because a sleep is a guess about how long
  something takes and is wrong in both directions. The order, and the exclusivity it implies,
  are then CHECKED from the task service's own event stream
  (`fleet_core.crossing_ledger`, used by `scripts/batch.py` for `crossing_exclusive` and
  `crossing_order`): the end-of-run snapshot is last-writer state and "who went first" is gone
  by then. Neither check holds a threshold -- an overlap is an inequality between two
  timestamps from the same file -- so no margin can be widened to make a failing case pass.
  See docs/DECISIONS.md D-P16-01. **Not yet run**: these four cases are runnable and none of
  the four has produced a verdict, so nothing here says they pass.
- `a case cannot pin a task to a robot` — `pinned_robot` exists in the task service but is
  set only for the charge runs the service generates itself; `SubmitTask.srv` has no pin field.
  The allocator chooses by travel cost, so a case that needs a PARTICULAR robot to take a
  particular task has to arrange the geometry (and the submission order) so that the intended
  assignment is also the cheapest one. N04-N07 are written that way and say so in their
  comments. See docs/DECISIONS.md D-P16-02.
- `batch_driver_extended` — **the trigger vocabulary this entry asked for now exists** (round
  17, 2026-09-18): `steps:`, an ordered list of `when <predicate> do <action>` in
  `config/scenarios/regression_v1.yaml`, with the predicates taken from `fleet_core.wait_for` and
  the actions enumerated in `fleet_core.trigger` and checked in the build by
  `scripts/check_batch_manifest.py`. F06, F07, F08, F09, F11, F12, F13, F14 and N08 moved onto it.
  **Three cases still declare this key and none of them is waiting for a trigger:** F03 needs a
  robot parked on the exit buffer (nothing drives a robot to a chosen pose), I03 needs a stale goal
  result injected at the ROS integration boundary, I08 needs a whole run terminated and another
  started. See docs/DECISIONS.md D-P17-01.
- `low_battery_while_executing` — **IMPLEMENTED (round 17, stage 2); retired from this list.**
  The gap this entry described was real: `task_service_node._service_battery` began with
  `if state.executing or not self._fresh(rid, now): continue`, so a robot whose charge went
  critical mid-task drove on to the end of its leg — the opposite of CONTRACTS section 5's
  rule (持货低电：v1 不自动卸货、不自动把货带到不支持载货的充电位；安全停到可达合法位并
  NEEDS_ATTENTION). The skip is now split: freshness still skips, but an executing robot at
  CRITICAL goes to `_service_loaded_battery`, which decides through
  `fleet_core.loaded_battery_policy` (`decide` checks the loaded branch FIRST, so carrying
  outranks every other consideration), finds a legal park point with `legal_park_point`
  (which clears the gap and both pads), and parks or cancels-and-charges accordingly. 22
  tests in `tests/test_p17_loaded_battery.py`. Found in round 17 while wiring F05, which
  was filed under `batch_driver_extended` and was never a trigger problem at all. See
  D-P17-02. **Not yet run.**
- `obstacle_injection` — **IMPLEMENTED (round 17, stage 2); retired from this list.**
  `fleet_core.stage2_capabilities` carries the `spawn_obstacle` / `despawn_obstacle`
  actions with the free-space check this entry asked for: Section 5 still forbids
  teleporting a body into the robot's footprint, so `obstacle_precondition` refuses a
  spawn radius below `MIN_OBSTACLE_RADIUS_M` (0.20 m) or one that would land on a robot,
  and `obstacle_leaves_a_route` refuses a placement that seals the capacity-1 corridor
  completely — which is what F02 is about, so the check has to distinguish "blocks the
  corridor" from "blocks it for good". F01 and F02 now declare `steps:` using them.
  **Not yet run**: neither F01 nor F02 has produced a verdict.
- `robot_kill_trigger` — **retired in round 17 (2026-09-18).** F06, F07, F08, F09 and F13 each
  declare `steps:` with `kill_process` now, addressed by the argv the launch itself writes
  (`-r __ns:=/r01` for a namespace, `-r __node:=safety_gate` for one node). Section 6's split is
  kept: "Adapter 退出" stops `nav2_adapter` and leaves the Gate running (F07), "Gate 退出" stops
  `safety_gate` and nothing else (F13), and F09 stops the whole namespace because a robot that has
  failed inside the corridor is a robot that is gone.
- `pad_occupation_by_crossing` — **OPEN (2026-09-20, D-P17-30).** F03 declares it.
  This is NOT a missing action: `drive_to_pose` exists and `scripts/batch.py:fire_step`
  dispatches it. It is a missing LEGAL ARRIVAL POINT. `config/resources.yaml` records that
  arriving at a pad is refused (`RESOURCE_UNKNOWN` on `mid`) while a robot standing there is
  allowed, so a robot cannot be driven onto an exit buffer directly -- it has to earn its way
  there through a granted crossing. F03 wants a body ON the pad, so the capability it needs is
  "occupy a buffer by completing a crossing and staying", which nothing provides.
  Do NOT close this by pointing `drive_to_pose` at `align_east`: that IS a legal asking point,
  so the robot would stop beside the pad and the case would no longer test what its title says.
- `truth_collector_control` — **IMPLEMENTED (round 17, stage 2); retired from this list.**
  `start_truth_collector` / `stop_truth_collector` are registered actions, and
  `collector_precondition` / `truth_absence_is_reported` state the two halves of what I06
  asks: the collector can be stopped mid-run, AND the absence of truth that results is
  REPORTED as absence rather than silently becoming a pass. That second half is the point
  of the case — see the frozen-truth section at the end of this file for why "no truth"
  and "frozen truth" have to be told apart. **Not yet run.**
- `world_pause_control` — **IMPLEMENTED (round 17, stage 2); retired from this list.**
  `pause_world` / `resume_world` for I04, and `jump_clock` for I05. The clock reset needed
  its own action rather than being a side effect of pausing, because I05 asks what happens
  when time itself moves under the fleet and a pause does not imply that;
  `test_stage2_capabilities.py` pins the distinction
  (`test_the_clock_jump_is_a_registered_ACTION_not_a_side_effect_of_pausing`).
  `clock_jump_precondition` and `PauseObservation.staleness_was_observable` carry the
  preconditions. **Not yet run.**
- `db_fault_injection` — **IMPLEMENTED (round 17, stage 2); retired from this list.**
  `fail_ledger_writes` / `heal_ledger_writes`, with `LEDGER_FAULT_MODES = ("write_denied",
  "db_locked", "disk_full_sim")` — the third is a SIMULATED full disk, which is what
  Section 6's "controlled failure, not a full disk" actually requires.
  `db_fault_precondition` and `db_fault_confirm` hold the two halves. **Not yet run.**
- `nav2_fault_injection` — **retired in round 17.** F12 is the second half of its own title
  ("目标拒绝/服务器退出"): the server exits, and `kill_process` on that robot's `bt_navigator` is
  exactly that. "Nav2 refuses a goal" remains out of reach — it depends on Nav2 deciding to — and
  the case says which of the two it delivers.
- `localization_fault_injection` — **retired in round 17.** F14 stops that robot's `gz_bridge`,
  which is where its odometry and scan enter ROS; from there AMCL has nothing to correct against
  and the gate has no pose it trusts. The case's second clause (拒绝真值代驾) is NOT asserted —
  see D-P17-03.
- `permit_renewal_interrupt` — **IMPLEMENTED (round 17, stage 2); retired from this list.**
  `suspend_permit_renewal` / `resume_permit_renewal`, with
  `permit_interrupt_precondition` and `permit_withdrawal_confirm`. The case's difficulty
  was never starting the interruption; it was that the robot must KEEP REPORTING while the
  permit stops being renewed, so a check that merely observed a stopped renewal would pass
  on a robot that had also gone silent — which is a different fault (F06/F08's). The
  precondition separates them. **Not yet run.**

**What round 17 changed, in one line each**, so this section can be read without the manifest:

* the trigger vocabulary exists: `steps:` with `when <predicate> do <action>` (D-P17-01);
* `robot_kill_trigger`, `nav2_fault_injection` and `localization_fault_injection` are retired by
  it;
* `batch_driver_extended` is narrowed to F03, I03 and I08, none of which is a trigger problem;
* `low_battery_while_executing` is ADDED, for F05, which was filed under the wrong key (D-P17-02);
* two cases now run with a clause their manifest cannot decide, and both say so in their own
  `note:` (F13's downstream-of-the-gate claim, F14's refused-the-truth claim) — unasserted, not
  passing (D-P17-03);
* two fault-entry shapes now coexist (`faults:` for F04 and `steps:` for everything after it),
  which is debt (D-P17-04);
* **stage 2 lands the remaining six capabilities** (`obstacle_injection`,
  `truth_collector_control`, `world_pause_control`, `db_fault_injection`,
  `permit_renewal_interrupt`, `low_battery_while_executing`), so **all 38 runs are now runnable
  and 0 are blocked** — the count went from 16 runnable / 22 blocked (round 16) to **38 / 0**;
* **and the three-robot truth stream was found FROZEN** — see the section at the end of this
  file. That is the one entry here a reader must not skim: every three-robot safety verdict
  recorded before round 17 was taken against a truth stream in which no body ever moved, so
  those verdicts, including N07's `safety = PASS/PASS/PASS`, **are not safety passes**. The
  fix is in and guarded, and it has NOT yet been confirmed by a live three-robot run.

**Ground truth is attached to every case now, and the batch can be accepted.** Since
2026-09-16 `scripts/batch.py` starts `fleet_evaluation`'s recorder beside every case and judges
the recording with `fleet_evaluation.judge`. `safety_outcome` comes from the judge's two safety
properties derived from the simulator's own body poses -- not from the judge's summary, which
also carries pose error and the claim cross-checks. Measured on the five runnable cases:
**4 PASS, 0 FAIL, 1 UNKNOWN** (N03: see the truth-slot entry above; with two robots the
recording contains one robot's pose and one that never moves, so nothing about the second robot
can be judged independently).

Two things this does NOT establish, both registered rather than implied:

- **The runner publishes no claim of its own, so `driver_claims_corroborated_by_truth` is
  `NOT_RUN` on every batch case.** The runner IS a driver: its task outcomes and the
  dispatcher's crossing reports are exactly the claims the judge exists to contradict.
  Publishing them in a form `judge.load_case` reads is the next step. Until then a batch case's
  judge summary is `NOT_RUN` even when both safety properties pass -- which is why
  `safety_outcome` is derived from the properties and not from the summary.
- **`pose_estimate_error` FAILED on the cases where it could be judged**: worst p95
  |claim - truth| = 1.8588 m over 2995 samples (limit 0.35 m) on N02, and this was NOT
  investigated. It is consistent with the recorded finding that `map -> odom` is nearly
  constant, i.e. that AMCL barely corrects, but that is a note and not a diagnosis. Every
  safety verdict in the batch is derived from truth, so a drifting estimate does not move them;
  what it does mean is that containment judged from odometry would have been wrong, which is
  that check's own reason for existing.

**`scripts/run_demo.sh` is a stub for the real backend.** It exits 3 for
`--backend gazebo_nav2` with "not implemented yet (P2)", so TEST_AND_ACCEPTANCE section 9's
command contract is only partly implemented: the fleet is launched by `scripts/batch.py`,
`scripts/run_scenarios3456.sh` and `scripts/run_p7_matrix.sh` instead. The `fake` backend
works. `README.md` names the commands that do run.

## P8 wiring and instrumentation follow-ups (2026-09-17)

- **The gate's latch is GEOMETRIC and it judged from a pose 1.87 m from truth.** The gate now
  publishes the pose it used. At the latch (`reports/batch_20260917T092455Z/N01`) it believed
  `(3.488, -0.269)` and named the resource: `unpermitted ['mid_right'], holdings none
  (reserve 0.356 m)` -- i.e. its believed footprint was inside the east exit buffer's
  stopping-boundary expansion. At the end of the run it still believed `(3.389, -0.656)` while the
  simulator's truth for the same robot was `(5.223, -1.024)`: a gap of **1.87 m**, which is round
  5's p95 |claim - truth| = 1.8588 m again. **Two criteria, one envelope:** the gate refuses on
  `rect + reserve(speed) + footprint_margin` and the passage driver proves clearance on
  `rect + footprint_margin`, so "cleared" does not imply "clear" -- and the driver reported success
  at the same instant the gate began refusing. ****The alignment was then made, in the same round and without a new run**: the recorder logs its own start on absolute time and `runtime/events.jsonl` carries absolute timestamps, so the
  crossing's harvest locates the latch on the recorder's axis. At that instant the gate believed `(3.488, -0.269)` and the simulator's truth was `(4.403, -1.490)` -- a gap of
  **1.526 m**. The refusal is **correct in the gate's own frame** (re-derived by hand: a footprint corner at `(3.201, -0.510)` lies inside `mid_right` expanded by the printed
  0.506 m) and **wrong in reality** (the true pose is 1.553 m from that rectangle). So the gate refused on a pose error of 1.5 m, not on a bad rule, and the FIRST fix is the pose
  the gate is allowed to conclude from -- since `map -> odom` is nearly constant, the 1.5 m is a constant, and a constant can be measured directly. A retreat allowance is the second
  fix, as defence in depth. See `D-P8-03`.
- **The safety gate latches STOP after a permit lapses, and never clears it. Measured.**
  `permit expired without clearance` is written to the resource's block reason (`traffic.py:624`),
  and the same text is what makes every later refusal report `PERMIT_EXPIRED`
  (`traffic.py:302` tests `"expired" in self._block_reason.get(n)`). In the run that found it:
  the last `mode=NORMAL` was at **t=105.7** (the crossing's permit lapsing);
  from then until the end of the run **815 s later** the gate published
  `mode=STOP / reason=PERMIT_EXPIRED` at 20 Hz with `cmd=(0.0, 0.0)`, and `mode=NORMAL` did not
  occur once more. **This is why every N01 run's follow-on leg fails**: it is not the leg, and it
  is not Nav2 -- the gate is zeroing every command. The controller's `Failed to make progress` (32
  aborts, with `spin` and `backup` also timing out) is the correct response to a robot that is not
  allowed to move. **Not fixed here**: "clear the mark once the robot is outside every protected
  region" and "do not require a permit for a leg whose stop boundary touches no unpermitted
  resource" are different safety-layer changes, and the string that distinguishes them
  (`unpermitted [...], holdings [...]`) was truncated by this round's own probe. A safety-layer
  behaviour change chosen on a guess is what this project's rules forbid.
- **The batch does not record the gate's verdict either, and now can.**

- **The leg executor listened to a gate verdict nobody publishes.** Measured in a live fleet:
  `/r01/fleet/gate_state` had **0 publishers and 2 subscribers**; `/r01/gate_state` had
  **1 publisher (safety_gate) and 0 subscribers**. `fleet.launch.py` passed
  `gate_state_topic: "fleet/gate_state"` to `nav2_adapter_node` only, while the gate kept its own
  default `gate_state`. Fixed by making both name `gate_state`, which is what the other four
  consumers already use. **Cost, measured rather than assumed:** `_on_gate_state` sets only
  `_gate_reason`, which is copied into the published `RobotState` -- so every `RobotState` this
  fleet has published carries an **empty `gate_reason`**, and motion is unaffected (the adapter's
  own docstring forbids the log from entering the control path). **This does NOT explain the
  stalled base.** What was missing is a check that the five consumers AGREE: each was locally
  consistent, and a subscription nobody feeds is silent -- no error, no warning, no log line.
- **`reports/frozen_manifest.json` went stale and §10 item 10.1 was FAIL because of it.**
  The manifest was frozen at **2026-09-16T14:50:09Z** with 198 files; rounds 5-8 then changed two
  files and added six more (`config/scenarios/n01_crossing.yaml`, `scripts/probe_acquire.py`,
  `scripts/probe_gate_state.py`/`.sh`, `scripts/probe_recorder_stop.py`/`.sh`,
  `tests/test_p5_crossing_from_tasks.py`, `tests/test_p7_batch_truth.py`) without regenerating it.
  `verify_independence.sh` step 7 then reported CHANGED/EXTRA for each, which reads as "the copy
  is not the same tree". The manifest must be regenerated **after** the changes and **before** the
  relocation check; that ordering was already written down in this project's notes and was still
  missed. Item 10.1 is now measured **after** a regeneration, not before it.
- **The batch does not record the gate's verdict either, and now can.**
  `scripts/probe_gate_state.py` subscribes to `<ns>/gate_state`, prints every change with a
  publisher count, and writes a signal-safe summary. It is the instrument the "gate withheld it
  or the base ignored it" question needs.

## P7 crossing follow-ups (2026-09-17)

- **The `S_right_a -> S_right_b` drive fails reproducibly, and it is not the crossing.**
  Both N01 runs fail on that one leg, with the crossing healthy in each: round 6 crossed at
  `to_drop-2` and then failed `to_pick-0` (S_right_b); round 7 crossed at `to_pick-0` and then
  failed `to_drop-2` (S_right_b). Same origin, same destination, same failure -- `nav2 status=6`,
  both attempts spent, task `FAILED`, `expected_behavior: FAIL`. The Nav2 status is
  `error_code 105 'Failed to make progress'` from the controller's progress checker (32 times
  across the two attempts); both recoveries also time out (`spin` 701 four times, `backup` 711
  four times), and `bt_navigator` then aborts with the same 105. **What the truth stream adds**:
  the robot is motionless for the rest of the run -- in round 6 its world pose is
  `(5.241, -0.557)` unchanged for **372 s**, and the raw odometry is unchanged over the same
  window, so this is neither a localisation artefact nor a bad plan: nothing is moving the base.
  It stops about 1.9 m short of a goal in open floor between two stations, and the leg is
  entirely inside the east half, so the corridor is not involved. Two candidate mechanisms,
  and the evidence does not separate them: the gate withheld the command, or the base did not
  apply it. Registered together with the instrument gap below rather than guessed at.
- **The safety gate can zero a command silently, and the topic that would say so is not
  recorded.** `gate_node.py` has three paths that end in a zeroed command and only one of them
  speaks: `input_timeout_s` exceeded (silent), the geofence reducing the request (silent; the
  detail goes to `gate_state_topic`), and "no command has ever arrived" (logged once, never
  again -- `_published_zero` latches). In both N01 runs the gate printed nothing but its
  periodic "sole publisher check passed" line for the whole 6-minute window in which the robot
  did not move. The separating measurement exists and is published at 20 Hz on
  `<ns>/fleet/gate_state` (`zero_published_at`, `zero_is_not_stopped`, the solo-publisher flag)
  -- and the batch runner does not record it. **An instrument that could say which half failed
  was available and unwired**, which is the same shape as the `resources` field the judge read
  and nobody wrote. Next: record `gate_state_topic` in the case directory, and log the
  zero/no-zero transition (rate-limited) so a 6-minute silence is distinguishable from a
  6-minute pass-through.

## P10 follow-ups (2026-09-17): the boundary, the pose, and the truth stream

- **One boundary, and its bound.** `D-P10-01`: the refusal and the clearance proof now share
  `CrossingManager.protected_region`. The containment is monotone only up to the fastest speed the
  stop model was measured at (0.35 m/s) and a test asserts it fails above that; the gate's own
  clamp is tied to it by test because the two numbers live in different files.
  **Not established: that the crossing completes.** With `D-P10-02` unfixed, the stricter proof
  converts a silent 830-second freeze into a loud `CLEARANCE_NOT_PROVEN`.
- **The gate's pose error is a RATE, not an offset.** `D-P10-02`, measured in one process on one
  clock: the gate's pose is `spawn (+) odom` to 0.12 mm mean, and its error against the
  simulator's truth is `0.0809 x distance travelled` (R^2 = 0.923) -- 0.281 m in the first
  quarter of a 21.5 m route, 1.617 m in the last. At the same instant AMCL was 0.1144 m from
  truth. **So this is not a frame-convention bug and not a constant that can be subtracted**, and
  the fix is at the pose source: the gate concludes from raw odometry when a localised pose 11 cm
  away from truth is already published on the same machine. Not applied -- it is a safety-layer
  change with staleness semantics of its own, and one variable changes at a time here.
- **The truth stream's slot index is not an identity.** `D-P10-03`. The width of
  `dynamic_pose/info` varied 8/16/24 within one single-robot run, so `range(width //
  LINKS_PER_MODEL)` is not a stable slot set; and in the two-robot case N03 the stream carried
  ONE model while r02 travelled 9.902 m. **A two-robot safety verdict is unobtainable from this
  channel as bridged**, because the second robot's pose is not in it -- not because the judge
  cannot attribute slots (it can, and it names the missing robot instead of guessing).
- **`fleet_recorder`'s per-slot accounting is not read by anything.** It records
  `truth_widths`, and the number 0/8/16/24 is the evidence for the paragraph above, but no check
  fails when the width changes mid-run. Next: make a varying width a reported condition rather
  than a number in an audit block nobody diffs.

## P11 follow-ups (2026-09-17): the asking-point geometry, and what P4's acceptance now describes

* **The asking points moved after P4 was accepted.** `config/resources.yaml` sets
  `align_west`/`align_east` to |x| = 3.35, from 2.40. P4's acceptance run and its recorded
  arrival and crossing numbers describe the **previous** coordinates. The change is derived
  (`docs/DECISIONS.md` D-P11-01) and the invariant it restores is now a test, but P4's
  acceptance is not re-established until it is re-run on this geometry. Do not quote P4's
  crossing timings as describing the current map.

* **The crossing has not been run end to end on the new geometry.** `check_asking_points.py`
  and the suite say the robot can now legally *reach* the asking point, which is necessary and
  not sufficient. Whether the crossing then acquires, crosses and clears is a separate
  measurement, not yet taken.

* **The gate's resource states are still fed only by grants.** `GateNode._on_permit` ignores
  refusals on purpose, and `ResourcePermit` carries no resource states, so a resource the gate
  has never been granted stays `UNKNOWN` in the gate's own manager however clear the
  coordinator's sweep found it. The move removes the *observed* deadlock; the structural
  asymmetry — two components answering "what state is this resource in" with different inputs —
  remains. Registered, not fixed: making the gate adopt the coordinator's verified-clear state
  is a safety-layer change and needs its own argument.

* **`err_gate_truth` in the pose track measures the wrong number.** The gate publishes both the
  pose it decided from (`pose_trusted`) and the composed pose (`pose`), and the probe carried
  only the latter, so a reader saw the gate concluding from odometry while its own start-up
  line said `localiser`. The probe now carries `pose_source`, `pose_usable`, `pose_trusted` and
  `err_trusted_truth`; any analysis of a run recorded before 2026-09-17T11:33Z must not use
  `err_gate_truth` as "the gate's pose error".

* **`align_west` clears the boundary by 0.027 m of design headroom.** The derived requirement is
  3.323 and the configured position is 3.35, so the worst point of the legal asking ball sits
  0.027 m outside the boundary at the fastest measured speed. That is a real clearance and it is
  asserted, but it is small: any increase in the stop envelope, either margin, the footprint, or
  `node_reach_tolerance_m` will fail the test, and the node must then be re-derived rather than
  the test edited.

## P11 follow-ups, second batch (2026-09-17): the coordinator's pose, and what the ladder now proves

* **The coordinator judges the asking-point precondition from composed odometry.** Measured error
  of that pose this run: **1.5516 m median, 2.6825 m p95**, against a `node_reach_tolerance_m` of
  **0.80 m**. Task 1 of N01 succeeded and task 2 failed at `acquire` for this reason
  (`D-P11-03`). The gate's equivalent defect is fixed; the supervisor's is not.

* **`PERMIT_EXPIRED` still occurs — now transiently.** 34 rows this run (round 8's latch, whose
  mark re-arms its own refusal). The robot kept moving afterwards (`2580 NORMAL rows after the
  first STOP`, 34.999 m of travel), so the latch is not currently permanent, but the mechanism
  registered as `D-P8-02` is unchanged and this run does not show it is harmless.

* **The crossing is not robust.** One success, with no contention, no queue, no cancellation and
  no fault injection in the path. `D-P11-01` moved two nodes by a derived amount; whether the
  crossing survives a second robot, a lapsed permit mid-passage (`D-P4-12`) or a late renewal is
  untested.

* **`batch.py` still reports the case as a failure.** `safety: N01=PASS` and
  "a case's declared outcome did not happen (1 task, 1 behaviour) -- exit 4": behaviour is FAIL
  because task 2 did not complete. Exit 4, not 5, for the first time on this case.

* **AMCL's `update_min_d` / `update_min_a` are load-bearing for the gate's liveness.** The gate's
  escape hatch ("the robot has not moved, so a stale estimate still describes it") is what keeps a
  stationary robot from being refused for ever, because `amcl_pose` is motion-gated. Changing
  those parameters changes the gate's behaviour; they are not Nav2-tuning details.
  **2026-09-18 (`D-P17-06`): load-bearing was an understatement — the two were incompatible.**
  Stock `update_min_d` 0.25 m is larger than the hatch's `max_moved_m` 0.10 m, so the hatch was
  guaranteed to close partway through every update interval. Measured over N05's three seeds:
  227-262 refusals per run, and 17-18 Nav2 `Failed to make progress` aborts against 0 on the run
  that happened to keep up — which is how a 68 s approach became 194 s. `update_min_d` is now
  0.10 and `scripts/check_pose_freshness.py` fails the build if it ever exceeds the hatch again.
  **The hatch itself is untouched**: still 0.10 against a localiser p95 of 0.2515 m, which is
  `D-P15-02` and still open.

## P12 follow-ups (2026-09-17): the coordinator's source, and what the run after it showed

* **The start-up occupancy sweep now needs a localised pose.** Measured: `start-up sweep: ['r01']
  have no fresh pose; resources stay UNKNOWN`, then `re-verify (automatic): promoted ['mid',
  'mid_left', 'mid_right'] from UNKNOWN to FREE on fresh poses`. The re-verify recovers it, but the
  window in which nothing is grantable is longer than it was under `spawn_odom`, and nothing measures
  how much longer. A robot that asks inside that window is refused with `RESOURCE_UNKNOWN`.

* **The coordinator's judgements have not been re-measured.** `acquire`, `confirm_clear`, the sweep
  and the re-verify all now read a pose 8.3x closer to truth (0.1866 m vs 1.5516 m median), but only
  the pose has been measured, not the four decisions made from it.

* **A goal rejection with no attributed cause.** In the run after this change, the crossing driver's
  first goal was refused three times -- "another navigator is still processing one" -- while
  `nav2_adapter_node` also had goals refused and the controller had been aborting `Failed to make
  progress` for ~150 s beforehand. That is contention over the single `NavigateToPose` server between
  the leg executor and the crossing driver. Whether this change made it more likely is **not
  established**; one run cannot separate the two, and the previous run did not show it.

* **`PoseSource` has one consumer, not two, so far.** The coordinator uses it; the gate still carries
  its own equivalent bookkeeping inline (`_on_localiser`, `_on_odom`, `_moved_since_estimate`,
  `_trusted_pose`), pinned by `tests/test_p11_pose_source.py`. That is the arrangement this class
  exists to end, and leaving it is a deliberate ordering choice: the gate's path was verified live in
  the same session, and migrating it is a no-behaviour-change refactor that deserves its own
  verification run rather than riding along with a behavioural change. `D-P11-02` was a bug in exactly
  that bookkeeping, which is why this is a debt with a date on it, not a preference.

* **The gate's `err_gate_truth` remains the reported pose, not the acting one.** Unchanged since
  round 11 and still true: `pose_trusted` is the field to read, `err_trusted_truth` the number.

* **The localiser hatch is self-sustaining once the displacement exceeds its threshold.** Measured:
  the displacement froze at 0.283 m against a 0.100 m bound, so every subsequent row was unusable;
  `amcl_pose` is motion-gated, so refusing motion keeps it that way (`D-P12-03`). The hatch closes one
  regime (`D-P11-02`) and not the other.

* **`localization_margin_m` (0.10 m) and `max_moved_m` (0.10 m) are both below the localiser's own
  measured error (p50 0.101 m, p95 0.247-0.348 m).** They have to be re-derived together, with the
  position uncertainty entering the stop boundary instead of sitting in a constant. Until then,
  statements about the corridor boundary are statements about a boundary computed from a pose the gate
  may be up to ~0.63 m wrong about.

* **The goal rejection reproduces.** Two N01 runs with the coordinator on the localiser, both failing
  at `stage_wait: goal to wait_west was not accepted` after ~9.0 s, with `crossing_started` twice and
  `crossing_failed` twice. The cause is still not attributed -- the shape is contention over the single
  `NavigateToPose` server between the leg executor and the crossing driver -- but it is no longer
  "one run against one run": it is reproducible, and the run before the change did not show it.

## P14 follow-ups (2026-09-18)

* **`probe_gate_state.py` carries one of the gate's two poses.** The gate publishes
  `pose` (spawn + odom) and `pose_trusted` (what it decides with). The probe carries the
  first, so a run shows `|gate - truth| = 0.650 m` beside `|amcl - truth| = 0.072 m` and
  reads as "the gate is still on odometry" while its own startup line says `localiser`.
  This is `LESSONS.md` item 15, repeated. **Fix before using the probe to judge the pose
  source**, because otherwise the fact under test is not in the data.

* **The retreat allowance has two halves; only one was ever described.** `D-P4-12` names
  the safety-layer permission. The second half is a retreat *requestor* in the dispatcher:
  `check_movement` sees only the current pose and the commanded twist, so no amount of
  permission moves a robot nobody has asked to move. See `D-P14-02`.

* **`stopped_detail` exits did not set `elapsed_s`** (it read 0.0 while the sentence said
  162.6 s). Fixed the same day; the rule is now that every `_set_stage` exit carries `t0`.

* **N01's current binding constraint is `D-P12-03`**, not the goal rejection. The run of
  2026-09-18 reached `|x| ~ 3.25` with the localiser at 0.072 m from truth, then latched
  `POSITION_UNKNOWN` at t=85.5 s and stayed there for the remaining 355 s of the recording:
  the pose source's staleness hatch freezes at 0.283 m against its own 0.100 m bound, and
  `amcl_pose` is motion-gated, so the refusal keeps itself alive.

### Correction (2026-09-18, same day): the probe DOES carry both poses

The bullet below says `probe_gate_state.py` "carries one of the gate's two poses". **That is
wrong.** The probe forwards the gate's whole payload, so `pose_track.jsonl` carries
`pose_usable`, `pose_source`, `pose_trusted`, `pose_disagreement_m` and precomputes
`err_amcl_truth` / `err_gate_truth` / `err_trusted_truth`. What is missing is only the
**text log**, which prints the composed pose under the name `pose=` and never prints the
usable flag -- so `grep pose_usable gate_state_probe.txt` finds nothing and the fact under
test looks absent.

It was read as "the instrument carries one of two facts". The actual fault was **reading a
field that does not answer the question**: `err_gate_truth` measures the REPORTED pose, and
`err_trusted_truth` is the one that measures the pose the gate ACTED ON. `LESSONS.md` item
15 says "ask which fact a field is"; the refinement is that a field being *present* is not
the same as it being *the one you need*.

The measured numbers, once read correctly (516 samples of N01, 2026-09-18):

    |amcl - truth|            p50 0.1021  p90 0.2201  p95 0.2515  max 0.3385
    |gate_trusted - truth|    p50 0.1021  (identical -- the gate IS on the localiser)
    pose_usable               True 463, False 52
    localiser_moved_m (refused)   p50 0.2583  p95 0.3117  max 0.3397, frozen
    localiser_age_s (refused)     p50 316 s   max 813 s

## P15 follow-ups (2026-09-18)

* **`nomotion_service` 已实测为 `/r01/request_nomotion_update`**，门禁从图里发现它并调用了一次
  （见 `D-P15-01` 的实测结果）。**"刷新救回了死锁"仍是 NOT_RUN**：本轮的机器人从未进入完全停死
  状态，所以"改动"与"运行形态"混在一起，一次运行分不开。**复现才算数。**

* **探针的行是白名单**，所以门禁新增的 payload 字段必须**逐条加进去**才能被读到。
  `nomotion_*` 三个字段是在 N01 已经通过之后才补的 —— 也就是说那次成功里
  "问了几次"这个事实**在数据里不存在**。**加字段时同时加白名单，否则加了等于没加。**

* **两条 safety 余量仍未重推**（`D-P15-02`）。它们比必须覆盖的量小 2.5–2.6 倍：
  `localization_margin_m` 0.10 对定位 p95 0.2515；`max_moved_m` 0.10 对冻结位移 p50 0.2583。
  耦合已写清：抬 `localization_margin_m` 会让拒绝边界外移 0.1515 m，撞上只有 0.027 m 余量的
  申请点不变式（`tests/test_p11_asking_points.py`），必须与 `align_*` / `pad_release_*` 一起重推。
  **单独改那个数字是被禁止的**。

* **门禁的"位姿不可用"警告原来按 20 Hz 打**（355 s ≈ 7100 行）。已改成理由变化 + 最少间隔
  10 s。**重复陈述一个事实的日志会把它后面那个新事实埋掉** —— 这与"仪器沉默"是同一族缺陷的
  反面，值得单列。

* **`probe_gate_state.py` 的文本日志不打印 `pose_usable`**（JSONL 里有，文本里没有）。
  ⇒ `grep pose_usable <text log>` 找不到，事实看起来不存在。文本与结构化输出
  **必须都能回答同一个问题**，否则用哪一个就决定了你能得出什么结论。

## The three-robot truth stream was FROZEN, and it invalidates those verdicts (2026-09-18, mechanism corrected 2026-09-19)

**The most consequential finding of round 17. Read it before quoting any three-robot safety
result.**

The judge reads Gazebo's own `dynamic_pose/info` as the arbiter of where each robot is.
Measured across every recording in `reports/`, that stream carried **no motion at all** on
three-robot runs:

| recording | samples | truth-bearing | samples where a body MOVED | slots in the message |
|---|---|---|---|---|
| `N07` / `batch_20260918T103152Z` | 9141 | 8738 | **0** | 24 |
| `N07_seed2` | 9956 | 9609 | **0** | 24 |
| `N08_seed2` | 38960 | 38679 | **0** | 24 |
| `N05` (two robots, for contrast) | 8545 | 8236 | **6985** | 16 |

The message is not malformed -- the width is a correct 24 (three models x eight links) on
every sample -- and all 24 links hold their spawn pose for the whole run while the odometry
in the same file accumulates 94.596 m.

**The mechanism is publisher starvation, not delivery loss.** An earlier version of this
section said "the publisher did not stop; what collapsed is DELIVERY". The arrival rate
settles it -- `truth_messages` divided by the run duration:

| run | robots | `truth_messages` | samples | **msgs/s** | samples/s | duration s |
|---|---|---|---|---|---|---|
| `N07` T103152Z | 3 | 60 | 9144 | **0.13** | 20.01 | 457 |
| `N07_seed2` T103152Z | 3 | 641 | 9958 | **1.29** | 20.01 | 498 |
| `N07_seed3` T103152Z | 3 | 570 | 12319 | **0.93** | 20.00 | 616 |
| `N05_seed1` T015759Z | 2 | 22547 | 8548 | **52.65** | 19.96 | 428 |

Three robots: **0.13-1.29 Hz**. Two robots: **52.65 Hz**. A `KEEP_LAST` subscription can
drop messages that arrived; it cannot turn 52 Hz into 0.13 Hz. The subscriber's own sample
rate stays near 20 Hz throughout, so the loss is upstream of the subscription.

**What follows from this**

* No three-robot `safety` verdict recorded while the truth source was starved may be read
  as a pass. `no_unauthorised_entry_from_truth` cannot see a robot enter a corridor it has
  no permit for if nothing in the truth ever moves.
* `recorder.truth_qos()` (depth 10 -> 2000) was verified on a **two-robot** run only. Its
  necessity or sufficiency at three robots is **unproven**.
* `scripts/check_truth_liveness.py` reports `FROZEN` and exits 1 in guard mode, so this
  cannot be quoted as a pass by accident.

## A robot's BODY can be missing while its odometry is live

**This section is retracted, and the retraction is the finding.**

An earlier note here blamed a `ros_gz_sim/create` SIGSEGV for a 16-slot recording.
That was wrong. The recording it was based on
(`reports/qosverify_20260919T011913Z`) asked the recorder for three robots but its
launch started two, because `fleet.launch.py`'s `robots:=` argument defaults to
`r01,r02` and that run did not pass it. **16 slots = two models x eight links was
correct for the world that was actually launched.** The guard's arithmetic was
right; the expectation fed to it came from the case's declaration.

The SIGSEGV is real but was observed in a different run and **did not reproduce**:
a controlled three-robot spawn produced `Entity creation successful.` for r01 and
r02 and both `create` processes **finished cleanly** (no `-11`).

**And the three-robot recordings DO hold all three bodies.** `N07` T103152Z reports
`truth_slots_recorded = 24`, three model indices (`slot // 8` in `{0, 1, 2}`), and
three model frames each pinned at its own spawn: `(-6.0,-2.0)`, `(-6.0,0.6)`,
`(6.0,-2.0)`. No body was missing from that run.

**The real defect the mistake exposed** is the silent one: `robots:=` defaults to a
two-robot set while cases N07/N08 declare `runners: [r01, r02, r03]`. A caller that
omits the argument gets a world with fewer bodies than the recorder audit claims,
and every odometry-derived check still passes, because a robot with no body still
has an odometry bridge.

* `scripts/check_truth_liveness.py` computes `models_in_world = slots // LINKS_PER_MODEL`
  and compares it with the audit's `robots`, reporting **SHORT** and exiting 1.
  It cannot tell a short world from a mis-declared one, which is why the fix is at
  the caller.
* `scripts/batch.py` derives `robots:=` from the case body's own `runners`
  (`runs = list(body.get("runners") or ["r01"])`).
* `scripts/check_launch_robots.py` (11th static guard) keeps it that way: it fails
  any case naming an unknown runner, and any script that spawns
  `fleet.launch.py` without `robots:=`. It distinguishes a launcher (an argv
  containing `"ros2"`) from a file that merely mentions the launch path.

**Still not established**: no three-robot case has been re-run on a recording whose
truth source was live, so **no three-robot safety verdict is quotable today.**

## `check_guards.sh` could not fail a build (fixed 2026-09-19)

The runner that was supposed to enforce every guard could not detect a failure.

1. `set -uo pipefail` does **not** enable `pipefail`. It parses as `set -u`,
   `set -o`, `pipefail` -- `o` becomes a positional parameter and `pipefail` is
   ignored -- so the pipeline inside `guard()` returned `sed`'s status, which is
   always 0. Correct form: `set -u -o pipefail`.
2. `guard()` read the pipeline's status rather than the guard's. It now reads
   `${PIPESTATUS[0]}` and treats an unreadable status as a failure
   (`[ "${rc:-1}" -eq 0 ]`), because "missing" must not read as "fine".

Measured: `/usr/bin/python3 script.py` with `sys.exit(7)` returns **7** directly,
but **0** through `| sed 's/^/  /'`. Exit codes were never the problem; the pipe
was.

**Consequence**: before this fix, "the build is green" meant only that colcon
compiled. Every `[FAIL]` line this project had seen came from a guard printing its
own text, not from the runner deciding. The runner now carries a self-test --
inject `exit 7`, require a non-zero exit, restore, require 0 -- because a check
that cannot be shown to fail is not a check.
