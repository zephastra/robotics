# 009 — Architecture

A multi-robot fleet simulator: three four-wheeled AMRs in a warehouse split by a barrier,
one gap in the middle, and a rule set that decides who may be in that gap.

The point of the project is not the robots. It is the **coordination rules** — and the
discipline of proving they hold rather than asserting that they should.

---

## 1. Layers, and the one rule that keeps them honest

```
        fleet_bringup      launch files, URDF, parameters          (needs ROS + sim)
        fleet_ros          nodes: gate, coordinator, adapters      (needs ROS)
        fleet_adapter      gate policy, zone guard, fake backend   (pure Python)
        fleet_core         domain, ledger, traffic, tasks          (pure Python, no ROS)
```

The load-bearing rule: **`fleet_core` imports no ROS and no simulator.** That is why the
rules that decide whether a robot is sent somewhere it cannot come back from can be checked
in under a second, by 372 tests, with no Gazebo. If a rule cannot be expressed without ROS,
it belongs in `fleet_ros`; if it is in `fleet_core`, it must be testable alone.

This is enforced, not just intended: `scripts/test_core.sh` refuses to run when `ROS_DISTRO`
is set.

## 2. `fleet_core` — the rules

| module | responsibility |
|---|---|
| `domain.py` | `TaskState`, `PayloadState`, `ReasonCode`, `TaskSpec`, `RobotState`, `TrafficConfig`. All the vocabulary. Reason codes are spelled the way `CONTRACTS.md` spells them, so a report can be checked against the design document. |
| `clock.py` | `Clock` / `FakeClock`, so time is an input rather than an ambient fact |
| `events.py` | append-only event log; at-least-once delivery, consumers de-duplicate |
| `ledger.py` | SQLite. Tasks, assignments, payloads, resources, requests, events. Every mutation goes through `_tx()`, which serialises on one lock. `submit` is idempotent on `request_id`. |
| `resources.py` | the lease book: who holds what, FIFO, permit TTL |
| `traffic.py` | the crossing state machine: `UNKNOWN → FREE → RESERVED → OCCUPIED → CLEARING → FREE`, atomic bundle acquisition, verified clearance, per-resource authorisation |
| `geometry.py` | rectangles, oriented bodies, separating-axis overlap, stopping envelope, `compose_2d` |
| `task_machine.py` | the legal transition map, two-phase cancel, `takeover_allowed` |
| `allocator.py` | capability filter, battery admission, low-battery detection |
| `battery.py`, `charging.py` | linear energy model and per-pad capacity with FIFO |
| `legs.py` | task → legs, and `approach_direction()` — which side of the barrier a robot is on |
| `recovery.py` | `CONTRACTS.md` §5 takeover table, as code, returning an **action string** rather than a bool |
| `validation.py` | config parsing that refuses rather than defaults |

### Three invariants worth naming

1. **Nothing is `FREE` until occupancy has been checked.** The initial state is `UNKNOWN`,
   and every transition out of it requires a stated source. A boot sweep, and continuous
   re-verification, are the same function (`verified_clear`) so they cannot disagree.
2. **A permit expiring is not evidence that the region is empty.** `expire_due` demotes the
   resource to `UNKNOWN` and leaves it blocked. Only a verified geometric clearance frees it.
   A resource can also never be freed by a caller's claim: `confirm_clear` takes a pose and
   recomputes, so parking on the exit buffer is refused rather than accepted.
3. **The bundle is atomic.** A corridor and the exit buffer the robot will leave into are
   granted together or not at all. Granting the corridor alone is a robot that can enter and
   cannot leave, which is how a fleet deadlocks.

## 3. `fleet_adapter` — policy, still testable

`safety_gate.py` holds the decision logic: staleness, heartbeat, emergency stop, permit
life. `zone_guard.py` supplies the geometric authority: given a pose and a twist, it asks
`CrossingManager.check_movement` and returns a verdict.

The important property: **the gate computes the geometry itself.** An earlier version took
`wants_protected_zone` as a caller-supplied boolean, which is not a safety check — it is
"please approve yourself", and it fails open by default. With a guard installed, the caller's
flag is ignored. A `zone` call that raises is treated as unknown occupancy and stops the
robot, because an exception in a ROS timer callback is not a stop.

## 4. `fleet_ros` — nodes

| node | scope | responsibility |
|---|---|---|
| `gate_node` | one per robot | the **only** publisher of `/rXX/cmd_vel`. Zero-published is not stopped-confirmed. Mirrors permits and judges with its own geometry. |
| `coordinator_node` | fleet singleton | owns the single `CrossingManager`; serves `AcquirePassage` / `RenewPermit` / `ConfirmClear`; runs the boot occupancy sweep and continuous re-verification; publishes permits |
| `nav2_adapter_node` | one per robot | the **only** caller of `/rXX/navigate_to_pose`. Serves `ExecuteLeg`. Refuses stale commands from a superseded generation, discards late results, and reports `stopped_confirmed` from measured odometry. Never degrades to a fake backend. |
| `task_service_node` | fleet singleton | owns `Ledger` + `TaskMachine` + `Allocator` + `EventLog`. Serves `SubmitTask` / `CancelTask` / `FaultInject` and the status snapshot. Applies the admissions the allocator cannot know: charge-pad occupancy, low-battery diversion, route connectivity, payload custody. |
| `stop_distance_calibrator` | one per robot | measures real stopping distance before anything is allowed to plan on it |

### Ownership rules that are not negotiable

- **One writer per `cmd_vel`.** The gate verifies this itself at startup and latches an
  emergency stop if a second publisher appears.
- **One owner per permit decision.** The coordinator and the gate run the *same* geometry and
  do not substitute each other's answers. The permit only tells a robot what it was granted.
- **Permits are scoped to the coordinator's epoch**, and the caller cannot supply one. A
  grant scoped to an epoch the caller chose is invisible to the coordinator's own publisher —
  that was a real defect, and it made every permit unreadable by the node that issued it.
- **No nested spins.** Nodes run on `MultiThreadedExecutor`; futures are polled.

## 5. `fleet_bringup` — launch

`fleet.launch.py` brings up the world, then the coordinator and task service, then each
robot with `traffic_guard:=true`. `robot.launch.py` brings up one robot: Gazebo spawn,
bridge, Nav2 stack, gate, adapter. `robots:=r01,r02,r03` is a plain parameter, and
`config/spawns.yaml` is the only place a spawn pose is written down — `validate_assets.py`,
the launch file, AMCL's initial pose and the stop-distance calibrator all read it.

## 6. The instruments

| script | what it is for |
|---|---|
| `test_core.sh` | 372 pure-Python tests, no ROS, under a second |
| `check_guards.sh` | the five static guards, called from `build.sh` |
| `check_undefined_names.py` | undefined capitalised names, `Node` member shadowing, and out-of-function-scope names |
| `check_xml_wellformed.py` | every XML in `assets/`, `config/`, `src/` |
| `check_ros_params.py` | every parameter read is declared, and none are dead |
| `validate_assets.py` | spawn poses, pads, walls, footprint, URDF-vs-SDF kinematics, map extent |
| `validate_traffic_geometry.py` | traffic config vs world vs fleet config, 57 checks |
| `acceptance_p4.*` | two-robot opposing traffic, both request orders, with sampling and a self-audit |
| `run_p5.sh` | the three-robot task probe: submissions, refusals, a fault with a real precondition, a cancel with a real precondition |
| `run_demo.sh` / `stop_demo.sh` | bring the demo up, and stop it with a *verified* teardown |

## 7. Why the evidence is shaped the way it is

- **Three outcomes are kept separate** (task / safety / expected_behavior), because a run can
  pass one and fail another and the average of them is meaningless.
- **`backend` is labelled** on every result. A `fake` backend can never produce a safety
  PASS; it is `NOT_RUN`.
- **Ground truth never reaches control** — it is only used to check estimates afterwards.
- **Instrument self-audit.** Any rate or latency claim must compare a stamp-derived rate
  against a delivered rate, or be withheld. Sampling that cannot pass its own audit is
  reported as `WITHHELD`, and the numbers it produced are indicators, not measurements.
- **A run whose teardown did not verify is not evidence.** Two fleets on one DDS domain
  produce numbers that look like results and are contamination.
