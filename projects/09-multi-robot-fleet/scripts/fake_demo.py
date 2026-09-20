"""P1 demonstration: two robots contending for one corridor. **BACKEND: FAKE.**

This script exists to show the *coordination logic*, not to prove anything about
a robot. It uses the fake adapter, which moves a point at constant speed with no
physics, no collisions and no sensors. Every line of output is therefore labelled
with the backend, and no physical or safety claim is made anywhere.

What it does demonstrate, and what the tests assert:

  * the corridor plus its exit buffer is granted as ONE bundle
  * exactly one robot holds the corridor at a time
  * the loser queues, and the order follows arrival, not robot id
  * a lapsed permit does NOT free the corridor until something explicitly
    confirms it is empty
  * a robot that already holds the payload cannot simply be re-homed

Run:  python3 scripts/fake_demo.py [--scenario opposing_requests]
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[1]
for pkg in ("fleet_core", "fleet_adapter"):
    p = ROOT / "src" / pkg
    if str(p) not in sys.path:
        sys.path.insert(0, str(p))

from fleet_adapter import AdapterPhase, FakeAdapter, MotionCommand  # noqa: E402
from fleet_core import (  # noqa: E402
    Allocator,
    FakeClock,
    Ledger,
    ReasonCode,
    ResourceBook,
    RobotState,
    TaskMachine,
    TaskSpec,
    validate_fleet_config,
    validate_task_spec,
)

BACKEND = "fake"
BANNER = "=" * 74


def banner(title: str) -> None:
    print()
    print(BANNER)
    print(f"  {title}")
    print(f"  backend = {BACKEND}   (NO physics, NO collisions, NO sensors)")
    print(BANNER)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--scenario", default="opposing_requests")
    ap.add_argument("--robots", type=int, default=2)
    ap.add_argument("--config", default=str(ROOT / "config" / "fleet.yaml"))
    ap.add_argument("--runtime", default=str(ROOT / "runtime"))
    args = ap.parse_args()

    raw = yaml.safe_load(Path(args.config).read_text(encoding="utf-8"))
    cfg = validate_fleet_config(raw)

    runtime = Path(args.runtime)
    runtime.mkdir(parents=True, exist_ok=True)
    db = runtime / "fake_demo.sqlite3"
    if db.exists():
        db.unlink()  # demo only: a fresh ledger each run

    clock = FakeClock()
    ledger = Ledger(db)
    machine = TaskMachine(ledger, epoch=clock.epoch)
    book = ResourceBook(permit_ttl_s=cfg.permit_ttl_s)
    book.capacity[cfg.corridor] = 1
    for rid in cfg.exit_buffers.values():
        book.capacity[rid] = 1
    alloc = Allocator(cfg)

    robot_ids = [f"r{i:02d}" for i in range(1, args.robots + 1)]

    banner(f"scenario: {args.scenario}   robots: {', '.join(robot_ids)}")

    specs = [
        TaskSpec(
            request_id="opp-1",
            pick_station="S_left_a",
            drop_station="S_right_a",
            payload_id="demo_payload",
        ),
        TaskSpec(
            request_id="opp-2",
            pick_station="S_right_a",
            drop_station="S_left_a",
            payload_id="demo_payload_b",
        ),
    ]
    tasks = []
    for spec in specs:
        validate_task_spec(spec, cfg)
        task, created = ledger.submit(spec, now=clock.sim_now(), epoch=clock.epoch)
        tasks.append(task)
        print(f"  submit {spec.request_id}: task_id={task.task_id} created={created}")

    # Replaying the first request verbatim must change nothing.
    _, created_again = ledger.submit(specs[0], now=clock.sim_now(), epoch=clock.epoch)
    print(f"  replay {specs[0].request_id}: created={created_again}  "
          f"(must be False; tasks in ledger = {len(ledger.all_tasks())})")

    # Robots start at opposite ends, otherwise both opposing tasks would be
    # allocated to the same (nearest in every direction) robot and the scenario
    # would prove nothing about contention between two robots.
    starts = {"r01": (-5.0, 2.5), "r02": (5.0, 2.5)}

    for task in tasks:
        machine.accept(task.task_id, now=clock.sim_now())
        robots = {}
        for rid in robot_ids:
            x, y = starts.get(rid, (0.0, 0.0))
            robots[rid] = RobotState(rid, x=x, y=y, battery_wh=100.0)
        result = alloc.allocate(task, robots, now=clock.sim_now())
        assert result.reason is ReasonCode.OK, result.reason
        machine.assign(task.task_id, result.robot_id, now=clock.sim_now())
        print(f"  {task.spec.request_id} -> {result.robot_id}  (cost {result.cost:.2f})")

    banner("corridor contention")

    exit_left = cfg.exit_buffers["left"]
    bundle = (cfg.corridor, exit_left)

    first, second = tasks[0], tasks[1]
    r1 = ledger.get(first.task_id).robot_id
    r2 = ledger.get(second.task_id).robot_id

    gen = ledger.next_generation()
    res1 = book.acquire(
        task_id=first.task_id, robot_id=r1, resources=bundle,
        now=clock.sim_now(), wall_now=clock.wall_now(),
        boot_id=ledger.boot_id, revision=0, generation=gen, epoch=clock.epoch,
    )
    print(f"  {r1} requests corridor+buffer -> granted={res1.granted} ({res1.reason.value})")

    gen = ledger.next_generation()
    res2 = book.acquire(
        task_id=second.task_id, robot_id=r2, resources=bundle,
        now=clock.sim_now(), wall_now=clock.wall_now(),
        boot_id=ledger.boot_id, revision=0, generation=gen, epoch=clock.epoch,
    )
    print(f"  {r2} requests corridor+buffer -> granted={res2.granted} ({res2.reason.value})")
    print(f"    blocked_by : {[str(r) for r in res2.blocked_by]}")
    print(f"    queued_behind: {list(res2.queued_behind)}")
    print(f"  corridor owner: {book.owner_of(cfg.corridor)}   "
          f"exit buffer owner: {book.owner_of(exit_left)}")
    assert book.owner_of(cfg.corridor) == first.task_id
    assert book.owner_of(exit_left) == first.task_id

    banner("the loser waits; the winner leaves and confirms the area is clear")

    if res1.permit is not None:
        ledger.add_permit(res1.permit)

    # Force the permit to lapse on WALL time while sim is frozen.
    clock.pause()
    clock.advance_wall(cfg.permit_ttl_s + 1.0)
    expired = book.expire_due(wall_now=clock.wall_now())
    print(f"  sim paused at {clock.sim_now():.1f}s; wall now {clock.wall_now():.1f}s")
    print(f"  permits lapsed: {[p.permit_id for p in expired]}")
    print(f"  corridor still free? {book.is_free(cfg.corridor)}  "
          f"<- expiry is NOT clearance")
    print(f"  reason: {book.unknown_reason(cfg.corridor)}")
    assert book.is_free(cfg.corridor) is False

    cleared = book.confirm_clear(resources=bundle, proof="demo: evaluator says empty")
    print(f"  explicit clearance of {[str(r) for r in cleared]}")
    print(f"  corridor free now? {book.is_free(cfg.corridor)}")
    clock.resume()

    gen = ledger.next_generation()
    res3 = book.acquire(
        task_id=second.task_id, robot_id=r2, resources=bundle,
        now=clock.sim_now(), wall_now=clock.wall_now(),
        boot_id=ledger.boot_id, revision=0, generation=gen, epoch=clock.epoch,
    )
    print(f"  {r2} re-requests after clearance -> granted={res3.granted} ({res3.reason.value})")
    assert res3.granted is True
    assert book.owner_of(cfg.corridor) == second.task_id

    banner("fake motion (stand-in only)")

    adapters = {rid: FakeAdapter(robot_id=rid, speed_mps=0.35) for rid in robot_ids}
    for task in tasks:
        rid = ledger.get(task.task_id).robot_id
        machine.begin_execution(task.task_id, rid, now=clock.sim_now())
    for rid in robot_ids:
        target = cfg.stations["S_right_a"] if rid == r1 else cfg.stations["S_left_a"]
        adapters[rid].start(
            MotionCommand(
                task_id="demo", robot_id=rid, generation=ledger.next_generation(),
                target_x=target.x, target_y=target.y,
            )
        )
    for _ in range(400):
        clock.advance_sim(0.05)
        for a in adapters.values():
            a.tick(0.05)
        if all(a.status().phase is AdapterPhase.ARRIVED for a in adapters.values()):
            break
    for rid, a in adapters.items():
        st = a.status()
        print(f"  {rid}: phase={st.phase.value:8s} pos=({st.x:6.2f},{st.y:6.2f}) "
              f"travelled={a.distance_travelled_m:6.2f} m  backend={st.backend}")

    banner("payload ownership")

    held_task = tasks[0]
    ledger.pick("demo_payload", task_id=held_task.task_id, robot_id=r1)
    print(f"  {r1} picks demo_payload -> state={ledger.payload_state('demo_payload')[0].value}")
    moved = ledger.transfer_payload("demo_payload", to_robot=r2, task_id=second.task_id)
    print(f"  attempt to hand the HELD payload to {r2}: allowed={moved}  "
          f"(must be False)")
    holder = ledger.payload_state("demo_payload")[1]
    print(f"  holder is still: {holder}")
    assert moved is False and holder == r1

    banner("summary")
    print(f"  backend          : {BACKEND}   <- nothing here is a physical result")
    print(f"  tasks in ledger  : {len(ledger.all_tasks())}")
    print(f"  corridor owner   : {book.owner_of(cfg.corridor)}")
    print(f"  payload holder   : {holder}")
    print(f"  sim time         : {clock.sim_now():.2f}s   wall time: {clock.wall_now():.2f}s")
    print()
    print("  PHYSICAL / SAFETY RESULT: NOT_RUN -- the fake backend has no physics.")
    print("  Per TEST_AND_ACCEPTANCE §1 a fake run can never yield safety_outcome=PASS.")
    print(BANNER)

    ledger.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
