"""L1 §2, §9, §10, §13: idempotency, atomic payload moves, restart recovery.

Maps to TEST_AND_ACCEPTANCE L1:
  * 2  request_id same-content dedupe / different-content conflict / post-restart
  * 9  pick-deliver atomicity, held payload does not teleport, no double pickup
  * 10 lost ack idempotent resend (a replayed submit changes nothing)
  * 13 crash recovery reconciliation and failure handling
"""

from __future__ import annotations

import sqlite3

import pytest

from fleet_core import Ledger, LedgerError, TaskState, TaskSpec
from fleet_core.domain import ConflictError, PayloadState


# --------------------------------------------------------------------------- #
# §2 idempotency
# --------------------------------------------------------------------------- #


def test_first_submit_creates(ledger, spec_factory):
    task, created = ledger.submit(spec_factory("req-A"), now=1.0, epoch=0)
    assert created is True
    assert task.task_id == "task-req-A"
    assert task.state is TaskState.SUBMITTED


def test_same_request_same_content_is_a_noop(ledger, spec_factory):
    t1, c1 = ledger.submit(spec_factory("req-A"), now=1.0, epoch=0)
    t2, c2 = ledger.submit(spec_factory("req-A"), now=99.0, epoch=0)
    assert c1 is True and c2 is False
    assert t1.task_id == t2.task_id
    assert t2.created_at == 1.0, "the replay must not rewrite the stored task"
    assert len(ledger.all_tasks()) == 1


def test_same_request_different_content_conflicts(ledger, spec_factory):
    ledger.submit(spec_factory("req-A", drop_station="S_right"), now=1.0, epoch=0)
    with pytest.raises(ConflictError, match="REQUEST_CONFLICT"):
        ledger.submit(spec_factory("req-A", drop_station="S_left2"), now=2.0, epoch=0)
    assert len(ledger.all_tasks()) == 1, "a conflict must not create a second task"


def test_different_payload_same_request_id_conflicts(ledger, spec_factory):
    ledger.submit(spec_factory("req-A", payload_id="p1"), now=1.0, epoch=0)
    with pytest.raises(ConflictError):
        ledger.submit(spec_factory("req-A", payload_id="p2"), now=2.0, epoch=0)


def test_dedupe_survives_process_restart(tmp_path, spec_factory):
    """The dedupe key lives in the database, not in memory."""
    path = tmp_path / "fleet.sqlite3"
    led1 = Ledger(path, sim_t=0.0, wall_t=0.0)
    t1, c1 = led1.submit(spec_factory("req-A"), now=1.0, epoch=0)
    led1.close()

    led2 = Ledger(path, sim_t=0.0, wall_t=0.0)
    try:
        t2, c2 = led2.submit(spec_factory("req-A"), now=5.0, epoch=0)
        assert c1 is True and c2 is False
        assert t2.task_id == t1.task_id
        assert len(led2.all_tasks()) == 1
    finally:
        led2.close()


def test_restart_conflict_still_detected(tmp_path, spec_factory):
    path = tmp_path / "fleet.sqlite3"
    led1 = Ledger(path)
    led1.submit(spec_factory("req-A", drop_station="S_right"), now=1.0, epoch=0)
    led1.close()

    led2 = Ledger(path)
    try:
        with pytest.raises(ConflictError):
            led2.submit(spec_factory("req-A", drop_station="S_left2"), now=2.0, epoch=0)
    finally:
        led2.close()


def test_replayed_submit_ten_times_yields_one_task(ledger, spec_factory):
    """L1 requirement I01: 10 identical submissions -> exactly one task."""
    for i in range(10):
        ledger.submit(spec_factory("req-A"), now=float(i), epoch=0)
    assert len(ledger.all_tasks()) == 1


# --------------------------------------------------------------------------- #
# §9 payload atomicity
# --------------------------------------------------------------------------- #


def test_payload_starts_free_not_held(ledger, spec_factory):
    ledger.submit(spec_factory("req-A", payload_id="p1"), now=1.0, epoch=0)
    state, holder = ledger.payload_state("p1")
    assert state is PayloadState.FREE
    assert holder is None, "submitting a task must not claim the payload"


def test_pick_claims_payload(ledger, spec_factory):
    task, _ = ledger.submit(spec_factory("req-A", payload_id="p1"), now=1.0, epoch=0)
    assert ledger.pick("p1", task_id=task.task_id, robot_id="r01") is True
    state, holder = ledger.payload_state("p1")
    assert state is PayloadState.HELD
    assert holder == "r01"


def test_second_robot_cannot_pick_a_held_payload(ledger, spec_factory):
    task, _ = ledger.submit(spec_factory("req-A", payload_id="p1"), now=1.0, epoch=0)
    ledger.pick("p1", task_id=task.task_id, robot_id="r01")
    assert ledger.pick("p1", task_id=task.task_id, robot_id="r02") is False
    assert ledger.payload_state("p1")[1] == "r01"


def test_same_robot_pick_is_idempotent(ledger, spec_factory):
    task, _ = ledger.submit(spec_factory("req-A", payload_id="p1"), now=1.0, epoch=0)
    assert ledger.pick("p1", task_id=task.task_id, robot_id="r01") is True
    assert ledger.pick("p1", task_id=task.task_id, robot_id="r01") is True


def test_deliver_requires_the_holder(ledger, spec_factory):
    task, _ = ledger.submit(spec_factory("req-A", payload_id="p1"), now=1.0, epoch=0)
    ledger.pick("p1", task_id=task.task_id, robot_id="r01")
    assert ledger.deliver("p1", task_id=task.task_id, robot_id="r02") is False
    assert ledger.deliver("p1", task_id=task.task_id, robot_id="r01") is True
    assert ledger.payload_state("p1")[0] is PayloadState.DELIVERED


def test_no_double_delivery(ledger, spec_factory):
    task, _ = ledger.submit(spec_factory("req-A", payload_id="p1"), now=1.0, epoch=0)
    ledger.pick("p1", task_id=task.task_id, robot_id="r01")
    assert ledger.deliver("p1", task_id=task.task_id, robot_id="r01") is True
    assert ledger.deliver("p1", task_id=task.task_id, robot_id="r01") is False


def test_held_payload_does_not_teleport_on_transfer(ledger, spec_factory):
    """F08: a robot failing while holding a payload must not leak it."""
    task, _ = ledger.submit(spec_factory("req-A", payload_id="p1"), now=1.0, epoch=0)
    ledger.pick("p1", task_id=task.task_id, robot_id="r01")
    assert ledger.transfer_payload("p1", to_robot="r02", task_id="task-other") is False
    state, holder = ledger.payload_state("p1")
    assert state is PayloadState.HELD and holder == "r01"


def test_transfer_allowed_while_only_reserved(ledger, spec_factory):
    task, _ = ledger.submit(spec_factory("req-A", payload_id="p1"), now=1.0, epoch=0)
    assert ledger.transfer_payload("p1", to_robot="r02", task_id="task-req-A") is True
    state, holder = ledger.payload_state("p1")
    assert state is PayloadState.RESERVED and holder == "r02"


def test_canceling_a_task_releases_its_reserved_payload_in_one_transaction(ledger, spec_factory):
    task, _ = ledger.submit(spec_factory("req-A", payload_id="p1"), now=1.0, epoch=0)
    ledger.set_state(task.task_id, TaskState.CANCELED, now=2.0)
    state, holder = ledger.payload_state("p1")
    assert state is PayloadState.FREE
    assert holder is None


def test_canceling_does_not_release_a_held_payload(ledger, spec_factory):
    """Canceling the task does not mean the physical payload left the robot."""
    task, _ = ledger.submit(spec_factory("req-A", payload_id="p1"), now=1.0, epoch=0)
    ledger.pick("p1", task_id=task.task_id, robot_id="r01")
    ledger.set_state(task.task_id, TaskState.CANCELED, now=2.0)
    state, holder = ledger.payload_state("p1")
    assert state is PayloadState.HELD
    assert holder == "r01", "a held payload needs reconciliation, not silent release"


# --------------------------------------------------------------------------- #
# §13 recovery and failure handling
# --------------------------------------------------------------------------- #


def test_boot_id_changes_across_instances(tmp_path):
    a = Ledger(tmp_path / "a.sqlite3", sim_t=0.0, wall_t=0.0)
    b = Ledger(tmp_path / "b.sqlite3", sim_t=9.0, wall_t=9.0)
    try:
        assert a.boot_id != b.boot_id
    finally:
        a.close()
        b.close()


def test_reconcile_reports_open_tasks_without_resuming_them(ledger, machine, spec_factory):
    task, _ = ledger.submit(spec_factory("req-A"), now=1.0, epoch=0)
    machine.accept(task.task_id, now=2.0)

    report = ledger.reconcile(now=100.0, epoch=0)
    assert report["open_tasks"], "an unfinished task must be reported"
    # Conservative: reconciliation reports, it does not silently resume.
    assert ledger.get(task.task_id).state is TaskState.ACCEPTED


def test_reconcile_frees_orphan_held_payloads(ledger, spec_factory):
    task, _ = ledger.submit(spec_factory("req-A", payload_id="p1"), now=1.0, epoch=0)
    ledger.pick("p1", task_id=task.task_id, robot_id="r01")
    ledger.set_state(task.task_id, TaskState.SUCCEEDED, now=2.0)

    report = ledger.reconcile(now=100.0, epoch=0)
    assert "p1" in report["orphan_payloads"]
    assert ledger.payload_state("p1")[0] is PayloadState.FREE


def test_reconcile_revokes_foreign_boot_permits(tmp_path, cfg, spec_factory):
    from fleet_core import Permit, ResourceId, ResourceKind

    path = tmp_path / "fleet.sqlite3"
    led1 = Ledger(path, sim_t=0.0, wall_t=0.0)
    led1.submit(spec_factory("req-A"), now=1.0, epoch=0)
    rid = ResourceId(ResourceKind.CORRIDOR, "mid")
    led1.add_permit(
        Permit(
            permit_id="permit-old",
            task_id="task-req-A",
            robot_id="r01",
            resources=(rid,),
            boot_id="boot-from-a-previous-life",
            revision=0,
            generation=1,
            epoch=0,
            granted_at=0.0,
            expires_at=1e9,
        )
    )
    led1.close()

    led2 = Ledger(path, sim_t=0.0, wall_t=0.0)
    try:
        report = led2.reconcile(now=50.0, epoch=1)
        assert "permit-old" in report["revoked_permits"]
        assert led2.all_permits() == [], "an unverifiable permit must not survive restart"
    finally:
        led2.close()


def test_unwritable_ledger_fails_closed_rather_than_losing_work(ledger, spec_factory):
    """A ledger that cannot persist must refuse new tasks, not accept and forget."""
    ledger.close()  # simulate the connection going away underneath us
    with pytest.raises(LedgerError):
        ledger.submit(spec_factory("req-B"), now=1.0, epoch=0)
    assert ledger.healthy is False, "an un-writable ledger must report itself unhealthy"


def test_healthy_ledger_reports_healthy(ledger):
    assert ledger.healthy is True


def test_schema_version_present(ledger):
    row = ledger._db.execute("SELECT value FROM meta WHERE key='schema_version'").fetchone()
    assert row is not None


def test_transaction_rolls_back_on_failure(ledger, spec_factory):
    """A failed transaction must leave no partial row behind."""
    task, _ = ledger.submit(spec_factory("req-A"), now=1.0, epoch=0)
    before = len(ledger.all_tasks())
    with pytest.raises(Exception):
        with ledger._tx() as db:
            db.execute(
                "INSERT INTO tasks(task_id, request_id, fingerprint, pick_station,"
                " drop_station, capability, payload_id, state, robot_id, revision, epoch,"
                " created_at, updated_at, reason, detail, cancel_requested)"
                " VALUES('dup','dup','dup','a','b','CARRY',NULL,'SUBMITTED',NULL,0,0,0,0,'OK','',0)"
            )
            raise sqlite3.IntegrityError("forced")
    assert len(ledger.all_tasks()) == before


def test_events_are_append_only(ledger):
    n1 = ledger.append_event(sim_t=0.0, wall_t=0.0, epoch=0, kind="k", subject="s")
    n2 = ledger.append_event(sim_t=1.0, wall_t=1.0, epoch=0, kind="k", subject="s")
    assert n2 > n1
    assert ledger.event_count() == 2
