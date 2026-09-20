"""The ledger must remember what KIND of task it stored.

This file exists because of a defect that a live run found and no unit test could have:
`tasks` had no `kind` column, so every task read back from the ledger came out as
`STATION_TRANSFER`. The dispatcher decides `charging_task` from that field, so a
low-battery robot was refused **its own charge task** for being low -- once per tick,
for ever. The fleet held the pad, the refusal counter climbed, the status page looked
plausible, and nothing was charging.

Why nothing else caught it: `Ledger.submit` returns the in-memory `Task` it just built,
so every test that submits and inspects the return value sees the right `kind`. The
field only disappears when the row is READ BACK, and the only callers that read back
are the dispatcher's `open_tasks()` loop and the status snapshot.
"""

from __future__ import annotations

import sqlite3
import sys
from pathlib import Path

SRC = Path(__file__).resolve().parents[1] / "src"
for _pkg in ("fleet_core", "fleet_adapter"):
    _p = SRC / _pkg
    if str(_p) not in sys.path:
        sys.path.insert(0, str(_p))

from fleet_core import Ledger, PayloadMode, TaskKind, TaskSpec  # noqa: E402


def _spec(request_id: str, kind: TaskKind, **kw) -> TaskSpec:
    base = dict(request_id=request_id, pick_station="S_left_a", drop_station="S_left_b")
    base.update(kw)
    return TaskSpec(kind=kind, **base)


def test_a_charge_task_read_back_is_still_a_charge_task(tmp_path):
    led = Ledger(tmp_path / "fleet.sqlite")
    spec = _spec("auto-charge-r01-1", TaskKind.RETURN_TO_CHARGE, drop_station="C_left")
    task, created = led.submit(spec, now=0.0, epoch=1, task_id=spec.request_id)
    assert created

    back = led.get(task.task_id)
    assert back.spec.kind is TaskKind.RETURN_TO_CHARGE, (
        "a charge task read back from the ledger came out as a station transfer; the "
        "dispatcher decides whether the low-battery rule applies from this field, so it "
        "would refuse the charge task for a robot that is low"
    )
    # and it must survive the path the dispatcher actually uses
    assert led.all_tasks()[0].spec.kind is TaskKind.RETURN_TO_CHARGE


def test_the_fingerprint_round_trips_so_a_replay_is_idempotent(tmp_path):
    """`fingerprint()` includes `kind`, so losing `kind` on read turned a replay into a
    ConflictError against the row's own stored fingerprint."""
    led = Ledger(tmp_path / "fleet.sqlite")
    spec = _spec("auto-charge-r01-1", TaskKind.RETURN_TO_CHARGE, drop_station="C_left")
    task, _ = led.submit(spec, now=0.0, epoch=1, task_id=spec.request_id)

    same, created = led.submit(spec, now=1.0, epoch=1, task_id=spec.request_id)
    assert created is False
    assert same.task_id == task.task_id


def test_payload_mode_round_trips_too(tmp_path):
    """Same shape of defect, same fix: it is part of the fingerprint as well."""
    led = Ledger(tmp_path / "fleet.sqlite")
    spec = _spec("req-physical", TaskKind.STATION_TRANSFER,
                 payload_mode=PayloadMode.PHYSICAL)
    task, _ = led.submit(spec, now=0.0, epoch=1)
    back = led.get(task.task_id)
    assert back.spec.payload_mode is PayloadMode.PHYSICAL
    assert back.spec.fingerprint() == spec.fingerprint()


def test_a_transfer_is_still_a_transfer(tmp_path):
    """The fix must not relabel everything as a charge run."""
    led = Ledger(tmp_path / "fleet.sqlite")
    task, _ = led.submit(_spec("req-1", TaskKind.STATION_TRANSFER), now=0.0, epoch=1)
    assert led.get(task.task_id).spec.kind is TaskKind.STATION_TRANSFER


# --------------------------------------------------------------------------- #
# the migration
# --------------------------------------------------------------------------- #

_OLD_TASKS_DDL = """
CREATE TABLE tasks (
    task_id        TEXT PRIMARY KEY,
    request_id     TEXT NOT NULL,
    fingerprint    TEXT NOT NULL,
    pick_station   TEXT NOT NULL,
    drop_station   TEXT NOT NULL,
    capability     TEXT NOT NULL,
    payload_id     TEXT,
    state          TEXT NOT NULL,
    robot_id       TEXT,
    revision       INTEGER NOT NULL DEFAULT 0,
    epoch          INTEGER NOT NULL DEFAULT 0,
    created_at     REAL NOT NULL,
    updated_at     REAL NOT NULL,
    reason         TEXT NOT NULL DEFAULT 'OK',
    detail         TEXT NOT NULL DEFAULT '',
    cancel_requested INTEGER NOT NULL DEFAULT 0
);
"""


def _columns(path: Path) -> set[str]:
    """Read the schema through a separate connection, so the assertion is about the FILE
    rather than about the object under test's own view of it."""
    con = sqlite3.connect(f"file:{path}?mode=ro", uri=True)
    try:
        return {row[1] for row in con.execute("PRAGMA table_info(tasks)")}
    finally:
        con.close()


def _make_old_ledger(path: Path) -> None:
    """A database written by the version before `kind` existed."""
    con = sqlite3.connect(path)
    con.executescript(_OLD_TASKS_DDL)
    con.execute(
        "INSERT INTO tasks(task_id, request_id, fingerprint, pick_station,"
        " drop_station, capability, payload_id, state, robot_id, revision, epoch,"
        " created_at, updated_at, reason, detail, cancel_requested)"
        " VALUES(?,?,?,?,?,?,?,?,NULL,0,?,?,?,?,?,0)",
        ("auto-charge-r02-1", "auto-charge-r02-1", "old|fingerprint", "S_left_a",
         "C_left", "CARRY", None, "NEEDS_ATTENTION", 1, 0.0, 0.0, "CANCEL_NOT_CONFIRMED",
         "interrupted by a restart"),
    )
    con.commit()
    con.close()


def test_an_older_ledger_is_upgraded_in_place_rather_than_refused(tmp_path):
    """Clearing the database is not recovery (CONTRACTS section 11)."""
    path = tmp_path / "old.sqlite"
    _make_old_ledger(path)

    led = Ledger(path)  # must not raise
    rows = led.all_tasks()
    assert len(rows) == 1, "the existing row must survive the migration"
    assert rows[0].task_id == "auto-charge-r02-1"
    assert rows[0].cancel_requested is False

    # The migration cannot recover what was never written, and it says so rather than
    # guessing: an old charge row reads back as a transfer.
    assert rows[0].spec.kind is TaskKind.STATION_TRANSFER


def test_the_migration_is_idempotent(tmp_path):
    path = tmp_path / "old.sqlite"
    _make_old_ledger(path)
    Ledger(path).close()
    again = Ledger(path)  # second open must not try to add the column twice
    assert len(again.all_tasks()) == 1
    assert {"kind", "payload_mode"} <= _columns(path)


def test_the_migration_records_what_it_added(tmp_path):
    """A schema change that leaves no trace is one nobody can audit later."""
    path = tmp_path / "old.sqlite"
    _make_old_ledger(path)
    Ledger(path)
    con = sqlite3.connect(f"file:{path}?mode=ro", uri=True)
    try:
        row = con.execute(
            "SELECT value FROM meta WHERE key='migrated_columns'").fetchone()
    finally:
        con.close()
    assert row is not None
    assert "kind" in row[0]


def test_a_new_database_needs_no_migration(tmp_path):
    path = tmp_path / "fresh.sqlite"
    Ledger(path)
    con = sqlite3.connect(f"file:{path}?mode=ro", uri=True)
    try:
        row = con.execute(
            "SELECT value FROM meta WHERE key='migrated_columns'").fetchone()
    finally:
        con.close()
    assert row is None, "a database created by this code never needed the legacy columns"
