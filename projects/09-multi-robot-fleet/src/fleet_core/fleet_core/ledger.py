"""SQLite-backed ledger: the single writer for task, permit and payload state.

Design points that exist for safety, not for convenience:

  * **One writer.** All mutations go through `_tx()`, which serialises on a
    process lock and commits before returning. ROS callbacks must call in here
    rather than touching dicts directly.
  * **Idempotent submit by content.** Same `request_id` + same content is a
    no-op returning the existing task. Same `request_id` + different content is
    a hard conflict, never an overwrite. This survives process restart because
    the dedupe key lives in the database, not in memory.
  * **Atomic payload moves.** `pick()` and `deliver()` each happen in one
    transaction. A crash can therefore never leave a payload both HELD and FREE.
  * **Fail closed on write errors.** If the database cannot be written the
    ledger refuses new work instead of accepting tasks it cannot remember.
"""

from __future__ import annotations

import sqlite3
import threading
from contextlib import contextmanager
from pathlib import Path
from typing import Any, Iterator

from .domain import (
    ConflictError,
    PayloadState,
    Permit,
    ReasonCode,
    ResourceId,
    ResourceKind,
    Task,
    TaskSpec,
    TaskState,
)

SCHEMA_VERSION = 1


class LedgerError(RuntimeError):
    """The ledger cannot be trusted (bad schema, unreadable, un-writable)."""


_SCHEMA = """
CREATE TABLE IF NOT EXISTS meta (
    key   TEXT PRIMARY KEY,
    value TEXT NOT NULL
);

CREATE TABLE IF NOT EXISTS tasks (
    task_id        TEXT PRIMARY KEY,
    request_id     TEXT NOT NULL,
    fingerprint    TEXT NOT NULL,
    -- `kind` and `payload_mode` are part of the task's IDENTITY: they are inputs to
    -- TaskSpec.fingerprint(), and the dispatcher branches on `kind` to decide whether
    -- the low-battery admission rule applies. Leaving them out of the table meant a
    -- charge task read back from the ledger looked like an ordinary transfer, and the
    -- robot that was supposed to charge was refused the charge task for being low.
    kind           TEXT NOT NULL DEFAULT 'station_transfer',
    payload_mode   TEXT NOT NULL DEFAULT 'logical',
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
CREATE UNIQUE INDEX IF NOT EXISTS tasks_request_id ON tasks(request_id);

CREATE TABLE IF NOT EXISTS permits (
    permit_id   TEXT PRIMARY KEY,
    task_id     TEXT NOT NULL,
    robot_id    TEXT NOT NULL,
    resources   TEXT NOT NULL,
    boot_id     TEXT NOT NULL,
    revision    INTEGER NOT NULL,
    generation  INTEGER NOT NULL,
    epoch       INTEGER NOT NULL,
    granted_at  REAL NOT NULL,
    expires_at  REAL NOT NULL,
    cleared     INTEGER NOT NULL DEFAULT 0
);
CREATE INDEX IF NOT EXISTS permits_task ON permits(task_id);

CREATE TABLE IF NOT EXISTS payloads (
    payload_id TEXT PRIMARY KEY,
    state      TEXT NOT NULL,
    holder     TEXT,
    task_id    TEXT
);

CREATE TABLE IF NOT EXISTS events (
    seq        INTEGER PRIMARY KEY AUTOINCREMENT,
    sim_t      REAL NOT NULL,
    wall_t     REAL NOT NULL,
    epoch      INTEGER NOT NULL,
    kind       TEXT NOT NULL,
    subject    TEXT NOT NULL,
    data       TEXT NOT NULL
);
"""


class Ledger:
    def __init__(self, path: str | Path, *, sim_t: float = 0.0, wall_t: float = 0.0) -> None:
        self.path = str(path)
        self._lock = threading.RLock()
        self._healthy = True
        self._closed = False
        try:
            # check_same_thread=False is a requirement here, not a shortcut.
            #
            # This class already serialises every access on `self._lock`, and that is
            # exactly the condition under which sqlite's thread check may be waived.
            # It HAS to be waived, because the ROS nodes run on a MultiThreadedExecutor
            # and therefore call the ledger from several threads: the connection is
            # opened in the thread that constructs the node, and the first service or
            # timer callback that touches it arrives on another thread. Without this,
            # sqlite raises ProgrammingError and the node dies -- taking the whole
            # task layer with it, on the first `status` call.
            #
            # The alternative, marshalling every ledger call onto a single thread,
            # would put a queue between the ledger and its callers for no gain: the
            # lock is already the serialisation point, and the ledger is deliberately
            # the single writer.
            self._db = sqlite3.connect(
                self.path, isolation_level=None, timeout=10.0, check_same_thread=False
            )
            self._db.row_factory = sqlite3.Row
            self._db.execute("PRAGMA journal_mode=WAL")
            self._db.execute("PRAGMA foreign_keys=ON")
            self._db.executescript(_SCHEMA)
            self._migrate()
            self._ensure_version()
        except sqlite3.Error as exc:  # pragma: no cover - environment specific
            raise LedgerError(f"cannot open ledger at {self.path}: {exc}") from exc
        self._boot_id = f"boot-{abs(hash((self.path, sim_t, wall_t))) % 10**8:08d}"
        self._generation = 0

    # ------------------------------------------------------------------ #
    # plumbing
    # ------------------------------------------------------------------ #

    @property
    def boot_id(self) -> str:
        """Identity of this process's ledger instance; changes every restart."""
        return self._boot_id

    @property
    def healthy(self) -> bool:
        return self._healthy and not self._closed

    #: Columns added after the first databases were written. Additive and defaulted,
    #: so an existing ledger is upgraded in place rather than refused -- CONTRACTS
    #: section 11 is explicit that clearing the database is not recovery.
    _ADDED_COLUMNS = (
        ("kind", "TEXT NOT NULL DEFAULT 'station_transfer'"),
        ("payload_mode", "TEXT NOT NULL DEFAULT 'logical'"),
    )

    def _migrate(self) -> None:
        """Add columns an older database does not have.

        Honest about what it cannot recover: rows written before `kind` existed are
        relabelled `station_transfer`, and for a charge row that is simply wrong. It is
        still better than refusing to open the file, because the tasks in it are parked
        for a human either way and the alternative is losing the whole record.
        """
        present = {row[1] for row in self._db.execute("PRAGMA table_info(tasks)")}
        added = []
        for name, decl in self._ADDED_COLUMNS:
            if name in present:
                continue
            self._db.execute(f"ALTER TABLE tasks ADD COLUMN {name} {decl}")
            added.append(name)
        if added:
            self._db.execute(
                "INSERT OR REPLACE INTO meta(key, value) VALUES(?, ?)",
                ("migrated_columns", ",".join(added)),
            )
        return None

    def _ensure_version(self) -> None:
        row = self._db.execute("SELECT value FROM meta WHERE key='schema_version'").fetchone()
        if row is None:
            self._db.execute(
                "INSERT INTO meta(key, value) VALUES('schema_version', ?)",
                (str(SCHEMA_VERSION),),
            )
            return
        if int(row["value"]) != SCHEMA_VERSION:
            raise LedgerError(
                f"schema_version mismatch: db has {row['value']}, code expects {SCHEMA_VERSION}"
            )

    @contextmanager
    def _tx(self) -> Iterator[sqlite3.Connection]:
        """Serialise + commit. Any sqlite error marks the ledger unhealthy."""
        with self._lock:
            if not self.healthy:
                raise LedgerError("ledger is not healthy; refusing to write")
            try:
                self._db.execute("BEGIN IMMEDIATE")
                yield self._db
                self._db.execute("COMMIT")
            except sqlite3.Error as exc:
                try:
                    self._db.execute("ROLLBACK")
                except sqlite3.Error:
                    pass
                # Fail closed: we can no longer promise what we have persisted.
                self._healthy = False
                raise LedgerError(f"ledger write failed, now unhealthy: {exc}") from exc

    def close(self) -> None:
        with self._lock:
            if not self._closed:
                self._db.close()
                self._closed = True

    def _read(self, sql: str, params: tuple = ()) -> sqlite3.Cursor:
        """Reads fail closed too.

        A ledger that cannot be read cannot be trusted, so a read error marks it
        unhealthy and surfaces as LedgerError rather than a bare sqlite error
        leaking out of whichever caller happened to touch it first.
        """
        with self._lock:
            try:
                return self._db.execute(sql, params)
            except sqlite3.Error as exc:
                self._healthy = False
                raise LedgerError(f"ledger read failed, now unhealthy: {exc}") from exc

    def next_generation(self) -> int:
        """A new command generation. Only ever increases."""
        with self._lock:
            self._generation += 1
            return self._generation

    # ------------------------------------------------------------------ #
    # events
    # ------------------------------------------------------------------ #

    def append_event(
        self, *, sim_t: float, wall_t: float, epoch: int, kind: str, subject: str, data: str = "{}"
    ) -> int:
        with self._tx() as db:
            cur = db.execute(
                "INSERT INTO events(sim_t, wall_t, epoch, kind, subject, data) "
                "VALUES(?,?,?,?,?,?)",
                (sim_t, wall_t, epoch, kind, subject, data),
            )
            return int(cur.lastrowid or 0)

    def event_count(self) -> int:
        with self._lock:
            return int(self._db.execute("SELECT COUNT(*) AS n FROM events").fetchone()["n"])

    # ------------------------------------------------------------------ #
    # tasks
    # ------------------------------------------------------------------ #

    def find_by_request_id(self, request_id: str) -> Task | None:
        row = self._read("SELECT * FROM tasks WHERE request_id=?", (request_id,)).fetchone()
        return None if row is None else _row_to_task(row)

    def submit(
        self,
        spec: TaskSpec,
        *,
        now: float,
        epoch: int,
        task_id: str | None = None,
    ) -> tuple[Task, bool]:
        """Idempotent submit.

        Returns `(task, created)`. `created=False` means an identical request was
        already present -- replaying it changes nothing. A different payload for
        the same request_id raises ConflictError rather than overwriting.
        """
        with self._lock:
            existing = self.find_by_request_id(spec.request_id)
            if existing is not None:
                if existing.spec.fingerprint() == spec.fingerprint():
                    return existing, False
                raise ConflictError(
                    f"request_id {spec.request_id!r} already exists with different content "
                    f"({ReasonCode.REQUEST_CONFLICT.value})"
                )

            tid = task_id or f"task-{spec.request_id}"
            with self._tx() as db:
                db.execute(
                    "INSERT INTO tasks(task_id, request_id, fingerprint, kind,"
                    " payload_mode, pick_station, drop_station, capability, payload_id,"
                    " state, robot_id, revision, epoch, created_at, updated_at, reason,"
                    " detail, cancel_requested)"
                    " VALUES(?,?,?,?,?,?,?,?,?,?,NULL,0,?,?,?,?,?,0)",
                    (
                        tid,
                        spec.request_id,
                        spec.fingerprint(),
                        spec.kind.value,
                        spec.payload_mode.value,
                        spec.pick_station,
                        spec.drop_station,
                        spec.required_capability.value,
                        spec.payload_id,
                        TaskState.SUBMITTED.value,
                        epoch,
                        now,
                        now,
                        ReasonCode.OK.value,
                        "",
                    ),
                )
                if spec.payload_id:
                    # Reserve, never assume HELD. HELD is only ever set by pick().
                    db.execute(
                        "INSERT OR IGNORE INTO payloads(payload_id, state, holder, task_id)"
                        " VALUES(?,?,NULL,?)",
                        (spec.payload_id, PayloadState.FREE.value, tid),
                    )
            return self.get(tid), True

    def get(self, task_id: str) -> Task:
        row = self._read("SELECT * FROM tasks WHERE task_id=?", (task_id,)).fetchone()
        if row is None:
            raise KeyError(task_id)
        return _row_to_task(row)

    def all_tasks(self) -> list[Task]:
        with self._lock:
            rows = self._db.execute("SELECT * FROM tasks ORDER BY created_at, task_id").fetchall()
        return [_row_to_task(r) for r in rows]

    def open_tasks(self) -> list[Task]:
        return [t for t in self.all_tasks() if t.is_open()]

    def set_state(
        self,
        task_id: str,
        state: TaskState,
        *,
        now: float,
        reason: ReasonCode = ReasonCode.OK,
        detail: str = "",
        robot_id: str | None = None,
        bump_revision: bool = True,
    ) -> Task:
        with self._tx() as db:
            row = db.execute("SELECT * FROM tasks WHERE task_id=?", (task_id,)).fetchone()
            if row is None:
                raise KeyError(task_id)
            revision = int(row["revision"]) + (1 if bump_revision else 0)
            db.execute(
                "UPDATE tasks SET state=?, reason=?, detail=?, updated_at=?, revision=?,"
                " robot_id=COALESCE(?, robot_id) WHERE task_id=?",
                (state.value, reason.value, detail, now, revision, robot_id, task_id),
            )
            # A task that stops holding a payload must release it in the same
            # transaction -- otherwise a crash can leave a phantom holder.
            if state in (TaskState.CANCELED, TaskState.FAILED):
                db.execute(
                    "UPDATE payloads SET state=?, holder=NULL WHERE task_id=? AND state=?",
                    (PayloadState.FREE.value, task_id, PayloadState.RESERVED.value),
                )
        return self.get(task_id)

    def request_cancel(self, task_id: str, *, now: float) -> Task:
        with self._tx() as db:
            row = db.execute("SELECT * FROM tasks WHERE task_id=?", (task_id,)).fetchone()
            if row is None:
                raise KeyError(task_id)
            db.execute(
                "UPDATE tasks SET cancel_requested=1, updated_at=? WHERE task_id=?",
                (now, task_id),
            )
        return self.get(task_id)

    # ------------------------------------------------------------------ #
    # payloads -- atomic claim/handover
    # ------------------------------------------------------------------ #

    def payload_state(self, payload_id: str) -> tuple[PayloadState, str | None]:
        with self._lock:
            row = self._db.execute(
                "SELECT state, holder FROM payloads WHERE payload_id=?", (payload_id,)
            ).fetchone()
        if row is None:
            raise KeyError(payload_id)
        return PayloadState(row["state"]), row["holder"]

    def payload_holders(self) -> dict[str, str]:
        """payload_id -> robot_id for every payload currently HELD.

        Read-only, and the single source for "which robot is carrying something". The
        dispatcher needs it to refuse loading a second cargo onto a robot that already
        has one. Without that guard a faulted robot keeps its cargo, gets reallocated,
        and silently ends up holding two: both rows read HELD, the ledger looks
        consistent, and the first cargo is never delivered.
        """
        with self._lock:
            rows = self._db.execute(
                "SELECT payload_id, holder FROM payloads "
                "WHERE state=? AND holder IS NOT NULL",
                (PayloadState.HELD.value,),
            ).fetchall()
        return {row["payload_id"]: row["holder"] for row in rows}

    def pick(self, payload_id: str, *, task_id: str, robot_id: str) -> bool:
        """Atomically claim a payload. False if someone else already holds it."""
        with self._tx() as db:
            row = db.execute(
                "SELECT state, holder FROM payloads WHERE payload_id=?", (payload_id,)
            ).fetchone()
            if row is None:
                raise KeyError(payload_id)
            if row["state"] == PayloadState.HELD.value and row["holder"] != robot_id:
                return False
            db.execute(
                "UPDATE payloads SET state=?, holder=?, task_id=? WHERE payload_id=?",
                (PayloadState.HELD.value, robot_id, task_id, payload_id),
            )
        return True

    def deliver(self, payload_id: str, *, task_id: str, robot_id: str) -> bool:
        with self._tx() as db:
            row = db.execute(
                "SELECT state, holder FROM payloads WHERE payload_id=?", (payload_id,)
            ).fetchone()
            if row is None:
                raise KeyError(payload_id)
            if row["state"] == PayloadState.DELIVERED.value:
                return False  # already delivered -- never double-deliver
            if row["holder"] != robot_id:
                return False
            db.execute(
                "UPDATE payloads SET state=?, holder=NULL, task_id=? WHERE payload_id=?",
                (PayloadState.DELIVERED.value, task_id, payload_id),
            )
        return True

    def transfer_payload(self, payload_id: str, *, to_robot: str, task_id: str) -> bool:
        """Hand a payload over. Refused while it is HELD -- a held payload is not
        a free resource, so a failing robot must not leak it to another one."""
        with self._tx() as db:
            row = db.execute(
                "SELECT state, holder FROM payloads WHERE payload_id=?", (payload_id,)
            ).fetchone()
            if row is None:
                raise KeyError(payload_id)
            if row["state"] == PayloadState.HELD.value:
                return False
            if row["state"] == PayloadState.DELIVERED.value:
                return False
            db.execute(
                "UPDATE payloads SET state=?, holder=?, task_id=? WHERE payload_id=?",
                (PayloadState.RESERVED.value, to_robot, task_id, payload_id),
            )
        return True

    # ------------------------------------------------------------------ #
    # permits
    # ------------------------------------------------------------------ #

    def add_permit(self, permit: Permit) -> None:
        with self._tx() as db:
            db.execute(
                "INSERT INTO permits(permit_id, task_id, robot_id, resources, boot_id,"
                " revision, generation, epoch, granted_at, expires_at, cleared)"
                " VALUES(?,?,?,?,?,?,?,?,?,?,?)",
                (
                    permit.permit_id,
                    permit.task_id,
                    permit.robot_id,
                    ",".join(str(r) for r in permit.resources),
                    permit.boot_id,
                    permit.revision,
                    permit.generation,
                    permit.epoch,
                    permit.granted_at,
                    permit.expires_at,
                    1 if permit.cleared else 0,
                ),
            )

    def permits_for_task(self, task_id: str) -> list[Permit]:
        with self._lock:
            rows = self._db.execute(
                "SELECT * FROM permits WHERE task_id=? ORDER BY granted_at", (task_id,)
            ).fetchall()
        return [_row_to_permit(r) for r in rows]

    def all_permits(self) -> list[Permit]:
        with self._lock:
            rows = self._db.execute("SELECT * FROM permits ORDER BY granted_at").fetchall()
        return [_row_to_permit(r) for r in rows]

    def mark_cleared(self, permit_id: str) -> bool:
        """Record an *explicit* proof that the protected area is empty."""
        with self._tx() as db:
            cur = db.execute("UPDATE permits SET cleared=1 WHERE permit_id=?", (permit_id,))
            return cur.rowcount > 0

    # ------------------------------------------------------------------ #
    # restart reconciliation
    # ------------------------------------------------------------------ #

    def reconcile(self, *, now: float, epoch: int) -> dict[str, Any]:
        """After a restart, decide what to do with whatever was in flight.

        Conservative by policy: nothing is assumed finished, nothing is silently
        resumed, and permits whose owner cannot be confirmed are revoked rather
        than trusted. Returns a report the caller must surface.
        """
        report: dict[str, Any] = {
            "epoch": epoch,
            "resumed": [],
            "revoked_permits": [],
            "orphan_payloads": [],
            "open_tasks": [],
        }
        with self._tx() as db:
            # Any permit from a previous boot is unverifiable -> revoke.
            rows = db.execute("SELECT * FROM permits WHERE boot_id != ?", (self._boot_id,)).fetchall()
            for r in rows:
                report["revoked_permits"].append(r["permit_id"])
            db.execute("DELETE FROM permits WHERE boot_id != ?", (self._boot_id,))

            # Payloads claimed by a task that no longer exists are orphans.
            held = db.execute(
                "SELECT payload_id, holder, task_id FROM payloads WHERE state=?",
                (PayloadState.HELD.value,),
            ).fetchall()
            for r in held:
                trow = db.execute(
                    "SELECT state FROM tasks WHERE task_id=?", (r["task_id"],)
                ).fetchone()
                if trow is None or trow["state"] in (
                    TaskState.CANCELED.value,
                    TaskState.FAILED.value,
                    TaskState.SUCCEEDED.value,
                ):
                    report["orphan_payloads"].append(r["payload_id"])
                    db.execute(
                        "UPDATE payloads SET state=?, holder=NULL WHERE payload_id=?",
                        (PayloadState.FREE.value, r["payload_id"]),
                    )

            open_rows = db.execute(
                "SELECT task_id, state FROM tasks WHERE state NOT IN (?,?,?)",
                (TaskState.SUCCEEDED.value, TaskState.FAILED.value, TaskState.CANCELED.value),
            ).fetchall()
            for r in open_rows:
                report["open_tasks"].append({"task_id": r["task_id"], "state": r["state"]})
        return report


# --------------------------------------------------------------------------- #
# row helpers
# --------------------------------------------------------------------------- #


def _row_to_task(row: sqlite3.Row) -> Task:
    from .domain import Capability, PayloadMode, TaskKind

    # `kind` and `payload_mode` are read back, not defaulted. Defaulting them here is
    # what made a charge task indistinguishable from a transfer once it had been
    # through the database -- and the dispatcher decides whether the low-battery
    # admission rule applies from exactly this field.
    spec = TaskSpec(
        request_id=row["request_id"],
        pick_station=row["pick_station"],
        drop_station=row["drop_station"],
        required_capability=Capability(row["capability"]),
        payload_id=row["payload_id"],
        kind=TaskKind(row["kind"]),
        payload_mode=PayloadMode(row["payload_mode"]),
    )
    return Task(
        task_id=row["task_id"],
        spec=spec,
        state=TaskState(row["state"]),
        robot_id=row["robot_id"],
        revision=int(row["revision"]),
        epoch=int(row["epoch"]),
        created_at=float(row["created_at"]),
        updated_at=float(row["updated_at"]),
        reason=ReasonCode(row["reason"]),
        detail=row["detail"] or "",
        cancel_requested=bool(row["cancel_requested"]),
    )


def _row_to_permit(row: sqlite3.Row) -> Permit:
    resources: list[ResourceId] = []
    for token in (row["resources"] or "").split(","):
        token = token.strip()
        if not token:
            continue
        kind, _, name = token.partition(":")
        resources.append(ResourceId(ResourceKind(kind), name))
    return Permit(
        permit_id=row["permit_id"],
        task_id=row["task_id"],
        robot_id=row["robot_id"],
        resources=tuple(resources),
        boot_id=row["boot_id"],
        revision=int(row["revision"]),
        generation=int(row["generation"]),
        epoch=int(row["epoch"]),
        granted_at=float(row["granted_at"]),
        expires_at=float(row["expires_at"]),
        cleared=bool(row["cleared"]),
    )
