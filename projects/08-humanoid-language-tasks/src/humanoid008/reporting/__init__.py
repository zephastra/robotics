"""Reporting layer: run directories, manifests, event logs and final reports.

This package only records. It must never influence control decisions, and it
must never be a source of truth that flows back into task execution.
"""

from .events import EventLog
from .report import (
    SCHEMA_VERSION,
    create_run_directory,
    new_run_id,
    utc_stamp,
    write_input,
    write_json,
    write_jsonl,
    write_manifest,
    write_report,
)

__all__ = [
    "EventLog",
    "SCHEMA_VERSION",
    "create_run_directory",
    "new_run_id",
    "utc_stamp",
    "write_input",
    "write_json",
    "write_jsonl",
    "write_manifest",
    "write_report",
]
