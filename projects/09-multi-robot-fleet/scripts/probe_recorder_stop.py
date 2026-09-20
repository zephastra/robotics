#!/usr/bin/env python3
"""Does stopping the recorder produce a recording, or an empty directory?

WHY THIS EXISTS
---------------
Measured before it was fixed: `ros2 run` handles SIGINT itself AND forwards it to the child,
so the recorder received it twice. The first one raised out of the sleep, the second landed
inside `write_text` and killed the flush -- exit 254, zero files, no samples. SIGTERM was not
handled at all, so a supervisor that used it got zero files too. That combination means the
ONLY way to stop this recorder without losing the recording was a single clean SIGINT, and the
failure mode is an empty directory rather than an error: a witness that returns nothing, which
is indistinguishable from a run in which nothing was inside a protected region.

So this probe stops the recorder four ways and requires a flushed, non-empty recording from
each. It needs no simulator and no fleet: the recorder appends one sample per tick whether or
not any robot is publishing, and the file is the only thing under test.

    scripts/probe_recorder_stop.py            # all four
    scripts/probe_recorder_stop.py --verbose
"""

from __future__ import annotations

import argparse
import json
import os
import pathlib
import signal
import subprocess
import sys
import time

ROOT = pathlib.Path(__file__).resolve().parents[1]

#: (name, how to stop, how long to run first). `killpg` reproduces the double delivery that
#: `ros2 run` produces, which is the case that lost the recording.
STOPS = (
    ("sigterm_to_process", "terminate", 6.0),
    ("sigint_to_process_group", "killpg_int", 6.0),
    ("sigint_to_process_only", "signal_int", 6.0),
    ("duration_expiry", "duration", 4.0),
)


def one(name: str, how: str, run_s: float, out_root: pathlib.Path,
        verbose: bool) -> dict:
    out = out_root / name
    out.mkdir(parents=True, exist_ok=True)
    duration = "4.0" if how == "duration" else "0"
    cmd = ["ros2", "run", "fleet_evaluation", "fleet_recorder",
           "--robots", "r01", "--out", str(out), "--label", name,
           "--sample-hz", "5.0", "--duration", duration]
    proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                            text=True, start_new_session=True)
    time.sleep(run_s)
    alive_before = proc.poll() is None

    if how == "terminate":
        proc.terminate()
    elif how == "killpg_int":
        os.killpg(os.getpgid(proc.pid), signal.SIGINT)
    elif how == "signal_int":
        proc.send_signal(signal.SIGINT)
    # "duration": it stops by itself

    killed = False
    try:
        proc.wait(timeout=20.0)
    except subprocess.TimeoutExpired:
        os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
        proc.wait(timeout=10.0)
        killed = True

    files = sorted(p.name for p in out.glob("samples-*.jsonl"))
    samples = 0
    audit: dict = {}
    if files:
        lines = [ln for ln in (out / files[0]).read_text(encoding="utf-8").splitlines()
                 if ln.strip()]
        samples = len(lines)
        audit_file = out / files[0].replace("samples-", "recorder-").replace(
            ".jsonl", ".json")
        if audit_file.is_file():
            audit = json.loads(audit_file.read_text(encoding="utf-8"))

    result = {
        "stop": name,
        "how": how,
        "alive_before_stop": alive_before,
        "had_to_sigkill": killed,
        "exit_code": proc.returncode,
        "samples_file": files[0] if files else "",
        "samples": samples,
        # The book columns: the recorder now polls the coordinator as well as the task
        # service, and this is where the absence of that shows up.
        "audit_book_available": audit.get("book_available"),
        "audit_claim_available": audit.get("claim_available"),
        "flushed": bool(files) and samples > 0,
    }
    if not result["flushed"]:
        result["stdout_tail"] = (proc.stdout.read() or "").strip().splitlines()[-8:]
    elif verbose:
        result["stdout_tail"] = (proc.stdout.read() or "").strip().splitlines()[-3:]
    return result


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--out", default="")
    ap.add_argument("--verbose", action="store_true")
    args = ap.parse_args(argv)

    stamp = time.strftime("%Y%m%dT%H%M%SZ", time.gmtime())
    out_root = pathlib.Path(args.out) if args.out else ROOT / "reports" / \
        f"recorder_stop_{stamp}"
    out_root.mkdir(parents=True, exist_ok=True)
    print(f"probe_recorder_stop: {out_root}")

    results = []
    for name, how, run_s in STOPS:
        entry = one(name, how, run_s, out_root, args.verbose)
        results.append(entry)
        mark = "PASS" if entry["flushed"] else "FAIL"
        print(f"  [{mark}] {name:<28} exit={entry['exit_code']!s:<6} "
              f"samples={entry['samples']:<4} book={entry['audit_book_available']} "
              f"claim={entry['audit_claim_available']}")
        if not entry["flushed"]:
            for line in entry.get("stdout_tail") or []:
                print(f"          | {line}")

    (out_root / "probe.json").write_text(json.dumps(results, indent=2, sort_keys=True),
                                         encoding="utf-8")
    bad = [r["stop"] for r in results if not r["flushed"]]
    print(f"  RESULT: {len(results) - len(bad)}/{len(results)} stop methods produced a "
          f"recording" + (f"; lost: {bad}" if bad else ""))
    return 0 if not bad else 5


if __name__ == "__main__":
    raise SystemExit(main())
