"""Summarize a run report.

Defaults to the newest ``reports/*/report.json``; pass ``--report`` to pick one
explicitly (MASTER_PLAN section 18: never guess the last directory in scripts).
"""

import argparse
import glob
import json
import os


def main():
    parser = argparse.ArgumentParser(description="Summarize a run report")
    parser.add_argument("--report", default=None, help="path to report.json")
    args = parser.parse_args()
    path = args.report
    if path is None:
        matches = sorted(glob.glob("reports/*/report.json"), key=os.path.getmtime)
        if not matches:
            print("no reports found")
            return 1
        path = matches[-1]
    report = json.load(open(path))
    print(f"report: {path}")
    print(
        f"status: {report.get('status')}  reason: {report.get('reason_code')}  "
        f"physical: {report.get('physical_outcome')}  "
        f"exec_backend: {report.get('execution_backend')}  "
        f"planner: {report.get('planner_backend')}  clock: {report.get('clock_mode')}"
    )
    for s in report.get("skill_results", []):
        print(f"  {s['skill']:18s} {s['status']:10s} {s.get('reason_code', '')}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
