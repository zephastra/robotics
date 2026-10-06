"""Read-only raw initialized loaded-nav forensics; no simulation or control."""
import argparse
import json
import math
from pathlib import Path

ROOT=Path(__file__).resolve().parents[1]


def summarize(report):
    rows=report.get('truth_judge_only',[])
    initial=rows[0]['tray_in_deck_judge_only'] if rows else None
    def slip(row):
        return math.dist(row['tray_in_deck_judge_only'][:2],initial[:2])
    first=next((r for r in rows if initial and slip(r)>.005),None)
    refusal=next((r for r in report.get('sensor_frames',[])
        if r.get('sim_s',0)>6 and (r.get('observation',{}).get('status')!='RESOLVED'
            or r.get('observation',{}).get('support')!='deck')),None)
    t=refusal['sim_s'] if refusal else first['sim_s'] if first else None
    selected=[r for r in rows if t is not None and t-.5<=r['sim_s']<=t+.5]
    return dict(scope='RAW_FORENSICS_NOT_ACCEPTANCE',
        initialization_only=True,first_5mm_crossing=first,
        first_sensor_refusal=refusal,near_refusal=selected,
        motion_refusal=report.get('motion_refusal'),
        original_report_result=report.get('commissioning_result'),
        full_order='NOT_RUN',v1_complete=False)


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--input',required=True)
    a=p.parse_args();path=Path(a.input)
    if not path.is_absolute():path=ROOT/path
    print(json.dumps(summarize(json.loads(path.read_text())),indent=2))


if __name__=='__main__':main()
