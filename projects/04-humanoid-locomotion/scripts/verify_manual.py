"""Opt-in GUI integration test of real Tk bindings + the actual simulation loop.

Generates one W press, sustains it without autorepeat, releases, then cancels.
Only our own control window receives events. No global input hooks or hardware.
"""
import argparse
import json
from pathlib import Path
import sys
from unittest.mock import patch

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src'))
from humanoid004 import app
from humanoid004.manual_panel import ManualPanel


class TestPanel(ManualPanel):
    def __init__(self,*args):
        super().__init__(*args)
        self.stage=0

    def status(self,now,phase,command):
        super().status(now,phase,command)
        # Native Tk event dispatch reaches the same callbacks as user input.
        if self.stage==0 and now>=2.5:
            self.root.event_generate('<KeyPress>',keysym='w')
            self.stage=1
        elif self.stage==1 and now>=7.5:
            self.root.event_generate('<KeyRelease>',keysym='w')
            self.stage=2
        elif self.stage==2 and now>=11:
            self.root.event_generate('<KeyPress>',keysym='c')
            self.stage=3


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--robot',choices=['g1','t800'],default='t800')
    parser.add_argument('--no-viewer',action='store_true',help='Keep the real Tk panel and physics, but omit the MuJoCo viewer for isolated GUI tests')
    option=parser.parse_args()
    args=argparse.Namespace(robot=option.robot,mode='manual',headless=option.no_viewer,realtime=True,
                            snapshot=True,duration=45.,command=[0,0,0],seed=0,vary_initial=False,
                            cancel_at=None,push_at=8.,push_force=30.,push_duration=.2)
    before=set((ROOT/'reports').glob('*/report.json'))
    with patch.object(app,'ManualPanel',TestPanel):
        code=app.run(args)
    created=set((ROOT/'reports').glob('*/report.json'))-before
    matching=[p for p in created if json.loads(p.read_text()).get('robot')==option.robot
              and json.loads(p.read_text()).get('mode')=='manual']
    path=max(matching,key=lambda p:p.stat().st_mtime)
    report=json.loads(path.read_text())
    import csv
    with (path.parent/'trajectory.csv').open() as stream:
        rows=list(csv.DictReader(stream))
    held=[r for r in rows if 3<=float(r['t'])<=7]
    stopped=[r for r in rows if 8<=float(r['t'])<=10.5]
    checks={'one_w_press':sum(e.get('event')=='KEY_DOWN' and e.get('key')=='W' for e in report['input_events'])==1,
            'sustained_command':bool(held) and all(float(r['input_vx'])==.4 for r in held),
            'released_command':bool(stopped) and all(float(r['input_vx'])==0 for r in stopped),
            'actual_forward_motion':max(float(r['x']) for r in rows)>.6,
            'controlled_stop':bool(report.get('stop_check',{}).get('passed'))}
    evidence=dict(robot=option.robot,report=str(path.relative_to(ROOT)),checks=checks,
                  passed=code==0 and all(checks.values()),
                  scope='Programmatically generated Tk events and actual GUI simulation, not a human keyboard acceptance test')
    (path.parent/'manual-input-verification.json').write_text(json.dumps(evidence,indent=2)+'\n')
    print(json.dumps(evidence,indent=2),flush=True)
    return 0 if evidence['passed'] else 1


if __name__=='__main__':
    raise SystemExit(main())
