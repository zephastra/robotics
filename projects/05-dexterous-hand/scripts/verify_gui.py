"""Actual Tk widgets + MuJoCo viewer test, not human WSLg input acceptance."""
import argparse
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'src'))
from hand005 import app, panel

Base = panel.Panel


class TestPanel(Base):
    def __init__(self, ranges):
        super().__init__(ranges)
        self.checked = []
        self.root.after(150, self.test_widgets)
        self.root.after(1500, lambda: self.buttons['Run grasp demo'].invoke())

    def test_widgets(self):
        self.buttons['2 Power pose'].invoke()
        assert self.target[1] > .7
        self.buttons['1 Open'].invoke()
        assert self.target[1] == 0.
        self.slider(0, .1)
        assert self.target[0] == .1
        self.buttons['1 Open'].invoke()
        self.pause()
        assert self.paused
        self.pause()
        self.checked = ['power_button', 'open_button', 'joint_target', 'pause_resume']

    def display(self, now, phase, obs):
        super().display(now, phase, obs)
        if phase in ('COMPLETED', 'FAILED'):
            self.close()


def main():
    instances = []
    def factory(ranges):
        p = TestPanel(ranges)
        instances.append(p)
        return p
    panel.Panel = factory
    report, out = app.run(argparse.Namespace(mode='manual', headless=False, keep_open=True,
                                             snapshot=True, duration=35., seed=0, offset=0., mass=.05))
    checks = dict(widgets=len(instances[0].checked)==4, demo_button=any(
        e['action']=='DEMO_ACCEPTED' for e in report['events']), completed=any(
            trial['result']=='COMPLETED' for trial in report['trials']))
    result = dict(checks=checks, passed=all(checks.values()),
                  scope='Programmatically invoked real Tk widgets on WSLg, with actual MuJoCo viewer; not human mouse acceptance')
    (out/'gui-verification.json').write_text(json.dumps(result, indent=2)+'\n')
    print(result)
    raise SystemExit(0 if result['passed'] else 1)


if __name__ == '__main__':
    main()
