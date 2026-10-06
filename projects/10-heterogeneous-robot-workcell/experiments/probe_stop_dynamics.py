"""Diagnose stop transients once; no threshold or world changes."""
import argparse
import json
import time
from pathlib import Path
import numpy as np

import probe_transfer_recovery as recovery
from workcell.runtime_permit import RuntimePermit


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--run-id', required=True)
    args = ap.parse_args()
    out = recovery.ROOT / 'reports' / args.run_id
    out.mkdir(exist_ok=False)
    plant, _ = recovery.build()
    m, d = plant.model, plant.data
    def observe():
        return dict(owner='stop_diagnostic', epoch=1, generation=1, sequence=0,
                    observed_wall_s=time.monotonic(), cancel_requested=d.time >= .3,
                    zone_clear=True, stop_chain_healthy=True)
    plant.motion_guard = RuntimePermit(observe, owner='stop_diagnostic', epoch=1,
                                      generation=1, max_age_s=.5)
    first = plant.transfer(direction='onto_deck', timeout_s=5.)
    body = m.body('c_payload').id
    roller_actuators = [i for i in range(m.nu)
        if (m.actuator(i).name or '').startswith('c_fixed_roller')]
    windows = []
    for _ in range(6):
        q0 = np.array(d.xquat[body])
        brake = plant._stop_transfer()
        brake['orientation_change_rad'] = float(2*np.arccos(np.clip(
            abs(np.dot(q0, d.xquat[body])), 0, 1)))
        brake['roller_velocity_radps'] = [float(d.qvel[m.jnt_dofadr[
            int(m.actuator_trnid[i, 0])]]) for i in roller_actuators]
        windows.append(brake)
        (out/'report.json').write_text(json.dumps(dict(first=first, windows=windows), indent=2)+'\n')
        print(brake['end_s'], brake['tray_held_speed_bound_mps'],
              brake['stopped_confirmed'], flush=True)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
