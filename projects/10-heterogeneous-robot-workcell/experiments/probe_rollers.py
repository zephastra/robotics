"""Real rotating-roller diagnostic. No AMR docking or complete C claim."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import time
import xml.etree.ElementTree as ET

import mujoco
import numpy as np

ROOT = Path(__file__).resolve().parents[1]


def make_model():
    root = ET.Element('mujoco', model='010 roller component probe')
    ET.SubElement(root, 'option', timestep='.002', integrator='implicitfast')
    default = ET.SubElement(root, 'default')
    ET.SubElement(default, 'geom', friction='.8 .005 .0001', condim='4')
    world = ET.SubElement(root, 'worldbody')
    ET.SubElement(world, 'geom', name='floor', type='plane', size='5 5 .1')
    actuators = ET.SubElement(root, 'actuator')
    # Three stationary sections. The middle is NOT claimed to be an AMR.
    for section in range(3):
        for index in range(8):
            x = .08*(section*8+index)
            name = f'roller_{section}_{index}'
            body = ET.SubElement(world, 'body', name=name, pos=f'{x} 0 .45')
            ET.SubElement(body, 'joint', name=name, type='hinge', axis='0 1 0', damping='.01')
            ET.SubElement(body, 'geom', name=name, type='cylinder', size='.035 .22',
                          quat='.70710678 .70710678 0 0', mass='.4')
            ET.SubElement(actuators, 'velocity', name=name, joint=name, kv='2',
                          ctrllimited='true', ctrlrange='-5 5', forcelimited='true', forcerange='-2 2')
    tray = ET.SubElement(world, 'body', name='tray', pos='.20 0 .508')
    ET.SubElement(tray, 'freejoint')
    ET.SubElement(tray, 'geom', name='tray', type='box', size='.12 .15 .02', mass='.3')
    return ET.tostring(root, encoding='unicode')


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--run-id', required=True)
    args = parser.parse_args()
    if Path(args.run_id).name != args.run_id:
        raise ValueError('Invalid run id')
    out = ROOT/'reports'/args.run_id
    out.mkdir(parents=True, exist_ok=False)
    xml = make_model()
    (out/'model.xml').write_text(xml)
    m = mujoco.MjModel.from_xml_string(xml)
    d = mujoco.MjData(m)
    records = []
    start = time.monotonic()
    tray = m.body('tray').id
    max_travel = 0.
    stop_x = None
    status = 'ERROR'
    while d.time < 21:
        if time.monotonic()-start > 45:
            status = 'WALL_TIMEOUT'
            break
        d.ctrl[:] = 3. if 1 <= d.time < 18 else 0.
        mujoco.mj_step(m,d)
        if int(round(d.time/.002)) % 50 == 0:
            pos = d.xpos[tray].copy()
            max_travel = max(max_travel, float(pos[0]-.2))
            contacts = sum(1 for c in d.contact if tray in (m.geom_bodyid[c.geom1],m.geom_bodyid[c.geom2]))
            records.append(dict(t=float(d.time), xyz=pos.tolist(), contacts=contacts))
            if d.time >= 18 and stop_x is None:
                stop_x = float(pos[0])
            if pos[2] < .45:
                status = 'PAYLOAD_FALL'
                break
    else:
        # Final section begins at x=1.28; the entire .24 m tray must be over it.
        contained = 1.40 <= float(d.xpos[tray,0]) <= 1.72 and abs(float(d.xpos[tray,1])) <= .07
        status = 'COMPONENT_PASS' if contained and max_travel > .7 and abs(float(d.xpos[tray,0])-stop_x)<.08 else 'COMPONENT_FAIL'
    report = dict(scope='ROLLER_COMPONENT_ONLY; no mobile support/docking/handshake',
                  status=status, pid=os.getpid(), sim_seconds=float(d.time), wall_seconds=time.monotonic()-start,
                  max_travel_m=max_travel, post_stop_travel_m=None if stop_x is None else float(d.xpos[tray,0])-stop_x,
                  model_sha256=hashlib.sha256(xml.encode()).hexdigest(),
                  script_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
                  mujoco=mujoco.__version__, records=records, full_C_acceptance='NOT_RUN')
    (out/'report.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps({k:v for k,v in report.items() if k!='records'},indent=2))
    return 0 if status=='COMPONENT_PASS' else 1


if __name__ == '__main__':
    raise SystemExit(main())
