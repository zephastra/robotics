"""Independent v6 visible edge-pressure candidate; static audit NOT qualification.

User approved measuring first, then the smallest suitable retaining mechanism.
The measured 2ms/micrometer contact gaps do not justify a large clamp. Two
releasable edge shoes address bounded preload/slip separately from deck yaw.
Old worlds, contact parameters, cargo freedom and acceptance thresholds stay.
"""
import argparse
import hashlib
import json
import re
import xml.etree.ElementTree as ET
from pathlib import Path
import mujoco
import numpy as np
from build_p5_candidate_world import compile_text

ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT/'assets/world_p5_candidate_v5.xml'
TARGET = ROOT/'assets/world_p5_candidate_v6_retainer.xml'
# New declared fixture dimensions, NOT acceptance tolerances.
TRAY_CENTER_IN_DECK_X = .12  # measured/declarative source-transfer setup
RELEASE_TRAVEL = .03
PRELOAD_TARGET = -.001
FORCE_LIMIT = 2.  # each shoe, candidate only; physical qualification required


def build(raw):
    root = ET.fromstring(re.sub(r'<!--.*?-->', '', raw, flags=re.S))
    deck = next(b for b in root.iter('body') if b.get('name') == 'c_deck')
    tray = next(b for b in root.iter('body') if b.get('name') == 'c_payload')
    wall = tray.find("geom[@name='c_tray_wall_yp']")
    pos = np.fromstring(wall.get('pos'), sep=' ')
    size = np.fromstring(wall.get('size'), sep=' ')
    # Original floor underside -0.04 rests at deck crown local z=0.
    # Derive wall contact height from catalog geometry, not runtime cargo truth.
    floor = tray.find("geom[@name='c_tray_floor']")
    underside = float(floor.get('pos').split()[2])-float(floor.get('size').split()[2])
    rim_top = pos[2]+size[2]-underside
    shoe_half = [.035, float(size[1])*.75, .003]
    actuator = root.find('actuator')
    if actuator is None: actuator = ET.SubElement(root, 'actuator')
    added = []
    for side in (-1, 1):
        name = f'c_retainer_{side}'
        # Visible outriggers clear original handle outer edge ±0.335m.
        ET.SubElement(deck, 'geom', name=name+'_mast', type='capsule',
            fromto=f'{TRAY_CENTER_IN_DECK_X} {side*.36} -.07 {TRAY_CENTER_IN_DECK_X} {side*.36} .12',
            size='.006', mass='.025', rgba='.8 .6 .1 1')
        ET.SubElement(deck, 'geom', name=name+'_beam', type='capsule',
            fromto=f'{TRAY_CENTER_IN_DECK_X} {side*.36} .12 {TRAY_CENTER_IN_DECK_X} {side*pos[1]} .12',
            size='.005', mass='.015', rgba='.8 .6 .1 1')
        shoe = ET.SubElement(deck, 'body', name=name,
            pos=f'{TRAY_CENTER_IN_DECK_X} {side*pos[1]} {rim_top+shoe_half[2]+RELEASE_TRAVEL}')
        ET.SubElement(shoe, 'joint', name=name+'_joint', type='slide', axis='0 0 1',
            ref=str(RELEASE_TRAVEL), range=f'-.002 {RELEASE_TRAVEL}', limited='true', damping='.5')
        ET.SubElement(shoe, 'geom', name=name+'_shoe', type='box',
            size=' '.join(map(str, shoe_half)), mass='.025', friction='.4 .005 .001',
            condim='4', rgba='.9 .65 .1 1')
        ET.SubElement(actuator, 'position', name=name+'_drive', joint=name+'_joint',
            kp='500', ctrllimited='true', ctrlrange=f'-.002 {RELEASE_TRAVEL}',
            forcelimited='true', forcerange=f'-{FORCE_LIMIT} {FORCE_LIMIT}')
        added.append(name)
    # Remap old keyframes by joint identity/body ordinal; new actuators appended.
    old = compile_text(raw)
    keys = root.find('keyframe')
    if keys is not None: root.remove(keys)
    candidate = compile_text(ET.tostring(root, encoding='unicode'))
    if [candidate.actuator(a).name for a in range(old.nu)] != [old.actuator(a).name for a in range(old.nu)]:
        raise ValueError('old actuator order changed')
    keys = ET.SubElement(root, 'keyframe')
    widths = {0: (7, 6), 1: (4, 3), 2: (1, 1), 3: (1, 1)}
    for k in range(old.nkey):
        q = candidate.qpos0.copy(); v = np.zeros(candidate.nv)
        ctrl = np.full(candidate.nu, RELEASE_TRAVEL); ctrl[:old.nu] = old.key_ctrl[k]
        for j in range(old.njnt):
            body = old.body(int(old.jnt_bodyid[j])).name
            target_body = candidate.body(body).id
            target = int(candidate.body_jntadr[target_body])+j-int(old.body_jntadr[old.jnt_bodyid[j]])
            if old.joint(j).name != candidate.joint(target).name or old.jnt_type[j] != candidate.jnt_type[target]:
                raise ValueError('old joint identity changed')
            nq, nv = widths[int(old.jnt_type[j])]
            qa, qb = int(old.jnt_qposadr[j]), int(candidate.jnt_qposadr[target])
            va, vb = int(old.jnt_dofadr[j]), int(candidate.jnt_dofadr[target])
            q[qb:qb+nq] = old.key_qpos[k, qa:qa+nq]
            v[vb:vb+nv] = old.key_qvel[k, va:va+nv]
        attrs = dict(name=old.key(k).name or f'key{k}', time=str(old.key_time[k]),
            qpos=' '.join(map(str, q)), qvel=' '.join(map(str, v)), ctrl=' '.join(map(str, ctrl)))
        if old.na or old.nmocap: raise ValueError('extra key state requires explicit mapper')
        ET.SubElement(keys, 'key', **attrs)
    text = ET.tostring(root, encoding='unicode')
    compiled = compile_text(text)  # compile before writing any candidate asset
    return text, dict(added=added, old_nu=old.nu, new_nu=compiled.nu,
        old_nq=old.nq, new_nq=compiled.nq, rim_top_in_deck_m=float(rim_top),
        release_travel_m=RELEASE_TRAVEL, preload_target_m=PRELOAD_TARGET,
        shoe_force_limit_n=FORCE_LIMIT, physical_steps=0, qualified=False,
        scope='FIRST_VEHICLE_STATIC_CANDIDATE_ONLY',
        second_vehicle_retainer='NOT_RUN', deck_yaw_fix='NOT_RUN',
        old_joint_and_actuator_identity_preserved=True,
        runtime_cargo_qpos_writes=0, cargo_attachment=False)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run-id', required=True)
    parser.add_argument('--write', action='store_true')
    args = parser.parse_args()
    if Path(args.run_id).name != args.run_id: parser.error('bare run ID required')
    out = ROOT/'reports'/args.run_id; out.mkdir(exist_ok=False)
    raw = SOURCE.read_bytes(); text, report = build(raw.decode())
    report.update(source_sha256=hashlib.sha256(raw).hexdigest(),
                  candidate_sha256=hashlib.sha256(text.encode()).hexdigest(), verdict='STATIC_ONLY')
    if args.write:
        if TARGET.exists(): raise FileExistsError('candidate already exists; do not overwrite')
        TARGET.write_text(text)
    (out/'report.json').write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps(report, indent=2))


if __name__ == '__main__': main()
