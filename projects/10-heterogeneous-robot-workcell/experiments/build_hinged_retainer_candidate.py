"""v7 outward-folding candidate; v6 failure/assets preserved, no qualification."""
import argparse
import hashlib
import json
import math
import xml.etree.ElementTree as ET
import numpy as np
from build_tray_retainer_candidate import build as linear_build, SOURCE, ROOT
from build_p5_candidate_world import compile_text

TARGET = ROOT/'assets/world_p5_candidate_v7_hinged_retainer.xml'
RELEASE_ANGLE = math.pi/2
PRELOAD_ANGLE = -.025
TORQUE_LIMIT = .06  # Nm, ~2N at 29mm arm; candidate, not qualified


def build(raw):
    text, _ = linear_build(raw)
    root = ET.fromstring(text)
    deck = next(b for b in root.iter('body') if b.get('name') == 'c_deck')
    actuator = root.find('actuator')
    for side in (-1, 1):
        name = f'c_retainer_{side}'
        for child in list(deck):
            if (child.get('name') or '').startswith(name): deck.remove(child)
        for child in list(actuator):
            if (child.get('name') or '').startswith(name): actuator.remove(child)
        # Mast x-clearance to handle sphere: 45 - 35 - 5 = 5mm declared.
        ET.SubElement(deck, 'geom', name=name+'_mast', type='capsule',
            fromto=f'.165 {side*.255} -.07 .165 {side*.255} .068', size='.005',
            mass='.025', rgba='.8 .6 .1 1')
        body = ET.SubElement(deck, 'body', name=name, pos=f'.165 {side*.255} .068',
            quat=f'{math.cos(RELEASE_ANGLE/2)} {-side*math.sin(RELEASE_ANGLE/2)} 0 0')
        ET.SubElement(body, 'joint', name=name+'_joint', type='hinge', axis=f'{-side} 0 0',
            ref=str(RELEASE_ANGLE), limited='true', range=f'-.08 {RELEASE_ANGLE}', damping='.03')
        ET.SubElement(body, 'geom', name=name+'_arm', type='capsule',
            fromto=f'0 0 0 -.045 {-side*.029} 0', size='.003', mass='.01', rgba='.9 .65 .1 1')
        ET.SubElement(body, 'geom', name=name+'_shoe', type='box',
            pos=f'-.045 {-side*.029} 0', size='.035 .003 .003',
            mass='.025', friction='.4 .005 .001', condim='4', rgba='.9 .65 .1 1')
        ET.SubElement(actuator, 'position', name=name+'_drive', joint=name+'_joint', kp='1',
            ctrllimited='true', ctrlrange=f'-.08 {RELEASE_ANGLE}', forcelimited='true',
            forcerange=f'-{TORQUE_LIMIT} {TORQUE_LIMIT}')
    # Same two added DOFs/actuator suffixes as v6, remap their initial values only.
    old_candidate = compile_text(text)
    for key in root.findall('keyframe/key'):
        q = np.fromstring(key.get('qpos'), sep=' ')
        ctrl = np.fromstring(key.get('ctrl'), sep=' ')
        for side in (-1, 1):
            name = f'c_retainer_{side}'
            q[old_candidate.jnt_qposadr[old_candidate.joint(name+'_joint').id]] = RELEASE_ANGLE
            ctrl[old_candidate.actuator(name+'_drive').id] = RELEASE_ANGLE
        key.set('qpos', ' '.join(map(str, q))); key.set('ctrl', ' '.join(map(str, ctrl)))
    result = ET.tostring(root, encoding='unicode')
    model = compile_text(result)
    return result, dict(verdict='STATIC_ONLY', qualified=False, physical_steps=0,
        added_actuators=2, new_nu=model.nu, release_angle_rad=RELEASE_ANGLE,
        preload_angle_rad=PRELOAD_ANGLE, torque_limit_nm=TORQUE_LIMIT,
        camera_visibility='NOT_RUN', bilateral_contact='NOT_RUN',
        deck_yaw_fix='NOT_RUN', second_vehicle='NOT_RUN', full_order='NOT_RUN')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run-id', required=True); parser.add_argument('--write', action='store_true')
    args = parser.parse_args()
    if ROOT.name == args.run_id or '/' in args.run_id or '\\' in args.run_id: parser.error('bare run ID required')
    out = ROOT/'reports'/args.run_id; out.mkdir(exist_ok=False)
    raw = SOURCE.read_bytes(); text, report = build(raw.decode())
    report.update(source_sha256=hashlib.sha256(raw).hexdigest(),
                  candidate_sha256=hashlib.sha256(text.encode()).hexdigest())
    if args.write:
        if TARGET.exists(): raise FileExistsError('preserve existing candidate')
        TARGET.write_text(text)
    (out/'report.json').write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps(report, indent=2))


if __name__ == '__main__': main()
