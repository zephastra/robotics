"""Build an independently auditable integration candidate, not a V1 acceptance run.

User authorization: 2026-10-02, add a candidate world, preserve old worlds/evidence.
Only declared build-time geometry and initial conditions change. No runtime teleport,
weld, collision disabling, threshold relaxation, or inherited PASS is authorized.
The original navigation platforms remain DECLARED TWO-WHEEL STAND-INS, not four-wheel
production AMRs. Capacity and IK below are necessary static checks, NOT execution proof.
"""
import argparse
import copy
import hashlib
import json
import re
import sys
import tempfile
import xml.etree.ElementTree as ET
from pathlib import Path

import mujoco
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT / 'experiments'), str(ROOT / 'src')]
import arm_bridge
import arm_rig
from workcell.tray import derive_cells, cell_centre_world
from w4_plant import collision_radius

SOURCE = ROOT / 'assets/world_w5_h085_loop.xml'
TARGET = ROOT / 'assets/world_p5_candidate.xml'
# New layout declarations, not changed acceptance tolerances.
ARM_SIDE_Y_M = -0.58
ARM_BASE_Z_M = 0.61
STOCK_SIDE_Y_M = -0.47
STOCK_OFFSET_X_M = 0.30
RECEIVER_ROLLERS = 20
BUFFER_CLEARANCE_M = 0.010
MARKER_RENDER_GROUP = 5  # only nonphysical, massless marker; cameras may omit this group


def vector(element, attr='pos'):
    return np.array([float(x) for x in element.get(attr, '0 0 0').split()])


def setvector(element, value, attr='pos'):
    element.set(attr, ' '.join(format(float(x), '.12g') for x in value))


def renamed_tree(element, old, new):
    result = copy.deepcopy(element)
    for node in result.iter():
        for attr, value in list(node.attrib.items()):
            # Names and cross-references only. Never alter numeric physical parameters.
            if attr in ('name', 'joint', 'body', 'site', 'geom', 'objname'):
                node.set(attr, ' '.join(new + v[len(old):] if v.startswith(old) else v
                                        for v in value.split()))
    return result


def compile_text(text):
    # Mesh names resolve relative to assets, never cwd. Unique temp file, owned here.
    with tempfile.NamedTemporaryFile(mode='w', suffix='.xml', prefix='_p5_candidate_',
                                     dir=SOURCE.parent, encoding='utf-8') as handle:
        handle.write(text)
        handle.flush()
        return mujoco.MjModel.from_xml_path(handle.name)


def old_home(model):
    data = mujoco.MjData(model)
    mujoco.mj_resetDataKeyframe(model, data, 0)
    mujoco.mj_forward(model, data)
    return data


def build(source_text, stock_offset_x=STOCK_OFFSET_X_M):
    if not np.isfinite(stock_offset_x) or stock_offset_x <= 0:
        raise ValueError('stock offset must be finite and positive')
    # Existing generator comments contain double hyphens accepted by MuJoCo but not
    # strict ElementTree. Remove comments ONLY in the derived representation.
    root = ET.fromstring(re.sub(r'<!--.*?-->', '', source_text, flags=re.S))
    wb = root.find('worldbody')
    acts = root.find('actuator')
    source_model = compile_text(source_text)
    source_data = old_home(source_model)
    # Choose a real source roller's center, away from the human's first handover rollers.
    loading_x = float(source_data.xpos[source_model.body('c_fixed_roller_1_2').id][0])
    crown_z = float(source_data.geom_xpos[source_model.geom('c_fixed_roller_1_2').id][2]
                    + source_model.geom_size[source_model.geom('c_fixed_roller_1_2').id][0])
    arm = wb.find("body[@name='a_link0']")
    setvector(arm, [loading_x, ARM_SIDE_Y_M, ARM_BASE_Z_M])
    arm.set('quat', '0.7071067811865476 0 0 0.7071067811865476')
    pedestal = ET.SubElement(wb, 'body', name='p5_arm_pedestal',
                             pos=f'{loading_x} {ARM_SIDE_Y_M} {ARM_BASE_Z_M / 2}')
    ET.SubElement(pedestal, 'geom', name='p5_arm_pedestal_geom', type='cylinder',
                  size=f'0.10 {ARM_BASE_Z_M / 2}', rgba='0.3 0.3 0.35 1')
    table = wb.find("body[@name='a_table']")
    table_half_z = float(table.find('geom').get('size').split()[2])
    setvector(table, [loading_x + stock_offset_x, STOCK_SIDE_Y_M,
                      crown_z - table_half_z])
    part = wb.find("body[@name='a_payload']")
    part_half_z = float(part.find('geom').get('size').split()[2])
    setvector(part, [loading_x + stock_offset_x, STOCK_SIDE_Y_M,
                     crown_z + part_half_z])
    # This is an optical annotation, not a lid. Keep mass/collisions/pose unchanged.
    marker = wb.find("body[@name='c_payload']/geom[@name='c_visual_marker']")
    if marker.get('contype') != '0' or marker.get('conaffinity') != '0' or marker.get('mass') != '0':
        raise ValueError('marker must remain nonphysical')
    marker.set('group', str(MARKER_RENDER_GROUP))
    # Copy the COMPLETE high-interface mechanism, including passive compliance and actuators.
    old_deck = wb.find("body[@name='n_base_link']/body[@name='c_deck']")
    new_deck = renamed_tree(old_deck, 'c_', 'c2_')
    wb.find("body[@name='n2_base_link']").append(new_deck)
    deck_joint_names = {node.get('name') for node in old_deck.iter('joint')}
    copied_actuators = []
    for actuator in list(acts):
        if actuator.get('joint') in deck_joint_names:
            duplicate = renamed_tree(actuator, 'c_', 'c2_')
            acts.append(duplicate)
            copied_actuators.append(duplicate.get('name'))
    # Extend the same receiver row by its DECLARED measured pitch; old rollers are unedited.
    rollers = sorted((node for node in wb.findall('body')
                      if node.get('name', '').startswith('c_recv_roller_')),
                     key=lambda node: int(node.get('name').rsplit('_', 1)[1]))
    first = vector(rollers[0])
    pitch = vector(rollers[1])[0] - first[0]
    if len(rollers) >= RECEIVER_ROLLERS or pitch <= 0:
        raise ValueError('unexpected receiver row declaration')
    template_act = next(node for node in acts if node.get('joint') == 'c_recv_roller_0_joint')
    for i in range(len(rollers), RECEIVER_ROLLERS):
        roller = renamed_tree(rollers[0], 'c_recv_roller_0', f'c_recv_roller_{i}')
        setvector(roller, first + [i * pitch, 0, 0])
        wb.append(roller)
        acts.append(renamed_tree(template_act, 'c_recv_roller_0', f'c_recv_roller_{i}'))
    # Preserve all existing state BY JOINT NAME, not shifted array addresses.
    keyframe = root.find('keyframe')
    root.remove(keyframe)
    model = compile_text(ET.tostring(root, encoding='unicode'))
    qpos, ctrl = model.qpos0.copy(), np.zeros(model.nu)
    width = {int(mujoco.mjtJoint.mjJNT_FREE): 7, int(mujoco.mjtJoint.mjJNT_BALL): 4,
             int(mujoco.mjtJoint.mjJNT_SLIDE): 1, int(mujoco.mjtJoint.mjJNT_HINGE): 1}
    for joint in range(source_model.njnt):
        name = mujoco.mj_id2name(source_model, mujoco.mjtObj.mjOBJ_JOINT, joint)
        # The humanoid free joint is unnamed; topology before this joint has not changed.
        target_joint = model.joint(name).id if name else joint
        oldadr, newadr = source_model.jnt_qposadr[joint], model.jnt_qposadr[target_joint]
        size = width[int(source_model.jnt_type[joint])]
        if name != 'a_payload_free':
            qpos[newadr:newadr + size] = source_data.qpos[oldadr:oldadr + size]
    for actuator in range(source_model.nu):
        name = mujoco.mj_id2name(source_model, mujoco.mjtObj.mjOBJ_ACTUATOR, actuator)
        ctrl[model.actuator(name).id] = source_data.ctrl[actuator]
    kf = ET.SubElement(root, 'keyframe')
    key = ET.SubElement(kf, 'key', name='home')
    setvector(key, qpos, 'qpos')
    setvector(key, ctrl, 'ctrl')
    root.set('model', '010_p5_integration_candidate_v1')
    return ET.tostring(root, encoding='unicode'), {
        'loading_x_m': loading_x, 'interface_crown_z_m': crown_z,
        'receiver_pitch_m': pitch, 'receiver_rollers': RECEIVER_ROLLERS,
        'arm_base_m': [loading_x, ARM_SIDE_Y_M, ARM_BASE_Z_M],
        'stock_table_top_m': crown_z, 'second_deck_actuators': copied_actuators,
        'stock_offset_x_m': float(stock_offset_x),
        'source_deck_actuator_count': len(copied_actuators),
        'render_only_marker_group': MARKER_RENDER_GROUP,
        'initialization': 'existing home mapped by joint name; only stock part relocated',
    }


def audit(text, declarations):
    model = compile_text(text)
    data = old_home(model)
    checks = {}
    for name, chassis in [('c_deck', 'n_base_link'), ('c2_deck', 'n2_base_link')]:
        checks[name + '_on_own_chassis'] = model.body(name).parentid[0] == model.body(chassis).id
    source_joints = {j for j in range(model.njnt)
                     if (model.joint(j).name or '').startswith(('c_deck_', 'c_pusher_'))}
    copied_joints = {j for j in range(model.njnt)
                     if (model.joint(j).name or '').startswith(('c2_deck_', 'c2_pusher_'))}
    source_driven = sum(int(model.actuator_trnid[a, 0]) in source_joints
                        for a in range(model.nu))
    copied_driven = sum(int(model.actuator_trnid[a, 0]) in copied_joints
                        for a in range(model.nu))
    checks['second_deck_has_actuation'] = (copied_driven == source_driven
                                          == declarations['source_deck_actuator_count']
                                          and copied_driven > 0)
    cargo = model.body('c_payload').id
    cargo_constraints = [e for e in range(model.neq)
                         if model.eq_type[e] in (mujoco.mjtEq.mjEQ_WELD,
                                                 mujoco.mjtEq.mjEQ_CONNECT)
                         and cargo in (model.eq_obj1id[e], model.eq_obj2id[e])]
    checks['cargo_is_free_no_weld'] = (model.joint('c_payload_free').type[0]
                                      == mujoco.mjtJoint.mjJNT_FREE and not cargo_constraints)
    marker = model.geom('c_visual_marker').id
    checks['marker_remains_nonphysical'] = (model.geom_contype[marker] == 0
                                          and model.geom_conaffinity[marker] == 0)
    # Static IK uses a separate initialized state at the DECLARED loading pose.
    # It does not execute transport, and it does not write a running simulation's cargo pose.
    probe = old_home(model)
    adr = model.jnt_qposadr[model.joint('c_payload_free').id]
    probe.qpos[adr] = declarations['loading_x_m']
    mujoco.mj_forward(model, probe)
    arm_bridge.install('a_')
    ids = arm_rig.ids(model)
    rotation = np.array(probe.xmat[model.body('a_hand').id]).reshape(3, 3)
    seed = probe.qpos[ids['arm_qadr']].copy()
    cells = derive_cells(model, probe)
    ik_rows = []
    for cell in cells['cells']:
        target = np.array(cell_centre_world(cells, cell['index']))
        # arm_rig.ik solves the actual pad-midpoint grasp frame, not the palm origin.
        target[2] = cells['deck']['top_z'] + 0.025
        try:
            q, residual, rotation_error = arm_rig.ik(model, probe, target, rotation, q_seed=seed)
            success, detail = True, None
        except RuntimeError as exc:
            success, residual, rotation_error, detail = False, None, None, str(exc)
        ik_rows.append({'cell': cell['index'], 'target_m': target.tolist(),
                        'converged': success, 'residual_m': residual,
                        'rotation_error_rad': rotation_error, 'detail': detail})
        checks[f'cell_{cell["index"]}_static_ik'] = bool(success)
    radius = collision_radius(model, model.body('c_payload').id)
    receiver_x = sorted(float(data.xpos[b][0]) for b in range(model.nbody)
                          if re.fullmatch(r'c_recv_roller_\d+',model.body(b).name or ''))
    receiver_length = receiver_x[-1] - receiver_x[0]
    checks['receiver_matches_declared_count_and_pitch'] = (
        len(receiver_x) == declarations['receiver_rollers']
        and np.allclose(np.diff(receiver_x), declarations['receiver_pitch_m'],
                        atol=1e-9, rtol=0))
    required = 4 * radius + 3 * BUFFER_CLEARANCE_M
    checks['two_conservative_longitudinal_slots_fit'] = receiver_length >= required
    # Audit home pose for new deck's collision overlaps with unrelated entities.
    allowed = {'n2_base_link', 'c2_deck'}
    contacts = []
    for contact in data.contact[:data.ncon]:
        if contact.dist >= -0.001:
            continue
        bodies = [mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY,
                                   int(model.geom_bodyid[g])) or 'world'
                  for g in (contact.geom1, contact.geom2)]
        if any(name.startswith('c2_') for name in bodies) and not all(
                name.startswith('c2_') or name in allowed for name in bodies):
            contacts.append({'bodies': bodies, 'penetration_m': float(-contact.dist)})
    checks['new_deck_no_unrelated_home_penetration'] = not contacts
    table_geom = model.geom('a_table_top').id
    arm_table_contacts = []
    for contact in data.contact[:data.ncon]:
        if table_geom not in (contact.geom1, contact.geom2) or contact.dist >= -1e-9:
            continue
        other = contact.geom2 if contact.geom1 == table_geom else contact.geom1
        body = model.body(model.geom_bodyid[other]).name or ''
        if body.startswith('a_') and body not in ('a_table', 'a_payload'):
            arm_table_contacts.append({'body': body, 'depth_m': float(-contact.dist)})
    checks['stock_table_does_not_penetrate_arm_at_home'] = not arm_table_contacts
    return {'checks': {k: 'PASS' if bool(v) else 'FAIL' for k, v in checks.items()},
            'static_ik': ik_rows, 'new_deck_contacts': contacts,
            'arm_table_home_contacts': arm_table_contacts,
            'receiver_center_span_m': receiver_length, 'required_buffer_span_m': required,
            'buffer_slot_half_envelope_m': radius, 'nq': model.nq, 'nu': model.nu,
            'diagnostic_result': 'PASS' if all(checks.values()) else 'FAIL',
            'scope': 'BUILD_AND_STATIC_FEASIBILITY_ONLY',
            'dynamic_grasp': 'NOT_RUN', 'two_vehicle_transport': 'NOT_RUN',
            'two_tray_buffer_delivery': 'NOT_RUN', 'full_order': 'NOT_RUN',
            'v1_complete': False}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run-id', required=True)
    parser.add_argument('--write', action='store_true')
    parser.add_argument('--check', action='store_true')
    parser.add_argument('--stock-offset-x', type=float, default=STOCK_OFFSET_X_M)
    parser.add_argument('--candidate-file', default=TARGET.name)
    args = parser.parse_args()
    if args.write and args.check:
        parser.error('--write and --check are mutually exclusive')
    if not re.fullmatch(r'world_p5_candidate(?:_v[0-9]+)?\.xml', args.candidate_file):
        parser.error('candidate must be a versioned world_p5_candidate XML filename')
    target = SOURCE.parent / args.candidate_file
    out = ROOT / 'reports' / args.run_id
    out.mkdir(parents=True, exist_ok=False)
    source_text = SOURCE.read_text(encoding='utf-8')
    source_hash = hashlib.sha256(SOURCE.read_bytes()).hexdigest()
    text, declarations = build(source_text, args.stock_offset_x)
    if target != TARGET:
        text = text.replace('model="010_p5_integration_candidate_v1"',
                            f'model="{target.stem}"', 1)
    report = audit(text, declarations)
    report.update(declarations=declarations, source=str(SOURCE.relative_to(ROOT)),
                  source_sha256=source_hash, candidate_sha256=hashlib.sha256((text + '\n').encode()).hexdigest(),
                  authorization='2026-10-02 user approved separate integration candidate')
    if SOURCE.read_text(encoding='utf-8') != source_text:
        raise RuntimeError('source changed during build')
    if args.write and report['diagnostic_result'] == 'PASS':
        if target.exists():
            raise RuntimeError('candidate exists: refuse overwrite; version explicitly')
        target.write_text(text + '\n', encoding='utf-8')
        report['written'] = str(target.relative_to(ROOT))
        report['candidate_sha256'] = hashlib.sha256(target.read_bytes()).hexdigest()
    else:
        report['written'] = None
    if args.check:
        same = target.exists() and target.read_bytes() == (text + '\n').encode()
        report['checks']['candidate_matches_builder'] = 'PASS' if same else 'FAIL'
        if not same:
            report['diagnostic_result'] = 'FAIL'
    (out / 'acceptance.json').write_text(json.dumps(report, indent=2) + '\n', encoding='utf-8')
    print(json.dumps(report, indent=2))
    return 0 if report['diagnostic_result'] == 'PASS' else 1


if __name__ == '__main__':
    raise SystemExit(main())
