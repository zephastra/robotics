"""P4-HUMAN-01's physics prerequisite: the humanoid must not intersect itself.

WHAT WAS RECORDED, AND WHAT IT ACTUALLY WAS
-------------------------------------------
`reports/w2-audit-05` reported the humanoid putting its own thumb tip 58.543 mm inside its own knee
and its elbow 25.107 mm inside its own source table, and that was carried forward as "the humanoid
at its home pose intersects itself". Re-measured, that sentence is wrong in a way that changes the
remedy completely:

  * at the keyframe the cell has ZERO deep contacts, and at the settled pose 0.044 mm;
  * the 58.543 mm belongs to the arm of the A/B that steps with the keyframe's `ctrl` held CONSTANT
    -- and in that arm the humanoid's base falls from 1.030 m to 0.553 m. It is not standing at its
    home pose, it is falling through it. `home_hold_ctrl` is state feedback whose own docstring says
    "evaluated every step", and the audit evaluated it once.

So the recorded finding was an artefact of the audit's own controller, and the number it produced is
reproducible only by reproducing that defect. This judge therefore keeps the defect as a MEASURED
ARM: the probe must still be able to see the deep contact, or the row that says "there is none" would
be a check that cannot fail.

WHAT IS ACTUALLY LEFT, AND THE CRITERION
----------------------------------------
With the law re-evaluated every step, the worst humanoid self-contact is 3.376 mm -- transient, gone
by step 100, and 0.044 mm once settled. The criterion is therefore about BEHAVIOUR rather than a
threshold: the self-contact must resolve itself inside the window, and the settled pose must be
clear. A number that decays to nothing is a settling artefact; a number that stays is a pose defect,
and those two need different fixes, so the row distinguishes them.
"""
import argparse
import json
import sys
from dataclasses import dataclass
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'experiments'))
sys.path.insert(0, str(ROOT / 'src'))

import mujoco  # noqa: E402
import numpy as np  # noqa: E402

import build_p3_world as bp3  # noqa: E402
import merge_world as mw  # noqa: E402

#: The deep figure the audit reported. Kept here as the number the A/B arm must still reproduce.
AUDIT_DEEP_MM = 58.543
#: Anything the probe should still be able to see, so "no deep contact" is a claim with a control.
FALSIFIABILITY_FLOOR_MM = 20.0
#: Settled means this. The measured value is 0.044 mm; a pose defect would not be this small.
SETTLED_ALLOWANCE_MM = 0.5
#: The arms are also run with a measured abduction, to show the criterion can be MET and to keep the
#: remedy as a measurement rather than a constant sitting unused in the source.
REMEDY_ABDUCTION_RAD = 0.05


def bname(model, index):
    return mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, index) or f'<body{index}>'


@dataclass
class Contact:
    depth_mm: float
    a: str
    b: str
    step: int


#: The five imported roles, by name prefix. `merge_world` prefixes every imported body, which is
#: what makes ownership derivable at all.
ROLE_PREFIXES = (('h_', 'humanoid'), ('a_', 'arm'), ('c_', 'conveyor'),
                 ('n2_', 'amr2'), ('n_', 'amr1'))
ROLES = frozenset(role for _prefix, role in ROLE_PREFIXES)


def owner(name):
    """`humanoid` / `arm` / ... for an imported body, else `cell` for the world's own geometry.

    ★ The first version returned `?` for anything that was not a role prefix, and the world's own
    geoms are attached to a body literally named `world` -- so `world x h_LINK_FOOT_L` (the humanoid's
    foot resting on the ground) came out as "two different roles overlapping" and failed the row.
    Unmatched is `cell`, not `unknown`, because the only thing in this model that is not an imported
    role IS the cell.
    """
    for prefix, role in ROLE_PREFIXES:
        if name.startswith(prefix):
            return role
    return 'cell'


def deepest(model, data, *, humanoid_self, step):
    worst = Contact(0.0, '', '', step)
    for i in range(data.ncon):
        g1, g2 = int(data.contact.geom[i][0]), int(data.contact.geom[i][1])
        n1 = bname(model, int(model.geom_bodyid[g1]))
        n2 = bname(model, int(model.geom_bodyid[g2]))
        if humanoid_self and not (n1.startswith('h_') and n2.startswith('h_')):
            continue
        d = float(data.contact.dist[i]) * 1000.0
        if d < worst.depth_mm:
            worst = Contact(d, n1, n2, step)
    return worst


def deepest_cross_role(model, data, step):
    """The deepest overlap between TWO DIFFERENT ROLES -- world contacts excluded on purpose.

    ★ The first version of this row counted every contact that was not humanoid-self, so it included
    `cell_ground x n_wheel_left_geom` at -0.604 mm and failed the row. That contact is a WHEEL
    CARRYING THE VEHICLE: a loaded contact rests at a small negative distance, and calling it an
    interpenetration is calling the floor an assembly fault. What this criterion is for is two
    CONSTRUCTED parts occupying the same space, which is a modelling error with no reading under
    which it is fine. The world contacts are still measured and reported, just not judged here.
    """
    worst = Contact(0.0, '', '', step)
    world = Contact(0.0, '', '', step)
    for i in range(data.ncon):
        g1, g2 = int(data.contact.geom[i][0]), int(data.contact.geom[i][1])
        n1 = bname(model, int(model.geom_bodyid[g1]))
        n2 = bname(model, int(model.geom_bodyid[g2]))
        d = float(data.contact.dist[i]) * 1000.0
        o1, o2 = owner(n1), owner(n2)
        # judge ONLY two distinct recognised roles: that is a modelling error with no benign
        # reading. A role resting on the cell is a load path and is measured, not judged.
        if o1 in ROLES and o2 in ROLES:
            if o1 != o2 and d < worst.depth_mm:
                worst = Contact(d, n1, n2, step)
            continue
        if (o1 in ROLES) != (o2 in ROLES) and d < world.depth_mm:
            world = Contact(d, n1, n2, step)
    return worst, world


def arm(*, feedback, abduction=0.0, seconds=1.0, trace_every=None):
    """One arm of the measurement. `feedback=False` reproduces the audit's defect on purpose."""
    bp3.install()
    _t, layout, model, _s, _k, _n, _a = mw.build_text()
    data = mujoco.MjData(model)
    qpos, ctrl, _a2, _notes = mw.merged_home(model, layout)
    names, target, kp, kd = mw.stand_law()
    target = np.array(target, dtype=float)
    if abduction:
        # Named here rather than imported: the assembly-level constant was TRIED and removed (it
        # regressed the settle gate), so importing it would be importing something that no longer
        # exists. The remedy is a measurement in this file, not a live setting.
        for joint in ('J14_SHOULDER_ROLL_L', 'J19_SHOULDER_ROLL_R'):
            position = names.index(joint)
            target[position] += abduction * (1.0 if joint.endswith('_L') else -1.0)
    index = mw._stand_actuator_index(model)

    def law():
        command = np.zeros(model.nu, dtype=float)
        affine = int(mujoco.mjtBias.mjBIAS_AFFINE)
        joint_trn = int(mujoco.mjtTrn.mjTRN_JOINT)
        for a in range(model.nu):
            if int(model.actuator_biastype[a]) != affine or \
                    float(model.actuator_biasprm[a][1]) == 0.0:
                continue
            if int(model.actuator_trntype[a]) != joint_trn:
                continue
            jid = int(model.actuator_trnid[a, 0])
            if int(model.jnt_type[jid]) not in (2, 3):
                continue
            adr = int(model.jnt_qposadr[jid])
            value = float(qpos[adr])
            if int(model.actuator_ctrllimited[a]):
                lo, hi = (float(v) for v in model.actuator_ctrlrange[a])
                value = min(max(value, lo), hi)
            command[a] = value
        for k, joint_name in enumerate(names):
            jid = model.joint('h_' + joint_name).id
            if jid < 0:
                continue
            adr, dof = int(model.jnt_qposadr[jid]), int(model.jnt_dofadr[jid])
            torque = ((float(target[k]) - float(data.qpos[adr])) * float(kp[k])
                      - float(data.qvel[dof]) * float(kd[k]))
            for a in index.get(int(jid), ()):
                command[a] = torque
        return command

    data.qpos[:] = qpos
    data.ctrl[:] = law() if feedback else ctrl
    steps = int(seconds / model.opt.timestep)
    worst_self = Contact(0.0, '', '', 0)
    worst_cross = Contact(0.0, '', '', 0)
    worst_world = Contact(0.0, '', '', 0)
    trace = []
    for step in range(steps):
        if feedback:
            data.ctrl[:] = law()
        mujoco.mj_step(model, data)
        s = deepest(model, data, humanoid_self=True, step=step)
        if s.depth_mm < worst_self.depth_mm:
            worst_self = s
        c, w = deepest_cross_role(model, data, step)
        if c.depth_mm < worst_cross.depth_mm:
            worst_cross = c
        if w.depth_mm < worst_world.depth_mm:
            worst_world = w
        if trace_every and step % trace_every == 0:
            trace.append({'t_s': round(float(data.time), 3), 'depth_mm': round(s.depth_mm, 3)})
    settled = deepest(model, data, humanoid_self=True, step=steps).depth_mm
    base = float(data.xpos[model.body('h_LINK_BASE').id][2])
    return {'worst_self': worst_self, 'worst_cross': worst_cross, 'worst_world': worst_world,
            'settled_mm': settled, 'base_z': round(base, 4), 'trace': trace}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--run-id', default='p4-human-sc-01')
    ap.add_argument('--reports-dir', default=None)
    args = ap.parse_args()
    out = Path(args.reports_dir) if args.reports_dir else (ROOT / 'reports' / args.run_id)
    out.mkdir(parents=True, exist_ok=True)

    checks = []

    def check(group, name, verdict, detail):
        checks.append({'group': group, 'check': name, 'verdict': verdict, 'detail': detail})

    # --- the arm that reproduces the audit's defect: the probe must still SEE the deep contact ----
    reference = arm(feedback=False)
    check('falsifiability',
          'the probe still DETECTS the deep self-contact when the law is evaluated once',
          'PASS' if reference['worst_self'].depth_mm < -FALSIFIABILITY_FLOOR_MM else 'FAIL',
          f"stepping with the keyframe's ctrl held constant reproduces "
          f"{reference['worst_self'].depth_mm:.3f} mm between {reference['worst_self'].a} and "
          f"{reference['worst_self'].b} at step {reference['worst_self'].step}, with the humanoid's "
          f"base falling to z {reference['base_z']:.4f} m. The audit reported {AUDIT_DEEP_MM} mm from "
          f"exactly this arm, which is what makes the recorded finding an artefact of that probe's "
          f"controller rather than a property of the pose. Without this arm, the row below would be "
          f'a check that cannot fail')

    # --- the real pose, with the law evaluated every step ----------------------------------------
    current = arm(feedback=True, trace_every=5)
    check('physics', 'with the standing law evaluated every step, the humanoid STANDS',
          'PASS' if current['base_z'] > 1.0 else 'FAIL',
          f"base z settles at {current['base_z']:.4f} m against {reference['base_z']:.4f} m for the "
          f"constant-ctrl arm. The two arms differ in one thing -- whether the state-feedback law is "
          f're-evaluated -- and one of them is standing while the other is falling')
    check('physics', 'the humanoid does not intersect ITSELF once it has settled',
          'PASS' if current['settled_mm'] >= -SETTLED_ALLOWANCE_MM else 'FAIL',
          f"deepest humanoid self-contact at the end of the window is {current['settled_mm']:.3f} mm "
          f'against an allowance of {SETTLED_ALLOWANCE_MM} mm. This is the criterion: a settled pose '
          f'is a pose, and a pose that intersects itself is a defect with no excuse')
    check('physics', 'the transient self-contact RESOLVES inside the window',
          'PASS' if (current['worst_self'].depth_mm < 0.0
                     and current['settled_mm'] >= -SETTLED_ALLOWANCE_MM) else 'FAIL',
          f"deepest during the transit {current['worst_self'].depth_mm:.3f} mm between "
          f"{current['worst_self'].a} and {current['worst_self'].b} at step "
          f"{current['worst_self'].step}, resolved to {current['settled_mm']:.3f} mm by the end. A "
          f'contact that decays to nothing is a settling artefact and a contact that stays is a pose '
          f'defect; this row exists because they need different fixes and both report as "a negative '
          f'number"')
    cross = current['worst_cross']
    world = current['worst_world']
    check('physics', 'no two ROLES occupy the same space in the assembled cell',
          'PASS' if cross.depth_mm > -0.05 else 'FAIL',
          f'deepest overlap between two different roles: {cross.depth_mm:.3f} mm '
          f'({cross.a} x {cross.b} at step {cross.step}). Separately, the deepest contact between a '
          f'role and the cell is {world.depth_mm:.3f} mm ({world.a} x {world.b} at step '
          f'{world.step}), and that one is NOT judged: a wheel carrying the vehicle rests at a small '
          f'negative distance, and calling a loaded contact an interpenetration would be calling the '
          f'floor an assembly fault')

    # --- the remedy, measured rather than declared ----------------------------------------------
    remedy = arm(feedback=True, abduction=REMEDY_ABDUCTION_RAD)
    check('remedy', 'the measured remedy clears it entirely, and the criterion can be MET',
          'PASS' if remedy['worst_self'].depth_mm >= -SETTLED_ALLOWANCE_MM else 'FAIL',
          f"a symmetric shoulder-roll abduction of {REMEDY_ABDUCTION_RAD} rad "
          f'({np.degrees(REMEDY_ABDUCTION_RAD):.3f} deg) per side takes the worst humanoid '
          f'self-contact from {current["worst_self"].depth_mm:.3f} mm to '
          f'{remedy["worst_self"].depth_mm:.3f} mm. It is NOT applied in the world: it was tried, '
          f'and it made the humanoid settle slower (the P3 world gate\'s settle row went PASS -> '
          f'FAIL at worst |qvel| 0.1095 against 0.10). It is measured here so the number in this '
          f'report is produced every run instead of a constant sitting unused in the source')

    failed = [c for c in checks if c['verdict'] == 'FAIL']
    verdict = 'FAIL' if failed else 'PASS'
    report = {'run_id': args.run_id,
              'scope': 'P4-HUMAN-01 physics prerequisite: no humanoid self-intersection',
              'verdict': verdict, 'checks': checks,
              'measurements': {
                  'audit_deep_mm_reproduced': round(reference['worst_self'].depth_mm, 3),
                  'current_worst_transient_mm': round(current['worst_self'].depth_mm, 3),
                  'current_worst_pair': [current['worst_self'].a, current['worst_self'].b],
                  'current_settled_mm': current['settled_mm'],
                  'remedy_abduction_rad': REMEDY_ABDUCTION_RAD,
                  'remedy_worst_mm': round(remedy['worst_self'].depth_mm, 3),
                  'base_z_standing_m': current['base_z'],
                  'base_z_constant_ctrl_m': reference['base_z'],
              },
              'trace': current['trace'],
              'failed': [c['check'] for c in failed]}
    (out / 'report.json').write_text(json.dumps(report, indent=2) + '\n', encoding='utf-8')
    lines = [f'# {args.run_id} — humanoid self-intersection', '', f'**verdict {verdict}**', '']
    for c in checks:
        lines += [f'- **{c["verdict"]}** ({c["group"]}) {c["check"]}', f'  - {c["detail"]}']
    (out / 'acceptance.md').write_text('\n'.join(lines) + '\n', encoding='utf-8')

    for c in checks:
        print(f'  [{c["verdict"]}] ({c["group"]}) {c["check"]}')
        print(f'        {c["detail"][:520]}')
    print()
    print(f'verdict {verdict}; measurements '
          f'{json.dumps(report["measurements"], ensure_ascii=False)}')
    return 0 if verdict == 'PASS' else 1


if __name__ == '__main__':
    raise SystemExit(main())
