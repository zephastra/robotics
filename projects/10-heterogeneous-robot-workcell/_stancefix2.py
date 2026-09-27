"""Two corrections to `probe_p4_stance.py`, both found by the first run of its own instrument.

1. WRONG OBJECT, AGAIN. `np.sum(m.body_mass)` is the mass of the WHOLE SCENE -- 251.16 kg, including
   the two vehicles, the presentation table, the roller bands and the trays -- not the humanoid. It
   gave `weight_N = 2463.9` against a humanoid weight of 845.7 N, so F_z was 2.9x too large. This is
   the same defect as `subtree_com[0]` (the scene's CoM, 3 m from the feet) and as the foot-load
   readout that counted the table; the fix is the same shape too: use the SUBTREE of `h_LINK_BASE`.

2. THE WRONG REFERENCE TO VALIDATE AGAINST. I compared the stance torque to `mj_inverse` at the
   standing pose. But a robot that is settled IS in static equilibrium, so the torque that holds it
   is precisely the torque its actuators ARE applying -- at standing, measured: knee -4.279 N.m,
   ankle +12.343 N.m. `mj_inverse` disagrees with that (+2.698 at the knee), which is the
   `qfrc_inverse != bias - constraint` discrepancy this project already recorded in D094. So the
   applied torque at the settled state is the reference, and `mj_inverse` is kept only as a second
   column so the disagreement stays visible.

3. A dead constant (`FOOT_GEOM_HINT`) removed.
"""
import ast
from pathlib import Path

P = Path('/home/ziling/projects/010_heterogeneous_robot_workcell/experiments/probe_p4_stance.py')
t = P.read_text(encoding='utf-8')


def sub(old, new, count=1):
    global t
    assert t.count(old) == count, f'anchor x{t.count(old)} (wanted {count}): {old[:60]!r}'
    t = t.replace(old, new, count)


sub("""FEET = PB.FOOT
FOOT_GEOM_HINT = 'foot'""", """FEET = PB.FOOT""")

# --- 1. the humanoid's mass, not the scene's ------------------------------------------------------
sub("""    g = abs(float(m.opt.gravity[2]))
    total = float(np.sum(m.body_mass))
    fz = share * total * g                      # declared symmetric share per foot""",
    """    g = abs(float(m.opt.gravity[2]))
    # \u2605 THE HUMANOID'S SUBTREE, not the scene. `np.sum(m.body_mass)` is 251.16 kg -- the two
    # vehicles, the presentation table, the roller bands and the trays included -- which made F_z
    # 2.9x too large. Same defect shape as `subtree_com[0]` (the scene CoM) and as the foot-load
    # readout that counted the table.
    total = humanoid_mass(m)
    fz = share * total * g                      # declared symmetric share per foot""")

sub("""def foot_contact_x_scratch(m, d, side):""",
    """def humanoid_mass(m):
    \"\"\"The mass of the HUMANOID: the subtree hanging off `h_LINK_BASE`. Not the scene's.\"\"\"
    return float(m.body_subtreemass[m.body(PB.BASE).id])


def foot_contact_x_scratch(m, d, side):""")

# --- 2. validate against the torque actually holding the robot -------------------------------------
old_ic = t[t.index('def instrument_check(rig):'):t.index('def main():')]
new_ic = '''def instrument_check(rig):
    """Validate the stance torque against the torque that is ACTUALLY holding the robot.

    A settled robot is in static equilibrium, so the static stance torque at its configuration is
    exactly what its actuators are applying there -- measured at standing: knee -4.279 N.m, ankle
    +12.343 N.m. That is the reference. `mj_inverse` is reported beside it because at this pose it
    DISAGREES (knee +2.698), which is the `qfrc_inverse != bias - constraint` discrepancy D094
    recorded; showing both keeps the disagreement visible instead of hiding it behind a choice.
    """
    q_stand = np.array(rig.d.qpos, dtype=float)
    t_stand = stance_tau(rig, q_stand)
    applied = {}
    for name in rig.names:
        jid = rig.m.joint('h_' + name).id
        if jid < 0:
            continue
        applied[name] = 0.0
        for a in rig.index.get(int(jid), ()):
            applied[name] += float(rig.d.ctrl[a])
    sc = mujoco.MjData(rig.m)
    sc.qpos[:] = q_stand
    sc.qvel[:] = 0.0
    sc.ctrl[:] = np.zeros(rig.m.nu)
    mujoco.mj_forward(rig.m, sc)
    sc.qacc[:] = 0.0
    mujoco.mj_inverse(rig.m, sc)
    inv = np.array(sc.qfrc_inverse, dtype=float)
    cmp = {}
    for n in ('J00_HIP_PITCH_L', 'J03_KNEE_PITCH_L', 'J04_ANKLE_PITCH_L',
              'J06_HIP_PITCH_R', 'J09_KNEE_PITCH_R', 'J10_ANKLE_PITCH_R'):
        dof = int(rig.m.jnt_dofadr[rig.m.joint('h_' + n).id])
        cmp[n] = {'stance_Nm': round(float(t_stand[n]), 4),
                  'applied_Nm': round(float(applied.get(n, float('nan'))), 4),
                  'mj_inverse_Nm': round(float(inv[dof]), 4)}
    gap = max(abs(t_stand[PB.LEG_CHAIN['L'][i]] - t_stand[PB.LEG_CHAIN['R'][i]]) for i in range(6))
    weight = humanoid_mass(rig.m) * abs(float(rig.m.opt.gravity[2]))
    err = max(abs(cmp[n]['stance_Nm'] - cmp[n]['applied_Nm'])
              for n in ('J00_HIP_PITCH_L', 'J03_KNEE_PITCH_L'))
    return {'standing': cmp, 'symmetry_gap_Nm': round(float(gap), 6),
            'worst_sagittal_vs_applied_Nm': round(float(err), 4),
            'weight_N': weight, 'humanoid_mass_kg': humanoid_mass(rig.m)}


'''
t = t.replace(old_ic, new_ic, 1)

# --- the printed table must show all three columns --------------------------------------------------
sub("""    for n, v in inst['standing'].items():
        print(f"   {n:>20}  stance {v['stance_Nm']:9.3f}   mj_inverse {v['mj_inverse_Nm']:9.3f}")""",
    """    print(f"   {'joint':>20} {'stance':>10} {'applied':>10} {'mj_inverse':>11}")
    for n, v in inst['standing'].items():
        print(f"   {n:>20} {v['stance_Nm']:10.3f} {v['applied_Nm']:10.3f} "
              f"{v['mj_inverse_Nm']:11.3f}")""")
sub("""    print(f"   leg-to-leg symmetry gap {inst['symmetry_gap_Nm']:.6f} N.m "
          f"(D094's contact projection had 30.4 N.m); total mass {inst['total_mass_kg']:.3f} kg")""",
    """    print(f"   leg-to-leg symmetry gap {inst['symmetry_gap_Nm']:.6f} N.m "
          f"(D094's contact projection had 30.4 N.m); humanoid mass "
          f"{inst['humanoid_mass_kg']:.3f} kg -> {inst['weight_N']:.2f} N")
    print(f"   worst sagittal |stance - applied| = {inst['worst_sagittal_vs_applied_Nm']:.4f} N.m")""")

sub("""             '| joint | stance torque | mj_inverse |', '|---|---|---|']
    for n, v in inst['standing'].items():
        lines.append(f"| `{n}` | {v['stance_Nm']:+.4f} N·m | {v['mj_inverse_Nm']:+.4f} N·m |")""",
    """             '| joint | stance torque | applied (the measured truth) | mj_inverse |',
             '|---|---|---|---|']
    for n, v in inst['standing'].items():
        lines.append(f"| `{n}` | {v['stance_Nm']:+.4f} N·m | {v['applied_Nm']:+.4f} N·m | "
                     f"{v['mj_inverse_Nm']:+.4f} N·m |")""")

P.write_text(t, encoding='utf-8')
ast.parse(t)
checks = {
    'scene mass gone': 'np.sum(m.body_mass)' not in t,
    'humanoid_mass used': 'total = humanoid_mass(m)' in t,
    'applied column present': "'applied_Nm'" in t,
    'dead constant gone': 'FOOT_GEOM_HINT' not in t,
}
for k, v in checks.items():
    print(f'  {k:28s} {v}')
assert all(checks.values())
print('patched probe_p4_stance.py')
