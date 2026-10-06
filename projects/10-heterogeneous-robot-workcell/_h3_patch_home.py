"""Patch _h3_home.sh: report the tolerance explicitly instead of a bare `identical: False`.

Done with a script because the Edit tool reports `ModifyBackup failed` on this path -- the same
failure the project's notes already record for `\\\\wsl$` targets.
"""
from pathlib import Path

path = Path('/home/ziling/projects/010_heterogeneous_robot_workcell/_h3_home.sh')
text = path.read_text(encoding='utf-8')

old = """    a, b = probe(kf), probe(mh)
    print("==", f)
    print("   keyframe:", {k: round(v, 5) for k, v in a.items()})
    print("   merged  :", {k: round(v, 5) for k, v in b.items()})
    delta = {k: round(b[k] - a[k], 6) for k in a}
    print("   delta   :", delta)
    print("   identical:", all(abs(v) < 1e-9 for v in delta.values()))
"""

new = """    a, b = probe(kf), probe(mh)
    print("==", f)
    print("   keyframe:", {k: round(v, 6) for k, v in a.items()})
    print("   merged  :", {k: round(v, 6) for k, v in b.items()})
    delta = {k: round(b[k] - a[k], 8) for k in a}
    worst = max(abs(v) for v in delta.values())
    print("   delta   :", delta)
    # NOT a bit-for-bit claim. `merged_home` places the vehicle by ITERATING its assembly parking to
    # a fixed point, so it lands within a float32 residue of the world's own keyframe. Measured as
    # the same 4 um in every world here, i.e. a property of `merged_home` rather than of one world.
    # Stated against the tolerance it could affect, so nobody has to guess whether 4 um matters.
    print("   worst delta %.8f m (%.4f um)" % (worst, worst * 1e6))
    print("   within 10 um:", worst < 1e-5)
    print("   the docking envelope's LATERAL tolerance is 0.002 m, so this is %.4f%% of it"
          % (worst / 0.002 * 100))
"""

if old not in text:
    raise SystemExit('[FAIL] the anchor was not found; refusing to patch blind')
path.write_text(text.replace(old, new, 1), encoding='utf-8')
print('patched _h3_home.sh')
print('anchor present after patch:', 'within 10 um' in path.read_text(encoding='utf-8'))
