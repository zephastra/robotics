set -u
P=/home/ziling/projects/010_heterogeneous_robot_workcell
cd "$P" || exit 1
# Is the plant's `_home` (from `merged_home`) the same state as the world's own keyframe?
# If they disagreed, `_derive()` would be building the plant's dock geometry from one state while
# the hold law maintained another -- which is D110's family, one level deeper.
timeout 900 ./.venv/bin/python - <<'PY'
import sys, numpy as np
sys.path.insert(0, "experiments"); sys.path.insert(0, "src")
import mujoco, build_p3_world as bp3
bp3.install()
import merge_world as mw

for f in ("assets/world_w5_h085_loop.xml", "assets/world_w5_h085.xml",
          "assets/world_w2_logistic.xml"):
    m = mujoco.MjModel.from_xml_path(f)
    kf = np.array(m.key_qpos[0], float)
    mh, _ctrl, _applied, _notes = mw.merged_home(m)
    mh = np.asarray(mh, float)

    def probe(q):
        d = mujoco.MjData(m); d.qpos[:] = q; mujoco.mj_forward(m, d)
        pick = lambda name: float(d.xpos[m.body(name).id][0])
        return {n: pick(n) for n in ('n_base_link', 'c_deck', 'c_payload', 'h_LINK_BASE')}

    a, b = probe(kf), probe(mh)
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
PY
