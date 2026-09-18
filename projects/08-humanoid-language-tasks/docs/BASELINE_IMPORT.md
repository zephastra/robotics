# BASELINE IMPORT — 008

How 008 obtains its physical baseline (T800 + bilateral Allegro + head RGB-D
camera + the 22-action walking policy) **independently of 007**, and what was
verified.

## 1. What was imported

| Artifact | Source (007 baseline) | Destination (008) |
| --- | --- | --- |
| Combined MuJoCo model | `assets/combined.xml` | `assets/combined.xml` |
| T800 serial model + meshes + textures | `assets/t800/` | `assets/t800/` |
| Allegro left/right hands | `assets/allegro/` | `assets/allegro/` |
| Walking policy weights | `policies/t800/walking.mnn` | `policies/t800/walking.mnn` |
| Licence notices | `licenses/*` | `licenses/*` |
| T800 policy/model/stand config | `config/t800/*.yaml` | `config/t800/*.yaml` |
| Fixed-fixture task/scene/workcell config | `config/{task,workcell,scene}.json` | `config/*.json` |

83 files, each hash-verified against the 007 source manifest
(`assets/manifest.json`), then recorded in `assets/baseline_manifest.json`.

## 2. Upstream provenance

| Component | Repository | Pinned revision | Licence |
| --- | --- | --- | --- |
| T800 model + walking policy | `engineai-robotics/engineai_robotics_native_sdk` | `335c60e88772c26c7852d0abd6b3c7439037dd8f` | BSD-3-Clause |
| Allegro hands | `google-deepmind/mujoco_menagerie` → `wonik_allegro` | `8161bba264d7fa7c99ca301e91e7fb44737676ad` | BSD-2-Clause |

Walking policy SHA-256:
`cbcb90f86dbb2fde39bdc5a25c8d0530d5c79c7a8f84b1f90863d8c9065b6427`

## 3. What is **not** imported

007's `task.py` and `app.py` are **not** copied. They are used only as a
reference for the unsplit baseline task that 008 re-implements against its own
`simulation/` modules (section 6.1). The adaptation mapping is recorded in
`assets/baseline_manifest.json` under `adapted_code_mapping`.

## 4. Reproduce the import

```bash
cd ~/projects/008_humanoid_language_tasks
.venv/bin/python scripts/import_baseline.py --source ~/projects/007_humanoid_visual_transport
```

Idempotent: it refuses to overwrite already-imported files, and it never moves
or deletes the source (section 6.4).

## 5. Status

- **Import**: complete, hash-verified.
- **Independence**: not yet proven by a clean-room run — see
  `scripts/check_independence.py` and the P4 gate.
- **Full 30-example regression**: **not re-run inside 008 yet** (experimental).
  Only a structural smoke test and a single unsplit transport episode are run in
  P4; the full regression is a batch run that must be started explicitly.
