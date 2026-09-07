# Third-party provenance

## EngineAI Native SDK — T800 (optional)

- Source: https://github.com/engineai-robotics/engineai_robotics_native_sdk
- Pinned revision: `335c60e88772c26c7852d0abd6b3c7439037dd8f`.
- Copyright (c) 2025- Shenzhen Zhongqing Robot Technology Co., Ltd. ("EngineAI").
- License: BSD-3-Clause, retained in `licenses/EngineAI-BSD-3-Clause.txt`.
- Source model: `assets/resource/robot/t800/xml/serial_t800.xml`, includes, referenced OBJ meshes/textures, and `assets/resource/environment/ground.xml`.
- Source policy: `assets/config/t800/rl_walking_example/policy/t800_260618_165257_30000.mnn`.
- Policy SHA-256: `cbcb90f86dbb2fde39bdc5a25c8d0530d5c79c7a8f84b1f90863d8c9065b6427`.
- `t800_policy.py` adapts the upstream walking runner's observation history, action ordering, normalization and PD convention. Our command filter replaces the SDK gamepad filter; the Python runner is simulation-only, not the native SDK hardware deployment path.
- This is the SDK's serial 25-joint T800 model, with 22 policy-controlled joints (legs and arms), and waist/head held by PD. It is not T800 Pro or a model of all commercial variants. No mass/inertia/collision/torque-limit changes are made.
- Assets are obtained from the BSD-licensed GitHub SDK, **not** the separately restricted competition bundle. The setup script retains notices; it does not download or execute hardware deployment scripts.
- Walking weights are pretrained by upstream, not trained in this project. Runtime resource hashes are separate from G1 in `assets/manifest-t800.json`.

MNN is an optional CPU inference dependency from Alibaba (Apache-2.0); installing it does not replace G1's PyTorch policy. Third-party dependencies retain their own licenses.

## Unitree RL Gym

- Source: https://github.com/unitreerobotics/unitree_rl_gym
- Pinned revision: `276801e46c5d433564f24658bac64f254b7d2d4b`
- Copyright (c) 2016-2023 HangZhou YuShu TECHNOLOGY CO.,LTD. ("Unitree Robotics")
- License: BSD-3-Clause. See the complete retained text in `licenses/Unitree-BSD-3-Clause.txt`.
- Model: `resources/robots/g1_description/g1_12dof.xml`, its referenced meshes, and `scene.xml`.
- Policy: `deploy/pre_train/g1/motion.pt` (not trained or owned by the authors of project 004).
- Policy SHA-256: `cf668f75b90d1abf73d2b87612a6e76bccc61ff7e083b63582d3f6aaa3c1759d`.
- Configuration: `deploy/deploy_mujoco/configs/g1.yaml`.
- Observation layout, gravity convention, action scaling and PD control in `src/humanoid004/policy.py` are adapted from upstream deployment code. Runtime timing, checks, interaction, tasks, metrics and reports are implemented locally.

The setup script copies only required assets, retains the upstream license, and creates a decorative lab scene with non-colliding goal markers. It does not change robot mass, inertia, joint limits or collision geometry. Upper-body geometry is fixed to the pelvis in this upstream 12-DOF model; it is not a full-body manipulation model.

No affiliation, endorsement or physical-robot readiness is claimed. Full source and binary redistribution must retain the relevant license notices. Downloaded dependencies (MuJoCo, PyTorch, NumPy, etc.) retain their respective licenses.
