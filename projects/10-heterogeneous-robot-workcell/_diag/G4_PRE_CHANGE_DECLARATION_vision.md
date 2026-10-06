# §23.3 变更前声明 — G4 集成资格 #1：v7 视觉正例/遮挡负例（2026-10-04，新增文件，不改任何现有代码）

## 1. 当前门
**G4（§7）把新机构放进完整世界** —— 子项「相机视线 / 零件识别（视觉正例 + 遮挡负例）」。
对应 `build_hinged_retainer_candidate.py` 自报欠账 `camera_visibility='NOT_RUN'`。G2 已 GREEN，G3 判定=证据不支持任何机构补救分支（0.08 m/s 三根因全反驳），按 §6.2「限速不是证明」不借 G2 数据冒充门通过。

## 2. 失败的可观察原因
v7 从未跑过 RGB-D 判决：`probe_vehicle_rgbd.py` 的 `WORLD` 常量钉死 v5、CLI 不暴露 world 参数。
⛔ v5 的 vision PASS **不可引用**到 v7（世界哈希不同）。

## 3. 这次只改变的一个因素
**新增 `experiments/probe_g4_vision_v7.py`**（新文件，零现有代码改动）：
- 判决机与 `probe_vehicle_rgbd.py` 逐行同源（同 observe / observed_load_gate / 同 4 场景 / 同判据）；
- **唯一差异 = `scene(world_path=v7)`**，即世界从 v5 换成 v7；
- 阈值原封继承（counts=2红1蓝、误差 ≤0.02 m、遮挡→UNKNOWN、零深度→UNKNOWN），
  contract 来源声明为 `config/joint_world_v5_loaded.profile.json`（继承；v7 专属配置是 §8/G5 的活，不在本步私造）；
- 零物理步、运行时 qpos 写 0、真值只进判决。

## 4. 预计如何反证
- 若 v7 上 reference/shifted ≠ `RESOLVED` + 2红1蓝 + 误差 ≤0.02 m ⇒ **FAIL，v7 视觉资格不成立**；
- 若 occluded ≠ `UNKNOWN` 或 missing_depth ≠ `UNKNOWN` ⇒ FAIL（把遮挡看穿=仪器不可信）；
- 若渲染通道本身崩溃（EGL/模型编译）⇒ 仪器无效，修仪器重跑，不算物理 FAIL。

## 5. 运行预算、输出路径、回滚/保留方式
- 预算：1 次进程（零物理步，4 场景编译+渲染），`timeout 260` + `run_bounded.py --timeout 240`；
- 输出：`reports/p4-belt-g4-vision-<ts>/report.json`（含 4 张 png + depth npy）；
- 回滚：新文件不引用即可，**不删**；v5/v6/v7 资产零改动，期望 `ad6a6fa9…` 逐位不变。
