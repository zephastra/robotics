# §23.3 变更前声明 — G4 集成资格 #4：v7 源辊道/甲板接缝 + `source_stopped` FAIL 归因（2026-10-04，新增文件，不改任何现有代码）

## 1. 当前门
**G4（§7）第 3 项「源辊道/甲板接缝」，兼作 `p4-belt-g4-h2v7-01` 唯一 FAIL 行的归因仪器。**
事实链：H2 六阶段在 v7 全 PASS、9 行中 8 PASS；`source_stopped` FAIL——最差关节总转 0.176 rad、
峰值 1.2319 rad/s、按 0.035 m 半径换算 6.17 mm > 5 mm 限。母运行 w5-03 同机器同阈值测得
0.0006 rad（0.02 mm）。事件时间定位：t=0→33.5 s 带速 ~1e-10 rad/s（完全静止），
t=34.1–34.3（托盘落辊/松手）尖峰 0.76–1.23 rad/s，此后 ~0.0009 rad/s 缓衰减至 t=49.9。

## 2. 失败/异常的可观察原因（归因假设，待本探针证实或证伪）
- 该行按**体名前缀** `c_fixed_roller_ / c_recv_roller_ / c_deck` 收集"带"：w5 收集 47 体/49 关节，
  v7 收集 **151 体/153 关节**。新增 104 体几乎全是**托辊 idler**（y=±0.10、z=0.80，半径小），
  其中 `c_fixed_roller_idler_0_side±1`（x=4.4537）**正好在托盘落点正下方**——w5 没有它们，
  w5 里托盘只压 2 根大辊。
- 该行把最差关节的转角统一按**大辊半径 0.035 m**换算成表面行程；若动的是小托辊（r≈0.01），
  实际表面行程 ≈ 0.176×0.010 ≈ 1.8 mm，机器高估 ~3.5×。
- 若归因不成立（大辊真的转了 ≥0.143 rad），则 v7 交接存在真缺陷，如实升级。

## 3. 这次只改变的一个因素
**新增 `experiments/probe_g4_seam_v7.py`**（新文件，零现有代码/世界改动；不改 H2 判定机、不动阈值、
FAIL 行维持原判）：
- 世界 = v7 直载；托盘 `c_payload` 以**声明并计数的初始化 qpos 写**放到声明落点
  （x=4.45372、y=0、rest z + 0.010 m 落差——落差值取自 H2 实测 place 间隙 9.99 mm），零初速自由落辊；
- **人形与机械臂钉在家态键帧位形**（每步 qpos/qvel 重置，声明的隔离边界条件；理由：无控制器时它们
  会塌倒砸地、污染落辊事件；钉住的实体距接缝 ≥0.35 m，不参与冲击）；
  带/甲板/托架/托盘全部自由，ctrl=key_ctrl[0]（与 H2 相同）；
- 逐步记录**全部 153 个收集关节的 qpos**（50 Hz），归因到关节名；每个关节用**自己的**辊半径换算表面行程；
- 双口径判决：(a) **w5 口径子集**（名字存在于 w5 收集的主辊/甲板主辊/收端主辊）每根 ≤5 mm 表面行程；
  (b) 全口径 top-10 动者如实列表（含自身半径）；
- 落定后托盘 xy 漂移 ≤5 mm（H2 band_carry 同值——"源有没有搬走托盘"的直接量）；
- 接缝静态几何（免 sim）：源带末辊 6.2537 → 甲板首辊 6.3237 的节距/表面间隙 vs 正常节距 0.080、高度对齐；
- 落点支撑接触计数（托盘到底压着哪些辊，含 idler）。

## 4. 预计如何反证
- 若 w5 口径子集任一主辊表面行程 >5 mm ⇒ 归因不成立，v7 源带在落辊冲击下真的动，升级为交接缺陷；
- 若落定后托盘 xy 漂移 >5 mm ⇒ 落点不稳定，交接面缺陷；
- 若 idler 归因成立但托盘根本没接触 idler（接触计数=0）⇒ 归因链断，重查；
- 接缝节距/高度超差 ⇒ 接缝几何缺陷，如实记录；
- 崩溃/EGL ⇒ 仪器无效，修仪器重跑。

## 5. 运行预算、输出路径、回滚/保留方式
- 预算：20 s sim（步数按 model.opt.timestep）+ 免 sim 静态几何；`run_bounded.py --run-id
  p4-belt-g4-seam-01 --timeout 560 experiments/probe_g4_seam_v7.py`；
- 输出：`reports/p4-belt-g4-seam-01/report.json`；
- 回滚：新文件不引用即可，**不删**；v7 期望 `ad6a6fa9…` 逐位不变（运行后复核哈希）。

---

## 修订 1（2026-10-04，seam-01 运行后、seam-02 前）

**seam-01（`p4-belt-g4-seam-01`）证据保留，run id 不复用；其 3 个 FAIL 行是仪器自身载荷错误，非世界缺陷。**
归因链补全后（全部来自已有数据）：
- H2-v7 自身报告：托盘在松手时**已经落定**（release 段速度 4.5e-6 m/s；exit 行"托盘最多动了 0.06 mm、
  速度从未超 0.0000 m/s"PASS）——9.99 mm 间隙在 33.5→34.4 s 之间**随手下放过程中**闭合，
  松手时不存在 10 mm 自由落体。v1 把 9.99 mm 误读为"松手时落差"⇒ 0.44 m/s 冲击 ⇒ 托盘被发射
  （80 m 行程、idler 750 rad）——这是**雪橇试验**，不是 H2 事件。
- H2-v7 自身数据：尖峰 t=34.1–34.2（0.68–0.76 rad/s 采样值）发生在**左手仍接触**（left:2）时
  （右手 34.1 起已离）；H2 记录过手指×主辊接触对。松手后托盘真实位移 30 µm（4.44924→4.44921）。
  ⇒ `source_stopped` 的 0.176 rad 是**撤手接触扰辊**，被"托盘被搬走"保护对象实际只动了 30–60 µm。

**seam-02（新文件 `experiments/probe_g4_seam2_v7.py`，仍零现有代码/世界改动）：**
- DROP = 0.000（托盘以零速度放到静止表面 = H2 落定态、无手）；10 s sim；本次**保存托盘轨迹**；
- 判决行：(1) 静置态安静（全带峰值 <0.05 rad/s）；(2) 托盘 xy 全程漂移 ≤5 mm；
  (3) 落定支撑接触含**两根指定主辊**；(4) w5 口径每根辊 ≤5 mm 自半径表面行程（静止版的诚实判决）；
  (5) top 动者归因（预期微小）；(6) 接缝几何（同 v1）。
- 预算：10 s sim；`run_bounded.py --run-id p4-belt-g4-seam-02 --timeout 560
  experiments/probe_g4_seam2_v7.py`；v7 期望 `ad6a6fa9…` 不变。

---

## 修订 2（2026-10-04，seam-02 运行后、seam-03 前）

**seam-02（`p4-belt-g4-seam-02`）证据保留。其 4 个 FAIL 全部源于仪器放置错误，非世界缺陷：**
实测托盘几何（逐 geom 读出）：底部板 lowest = 体原点下 **0.040 m**（v1/v2 用的 max 半高 0.030 是
侧壁值）⇒ 真实静止原点 z = 0.810 + 0.040 = **0.850**，与 H2 观测静止 z=0.85 **精确一致**。
v2 把原点放到 0.84 ⇒ 底板陷入辊面 10 mm ⇒ MuJoCo 穿透修正把托盘炸飞（2.6 m 行程、idler 678 rad）
——穿透爆炸，不是物理事件。v1 的"rest 0.84 + drop 0.01"碰巧也落在 0.85（切接触），
静置 ~1.5 s 后下沉 9 mm 再发射——v1 与 v3 初始条件相同，v3 必须复现并解释这 1.5 s。

**seam-03（新文件 `experiments/probe_g4_seam3_v7.py`，仍零现有代码/世界改动）：**
- 静止位姿改用**逐 geom 真实最低点**推导（box: pos_z−size_z；cylinder: pos_z−size_z[1]），
  并新增行 `tray_pose_matches_h2_rest`：|推导静止 z − 0.85（H2 实测）| ≤ 0.005；
- DROP = 0；其余边界条件同 v2（人形/臂/站台钉住，带/甲板/托架/托盘自由，ctrl=key_ctrl[0]）；
- **完整仪器化**：托盘 50 Hz 轨迹 + 带峰值速度按 0.5 s 窗口时间序列 + 接触普查按 0.5 s 时间序列
  ——若再发射，census 会点名 t* 时刻接触体，归因到实体；
- 判决行同 v2（quiet / static / designated / w5_scope / seam）+ 位姿交叉核对行。
- 预算：10 s sim；`run_bounded.py --run-id p4-belt-g4-seam-03 --timeout 560
  experiments/probe_g4_seam3_v7.py`；v7 期望 `ad6a6fa9…` 不变。


## Revision 3 (2026-10-04, before seam-04) — seam-03 result + energy-source localization run

**seam-03 result (`p4-belt-g4-seam-03`, wall 5.8 s, rc=0, v7 hash unchanged `ad6a6fa9…`):**
- rows 4/7 FAIL: `at_rest_state_is_quiet` (peak band 92.803 rad/s, limit 0.05; idler_0_side1
  coasts at 83.917 rad/s CONSTANTLY from t≈1.0 to t=10.0), `tray_static_at_declared_point`
  (excursion 80.671 m), `supported_on_designated_rollers` (end census only c_fixed_roller_0_2),
  `w5_scope_rollers_held` (c_fixed_roller_0_3: 7.589633 rad — IDENTICAL to seam-01's 7.589633,
  deterministic). PASS: `tray_pose_matches_h2_rest` (derived 0.85000 vs H2 0.850, delta 0.00000),
  `seam_geometry_continuous` (gap 0.0029 < 0.020, height delta 0), `worst_mover_reported`.
- Timeline from trace+census: quiet 0–0.40 s (micron settle) → first kick t≈0.41 (z −4.2 mm,
  slides −0.10 m to x=4.3524, hangs off band end on the single line 0_0, census win1/win2) →
  quiet 0.78–1.28 → second kick t≈1.30 (lifted +20 mm = rolled over 0_0's crown) → falls, flies.

**World facts established from the model dump (no sim needed):**
- ALL band actuator entries in key_ctrl[0] are 0.0 (24 mains, 18 recv, 16 deck rollers, deck
  drive). Nonzero ctrl: humanoid motors (hips ±231.44, knees 25, base actuator 255), arm
  targets, retainer drives 1.5708 (retainers at x≈6.5, far away). **The belt is NOT driven;
  the "at rest" premise holds.**
- Idlers: NO actuator, damping 0.0, frictionloss 0.0, armature 0.0 → once spun, spin forever
  (observed: 83.9 rad/s × 9 s with ZERO tray contact, windows 3–18).
- idler_0 pair at x=4.4537 (= PLACE_X exactly), y=±0.10, z=0.80, r=0.01, top 0.810 — flush
  with mains 0_0 (x=4.4137) / 0_1 (x=4.4937), tops 0.810. Tray rests on FOUR flush lines →
  metastable rocking support; undamped idlers act as an energy ratchet. This hardware does
  NOT exist in w5 (47 vs 151 band bodies) — consistent with H2-w5 quiet (0.00293 m) vs
  H2-v7 disturbed (0.00617 m FAIL) and with the seam-01/03 launches.

**Remaining suspects BEFORE convicting the world (both probe-side):**
(a) the pinning clamp (85 joints, qpos/qvel reset each step) with LARGE actuator forces on
clamped dofs (hips 231, base actuator 255) — possible solver pumping; (b) key_ctrl[0] drives
on free bodies (retainer drives).

**New probe `experiments/probe_g4_seam4_v7.py` (this revision authorizes it):** three-arm
paired experiment, same tangent placement, 6 s each — A clamp+key0 (reproduction control),
B clamp+ctrl=0 (drives excluded), C no-clamp+key0 (clamp excluded; humanoid-standing guard
base drop ≤ 0.10 m, else inconclusive). Judged rows: arm_A_launch_reproduced,
arm_B_zero_ctrl_still_launches, arm_C_humanoid_stood, arm_C_still_launches,
energy_source_localized (reporting row with explicit decision table).
Run id: `p4-belt-g4-seam-04`. No world file, no frozen machine, no threshold touched.


**seam-04 result (`p4-belt-g4-seam-04`, wall 8.2 s, rc=0, v7 hash `ad6a6fa9…` unchanged):**
- A clamp+key0: excursion 55.793035 m, first>5mm at 0.46 s, watch idler peak 83.917 rad/s —
  seam-03 launch reproduced (deterministic family).
- B clamp+ZERO ctrl: excursion 0.000088 m (88 um), never crosses 5 mm, watch idler peak
  0.080 rad/s, end pose (4.45363, -1.6e-6, 0.849998) — **QUIET. The world is acquitted.**
- C no-pin+key0: humanoid base drop 0.8462 m over 6 s (COLLAPSED; guard fired, arm C
  inconclusive by design). Tray kicked by the collapsing body (1.61 m), watch idler peak
  0.0002 rad/s. Boundary fact: `key_ctrl[0]` alone does NOT hold the humanoid up.
- Verdict row: **INSTRUMENT** — requires both clamp and drives: the pinning clamp
  (qpos/qvel reset each step) times LARGE actuator forces on clamped dofs (hips ±231.44,
  base actuator 255) pumps the solver. The undamped idlers are passive followers, not the
  energy source. seam-03's four dynamic FAIL rows are instrument artifacts; v7 at-rest
  stability at the place point is confirmed by arm B.
