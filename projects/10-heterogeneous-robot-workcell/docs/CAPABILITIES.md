# 能力登记（capability register）

指导文档 §9 要求：**每个新能力必须按下面八个字段提供**。本文件只登记**已完成、可引用**的能力；
未完成的写 NOT_RUN，并注明卡在哪。判决链在 `docs/DECISIONS.md`，逐项状态在 `docs/TASK_BOARD.md`。

```text
能力名称与支持范围：
代码/模型版本与报告路径：
输入、前置条件、在线观测来源：
实际物理过程：
成功判据与独立验收：
失败/超时和安全处理：
仍未验证的情况：
如何最小复现：
```

---

## C-001 · 人形双手抓持 V1 托盘（H 链，**DIAGNOSTIC**）

- **能力名称与支持范围**：T800 人形用**双手手指真实闭合**抓住 V1 轻托盘，抬起、悬持、放回、松手、退出。
  支持范围 = **单一 V1 托盘（0.098 kg）**、**声明的固定工装位姿**、**无视觉、无位姿扰动**。
- **代码/模型版本与报告路径**：`experiments/probe_h.py`（探针）+ `src/humanoid007/runtime.py`（`OPEN`/`GRASP`/`ExitPath`）
  + `src/humanoid007/tray_task.py`（判据，与探针分离）。权威证据 **`reports/p1-h-seq-10/`**（V1 托盘），`-08` 同判。
  世界 `assets/world_tray_v1.xml`（源自 007 快照 `assets/combined.xml`）。
- **输入、前置条件、在线观测来源**：输入 = 固定工装位姿下的初始状态；前置 = V1 托盘在世界里、人形在位。
  **在线观测来源 = 仿真内接触计数**（`hand_contacts{left,right}`）**与物体真值**（托盘高度/速度）——
  按指导文档，**真值只用于独立评测，本能力是 `DIAGNOSTIC_ONLY`，不是在线感知控制器**。
- **实际物理过程**：手指关节被命令到 `GRASP` 屈曲姿态 → 与托盘把手形成接触 → 双手抬起托盘
  （+0.1263 m，初始支撑接触消失）→ 悬持 14.0 s → 放回支撑 10 mm 内 → 手指张开 → 手臂沿笛卡尔直线退出。
- **成功判据与独立验收**：`tray_task.py` 的 `THRESHOLDS`（**声明为 DIAGNOSTIC**）：离台 ≥ 0.05 m、
  悬持 ≥ 2.0 s、**每侧接触 ≥ 1**、放回 10 mm 内、松手后速度 < 0.010 m/s 保持 0.5 s、
  退出后手臂偏差 ≤ 0.15 rad、退出位移 ≤ 5 mm。**判据与探针分离**，可用合成样本证明 PASS/FAIL/NOT_RUN 三者都可达到。
- **失败/超时和安全处理**：探针有内部 wall deadline（90 s）与外部 timeout（120 s）；
  `PHASE_WINDOWS` 只把**走完窗口**的阶段记为 driven ⇒ NOT_RUN 是诚实的。
- **仍未验证的情况**：⛔ **不是 V1 验收**（`full_H_acceptance = DIAGNOSTIC_PASS`：声明工装位姿、无视觉、无位姿扰动）；
  ⛔ **未在含有物流链的世界里跑过**（用的是未合并的 007 世界，命名 `LINK_BASE`/`payload`，与合并世界 `h_*` 不同）；
  ⛔ **未记录**：脚底接触与姿态、异常碰撞、控制来源、**运行 qpos 写入次数**（四项空缺，见 C-003）。
- **如何最小复现**：`cd ~/projects/010_heterogeneous_robot_workcell && .venv/bin/python experiments/probe_h.py`
  （详见 `reports/p1-h-seq-10/`）。

---

## C-002 · 候选 B：物流接口统一抬高到 0.85 m

- **能力名称与支持范围**：一个**版本化的候选世界** `assets/world_w5_h085.xml`，整个物流接口
  （源端辊排 + AMT 甲板 + 接收端辊排与导向件）处于**同一个**冠高 0.810 / 托盘起点 0.850。
  **支持范围仅限几何**——它**不是**一个已跑通的物流链。
- **代码/模型版本与报告路径**：`experiments/build_w5_h085_world.py`（有 `--check`）
  → `assets/world_w5_h085.xml`（sha256 `d7f684dfc9a8f7a7`）+ `assets/world_w5_h085.manifest.json`。
  决策 `D101`；验证 `_diag/out_b_verify.txt`。
- **输入、前置条件、在线观测来源**：输入 = `assets/world_w2_logistic.xml`（**只读**，sha256 `7aee7337ee0454c1`
  逐位未变）；无在线观测（纯几何构建，无控制器）。
- **实际物理过程**：**一个统一的 `dz = +0.325 m` 施加到 47 个接口硬件锚点**
  （24 源端辊 + 14 接收辊 + 7 个 `w2_*` 导向件 + `c_deck` + `c_payload`；甲板辊是 `c_deck` 子体随之上升）。
  **没有新增任何升降机构**：接口是**静态抬高**的几何，不是运行时搬运。
- **成功判据与独立验收**：建造器**逐行比对输出与源文件，要求每一行差异都命中一个声明的锚点**，
  越界则整个构建失败且不写入；`--check` 不写任何东西。实测：五组辊子**共一个冠高 0.81**、
  托盘起点 **0.8500**、0.8 s 后仍落在**同两个辊子**上、`--check` **[OK]**。
- **失败/超时和安全处理**：构建失败即不写文件；源文件从不被写。
- **仍未验证的情况**：⛔ **候选世界不得继承旧 W5 的 PASS**——`w2-logistic-05`/`w3-mechanism-07`/
  `w4-skills-05`/`w5-loop-07` 的判决**不适用**；⛔ **未核查**接缝、传感器、对接契约、质量/重心、
  完整碰撞包络（只核查了高度齐平与托盘支撑）；⛔ **人形尚未在此世界里供过盘**。
- **如何最小复现**：`.venv/bin/python experiments/build_w5_h085_world.py --check`（应为 `[OK]`）。

---

## C-003 · H1：真实双手握持并小幅抬起（在候选世界里）—— **PASS**

- **能力名称与支持范围**：人形在**候选世界**里用双手真实握持 V1 托盘，抬起、悬持、放回、释放、退出。
  范围 = 单一 V1 托盘（0.098 kg）、**声明的固定工位位姿**、无视觉、无位姿扰动。
- **代码/模型版本与报告路径**：探针 `experiments/probe_h_w5.py`；能力复用
  `src/humanoid007/{runtime,tray_task}.py`；世界 `assets/world_w5_h085.xml`（sha256 `054f50ac760508ff…`）。
  权威证据 **`reports/p4-h1-w5-01/`**（含 `acceptance.md`）。
- **输入、前置条件、在线观测来源**：输入 = 人形基准 x **4.0785**（可行带 [4.0, 4.15] 内）、
  托盘在供盘工位 x **4.34353** z **0.85**。**在线观测：无。** 锚点集是**声明的固定工位目标**
  （初始化时读一次），不是每步读取移动物体真值。
- **实际物理过程**：手指关节被命令到 `GRASP` 屈曲姿态 → 与托盘提手形成接触（left 1 / right 3）
  → 双手抬起 **+0.1259 m**（初始支撑接触消失）→ 悬持 **13.90 s** → 放回支撑 10 mm 内
  → 手指张开 → 沿笛卡尔直线退出（托盘只动 **0.04 mm**）。
  **不是 weld**：`equalities_involving_the_tray = []`、`free_base = True`。
- **成功判据与独立验收**：`tray_task.py` 的同一套冻结阈值，**一行未改**；判据与探针分离。
- **失败/超时和安全处理**：内部 wall deadline 300 s；`PHASE_WINDOWS` 只把**走完窗口**的阶段记为
  driven ⇒ NOT_RUN 是诚实的；体态失控（tilt > 35° 或 base z < 0.65）立即停并记 `BODY_FALL`。
- **仍未验证的情况**：⛔ 非视觉验收（`scope = DIAGNOSTIC_ONLY`）；⛔ **未接入 W5**
  （候选世界不能引用旧 W5/W3/W4 的 PASS）；⛔ H2/H3 未开始；
  ⚠️ 仍有 **1 对异常碰撞**（`c_fixed_roller_0_0` × `h_lh_rf_tip`，仅 2 个采样的擦过）。
- **如何最小复现**：`.venv/bin/python experiments/build_w5_h085_world.py --check && `
  `.venv/bin/python experiments/probe_h_w5.py --run-id p4-h1-w5-01`

## C-004 · H2：把托盘放到**停止的源端机构**上、释放、退出 —— **PASS**

- **能力名称与支持范围**：人形在候选世界里把 V1 托盘从供盘工装搬到**源端辊道的第一对辊子**上、
  松手、退到辊道包络之外。范围 = 单一 V1 托盘（0.098 kg）、**声明的固定工位站位**、无视觉、
  **本世界/本轨迹/本手指姿态**。
- **代码/模型版本与报告路径**：判据 `experiments/probe_h2_w5.py`；能力复用
  `src/humanoid007/{runtime,tray_task}.py`；世界 `assets/world_w5_h085.xml`（**CHANGE 4 后**）。
  权威证据 **`reports/p4-h2-w5-03/`**（含 `acceptance.md`）。⛔ 更早的 H2 报告（`-01`/`-02`/`y034`/`y036`）
  已标 `SUPERSEDED.md`。
- **输入、前置条件、在线观测来源**：输入 = 人形基准 x **4.13**（`D102` 可行带 [4.0,4.15] 内）、
  托盘起点 x 4.34353 z 0.85、放置点 x **4.45372**（辊 `_0_0`/`_0_1` 的中点，沿用源世界自己的约定）。
  **在线观测：无。** 锚点与放置点都是**声明的固定工位量**，初始化时读一次。
- **实际物理过程**：抓持（left 1 / right 2 接触）→ 抬 **+0.1264 m** → 悬持 **15.5 s** →
  **水平搬运 +0.1102 m** → 降到辊冠 0.810 → 落在 `c_fixed_roller_0_0` 与 `_0_1` 上
  （各 **157** 个接触采样）→ 松手（之后 15.6 s 无手接触）→ 退出（托盘只动 **2.84 mm**）。
  **不是 weld**：`equalities_involving_the_tray = []`、`free_base = True`、**运行 qpos 写入 0**。
- **成功判据与独立验收**：**H1 六阶段契约 = `humanoid007.tray_task` 的冻结阈值，一行未改**
  （`PASS: all six stages`）；另加 **9 条 H2 声明行**（`H2_THRESHOLDS`，报告内标为
  **DECLARED AND NOT FROZEN**，`PASS: all 9 H2 rows`）。
- **失败/超时和安全处理**：内部 wall deadline 300 s；任何涉及托盘的等式出现即**抛错终止**；
  未驱动的阶段记 `NOT_RUN`（从不算 PASS）；`source_stopped` 判**辊面总行程**（实测 0.00002 m，限 0.005）。
- ⛔ **仍未验证（就 H2 本身而言）**：H2 只证明**人形**能把托盘放到源端辊道上并释放；
  **它没有证明链会接过去** —— 那件事由 H3 完成（见 C-005）。候选世界不能引用旧 W5/W3/W4 的 PASS。
- ⚠️ 本能力的世界是 `assets/world_w5_h085.xml`；H3 用的是它的后继 `assets/world_w5_h085_loop.xml`
  （同一世界 ＋ 人形站位写进关键帧）。**H2 的数字只属于 H2 那个世界**。
- **如何最小复现**：
  `.venv/bin/python experiments/build_w5_h085_world.py --check && .venv/bin/python experiments/probe_h2_w5.py --run-id p4-h2-w5-03`


## C-005 · **H3：H 链接进 W5 物流链（同一世界 / 同一时间线 / 同一托盘）** —— **PASS**

- **能力名称与支持范围**：人形把 V1 托盘放到**停止的源端辊道**上并释放之后，**同一个托盘实体**
  被 W5 的单车物流链接走，经 `MOVE_TO_STATION / DOCK / START_TRANSFER / VERIFY_TRANSFER / UNDOCK`
  送到接收端并完成**物权转移**。范围 = 单一 V1 托盘（0.098 kg）、**声明的固定工位站位**、
  **无视觉**、`scope = DIAGNOSTIC_ONLY`、**只在站姿下成立**（`D_MAX = None`）。
- **代码/模型版本与报告路径**：判据 `experiments/probe_h3_w5.py`（**导入** `probe_h_w5` 的仪器与
  `probe_h2_w5` 的阶段驱动，**不复制**）；世界建造器 `experiments/w5_h085_plan.py`
  （audit-then-apply，`--check` 幂等，**先编译再写入**）；世界 `assets/world_w5_h085_loop.xml`
  sha256 `f964477fb8c2ffda…`。权威证据 **`reports/p4-h3-w5-01/`**（正例，含 `acceptance.md` 与
  `diagnostics/`）与 **`reports/p4-h3-w5-neg-01/`**（集成负例）。测试 `tests/test_h3_integration.py`（17 项）。
- **输入、前置条件、在线观测来源**：输入 = 世界关键帧里的人形基准 **x 4.13**（＝ H2 的站位，
  `D108` 说明为什么它不改）、托盘起点 x 4.343532 z 0.85、放置点 x 4.45372。
  **在线观测：无。** 锚点、站位、放置点都是**声明的固定工位量**，初始化时读一次。
  **前置**：世界被放到 home 位姿**一次**（唯一一次 qpos 写入），然后同一个 `MjModel`/`MjData`
  交给**人形运行时**与 `LogisticsPlant`（guest 模式，**拒绝 reset**）。
- **实际物理过程**：手指 **t=12.398 s** 合上 → 抬 +0.1264 m → 悬持 15.5 s → 水平搬运 → 落到
  `c_fixed_roller_0_0`/`_0_1` → **t=34.398 s** 松手 → 退出（离辊道 **0.1247 m**）→
  **链的第一次运动 t=51.286 s**（**比松手晚 16.888 s**）→ 11 条命令全 `SUCCEEDED` →
  托盘 x **4.448 → 12.596**（**链自己搬了 8.16 m**）→ 落在 `receiver_band`。
- **成功判据与独立验收**：**H 六阶段契约 = `humanoid007.tray_task` 的冻结阈值，一行未改**
  （`PASS: all six stages`）；另加 **18 条 H3 声明行**（`H3_THRESHOLDS`，报告内标为
  **DECLARED AND NOT FROZEN**）。正例 `PASS: all 18 judged H3 rows [positive arm], 1 NOT_RUN`；
  负例 `PASS: all 13 judged H3 rows [negative arm], 6 NOT_RUN`。**`NOT_RUN` 不计入 PASS 数。**
- **失败/超时和安全处理**：`WALL_DEADLINE_S = 3000`；任何涉及托盘的等式出现即**抛错终止**；
  **`ROW_ARMS` 强制分臂** —— 在本臂里没被评判的行若给出 PASS/FAIL 即 `RuntimeError`（`D114`）；
  任何行 detail 里留有未格式化的 `%s` 即 `RuntimeError`；未驱动的阶段/行记 `NOT_RUN`。
- ★ **集成契约（四条，全部实测）**：① 一个世界一个 `MjData`（`same_model` / `same_data`，
  `plant.owns_world = False`），全程**不重载**；② 同一个托盘实体（`c_payload` 首尾都是
  **body 124 / 0.098 kg**）；③ **下游等人形松手**（判据是**时间顺序**，且要求"松开在合上之后"）；
  ④ 托盘上的等式约束 **0**、`run` 阶段 qpos 写入 **0**（**结构性**：guest 的 plant 拒绝 reset）。
- ★ **集成负例**（`--negative-grasp`）：人形自己的裁判判 **FAIL**（`lift`/`hold`），
  链的 `START_TRANSFER`/`VERIFY_TRANSFER`/`VERIFY_DELIVERY` **全部拒绝**
  （`TRANSFER_TIMEOUT`、`PAYLOAD_LOST`），托盘**始终留在源端**。
  机制需要**两处替换**（`OPEN` 姿态 ＋ 搬运目标钉在托盘起始 x）—— 只张开手指时，
  **张开的手会把托盘推上辊道**，链照样送达（`D112`）。
- ⛔ **不能外推**：**"人形把托盘交给了车"**（交付的是"放到源端辊道上"，物权转移是链的 TRANSFER）；
  **"人形能走路"**；**"这是生产级工件线"**（无视觉、站位是声明的固定工位）；
  **"候选世界已是 W5 基线"**（不能引用旧 W5/W3/W4 的 PASS）；
  **"H3 覆盖订单 N01"**（无拣选/BOM/库存）；**蹲姿已具备**（`D_MAX = None`）。
  ⚠️ `merged_home` 与世界关键帧差 **3.94 µm**（float32 迭代残差），是横向容差 0.002 m 的 0.197%。
- **如何最小复现**：
  `.venv/bin/python experiments/w5_h085_plan.py --check && `
  `.venv/bin/python experiments/probe_h3_w5.py --run-id p4-h3-w5-01 && `
  `.venv/bin/python experiments/probe_h3_w5.py --run-id p4-h3-w5-neg-01 --negative-grasp`

## C-006 · P4：机械臂的**视觉抓放 + 数量核验**（在敞口的装盘工装里）—— **PASS**

```text
能力名称与支持范围：
  一台固定的 7 轴臂（panda：7 关节 + 1 个夹爪执行器）在**共同世界**里，用**渲染出来的深度+彩色**
  定位分开放置的零件，抓起来放进**从夹具几何推出来的料位**；并**独立核对**每个料位里是什么、有几个，
  看不清时报 UNKNOWN。范围是 `scope = DIAGNOSTIC_ONLY` 的仿真，**不含**转接真机、不含新传感器、
  不含 PCL（见"仍未验证"）。
代码/模型版本与报告路径：
  世界 `assets/world_p4_cell.xml`（**`d44db89876f48fb89be9c8b2130d13cebd5e4b40466c31eec91f084a02148dae`**）
  = 冻结的 `assets/world_p3_cell.xml`（`89ffeab8…`）＋ 敞口装盘工装；
  建造器 `experiments/build_p4_cell_world.py`（写入前编译，断言 nq/nkey/锚点 body/台面高度未变）；
  世界指针与声明的相机 `experiments/p4_fixture.py`；
  仪器 `experiments/probe_p4_arm.py`（02b）、`probe_p4_count.py`（02c）、`arm_bridge.py`（名字重绑）；
  证据 `reports/p4-arm-02/`（**PASS: all 7 judged rows**）、`reports/p4-count-02/`（**PASS: all 7**）、
  `reports/p4-interlock-01/`（互锁，**PASS: all 7**）；测试 `tests/test_p4_fixture.py`（5 条）。
输入、前置条件、在线观测来源：
  输入 = 一张 240x320 的深度图 + 彩色图 + 分割图。**前置**：工装与零件都**落在相机视场内**，
  而相机是**声明的机位**（台上 1.00 m、后撤 0.55 m，瞄在取件区与工装中点）——**默认机位读不到**
  （见"失败处理"）。观测来源只有这三张图 + 推出来的料位表；**真值只由裁判读**（`on_bench`/位姿误差）。
实际物理过程：
  ① `segment_parts` 把**高于 `bench_z + 0.5*min(part)` 且在台面足迹内**的点做 4-连通聚类；
  ② `decide()` 对每个类别要求**恰好一个**候选、≥50 像素、世界尺寸与零件相符，否则 UNKNOWN；
  ③ `ik()`（`arm_rig`）解到 GRASP/PLACE 位姿，位置伺服分阶段走完 approach→descend→close→lift→
  carry→lower→release→retreat，**夹爪是真的**（`a_actuator8`）；
  ④ 02c 用同一份观测把每个已解析的零件按**推出来的料位表**归位，得到占用图。
成功判据与独立验收：
  02b：零件被感知 **0.00577 m**（真值由裁判读）→ 抓起 → **落在推出来的第 1 格**、贴工装地板；
  02c：订单声明 `{'red': 2, 'blue': 1}` 而读者报 **`{'red': 1}`** ⇒ **报出不一致**（不是自证）；
  **把零件拿回自己的起始位姿，核验数 1 → 0**；放到另一格，**报在那一格**。
  ★ 独立验收：**五个相邻机位 x 三个料位**全部解析成功（高 ±0.12 m、后撤 ±0.09 m），
  且 `tests/test_p4_fixture.py` 把"世界可复现/料位约定与托盘一致/地板低于分割平面/工装可及"变成永久闸门。
失败/超时和安全处理：
  `decide()` 返回 UNKNOWN 时**不下任何指令**：实测零件移 **0.000000000 m**、**臂移 0.000000000000 rad**
  （同一函数给真位姿移 0.4989 rad）——**只有成功那一半的探针分不清"视觉抓放"和"永远往下抓"**；
  零件在料位外时**报 UNKNOWN 而不是静默数成 0**，且该消息**只有解析出类别之后才可达**（所以证明是"看见了但放不下"）；
  共享区互锁：人形未清空时**臂的许可被拒**（`REFUSED_NOT_CLEAR`，实测分离 0.191634 < 0.20），
  ★ **最强的两条**：默认机位下**零件后方的远侧隔板**落在图像行 90..93、零件落在行 94..108
  ⇒ 4-连通洪水填充**一步就从零件走进工装壁环**（holder = 1842 px、grey），
  ⇒ 报 `0 candidate(s) for a red part`——**并团的原因是投影压缩（0.45 m 深只映射约 90 行），不是遮挡**。
  这是相机成为**声明参数**的原因，也让 `p4-arm-01` / `p4-count-01` 被标 `SUPERSEDED`。
仍未验证的情况：
  ⛔ **不是 PCL**：`MASTER_PLAN` §5 要求的 PCL 路线**既没接入、也没正式改范围**；
  ⛔ **不是鲁棒感知**：无传感器噪声模型、无时间戳、无 ROS —— 相邻探针 `p4-vision-01` 已量到 σ≥5 mm 即 UNKNOWN；
  ⛔ **工装不保持零件**（肋高 12.5 mm，只做索引；保持属于 `P4-BELT-03`，而 `G-2` 已证明挡刀抬不起来）；
  ⛔ **世界原托盘不可被这台臂服务**（仍差 1.7264 m）；⛔ 不含真机、不含新硬件。
如何最小复现：
  `.venv/bin/python experiments/build_p4_cell_world.py --check && `
  `MUJOCO_GL=egl .venv/bin/python experiments/probe_p4_arm.py --run-id p4-arm-02 && `
  `MUJOCO_GL=egl .venv/bin/python experiments/probe_p4_count.py --run-id p4-count-02 && `
  `.venv/bin/python -m pytest tests/test_p4_fixture.py -q`
```
