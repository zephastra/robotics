# G7 开工计划（2026-10-05，v7 base 档路径）

> 门定义（CONTINUATION §10）：把隔离 Nav2 路径接到实际源端链：H 供盘 → RGB-D/PCL 装盘 → 视觉核数 → 源端到甲板交接 → 实际支持稳定 → 关闭保持鞋 → 资格成立 → Nav2 → 停车 → 接收许可 → 释放确认 → 输送卸盘 → 接收确认。
> 关键顺序红线：**不能夹着托盘强行输送；不能在运动中提前释放。**
> ledger 与每次支持/占用证据相符是验收的一部分，不止「最终在接收区」。

## 0. G6 继承边界（原样随行，不改门）
- 到达资格：v7 Nav2 三档 0.074–0.155 m 全 PASS（短腿 0.6 m 口径）。
- slip 门：三档全 FAIL（5.2/8.4/11.4 mm vs 5 mm），已穷尽控制空间并归因为「Nav2 驱动 × 弹性鞋保持」结构性质——**授权项挂账，不阻塞 G7 短腿**。
- 长腿（4.63 m）：posture 门会锁存（c_deck_yaw 出带）——G7 行程若超短腿口径，必须分段或记录边界。
- 鞋带 [−0.08,+0.05]、CLOSE_TARGET=−0.025、两阶段确认（1 s 保持+5e-3 漂移）——G7 关鞋沿用 run-03 已验证机制。

## 1. 现有资产映射（全部实测，不假设）
| 资产 | 状态 | G7 角色 |
|---|---|---|
| `src/workcell/physical_transfer_session.py`（74 行） | 完整可用 | `execute(request, launch_evidence, drive, confirm)` 编排：reserved→TRANSFERRING→COMMITTED；失败保持双锁；`release` 需 COMMITTED+量测清空 |
| `src/workcell/transfer.py` TransferLedger（567 行） | 完整可用 | request/advance/interrupt/resolve/reconcile/summary |
| `src/workcell/resources.py` ResourceTable（470 行） | 完整可用 | reserve/confirm_occupied/handover_occupied/release/expire |
| `src/workcell/ledger.py` OrderLedger（351 行） | 完整可用 | submit/advance/constrain/cancel——订单层 |
| `experiments/probe_h3_w5.py`（1463 行） | **H3 先例**：直接用 ResourceTable+TransferLedger（L346/351，不经编排器）；SAME WORLD ONE DATA 纪律（L911） | G7-a 的模板；硬编码 `world_w5_h085_loop.xml`（L65） |
| `p4-belt-g4-h2v7-01` | H2 换世界 阶段 6/6、行 8/9 | H 链在 v7 已有运行证据（H2 层） |
| `probe_vehicle_nav2.py` + worker + 档位 | `--candidate v7` 三档；鞋两阶段确认机制 | G7-b 的运输腿；需加「开局取盘」场景变体 |
| v7 世界 `ad6a6fa9…` | 冻结逐位未变 | 全程不重铸 |

## 2. 阶段拆解（每段独立验收，可分别回写）
- **G7-a：H3 链 v7 化冒烟**。复制 H3 探针为 v7 变体（世界常量+布局换 v7；H 站/源 band 坐标按 v7 盘点），先跑「H 供盘→transfer COMMITTED」切片。验收：transfer COMMITTED + ledger summary 与支持证据相符。前置侦察：H3 用到的 plant 方法在 v7 世界（station 位置、band 几何）是否成立。
- **G7-b：运输腿接真实开头**。probe_vehicle_nav2 场景变体：开局鞋**开**、托盘在源端（替代 declared shoes-closed）；探针先执行取盘（视觉核数→关鞋两阶段确认→资格成立）再进 Nav2 腿。验收：`runtime_qpos_writes=0` 保持、取盘有物理证据、运输腿判决照旧门。
- **G7-c：接收端卸盘**。到站→接收许可（ResourceTable）→释放鞋（量测确认托盘离开鞋约束）→输送卸盘→接收确认；全程 PhysicalTransferSession/TransferLedger 记账。验收：ledger 链与物理证据逐条相符。
- **G7-d：五失败测试**。源端拒绝 / 半程停止 / 夹持未释放 / 接收忙 / 卸盘失败——每个都要「中断或证据缺失不得变为 RELEASED/成功」的门行为证据（G1 监督钟域=sim 纪律沿用）。

## 3. 关键风险（开工前已知）
0. plant 依赖面已实证：H3 用到的 17 个 LogisticsPlant 方法/属性（chassis_at_home/chassis_x/clock/deck_row/dock_residuals/dock_x_source/model/park_control/pitch/qpos_writes/source_band/stations/tray_state/tray_support/zone_of/data）与 Nav2 探针同源，run-03..06 在 v7 已跑通其超集——G7-a 剩余未知仅 H3 专用属性在 v7 的**取值语义**（source_band/dock_residuals/zone_of 的坐标与 W5 布局不同）。
1. H3 的 W5 链与 v7 Nav2 链的**世界坐标语义不同**（H3 的源 band/receiver 位置是 W5 布局）——G7-a 不是改一行 WORLD 常量，布局语义要逐一对 v7 盘点值。
2. G7-b 改探针场景初始化=改 `scene()` 调用契约——blast-radius 涉及 run-03..06 的可复现性，需新 run-id 前缀隔离，不覆盖旧证据。
3. H3 在一个循环里驱动 H+W5 两执行体；v7 里 Nav2 腿是 ROS 进程链（bridge/worker），**事件循环结构不同**——不能硬套 H3 的同步循环，要定义交接点（谁在什么证据下把控制权交给探针主循环）。
4. slip FAIL 与长腿锁存作为「已归因边界」随行——G7 报告里必须如实列出，不许被「最终在接收区」掩盖。

## 3.5 G7-a 物理机制侦察结论（2026-10-05 第二轮，全部实测）
- **v7 执行器全景**（MuJoCo 枚举，共 137 个）：源端= 24 辊 +  推杆；甲板= +  +  鞋；接收端= 20 辊；双车 ；臂 。
- **带链交接机制既有**： 含完整阶段机 （transfer_id=，SkillSequence+SkillAdapter 编排，L101/311-313）——交接不是从零写，是**移植**。
- **world 语义参数化已被实证**： 实例化路径即 Nav2 链在 v7 的路径；probe_p4_belt 的 （=w2_logistic，模块常量）只是入口硬编码。
- **G7-a 落地方案定稿**：以 probe_p4_belt 阶段机为骨架做 v7 变体探针（新文件，复用其 xfer_source_to_deck 判据），TransferLedger/ResourceTable 记账挂 START/VERIFY 两点。剩余唯一侦察点：probe_p4_belt main 的场景构造对 v7 的适配面（下一轮第一步）。

## 4. 下一步（按序）
1. G7-a 前置侦察：probe_h3_w5.py 的 plant 依赖面 + v7 世界 H 站/源 band 坐标提取（只读）。
2. G7-a §23.3 声明 → v7 变体探针 → 冒烟 → 判决 → 回写。
