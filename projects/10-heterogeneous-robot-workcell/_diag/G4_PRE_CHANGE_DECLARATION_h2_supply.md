# §23.3 变更前声明 — G4 集成资格 #2：v7 人形供盘（H2 判定机世界交换）（2026-10-04，新增文件，不改任何现有代码）

## 1. 当前门
**G4（§7）「人形脚与供盘」在 v7 内复现。**
H 证据链（H1 `p4-h1-w5-01` / H2 `p4-h2-w5-03`）的世界是 `world_w5_h085(.xml)`；
⛔ 世界哈希不同 ⇒ 不可引用到 v7。§7 明确要求候选世界内重查「人形脚与供盘」。

## 2. 失败的可观察原因
`probe_h2_w5.py` 的 `WORLD` 常量钉死 `world_w5_h085.xml`，CLI 不暴露 world 参数。
已实测（dump5/dump6，2026-10-04）：v7 与 w5_h085_loop 共享命名元素 684 个中 679 个逐属性相同，
`only_in_w5=0`；H2 依赖的全部锚点在人形/带一侧逐位不变——站位 4.13（家态键帧第一位）、
供盘位 4.45372（滚轮 0_0/0_1 中点）、把手 y=0.30、托盘家态 (4.34353, 0, 0.85) 在源带上。
差异只在：臂/桌/零件挪位（5.21/5.71 一带）、车在接收坞 9.03、鞋家态张开、新增 c2/recv14-19/retainer 执行器（全在尾部）。
⇒ 工作区 (x≈4.0–4.6) 几何不受 v7 新增物影响，但**这只是预测，判决靠跑**。

## 3. 这次只改变的一个因素
**新增 `experiments/probe_g4_h2_v7.py`**（包装器，零现有代码改动）：
- `import probe_h2_w5 as H2; H2.WORLD = v7`——唯一改动因子；
- 六阶段合同（`tray_task`，冻结阈值）+ 9 行 H2 判决（声明阈值）逐行复用；
- `--station-x/--place-x/--handle-y/--hand-roll/--duration` 全默认；
- 报告内 `probe` 标签沿用母模块字符串"H2 in candidate B"——**溯源以 `world`+`world_sha256`+`command`（含本包装器文件名）为准**，不改证据文件。

## 4. 预计如何反证
- 六阶段任一 NOT PASS，或 `h2_overall` FAIL ⇒ **v7 供盘资格不成立**，如实记录；
- `source_stopped` / `no_abnormal_collision` 任一 FAIL ⇒ 同上，不许甩锅给"世界换了"；
- 进程崩溃/EGL 失败 ⇒ 仪器无效，修仪器重跑，不算物理 FAIL。

## 5. 运行预算、输出路径、回滚/保留方式
- 预算：sim 50 s（SEQUENCE_SECONDS），母模块自带 WALL_DEADLINE_S=300 s；
  `run_bounded.py --run-id p4-belt-g4-h2v7-01 --timeout 560 experiments/probe_g4_h2_v7.py`；
- 输出：`reports/p4-belt-g4-h2v7-01/report.json`（含 samples）；
- 回滚：新文件不引用即可，**不删**；v7 期望 `ad6a6fa9…` 逐位不变（运行后复核哈希）。
