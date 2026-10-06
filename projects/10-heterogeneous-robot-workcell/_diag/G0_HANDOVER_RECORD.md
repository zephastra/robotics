# G0 交接记录（§3.3 一页纸）— 2026-10-04

## 1. 准确当前门
**G1（保持机构可监督化）— 物理最小测试集 5/5 完成，G1 判 GREEN（带 3 条如实记录的遗留项）。**
已完成：纯测试 51/51（rc=0）· 突变 17/17 CAUGHT（0 DEAD / 0 BROKEN）·
物理用例 pos / neg（held-release）/ unknown（初始视觉遮挡）/ single（单侧闭合）/
relblk（释放被卡）全部按声明判决；无监督原版 pos / neg 回归 PASS 6/6。
**G2（甲板偏航/运动中保持）未开工。** P4-BELT-03 仍 PARTIAL；P5–P7 BLOCKED。

## 2. 源码 / 资产身份（sha256）
| 文件 | sha256（前 16） | 状态 |
|---|---|---|
| `assets/world_p5_candidate_v7_hinged_retainer.xml` | `ad6a6fa98a8c83d3` | **全程逐位未变** |
| `src/workcell/retainer_supervision.py` | `e6e457715ab21d9b` | 本 session 建 + 注释行 |
| `experiments/probe_tray_retainer.py` | `61885f733bdc430e` | 本 session 改 4 次 |
| `tests/test_retainer_supervision.py` | `6f4304cdc32ebfdc` | 51 测试 |

探针回退链：`.bak_pre_g1`（原版）→ `.bak_pre_g1_clock`（#2 前）→
`.bak_pre_g1_faults`（#3 前）→ `.bak_pre_g1_simclock`（#4 前）。
每步变更均有 §23.3 声明：`_diag/G1_PRE_CHANGE_DECLARATION*.md`（4 份）。

## 3. 活动进程
**无。** 全部 run 经 `scripts/run_bounded.py` 串行执行并已退出（exit 0/1 各有记录）；
无后台进程、无孤儿仿真。run 证据在 `reports/p4-belt-g1-*`（28 个目录，只归档不删）；
`…relblk-20261004T100633` 由声明 #4 标记 **INVALID**（墙钟竞速伪超时），保留不删。

## 4. 使用哪组基线
世界 = v7 候选 `ad6a6fa9…`（静止保持候选，**非** W5 基线）；
监督钟域 = **sim（`data.time`）**，声明预算 close 7.0 / release 2.0 sim s，
观察新鲜度 max_age 2.0（同域）；bench 阈值全部沿用冻结口径未动
（cargo_stop 0.01 m/s & 0.005 m、retention 0.005 m、release margin 0.01 rad）。
⛔ 已知未修：`actual_closed_preload` 只测接触不测位置（声明 −0.025 vs 实测 −0.000636 rad，
39.3×，`preload_check_is_positional=False` 如实报告，未授权不修）。

## 5. 前三个后续动作
1. **G2**：甲板偏航 / 运动中保持的三根因分离（2 ms 接触丢失 / >5 mm 滑移 /
   偏航 > ±0.01 rad），时间对齐证据，一次一个因素。
2. **契约分类锐化（需授权）**：`unknown` 用例根因（视觉 UNKNOWN）目前由
   `close_refusal` 字段承载，契约自身以 `CLOSE_TIMEOUT` 表达；以及 `matched`
   的粒度是"发生了拒绝"而非"指名的拒绝"。两项都是契约语义变更，须先声明再改。
3. **§23.4 回写**：`docs/CLAIMS.md`（G1 结果行）→ `docs/TASK_BOARD.md` →
   `make_handoff.py` 重生成 `docs/HANDOFF.md`（不手改派生文档）。

## 遗留项（如实记录，未修）
- `INITIAL_VISUAL_UNKNOWN` 在 CLOSING 相不快速失败（根因分类在探针侧字段）。
- `supervision_result.matched` 粒度偏粗（见上）。
- 模块 API 参数名 `now_wall_s` 实际承载 sim 钟（模块注释 + 探针
  `contract_clock_domain` 已声明；改名会破坏 51 测试，留给授权后的清理）。
- 明确 NOT_RUN：运动中保持、真实带载 Nav2、连续交接、第二台车、订单 N01、SLAM、
  36 实例冻结（G17–18）。
