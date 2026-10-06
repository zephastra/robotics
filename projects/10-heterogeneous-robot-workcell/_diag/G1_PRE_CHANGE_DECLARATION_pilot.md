# §23.3 变更前声明 — G1 pilot 时钟对齐（2026-10-04，第 2 次变更）

## 1. 当前门
**G1** — 保持机构从"实验探针"变为**受监督技能**（`CONTINUATION_TO_V1_COMPLETION.md` §4）。
本变更只处理 G1 最小测试集中**两个物理反例无法给出拒绝**的问题。

## 2. 失败的可观察原因
`neg`（`--fault-held-release --supervised`）与 `unknown`（`--supervised --initial-visual-unknown`）
两次 run 的实测结果（`reports/p4-belt-g1-neg-20261004T093411`、
`reports/p4-belt-g1-unknown-20261004T093451`）：

| 项 | `neg` | `unknown` |
|---|---|---|
| `supervision_state`（终态） | `CLOSING` | `CLOSING` |
| `observed_refusal` | `None` | `None` |
| `supervision_result.matched` | `False` | `False` |
| shoes 位置（全部采样） | `1.5708 / 1.5708` | `1.5708 / 1.5708` |
| `tray_contacts` | `0 / 0` | `0 / 0` |

根因（**已用 trace 证据定位，不是推测**）：这两个配置下机构**全程停在释放角**，
从未进入闭合，因此契约停在 `CLOSING`。而声明的 `close_deadline_s=40.0` 是**绝对墙钟预算**，
远大于一次真实 run 的 ~20 s 墙钟时长 ⇒ `CLOSE_TIMEOUT` **在任何真实 run 中都不可能有条件触发**。
这正是缺陷形态 #3（"检查不可能失败"族）与 #11（"只在最后才跑的那段代码，是缺陷存活的地方"）。

## 3. 这次只改变的一个因素
**把契约的期限基准从"绝对的、大于整场 run 的墙钟秒数"改为"探针自己声明的阶段时限 × 安全系数"。**

- 不修改：世界文件、几何、控制器、`src/workcell/retainer_supervision.py` 的任何判决逻辑、
  任何阈值、任何已有检查、`neg`/`unknown` 失败配置的物理行为。
- 具体：探针声明 `phase_deadline_s`（其自身 sim 排程换算出的墙钟量级），
  契约的 `close_deadline_s` / `release_deadline_s` 由它派生，并在报告里逐项记录。

## 4. 预计如何反证这个修复
**如果修复是假的**，会出现以下任一现象（任一即视为修复失败）：

1. `neg` / `unknown` 仍然报 `observed_refusal=None`（期限仍未触发）；
2. `pos` 正例开始误报 `CLOSE_TIMEOUT` / `RELEASE_TIMEOUT`（期限过紧 ⇒ 正例回归）；
3. 契约判决从 `CLOSING` 直接跳到 `CLOSED`（说明期限绕过了几何检查）；
4. 反例的拒绝原因**不可辨识**（例如 `neg` 与 `unknown` 报同一个 reason，或报成 `RELEASE_*`）；
5. `supervision_result.matched=True` 但 `transport_authorized_by_contract=True`（反例被授权）。

验收口径（文档原文）：*"每个反例都不给运输授权；所有失败可辨识；没有将 UNKNOWN 记为 PASS；
失效路径不篡改 cargo；原正常静止用例不回归。未测分支明确 NOT_RUN。"*

## 5. 运行预算、输出路径、回滚/保留方式
- **预算**：3 次物理 run（`pos` / `neg` / `unknown`），每次 ~40–45 s 墙钟、11 s sim，
  逐条串行执行，单条 `timeout 180`（`run_bounded.py` 内部再套墙钟守卫）。
- **输出路径**：`reports/<run-id>/`（每个新 run-id 一个目录）；
  诊断日志 `_diag/out/<run-id>.log`。
- **回滚**：本工程**无 git** ⇒ 改动前 `cp` 副本 + `sha256sum` 留档：
  - `experiments/probe_tray_retainer.py.bak_pre_g1`（已有，10495 B）作为**第一次变更前**基线
  - 本次再留 `experiments/probe_tray_retainer.py.bak_pre_g1_clock`（第二次变更前）
  - 真回退方式 = 副本覆盖 + 重算 sha256 核对，**不使用任何删除性命令**。
- **保留**：所有 run 目录与 `_diag` 产物**只归档不删**；旧 attempt 目录已在
  `_diag/archive/`。
- **世界文件**：`assets/world_p5_candidate_v7_hinged_retainer.xml` 本次
  **不触碰**，期望 sha256 逐位不变 `ad6a6fa98a8c83d3c4b6dc94b026398cfdfe8b379bb85f5ec603948f8a347f47`。
