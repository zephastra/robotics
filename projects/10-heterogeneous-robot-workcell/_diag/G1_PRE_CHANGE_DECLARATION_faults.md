# §23.3 变更前声明 — G1 反例注入开关（2026-10-04，第 3 次变更）

## 1. 当前门
**G1** — 最小物理测试集还差两项：**单侧保持缺失**、**释放不能完成**（文档原文：
"当前静止正例、保持释放位负例、初始视觉遮挡 UNKNOWN、单側保持缺失、释放不能完成"）。
前三项已跑通（`…093803` / `…095736` / `…095838`）。

## 2. 失败的可观察原因
探针目前只有 `--fault-held-release` 与 `--initial-visual-unknown` 两个注入开关，
**没有**构造"单侧闭合"与"闭合成功但释放被卡"这两个反例的手段。
（`release()` 的契约语义已核实：几何未到位 → `RELEASE_NOT_AT_TARGET`（非故障），
接触未清 → `RELEASE_NOT_CLEARED`，预算耗尽 → `RELEASE_TIMEOUT`。）

## 3. 这次只改变的一个因素
**给探针纯加法新增两个反例注入开关**（默认关闭，默认路径逐字不变）：
- `--fault-single-side`：闭合窗口内只把第一条 shoe 命令到闭合位，另一条保持在释放位；
- `--fault-release-block`：sim ≥ 7 后不再施加释放命令（闭合成功、释放被卡）。
并为二者补上**期望结果分支**（bench 期望行 + `supervision_result` 分类）。
不改：世界文件、`src/workcell/retainer_supervision.py`、任何既有检查与阈值、
既有两个开关的行为。

## 4. 预计如何反证这个修复
1. `--fault-single-side`（监督）：契约**不得**进入 `CLOSED`（单侧几何永不齐），
   预算耗尽 → `CLOSE_TIMEOUT`，`transport_authorized_by_contract=False`；
   bench 期望行 `expected_single_sided_close=PASS`。
2. `--fault-release-block`（监督）：闭合合法通过并被授权（`authorized_ever=True` 是**正确**的），
   但终态必须停在 `TRANSPORTING`（不得 `OPEN_VERIFIED`），release 决策的
   `reason_code=RELEASE_NOT_AT_TARGET`，bench `expected_release_blocked=PASS`、
   总 verdict=FAIL。
3. 监督正例回归：默认路径必须仍走完全生命周期 `OPEN_VERIFIED`、6/6 PASS。
   若正例因本次改动回归，修复视为失败。

## 5. 运行预算、输出路径、回滚/保留方式
- 预算：3 次物理 run（single-side / release-block / pos 回归），各 ~50 s 墙钟，
  串行，`timeout 180`。
- 输出：`reports/<run-id>/` + `_diag/out/`。
- 回滚：改动前 `cp` → `experiments/probe_tray_retainer.py.bak_pre_g1_faults` +
  sha256 留档；回退 = 副本覆盖。**无 git，不使用删除性命令。**
- 世界文件不触碰：期望 `ad6a6fa98a8c83d3c4b6dc94b026398cfdfe8b379bb85f5ec603948f8a347f47` 逐位不变。
