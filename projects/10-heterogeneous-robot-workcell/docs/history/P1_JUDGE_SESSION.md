# P1-JUDGE-08 会话记录 — 2026-09-20

## 要证明什么

在动手补能力之前，先让 H 的判据能失败。原状：`full_H_acceptance` 是一个写死的字符串
`NOT_RUN: empty tray lift/place/contact criteria not implemented`，
探针只在 `tilt > 35°` 或 `base_z < 0.65 m` 时判失败 —— 也就是说**没抓到盘也可能报"完成"**。

## 做了什么

1. `src/humanoid007/tray_task.py`（新）：六个阶段 + 集中阈值 + `evaluate()`。
2. `experiments/probe_h.py`：加 `--sequence`（50 s 驱动六阶段）、资产自检接入首行、
   逐阶段判据写入报告；`--lift` / `--duration` 旧模式保留。
3. `src/humanoid007/runtime.py`：`verify_assets()` 重写 —— 以前它引用了两个没被复制过来的
   文件（`config/scene.json`、`config/task.json`）而抛 FileNotFoundError，且**全工程零调用点**。
   现在：显式登记"声明未带走"的两个文件（并要求它们确实不存在）、拒绝缺失/哈希不符，
   未登记文件列为 `unrecorded` 而非硬失败（010 自己的整合文件不在来源 manifest 里）。
4. 测试 3 → 30 个。每条判据都双向验证。

## 三个自己犯的错（都由"跑一次真的"抓到）

- **`row['t']`**：`Runtime.snapshot()` 的键叫 `time`。判据块因此抛 KeyError。
- **外层 except 吞异常**：只在 `status == 'ERROR'` 时记录 `error`，于是上面那个 KeyError
  被吞掉，留下 `status = DIAGNOSTIC_COMPLETED`、`error: None`、**没有任何阶段判决**的报告。
  已改成无条件记录，status 置 `POST_RUN_ERROR`。
- **`release` / `exit` 锚点错了**：锚在"第一个无接触样本"，而每个 run 开头本来就无接触
  ⇒ 健康 run 因为错误的原因通过（`release` 曾报 `cleared_t = 0.10 s`）。
  已改为锚定最后一次手-盘接触，并加了回归测试。

## 观测到的物理事实（`reports/p1-h-seq-03`）

| 阶段 | 结果 | 数字 |
| --- | --- | --- |
| 抓取 | PASS | 双手同时接触 t=13.00 s |
| 离台 | PASS | +0.1224 m（门限 0.05 m） |
| 稳定持有 | PASS | 双手举持 13.30 s（门限 2.0 s） |
| 放下 | PASS | t=33.30 s 回到支撑面 10 mm 内，双手仍在 |
| 松手后稳定 | PASS | 松手 t=41.50 s，之后 8.50 s 盘速 < 0.01 m/s |
| 退出 | PASS（**险**） | 手臂 t=41.90 s 回到默认姿态，最差 0.148 rad 对上限 0.15 rad |

托盘全程 z 从 0.8500 抬到 0.9724，最后精确回到 0.8500 并静止（t≥42 s 速度恒为 0）。

**两条必须一起读的保留意见**：
- 退出时右手在 t=41.40 s 蹭到托盘（盘速 0.0412 m/s），托盘沿 x 位移 9 mm（0.230 → 0.2393）。
  `runtime.withdrawal_waypoints()` 正是为此而写，序列驱动还没用。
- `exit` 只余 1.3% 余量，不能当作稳健通过。

## 判定口径

`full_H_acceptance` 仍为 **NOT_RUN**：六阶段是在 007 旧托盘上过的，不是 V1 托盘。
下一轮 P1-H-03 建 V1 轻托盘、判据不动、换模型重跑。
