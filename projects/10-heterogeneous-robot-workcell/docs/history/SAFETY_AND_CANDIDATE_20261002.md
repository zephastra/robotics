# 010 · 运行中安全门与集成候选世界 · 2026-10-02

## 授权、范围与状态

用户明确回答：**“允许新增集成候选世界并重新验收”**。
其问题正文声明：把实际装盘位置放到 Panda 可达范围、第二车增加运输甲板、配置双托盘接收缓冲；保留原冻结资产、阈值、失败证据，不修改 001～009、不推送 GitHub。

本轮没有安装/下载/使用付费 API，没有停止其他工程。仍只有 `P4-BELT-03` 一个 ACTIVE_TASK，状态 **PARTIAL**。候选世界是其整链物理前置，不意味着同时领取或完成 P5。

## 安全门已实现的部分

- `workcell.interlock`：错误角色数量、任一角色无碰撞 geom、非有限参数/距离等，不再“测不到就认为安全”。缺证据拒绝。
- `workcell.runtime_permit`：owner / epoch / generation / 序号 / 墙钟新鲜度 / 取消 / 区域清空 / 停止链健康都必须有效。UNKNOWN 闭锁；恢复需明确的新 generation、停稳证据和新观察，不因租约过期自动清空资源。
- 观察先采样，再读核对墙钟；避免新观察被旧核对时刻误判为未来时间。旧错误仪器报告 `p4-runtime-recovery-20261002-01` 保留，不用于真实故障覆盖。
- `LogisticsPlant.transfer` 可接入 `motion_guard`，每个驱动步核对权限，许可丢失/超时先中断，不继续 seating，也不因制动时碰到接收排冒充成功。
- 停稳使用既定 0.01 m/s、0.5 s、5 mm 漂移预算，没有放宽。
- 停稳无法确认时，显式 `SIM_FROZEN_NO_PHYSICAL_STOP_PROOF`。植入的安全冻结阻止 plant 的 transfer / drive / settle / tick / reset 推进；重复命令零仿真时间、零运输位移。

**边界：**`motion_guard` 是可注入接口，不是已经接入全部技能的系统默认安全门。Guest plant 以外的共同世界所有者仍必须在自己的最终 `mj_step` 前遵守冻结标志；本轮尚未交付完整共享世界调度器，不能宣称任意外部调用已被封锁。

## 实测失败必须保留

`reports/p4-runtime-recovery-20261002-03/acceptance.json`：

- 四种故障：取消、理想测距闯入、墙钟过期、运输预算耗尽均实际中断；闯入是**声明的 mocap 故障球**，不是自由运动行人，不是碰撞预测系统。
- 安全保底 13/13 PASS；**mechanical_stop_result=FAIL、diagnostic_result=FAIL**。
- 自动事务恢复 `NOT_RUN`，完整订单 `NOT_RUN`，V1 未完成。
- 不把仿真冻结等同于机械停稳，不让“安全 PASS”盖掉整体诊断 FAIL。

`p4-stop-dynamics-20261002-01/report.json`：重复 6 个额外停止窗口后，托盘速度上界仍约 0.0781 m/s、角速度峰值约 0.210 rad/s；车已近零速，辊筒最终速率约 10^-4 rad/s。**不是单纯延长 1 s 就能消失**；具体接触振动根因尚未完成定位，不能宣称只是数值噪声。该诊断产生于引入冻结保底之前，当前冻结实现不允许靠重复停止推进物理。

## 新候选世界

资产：`assets/world_p5_candidate.xml`，源码：`experiments/build_p5_candidate_world.py`。
源：`assets/world_w5_h085_loop.xml`，sha256 `f964477fb8c2ffdaf383c613a6f9f9f2b40561b703e7fb040a9c6fb11cb95941`，源文件未改。
候选 sha256 `0b5c7f3bc77d1f9ff3a18efae3a2e725cffedb926cf5ee9e9fcecc3c43485e29`。

声明的布局变更：

1. 实际自由托盘的装盘锚点为源辊道现有 `c_fixed_roller_1_2` 的 x=5.21372 m；Panda 基座 `[5.21372,-0.58,0.61]`，朝 +y，增加真实固定支座；料台位于辊道侧面，顶面 z=0.810 m。
2. 第二车复制**整套**原高接口甲板：被动横向/偏航柔顺、8 个辊筒、驱动及推杆，11 个执行器；不是只加视觉外壳。原两台车仍是**双轮差速声明替身**，不能叫“四轮生产底盘”。
3. 接收排从 14 根延长到 20 根；保留 0.080 m 原冠距和既有辊径/摩擦，不宣称已解决辊隙。实际中心跨度 1.520 m ≥ 两个保守托盘包络与声明间隙所需 1.370 m。只是静态必要条件，不是双盘物理入区。
4. `c_visual_marker` **不是物理盖子**：原本 mass=0、contype=conaffinity=0。候选仅将其置于渲染 group=5，未来相机可排除该光学标签；没有关闭物理货物碰撞。相机管线仍须实际接入并验收。
5. 旧初始状态按 joint name 映射，新增机构使用声明的默认状态；仅库存单件零件随料台初始迁移。不是运行时 qpos 写入。**完整有限库存、第二托盘供给和站点注册仍未交付。**

静态证据：`p5-candidate-static-20261002-04`，使用当前源码，检查实际接收排长度/冠距，候选文件与重建内容逐字一致，PASS。
三格抓取参考 IK 残差分别约 0.0010 / 0.0047 / 0.0319 mm。**IK 收敛不证明碰撞自由路径、带载跟踪或抓放成功。**IK 审计使用独立初始化数据，不往运行世界移动托盘。

前两轮失败保留：`-01` 原 XML 注释内双横线导致严格解析失败；`-02` 审计误写执行器数 10（实际 11）、误把夹爪正常耦合也当货物约束。修正的是审计语义，不是放宽物理验收阈值。

动态稳定性独立探针：`experiments/probe_p5_candidate_settle.py`，复用原 P3 的 `check_settles`，3 s / 0.10 各角色 DOF 速度；新增 c2 角色明确纳入统计。**此判据不是运输停止契约，也不是全链验收。**结果以 `p5-candidate-settle-20261002-01/acceptance.json` 为准，不预填 PASS。

### v1 被动态证据拒绝，v2 单变量修改

v1 稳定性 `p5-candidate-settle-20261002-01` **FAIL**：a 角色关节速度 0.2264 > 0.10；失稳负例正确拒绝。补诊断的 `-02` 显示：初始料台与 Panda unnamed 碰撞网格穿透 **36.071 mm**；3 s 后 `a_joint3=-0.22644 rad/s`、`a_joint1=0.17155 rad/s`，库存零件本身已经停稳。不能将这组物理 FAIL 隐藏在静态 IK PASS 后面。

新版本 `assets/world_p5_candidate_v2.xml` **只将库存料台及其初始零件的 x 偏移从 0.30 改为 0.50 m**，机器人/装盘锚点/辊道/摩擦/阈值不改。原 v1 不覆盖。新增静态判项检验料台与机械臂无初始穿透；旧 0.30 布局必须被测试拒绝。
静态 `p5-candidate-v2-static-20261002-01` **12/12 PASS**；动态稳定性结果以 `p5-candidate-v2-settle-20261002-01` 为准，不沿用 v1 或旧世界的 PASS。
动态 `p5-candidate-v2-settle-20261002-01` **PASS**：3 s 后 a 角色速度由 0.2264 降为约 0，全部角色最大 0.0552（人形）< 既有 0.10 判据；失稳负例仍 FAIL（正确拒绝）。v2 sha256 `e39ae68d1dcb1cc071a22e760d7afb8e587c336a2c394194985120b71a0cc806`。只证明共同保姿的诊断门，不证明运动路径碰撞自由或运输机械停稳。

重建核对 v2（报告 ID 必须换新）：

```bash
.venv/bin/python scripts/run_bounded.py --run-id <新的唯一ID> --timeout 180 experiments/build_p5_candidate_world.py --candidate-file world_p5_candidate_v2.xml --stock-offset-x 0.50 --check
.venv/bin/python scripts/run_bounded.py --run-id <另一个唯一ID> --timeout 180 experiments/probe_p5_candidate_settle.py --world world_p5_candidate_v2.xml
```

## 测试和可复现命令

最终全量：**740 passed / 1 skipped，161.99 s，exit 0**；日志 `runtime/pytest-candidate-final-20261002.log`。冻结及派生交接检查均 exit 0；19 资产 / 78 源码 / 110 阈值。更新前指纹另存于 Windows 本轮暂存目录 `p3_freeze_before_candidate.json`；PowerShell 独立核对旧 17 资产哈希均未变，旧/新 110 条阈值完整 JSON 相等。本轮进程全部退出，无本轮后台运行。

安全修改后的全量日志 `runtime/pytest-safety-20261002.log`：**735 passed / 1 skipped，258.96 s**。输出日志重定向 wrapper 的结尾 shell 变量被外层展开，导致 wrapper exit 1（`exit: numeric argument required`），不是 pytest 失败；因此不宣称该 wrapper exit 0。后续全量需直接返回 pytest 的退出码，不再引用该 wrapper 的 rc。
候选短测试 **4 passed，22.66 s**：源不变与范围限制、完整甲板复制、缺执行器拒绝、真实排长缩短拒绝、物理标签不能被藏成光学标签。全量最终结果在当前状态单独登记。

```bash
cd ~/projects/010_heterogeneous_robot_workcell
.venv/bin/python scripts/run_bounded.py --run-id <新的唯一ID> --timeout 180 experiments/build_p5_candidate_world.py --check
.venv/bin/python scripts/run_bounded.py --run-id <另一个新的唯一ID> --timeout 180 experiments/probe_p5_candidate_settle.py
timeout 600 .venv/bin/python -m pytest -q
.venv/bin/python experiments/make_p3_freeze.py --check
.venv/bin/python experiments/make_handoff.py --check
```

`--write` 只用于建立不存在的候选；不自动覆盖已有候选或重用报告 ID。

## 下一步（最多三项）

1. 完成候选世界动态稳定性、全世界冻结最终写者，以及运输停止失败的接触根因诊断；保留受阻事务，不能靠重置货物恢复。
2. 在同一个候选世界中，人形供盘→辊道运动到实际装盘锚点→可测停车→Panda 对同一自由托盘视觉抓放/数量核验；先一件、后声明完整库存，实际 RGB-D UNKNOWN 必须拒绝动作。
3. 高接口满载连续交付完成后，再接共同世界 Nav2（不能用 chassis 真值位置冒充）、双车/双缓冲事务及 36 实例发布门。P5/P6/P7 仍 NOT_RUN，不提前写 DONE。
