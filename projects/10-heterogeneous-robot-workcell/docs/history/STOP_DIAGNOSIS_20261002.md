# 010 停止异常的短程对照诊断 · 2026-10-02

## 本轮推进了什么

新增 `experiments/probe_stop_kinematics.py`，只旁观现有停止过程，不改变运行控制，不绕过停止闭锁。每一步记录实际碰撞几何包围盒角点的刚体速度和位置差分、线/角速度、托盘接触及穿透。球/胶囊/圆柱使用包围盒角点，是保守边界而非表面采样；托盘地板 box 的角点是实际几何点。

结果不是整链验收，统一标 `RECORDED`。独立初始偏移在第一次物理步之前声明，不能用于恢复运行中的货物。小步长只改独立诊断模型，未修改冻结 XML 或停止阈值。

| 报告 | 条件 | 原速度上界峰值 m/s | 几何点边界峰值 m/s | 停止契约 |
|---|---|---:|---:|---|
| p4-stop-kinematics-20261002-01 | 运动至 0.3 s 取消，dt=0.002 | 0.190758 | 0.085321 | FAIL |
| p4-stop-midpoint-20261002-01 | 原初态，未驱动即取消 | 0.019374 | 0.017949 | FAIL |
| p4-stop-crown-20261002-01 | 初态 x 偏移 +0.040 m，未驱动即取消 | 0.109109 | 0.067330 | FAIL |
| p4-stop-fine-midpoint-20261002-01 | 原初态，dt=0.0005，未驱动即取消 | 0.009932 | 0.009057 | 单例 PASS，裕度很小 |
| p4-stop-fine-cancel-20261002-01 | 运动至 0.3 s 取消，dt=0.0005 | 0.160854 | 0.063815 | FAIL |

所有速度以各报告 `held_window` 为准，停止仍使用原来的 0.01 m/s、0.5 s 连续窗口及原漂移门槛。减小 dt 的静止初态单例通过，不代表机械修复或鲁棒性证明。几何点速度与逐步位置差分一致，例如第一例 0.0853208 / 0.0853216 m/s，不能把原保守球包络的偏大解释成纯假报警。

## 目前能得出的结论

1. 托盘发生真实运动，位置漂移小不等于每时刻都停稳。
2. 初始支撑相位是明确的影响因素：仅偏移 40 mm，就从双滚筒支撑附近转到单冠支撑附近，摇晃明显增加。没有证明这是唯一根因。
3. 地板沿输送方向长 130 mm，滚筒中心间距 80 mm；短托盘可能落在单冠支撑位置。运动取消例的最坏时刻只有一根滚筒与地板接触。小步长取消例也未达到机械停稳。
4. 双冠初态仍有接触/摇晃问题，减小步长有影响但不足以修复运动中取消；不得声称“只是求解噪声”。
5. 候选 v2 装盘锚点取在某根滚筒正上方，须重新审视其支撑裕度。仅换正常停车点不能解决任意位置的紧急停止。

## 决策边界与下一步（最多三项）

1. 用户已明确授权独立被动支撑候选与重验。旧物理世界和停止阈值保持。必须检查滚轮互不相交、真实自由转动、托盘/推杆/手臂净距；不得增加隐藏固定或吸附。
2. 授权后先做短程支撑/取消对照，空盘和三件载荷均测；所有判据保持原值，失败保留。证据改善后再接全局最终写者的冻结/恢复。
3. 然后推进候选同一托盘的视觉装盘和数量核验，最后接高接口满载交付、真实 Nav2、双车与发布矩阵。P4/P5/P6/P7 不因此升级 DONE。

## 运行方式

```bash
cd ~/projects/010_heterogeneous_robot_workcell
.venv/bin/python scripts/run_bounded.py --run-id <新ID> --timeout 180 experiments/probe_stop_kinematics.py
.venv/bin/python scripts/run_bounded.py --run-id <另一个新ID> --timeout 180 experiments/probe_stop_kinematics.py --no-drive --initial-offset-x 0.04
.venv/bin/python scripts/run_bounded.py --run-id <再一个新ID> --timeout 180 experiments/probe_stop_kinematics.py --diagnostic-timestep 0.0005
```

本轮五个有界运行均 exit 0，表示诊断完成而非机械 PASS；全部已退出。没有改旧资产、阈值、001～009 或推送 GitHub。

## 获批后的两个支撑候选

源码 `experiments/build_support_candidate.py`。候选仅针对 W2 源辊道，不是完整集成世界；不自动替代默认 WORLD。为新增被动关节移除旧宽度的 keyframe，诊断继续通过 `merged_home` 按名称初始化；不能用这两个无 keyframe 原型替换带集成状态的 P5 世界。

- v1 `assets/world_w2_support_v1.xml`：每个源滚筒间隙增加一根半径 10 mm 的被动自由转动小辊，23 根；支撑冠距变为 40 mm，原滚筒不动，最小滚筒净距 2.1699 mm。无驱动、关节 damping/frictionloss=0，无焊接。取消速度上界从 0.190758 降为 0.066228，但仍 FAIL；小步长变为 0.076726，仍 FAIL。保留原型及失败报告。
- v2 `assets/world_w2_support_v2.xml`：保持半径、冠距和冠高，将新增小辊改成左右两段独立转动，中心 y=±0.125 m、半长 0.0625 m，46 根；原长滚筒仍保留。支撑作用通过真实接触而非固定货物。
- v2 空盘运动取消 `p4-support-v2-stop-20261002-01`：原 2 ms 步长，停止上界 **7.17646e-6 m/s**，漂移 **7.60998e-7 m**，原机械停止判据通过。
- v2 初始偏移 40 mm `p4-support-v2-offset-stop-20261002-01`：停止上界 **7.89171e-6 m/s**，原机械停止判据通过。
- v2 三件自由载荷取消 `p4-support-v2-loaded-stop-20261002-01`：托盘停止上界 **0.00205092 m/s**，漂移 **3.46725e-6 m**，托盘及车原停止判据通过；三件仍在各自格内并与托盘接触。只是初始化载荷短程取消，不是视觉装盘，也不单独证明每件载荷的停止窗口或完整交付。
- 四故障/显式恢复 `p4-support-v2-recovery-20261002-01`：**27/27 checks PASS，安全项 13/13 PASS，机械停止 PASS**。取消/侵入/观测过期/预算到期均实际停稳；取消在源端显式重新授权后恢复输送到甲板 RECEIVED。订单事务恢复与完整订单仍 NOT_RUN。

这说明“短托盘 + 支撑相位 + 横向接触分布”值得作为机构问题修复，而不是把失败归咎为纯速度判据或调小步长。仍未证明全部位置、边界/载荷/高接口/转接缝都会通过。

复现候选（未存在时才允许 `--write`，已有时省略；run ID 必须换新）：

```bash
.venv/bin/python scripts/run_bounded.py --run-id <新ID> --timeout 180 experiments/build_support_candidate.py --split
.venv/bin/python scripts/run_bounded.py --run-id <新ID> --timeout 180 experiments/probe_stop_kinematics.py --world world_w2_support_v2.xml --loaded
.venv/bin/python scripts/run_bounded.py --run-id <新ID> --timeout 360 experiments/probe_transfer_recovery.py --world world_w2_support_v2.xml
```

新增几何测量/候选短测试的最近结果为 **11 passed / 1.06 s**；后补轴向分段测试，最终计数以新测试日志为准。第一次短测试因误写 `test_make_handoff.py` 路径 exit 1、未执行测试；改为现有 `test_handoff_diagnostic.py` 后 8 passed，不隐藏这个命令错误。

## 集成候选 v3（源辊道支撑迁入）

资产 `assets/world_p5_candidate_v3.xml`，sha256 `568a31060d6fd82b1bf345e6dd1ec1a9c78766b371ca18c6276229b47e5d9f30`。从 v2 另建，只加源辊道左右分段被动支撑；托盘、驱动、摩擦、甲板/接收排及阈值不改。原 keyframe qpos/qvel 按 body/关节身份映射，新增被动关节采用自身初始值；驱动顺序/维数保留并有短测试。

- `p5-support-v3-build-20261002-01` exit 0：原静态布局 12 项全部 PASS。
- `p5-support-v3-settle-20261002-01` exit 0：原 3 s / 0.10 DOF 整世界保姿门 PASS，失稳负例正确拒绝；不是运输停止门。
- `p5-support-v3-loaded-stop-20261002-03` exit 0：在装盘工位 x=5.21372、z=0.85 独立初始化，静置 0.2 s 建立重力接触，运动 0.3 s 后取消。原停止上界 0.00225541 m/s、漂移 4.83380e-6 m，托盘/车停稳门通过；三件仍在各自格内、与托盘接触。不是连续供盘、视觉装盘或订单。
- 前两次高接口试验保留：`-01` 前置拒绝后无停止窗口，探针 max(empty) 抛错；修正后 `-02` 明确报 NOT_RUN / NO_STOP_WINDOW_RECORDED。初态与冠面恰好相切、无支撑接触，原 transfer 正确返回 UNKNOWN，不驱动。未删除该前置条件，未把相切当支撑证据。
- 新增反例：前置拒绝不能拿初态静置 trace 冒充停止窗口。最后短测试 15 passed / 0.47 s。
- 较早 W2 满载报告 extra_initialization_writes=1 漏计三件初态偏移；无运行时写入。新探针准确记为 4 次显式初态写入，旧报告未覆盖。

中间全量：749 passed / 1 skipped，232.49 s，exit 0，`runtime/pytest-stop-diagnosis-20261002.log`，在 keyframe 映射/空窗口修正之前。**最终全量 752 passed / 1 skipped，177.74 s，exit 0**，`runtime/pytest-stop-final-20261002.log`。本轮有界仿真与测试进程均退出，没有本轮后台运行。
新冻结清单 22 资产 / 80 源码 / 110 阈值。PowerShell 按旧清单逐文件 SHA256 核验，旧 17 资产不变，110 阈值完整 JSON 相同。第一次比较把 artefacts 数组误当对象，输出无效；纠正后才作该结论。

```bash
.venv/bin/python scripts/run_bounded.py --run-id <新ID> --timeout 180 experiments/build_support_candidate.py --integrated --split
.venv/bin/python scripts/run_bounded.py --run-id <新ID> --timeout 180 experiments/probe_p5_candidate_settle.py --world world_p5_candidate_v3.xml
.venv/bin/python scripts/run_bounded.py --run-id <新ID> --timeout 180 experiments/probe_stop_kinematics.py --world world_p5_candidate_v3.xml --loaded --initialize-loading --settle-initial-s 0.2
```

下一步最多三项：①全局最终命令写者的冻结/恢复、甲板/接收段停止重验；②同一自由托盘的视觉装盘/数量核验；③高接口连续满载交付后接 Nav2、双车与发布矩阵。P4/P5/P6/P7 不升级 DONE。
