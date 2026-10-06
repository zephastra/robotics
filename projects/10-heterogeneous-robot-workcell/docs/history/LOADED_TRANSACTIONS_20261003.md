# 010 载货物理事务与在线点云续作 · 2026-10-03

## 当前有效结论

### 连续H供盘链已通过（不等于完整订单）

`p5-continuous-policy-transactions-v5-20261003-01` exit0，924.748 s wall，最终接收停止窗口结束sim190.418 s。相较下文初始化托盘实验，本轮托盘由人形真实供给，随后同世界RGB-D/PCL三件装盘、11物流技能、两段实际货权、接收独立数量/支撑/完整碰撞包络/停止均通过。12项顶层诊断判据PASS，七项接收/货权必需判据PASS。接收速度保守界0.000614647 m/s、漂移0.0000132293 m。装盘后底盘/甲板yaw均0；之前摔倒碰车根因经原网络零速站稳控制与连续本体守卫解决，原失败报告保留。

命令：`.venv/bin/python scripts/run_bounded.py --run-id p5-continuous-policy-transactions-v5-20261003-01 --timeout 1800 experiments/probe_candidate_multi_loading.py --world world_p5_candidate_v5.xml --handover --policy-balance --heading-feedback --transport --transactions --pcl`。

源端和甲板FREE、接收缓冲OCCUPIED归属loaded-diagnostic，不把交付当清空。整个链路仍是旧理想本体状态导航控制，不是Nav2，不含订单库存/双车/36故障矩阵或独立复现。`full_order=NOT_RUN` / `v1_complete=false`。当前全量回归日志`runtime/pytest-policy-continuous-20261003.log`正在运行，结果尚未声明。

### 历史初始化托盘实验

`p4-loaded-transactions-pcl-v5-20261003-02` exit 0，334.5017 s wall、101.526 s sim。
同一个 MjModel/MjData/仿真时间中完成：受限在线 PCL 平面估计、两红块一蓝圆柱实际抓放到自由托盘、11 个物流技能、两段真实货权交接、接收独立 RGB-D 三格计数、真实接触支撑、原停止窗口、缺深度 UNKNOWN。
这是**初始化空托盘在装盘位的单车诊断**，不是完整订单。没有 H 连续供盘、Nav2、双车、有限库存、故障矩阵、36 实例/复现验收声明。

接收停止速度保守界 0.000530730 m/s，漂移 0.000003578 m；原速度限 0.01 m/s、漂移限 0.005 m 未变。
最终源辊道、第一车甲板用实际清空证据释放为 FREE；接收位仍 OCCUPIED，归属 loaded-diagnostic，不能以交付成功冒充缓冲位已空。
两个账本事务都实际走到 COMMITTED/RELEASED，提交依据是接收支撑、源端清空、实际停止；不是技能自述或超时。

## 失败证据保留

| run | 退出 | 原因 | 处理 |
|---|---|---|---|
| p4-loaded-transactions-v4-20261003-01 | 1 | 许可计数闭包变量被 SkillSequence 覆盖 | 更名为 skill_sequence，结构短测保护；旧报告不改 |
| p4-loaded-transactions-v4-20261003-02 | 1 | 已上车但甲板托盘仍晃动，速度界 0.208773 m/s | NEEDS_ATTENTION、TRANSFERRING、两端 OCCUPIED；独立 v5 支撑重验 |
| p4-loaded-transactions-pcl-v5-20261003-01 | 1 | 真实交接/PCL/停止均有效，但视觉混入把手，轮廓多出 51.445 mm | 不放宽 6 mm 尺寸门；修读者并用新 run 验证 |
| p4-receiver-recorded-v5-handles-20261003-01 | 0 | 5/5 录制传感器重析 | 不是新执行；后续 -02 在线复测通过才声明载货链 PASS |

## 独立候选，不改旧世界

v4 SHA256 `642192d7ff8552fa5c6593994c0650d8708237ed159f65c76d8d7bb33c2f5717`。
v5 SHA256 `aa3e9085bf08236193537418f5cb1d32d6254f5cf0b2b7991f4796e82a2a8ac5`。
v5 仅在第一台车甲板相邻主辊间增加 14 个分段被动辊，半径 10 mm；保持主辊、执行器顺序、控制参数、接触参数、托盘自由刚体及旧版本。
新增辊继承甲板父坐标系，随车移动；不是世界坐标固定支撑。
静态审计 12/12、原 3 s 稳定性门正负例 2/2 PASS，报告 `p5-deck-support-v5-build-20261003-01`、`p5-deck-support-v5-settle-20261003-01`。

## 感知与安全边界

- PCL 1.15.1 已在本机，未安装/下载新依赖。实际编译 C++，在线调用 VoxelGrid＋SACSegmentation 平面；返回过期/缺失/异常时 UNKNOWN。
- RGB-D 颜色、轮廓、料位和数量为受限几何算法，不宣称全部由 PCL 实现、通用抓取或真机精度。
- 读者无订单数量/对象标签/运行真值输入。模版尺寸、相机标定和固定工装高度是已声明目录/校准信息。真值只在初始化和独立评测。
- 独立紧凑把手图像片与长壁轮廓分开，原尺寸拒绝门仍适用；缺深度、部分托盘无法解析仍拒绝。
- 理想本体状态、理想触觉、信任外部许可生产者、模型偏置补偿均为仿真假设。
- 一套共同最终写者逐步检查许可；冻结是 SIM_FROZEN_NO_PHYSICAL_STOP_PROOF，不是硬件急停。
- 新 `handover_occupied` 允许活跃授权换主，但资源一直 OCCUPIED；过期或错误令牌拒绝，不复活旧授权。

## 实际命令

```bash
cd ~/projects/010_heterogeneous_robot_workcell
.venv/bin/python scripts/run_bounded.py \
  --run-id p4-loaded-transactions-pcl-v5-20261003-02 --timeout 900 \
  experiments/probe_candidate_multi_loading.py \
  --world world_p5_candidate_v5.xml --transport --transactions --pcl
```

报告 ID 不可重用；复现时换新 ID。运行需要已编译 `runtime/pcl_receiver_plane`，原生构建证据见 `p4-pcl-recorded-floor-20261003-01`。

## 下一门，不能提前写完成

`p5-humanoid-visual-loading-v5-20261003-01` 已 exit 1（719.00 s）：H2 8/8 前置通过，实际抬盘 126.354 mm、视觉源端定位/停止通过、三件抓放及最后独立计数通过，但开始装盘前机械臂遮挡使空盘观察 UNKNOWN。该行 FAIL 阻止顶层通过，原报告保留。
准入缺口已经修复：至多两次真实退臂/重新观察；空盘不是 RESOLVED 且两类皆零则关许可、禁止装盘，不能只记 FAIL 后继续。读者原高度/尺寸门未改。
`p5-humanoid-loaded-transactions-v5-20261003-01` 已 exit124/WALL_TIMEOUT（900 s），供盘和三件装盘推进、第一条物流 MOVE_TO_STATION 成功，但没有完整最终报告，不宣称全链通过。阶段报告与 stdout 原样保留。
`p5-continuous-geometry-transactions-v5-20261003-01` exit1（1800 s墙钟预算未耗尽）：H供盘8/8和真实装盘10/10PASS，运输对接被拒，甲板世界偏航 -0.187305 rad。DOCK_FAIL后仍观察空接收端报PCL_UNKNOWN，是次生诊断错误，不是首因；已加入立即退出/保留首因的3项测试。新独立 `p4-heading-hold-baseline-20261003-01` 有界300 s，量底盘与甲板相对/世界偏航，未测不先定因。
新增不覆盖的阶段检查点；完整碰撞几何必须落在单个料格，使用原10 mm占地容差，不能只看中心。
人形 IK 只在 Runtime 消费目标的每5 tick求解；500项H采样轨迹与先前实际记录完全相同。退出后仅在站姿/手臂权重均为1时，使用与原控制器向量等价的反馈计算，16项纯测试通过。本私有 Runtime 此后不得恢复行走，因为神经网络历史不再推进；实际持续站姿结果仍待当前物理运行判定。
`evaluate_supply_rejection.py` 将预期任务失败与安全拒绝验收分开；11项纯测试通过，不视为物理负例。供盘不成功时应冻结最终写者、禁止观察装盘/抓放/运输/提交货权；机械停稳及恢复不由此证明。
完整 H 验收、失败供盘阻断、全量回归、Nav2 与双车等剩余门仍未收口。

## 航向卡点与真实供盘负例续作

组合链-01装盘10/10通过，但DOCK因甲板世界偏航-0.187305 rad拒绝，必须保持FAIL；不是PCL导致它停下。修复仅保留首因/阻断无意义接收观测，不改对接阈值。
`p4-heading-hold-baseline-20261003-01`初始-0.18 rad：30 sim秒后底盘-0.196559 rad，甲板相对仅-0.00000757 rad，原制动不会修航向。
`p4-heading-hold-feedback-20261003-01`及`p4-heading-installed-feedback-20261003-01`同条件候选差速均回到数值近零；不同源码哈希保留在各run，原停车分支没有被改成反馈默认。
显式`--heading-feedback`启用候选，原有轮速伺服与1.2 Nm限幅保留，皮带驱动期间也维持轮制动。理想航向反馈不等于Nav2定位，停车短测不等于完整运输。

真实失败供盘`p5-supply-rejected-heading-v5-20261003-01`exit1，156.28 wall秒：没有装盘phase、初始装盘观察、运输row或提交账本；WorldOwner冻结对象原因SUPPLY_UNCONFIRMED，机械停稳没有被声明。
独立裁判-01错误期待冻结值为布尔而FAIL，保留。修为核对完整冻结证据对象后-02对同一物理记录13/13安全检查PASS，任务仍FAILED，恢复/机械停止NOT_RUN；不是又执行一次。
当前helper追加原六阶段裁判和脚接触/异常碰撞原仪器；新更严格路径尚待物理复测，不能把负例旧8项前置自动算作新路径验收。
当前全量回归`runtime/pytest-heading-supply-20261003.log`执行中，旧771项不是当前结果。

较早检查点冻结24资产/94源码/110阈值且check通过，旧17资产及110阈值独立核验不变。该清单早于本次完整落格/供盘拒绝补丁，当前需重新生成和check，不能沿用旧绿灯。
当前源码最终全量 pytest **尚未运行**；771 passed / 1 skipped 是较早版本，不替代本次回归。
