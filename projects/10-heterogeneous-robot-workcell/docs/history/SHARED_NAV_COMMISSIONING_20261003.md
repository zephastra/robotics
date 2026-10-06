# 共同世界 Nav2 接入记录（2026-10-03）

## 边界

本轮只领取既有 P4-BELT-03，先做空载导航接入，不代表订单或 V1 完成。
不修改旧世界、旧阈值或 001～009，不推送。失败 run 原样保留。

## 几何与配置

- `p4-nav-geometry-inventory-20261003-01`：零物理步的实际模型审计，exit0。
- 车载机构初始本体包络为 x [-0.19,2.51]、y [-0.30,0.30]、z [-0.04,0.876] m。
  旧裸车半径 0.26m 不适用于这台带甲板机构的小车。
- `make_joint_nav_profile.py` 生成独立 map、ROS/仿真配置及 profile，`--check` 比较字节。
  导航双 costmap 使用同一个保守完整本体包络，collision_monitor 从实际 costmap footprint 订阅。
- 额外包络包含最大推杆位移、声明的甲板姿态区间。不包含货物：必须另证载荷包含关系。
- 地图来自初始化固定几何在激光高度的 AABB 测绘先验，**不是 SLAM，也不是 3D 防撞证明**。
  静态/运动/机器人/视觉几何分别列出，不用实体标签做运行定位。
- 运输姿态门为独立候选工作条件，原物理验收阈值不变。推杆升降允许 -1mm 的工作区下界
  是求解器微小穿透的显式候选假设，不是机构已经合格；实际运行仍须检查并拒绝越界。

## 执行链

`probe_joint_world_nav2.py` 是唯一物理 owner 的外层驱动：初始化之后没有 qpos 写入或 reset。
同一个 model/data/time 同时运行小车与原人形零速度平衡策略，不复用已跳过神经历史的静态控制。
ROS 子进程无 MuJoCo；输入只含轮速、射线和时钟。真实位置仅进入独立报告。

`joint_nav_worker.py` 启动 9 个必需 Nav2 节点，不启用 docking/following 等多余速度发布者。
必须全部生命周期 ACTIVE，并确认 /cmd_vel 唯一发布者 collision_monitor 才发 goal。
观察器用 wall deadline；goal 用收到的仿真时钟戳，不用 wall 时间。
过期命令在本体侧停止，父进程退出清理只操作核验的专属进程。

## 已保留失败

`p4-shared-world-nav2-empty-v5-20261003-01` exit1：最小启动器未展开
`$(find-pkg-share nav2_bt_navigator)`，bt_navigator 激活失败，没有导航命令。
读取实际错误后用 ament_index 明确解析安装包路径。专属进程核验后正常退出，无全局 kill。
第二轮 `p4-shared-world-nav2-empty-v5-20261003-02` exit0，74.843s wall，sim16.760s：
7项执行内判据PASS，到达误差0.142082468m，停止速度2.48648e-8m/s，漂移1.72805e-8m。
9节点全部ACTIVE，唯一速度写者collision_monitor，83个非零命令消息，车侧59条有效接收。
它缺少独立裁判要求的显式停止起点，旧报告不回填，第三轮重新采集。

新增 `evaluate_joint_nav.py` 从原始轨迹与ROS报告重新判12项，不信执行内PASS标签；
第三轮 `p4-shared-world-nav2-empty-v5-20261003-03` exit0（75.825s wall）；独立裁判
`p4-shared-world-nav2-empty-judge-20261003-03` exit0，12/12 PASS，原始输入哈希随判决保存。
缺轨迹/未知时间/重复时间/NaN/未激活/双写者/漂移/短窗口等负例被测试拒绝。

## 测试与下一门

新增几何/profile/启动拓扑共 19 项隔离测试通过（6.73s）。路径修复后拓扑3项再次通过（0.08s）。
这些不是物理成功证据。下一门：空载实际到达与停稳 → 载荷包络 → 同一连续供盘装载 Nav2 运输。
复杂路径的 3D 机构碰撞、双车、故障停止恢复、完整订单与 36 实例仍未完成。
