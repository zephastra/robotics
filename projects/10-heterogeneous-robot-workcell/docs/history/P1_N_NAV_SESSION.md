# P1-N-07 会话记录：单车 Nav2 实际到站（2026-09-20）

> 交付状态：**PARTIAL**。控制器确实把车开到目标点并停稳，到站误差已记录，
> 但真值到站误差 **0.431 m** 超过声明的 0.15 m 目标容差。
> 权威记录 `reports/p1-n-nav-06`（30 项判据：28 PASS / 2 FAIL，两个 FAIL 同一根因）。
> 判据文件 `experiments/evaluate_n_nav.py`；编排器 `experiments/probe_n_nav.py`。

## 这轮做了什么

在同一世界、同一桥上把 Nav2 整链接上：静态地图 → map_server → AMCL → planner_server →
controller_server → velocity_smoother → collision_monitor → 命令门 → MuJoCo 车轮。

新增：
- `experiments/make_map_n.py`：从**世界几何**生成静态地图（不手画），`--check` 幂等；
  free space 用从起点洪水填充得到的**可达**空间，所以墙外那圈被写成 occupied 而不是 free，
  不需要任何硬编码的墙坐标。同时导出 `raycast_grid()`，供判据把地图和真实 scan 对比。
- `experiments/make_nav2_params.py`：从 nav2 自带的默认参数文件**派生** `config/nav2_n_probe.yaml`，
  每一处偏离 stock 都是脚本里的一条**具名条目**并写明理由；`--check` 在 nav2 升级后强制人重读。
- `assets/maps/arena_n_probe.{pgm,yaml}`：181×181、0.05 m、origin `(-4.525, -4.525)`。
  原点选得让**起点正好落在像元中心**，这样"透过地图投射射线"与"真实 scan"才是同一件事；
  代价是 origin/resolution 不是整数（`StaticLayer` 会因此打一条 WARN，无害，已记录）。
- `experiments/check_n_nav.py`：独立观察者，记录话题计数、发布者**节点名**、**QoS**、
  **消息类型**、TF 边、首帧完整 scan、`amcl_pose` 采样。
- `experiments/probe_n_nav.py`：编排器。起 sim/桥/观察者/Nav2，等 lifecycle 到 ACTIVE，
  发目标，SIGUSR1 优雅收尾，清理并扫描残留。
- `experiments/evaluate_n_nav.py`：判据，纯 Python，可对合成输入判 FAIL。
- `experiments/patch_n07.py`、`patch_n07_config.py`：对既有文件的补丁（幂等 + 指纹校验）。
- `tests/test_p1_n_nav.py`：25 条。**测试 82 → 107 passed。**

## 四个必须记下来的坑（每个都花了一次运行）

### 1. `filters: []` 会把节点打死在构造期

第一个冒烟跑里 `controller_server` 和 `planner_server` 直接 SIGABRT：

```
parameter_value_from failed for parameter 'filters':
Invalid parameter value: rcl parameter structure contains no value
```

rclcpp 在**构造节点时**解析 YAML 参数覆盖，空的 YAML 序列会变成"没有值的结构"。
症状极不直接：lifecycle 节点停在 `unconfigured`，而 `lifecycle_manager_navigation`
疯狂刷 "Waiting for service controller_server/get_state"，把那次的 `nav2.log` 刷到 **177 MB**。
修法：**把 `filters` 键整个省掉**（不是清空），并让生成器**拒绝输出任何 `key: []`**。

### 2. 行为树的握手超时比一个时钟 tick 还短

第二个跑：目标在 206 ms 后被 BT 放弃（error_code 107），而 `controller_server`
**在放弃后 52 ms 才收到**这个目标。stock `default_server_timeout: 20` 是拿 20 Hz 的
`/clock`（一 tick 50 ms）当时间基准用的 —— 20 比一个 tick 还短。改成 1000 后，
feedback 从 7 条变成 3379 条，BT 能跑完整的 recovery 阶梯。

### 3. ★ 命令话题的类型是 `TwistStamped`，而类型不匹配是**静默**的

这是本轮最值钱的发现，也解释了为什么前三次运行"什么都对但车不动"：

```
cmd_vel_nav      geometry_msgs/msg/TwistStamped   (controller_server, behavior_server)
cmd_vel_smoothed geometry_msgs/msg/TwistStamped   (velocity_smoother)
cmd_vel          geometry_msgs/msg/TwistStamped   (collision_monitor, docking_server, following_server)
```

桥接订阅的是 `geometry_msgs/msg/Twist`。**DDS 会为 QoS 不匹配报警，但不会为类型不匹配报警** ——
没有一行日志、没有异常、计数为 0，症状与"控制器没发命令"完全一样。
而 nav2 自己内部是一致的，所以它自己也不报错。
我第一次把 0 条命令归因到 `/cmd_vel` 上那条 "incompatible QoS" 警告，**是错的**：
同一次跑里观察者自己用 RELIABLE 订阅也是 0 条，说明话题本身就没有流量。
修法：按**实测到的类型**订阅（默认 `twist_stamped`），并把"线上看到的类型"写进报告
（`publisher_types_seen` / `type_mismatch`），让下一次变化变成**响亮失败**而不是又一次静默。

### 4. 一次修一个变量、但一次量一整条链

第 4 次跑把 `cmd_vel_nav`、`cmd_vel_smoothed` 一起插桩，报告里直接给出
`{'cmd_vel_nav': 0, 'cmd_vel_smoothed': 0, 'cmd_vel': 0}`。
"车不动"对四跳是同一个症状，所以必须**要么四跳都量，要么就一直猜**。

## 结果

| 项 | 实测 |
| --- | --- |
| lifecycle | 13 个节点全部 `active`；两个 manager 声明的并集 = 发现的集合，无缺无多 |
| `/clock` 发布者 | 恰好 1 个（`workcell010_n_bridge`）|
| `/tf` 边 | 动态 `map→odom`（amcl）+ `odom→base_link`（桥）；静态 `base_link→lidar_link`；**每条边一个写者** |
| 地图 vs 首帧 scan | 181 线中 **99.4%** 在 0.2 m 内一致（起始位姿处实测）|
| 命令链 | `cmd_vel_nav` 267 / `cmd_vel_smoothed` 277 / `cmd_vel` 277；桥收到 277、限幅 0 |
| 命令门 | 接受 335、拒绝 0；HOLDING 8590 步、SILENT 9162 步；结束时已回退到零 |
| 目标 | 接受 → `status 4`（SUCCEEDED）、error_code 0、523 条 feedback，13.6 s 完成 |
| **到站（真值）** | 终点 `(2.231, 2.178, 0.066)`，离目标 **0.431 m**、航向差 0.066 rad |
| **绕障（真值）** | 路径长 3.184 m（直线 3.538 m）；离 `pillar_a` 最近 **0.514 m**（需 ≥0.430 m 才算没碰）|
| 停稳 | 最后 1.0 s 平均速度 0.0000 m/s |
| 轮式里程计 | 位置误差 **0.0664 m**、航向 0.0025 rad |
| **AMCL vs 真值** | 15 个配对样本：位置中位 **0.247 m** / p95 0.303 m；航向中位 0.013 rad |

**结论：整链通了，"到站"没达标。** 而且原因不是控制器 —— 它确实把车开到它认为的目标点并停下，
是**它认为的位置偏了**：AMCL 沿路径**超前**真值 0.24–0.31 m（航向却是对的，中位 0.013 rad），
于是目标检查器在真值 0.431 m 处就判成功（它自己算出来是 0.132 m < 0.15 m）。
同一段路上**轮式里程计只有 0.066 m** —— 也就是说，在这个 8×8 m、只有三根小柱子的场地里，
AMCL 的扫描匹配比它自己的轮式里程计**差 4.7 倍**。

这和 009 是同一类缺陷：**自报位姿说在容差内，真值说不在，而 `localization_valid` 一路为真。**
所以本轮把判据定成"到站必须用真值量，绝不采信 nav2 自己的结论"。

## 未做

- 只跑了一个目标点，**没有重复性**（不同 seed/起点/目标未测）。
- 地图是**世界几何先验**，不是 SLAM：`P1-N-07` 因此**不能声称建图**，报告里 `slam_exercised: false`。
- 无视觉、无位姿扰动、无动力学生成（底盘仍是 010 自制的差速代身，不是 AMR 选型）。
- `/cmd_vel` 上**有三个发布者**（collision_monitor、docking_server、following_server）。
  本轮的"一个最终命令写者"只能声称到**每条 TF 边**和**实测命令流**这一层；
  严格意义上的单写者需要把 docking/following 从 bringup 里去掉，未做。
- AMCL 超前未被修复，只被**定量**并写成 `P1-N-08`。

## 沉淀下来的操作事实

- 本机 nav2 是 `/opt/nav2/<pkg>/share/<pkg>` 的**非 merge 安装**，`ros2 pkg prefix` 返回 `/opt/nav2/<pkg>`。
- 用户级 `source setup.bash` **不能**在 `set -u` 下执行（`AMENT_TRACE_SETUP_FILES: unbound variable`）。
- 世界 XML 的注释里有 `--`，**MuJoCo 接受、XML 不接受**，所以 `ElementTree.parse` 会直接拒；
  栅格化前必须剥注释（有测试固定这条）。
- nav2 的 localisation 与 navigation 是**两个 lifecycle manager**；
  拿其中一个的 `node_names` 去比整个图，会把 `map_server`/`amcl` 判成"多出来的"。
- `wsl.exe` 会把 stdout/stderr 交错；要可读的顺序就把脚本内部重定向到文件。
