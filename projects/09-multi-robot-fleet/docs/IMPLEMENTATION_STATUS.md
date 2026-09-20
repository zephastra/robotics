# 009 实现状态（IMPLEMENTATION_STATUS.md）

更新时间：2026-09-15（P0 + P1 + P2）
当前阶段：**P0 / P1 / P2 完成；P3 未开始**

> 本文件只记录**实际发生**的事。未运行的一律 `NOT_RUN`。
> **"能动"不是"安全"。** 本工程至今没有产生任何物理或安全验收结论。
>
> ⚠️ **2026-09-15 重要更正**：此前记录的所有 P2 数字是在**轮子建模错误**的底盘上量的
> （四个轮子是平放圆盘，被拖着走）。**那一批数字全部作废**，本文件的数字是修复后重测的。
> 根因与修复见工作区根 `009_ROOTCAUSE_WHEELS.md`；重测全过程见 `009_P2_REVERIFY.md`。
>
> ⚠️ **2026-09-15 第二次更正**：曾记录的"头号缺陷：运动中 `amcl_pose` 落后真值最多 3.5–4.2 m"
> **已撤回**。那是两个探针在采样循环里自限速到 20 回调/秒、把自己队列积压报成系统故障所致。
> 仪器修好并自审通过后重测：位姿 vs 真值 **max 0.271 m**，输入年龄 **≤0.100 s**，
> gz 自身 `real_time_factor` 全程 **1.000**。**当前没有任何已知的影响控制质量的缺陷。**
> 详见工作区根 `009_INSTRUMENT_ARTIFACT.md`。

---

## 已验证能力

### P0（环境与脚手架）

| 能力 | 状态 | 证据 |
| --- | --- | --- |
| 交接包六份文档可读 | ✅ | `/mnt/c/.../GitHub/output/009_handoff`，6 文件 |
| 目标目录原本不存在，无覆盖 | ✅ | 创建前检查返回 `ABSENT` |
| `AGENTS.md` 完整正文按字节复制 | ✅ | `cmp` 验证 byte-identical |
| 环境实测已记录 | ✅ | `docs/ENVIRONMENT.md`（只读探测，未装依赖） |
| `scripts/doctor.sh` 可运行且退出码正确 | ✅ | **exit 0** |
| 001～008 未被修改 | ✅ | 仅 `ls` 读取 002 目录 |

### P1（纯 Python 协调核心）

| 能力 | 状态 | 证据 |
| --- | --- | --- |
| 核心测试全部通过 | ✅ | **`177 passed`**（0.20 s），exit 0 |
| 核心测试**不 import ROS** | ✅ | `ast` 扫描 + 运行时 `sys.modules` 双重检查 |
| 独立 `.venv`，解释器与 ROS 一致 | ✅ | venv Python 3.14.4 == `/usr/bin/python3` |
| fake demo 可运行并**明确标 fake** | ✅ | 每屏打印 `backend = fake (NO physics...)` |
| 未知 backend **不回退** | ✅ | `--backend gazebo_nav2` → **exit 3** |
| 幂等提交 / 冲突拒绝 / 资源互斥 / 原子 bundle | ✅ | 见 `tests/test_p1_*.py` |
| **租约到期 ≠ 清空** | ✅ | 到期后走廊仍 `is_free=False`，需显式 `confirm_clear` |
| 持货不可转移 / 两阶段取消 / 双时钟 / 崩溃对账 | ✅ | 同上 |

### P2（Gazebo 单车）

| 能力 | 状态 | 证据 |
| --- | --- | --- |
| 四包 colcon 构建通过 | ✅ | `fleet_core` / `fleet_adapter` / `fleet_ros` / `fleet_bringup` |
| gz 10.4.0 **headless** 加载世界并步进 | ✅ | exit 0 |
| 世界物理按实时节流 | ✅ | `<physics type="ode">`；修复前 `type="ignored"` 使实时因子完全失效 |
| ROS↔gz bridge 五路（scan/odom/joint_states/tf/cmd_vel） | ✅ | 话题出数据，`/tf` 与 URDF/SDF 逐位一致 |
| 资产不变量 **9/9** | ✅ | 含 URDF↔SDF 运动学交叉核对、泛洪填充证明走廊不可绕过 |
| **安全门独占 `/r01/cmd_vel`** | ✅ | 完整 Nav2 下仍 `Publisher count: 1`，名字 = `safety_gate` |
| Nav2 + AMCL lifecycle **8/8 active** | ✅ | `Managed nodes are active`，0 进程死亡 |
| **走廊穿越，方向交替 3/3 成功** | ✅ | W→E 34.4 s / E→W 41.8 s / W→E 37.0 s，全部 `SUCCEEDED` |
| **穿越确实走的是 1.2 m 缺口** | ✅ | 真值在缺口内最小 \|y\| = **0.015 / 0.075 / 0.084 m**（半宽 0.6 m），且泛洪填充已证无其它路线 |
| **停车距离（真值口径，3 遍）** | ✅ | 0.15 → 0.0292；0.25 → 0.0709；**0.35 → 0.1381 m**，极差 1.5–8.4 mm |
| **到达误差（真值口径）** | ✅ | **0.5051 m**（−4.0, 0.0）与 **0.1636 m**（−3.0, −3.0） |
| 穿越时离墙最近距离（机器人自己的激光） | ✅ | **0.5412 m**，出现在走廊内 `map (+0.183, −0.161)` |
| 落位/过廊用的是 safety gate 的整条链路 | ✅ | 标定器把速度发给**门的输入**，不是绕过门 |

### P2 未完成（NOT_RUN）

| 项 | 说明 |
| --- | --- |
| ~~`amcl_pose` 在运动中以旧值发布~~ | ⚠️ **此条已撤回（2026-09-15）**。它是一次**测量事故**：两个探针在采样循环里 `spin_once` 一次后 `sleep(50 ms)`，把自己限制在 **20 回调/秒**，而链路每秒送来约 50 条 ⇒ 探针在测自己的积压。修好（执行器独立线程 + 自审交付/节距比）后重测：**`amcl_pose` vs 真值 max 0.271 / mean 0.134 m**，`scan` 年龄 0.063 s，`odom` 年龄 0.000 s，自审 ratio 0.97–1.00。详见工作区根 `009_INSTRUMENT_ARTIFACT.md`。 |
| **`map→odom` 几乎恒为常数（≈零修正）** | ✅ **实测确认，且不是故障**：3.400 m 行程里 `map→odom` 只变 **0.077 m** ⇒ **AMCL 实际几乎不修正**，发布的位姿是"初始位姿 + 里程计"；因里程计已修好（yaw 误差 3.97°、3.5 m 漂 0.0655 m），结果准确。**但不得表述为"AMCL 正在定位"**，只能说"近常数修正叠加诚实里程计"。静止时 `amcl_pose` 年龄长到 5–6 s 同理属设计行为（`update_min_d` 门控：0.25 m 当初测量时、0.10 m 自 D-P17-06），不是故障。 |
| **有效滚动半径** | **未确定**。`probe_scale.py` 的 6 s 窗口未通过校验（里程计只相当于 5.03 s 的运动）⇒ 该仪器尚不可用，已加窗口自审。里程计尺度误差**不是常数**：14 s 直行 2.6%，6 s 直行 12.3%，由启动瞬态与打滑主导 |
| 停车**时间** / 由其派生的减速度 | 不可引用（真值口径比 odom 口径更长） |
| 转弯时轮子擦角 | 激光测不到扫描面以下的轮子，需独立方法 |
| 0.35 m/s 以上 / 载荷下的停车距离 | 未做 |
| 带名字的真值通道 | **未实现**。现通道丢 frame name、stamp 恒 0 ⇒ **只能单车按数组位置取** |
| GUI / RViz | 本机无 NVIDIA Vulkan ICD，headless |
| 多车（r02/r03）在线 | **P3，未开始** |
| 窄道预约、故障接管 | **P4+，未开始** |
| 物理 / 安全验收结论 | **未产生** |

### 一点测出来的操作边界

- **标定器从出生点起只够跑 2 遍。** 前两遍跑完机器人已到 `x ≈ −0.63`，正对 x=0 屏障；
  第 3 遍被净空护栏以 `ABORTED_OBSTACLE`（`forward clearance 0.344 m`）中止 —— **判对了，不是误报**。
  第 3 遍需先把机器人挪回出生点。
- 净空护栏**已实战出手一次**，挡住了"顶着墙继续发速度"这一类事故。
- 标定**必须排在穿越之前**（那时机器人还在出生点、前方 5.5 m 净空）。

---

## 真值通道（**必须读**）

- 现用：世界级 `/world/<world>/dynamic_pose/info`，bridge `gz.msgs.Pose_V → tf2_msgs/msg/TFMessage`，
  由 **`world.launch.py` 桥一次**（不是每台车各桥一次）。
- **本次已逐条验证**（`scripts/dump_truth_all.py`）：
  - 每条消息 **8 个 transform**，51–55 Hz；
  - **`transforms[0]` = 模型位姿，世界坐标**（静止时恰为出生点；直行时 2.4 m/10 s ≈ 指令 0.24 m/s）；
  - `transforms[1..7]` 是**同一台机器人的连杆**，坐标在**模型局部系**（不是别的模型）；
  - 世界里 24 个模型**全部 static**，不进该话题。
- ❌ 限制：**丢 frame name、stamp 恒为 0** ⇒ 只能按数组位置取、**只能单车**。
  多车必须换通道（gz-transport 小节点或 `ros_gz_interfaces`）。**列为 NOT_RUN。**
- **真值绝不回流控制**（AGENTS.md rule 13）。本工程的诊断工具都只读它。

## 一条贯穿全程的纪律（血泪版）

**"系统看起来坏了"的次数，远少于"观测手段在说谎"的次数。** 本项目已抓到 10 处，
逐条列在 `009_P2_REVERIFY.md` 第七节。其中代价最大的三条：

1. **`f"{v:.8g}"` 把 `-6.0` 变成 `"-6"`** ⇒ YAML 读成 int ⇒ AMCL 参数类型不符 ⇒ **整条 Nav2 起不来**，
   而唯一可见症状是 `no navigate_to_pose action server`。
2. **标定器盲发速度、无净空检查** ⇒ 顶着墙走 ⇒ 里程计积分到 14 m ⇒ 规划器报 `outside bounds`
   —— **看起来像定位 bug，其实是测试把车开进了墙。**
3. **3 s 收敛尾巴是忙等** ⇒ 3 s 追加 ~3000 样本 ⇒ 统计量无法自洽。
   ⇒ 现在采样器**自审节奏**（打印实际 Hz / 请求 Hz，偏离即 WARNING）。

**推论**：诊断工具必须先自证，再解读它给出的数据。

---

## 下一步（按依赖顺序）

1. ~~查 `amcl_pose` 滞后~~ ✅ **已结案**：不存在该缺陷，原结论系探针自限速所致（见上表与 `009_INSTRUMENT_ARTIFACT.md`）。
2. 转弯擦角 + 0.35 m/s 以上 + 载荷。
3. 冻结 footprint / 限速（停车距离与到达误差已各 3 遍，可冻结）。
4. **P3 两车** —— 注意真值通道必须换成带名字的。
5. **P4 窄道预约** —— 009 的核心价值所在。

**当前已知影响控制质量的缺陷：无。**

---

## P4.0 / P4.1（2026-09-15）：窄道协同的配置层与纯 Python 预约核心

更新时间：2026-09-15
当前阶段：**P4（窄道协同）前半 —— 配置 + 核心。P4.2～P4.5 未开始。**

### 已验证能力

- **原子捆绑预约**：通道 + 目标侧出口缓冲位作为一个单位申请；半个捆绑不存在。
- **许可 ≠ 占用**：`RESERVED`（有权进入）/ `OCCUPIED`（确认在里面）/ `CLEARING`（要走没证明走掉）
  / `UNKNOWN`（不知道 ⇒ 封锁）/ `FREE`（账本空 **且** 几何已验空）。
- **TTL 到期不释放**：`expire_due()` 只把资源降为 `UNKNOWN`，必须等显式几何清空。
- **清空由调度器自己验**：`confirm_clear()` 收的是**位姿**，不是 `clear=true`。
  车停在出口缓冲位上会被拒（`CLEARANCE_NOT_PROVEN`），因为缓冲位在设计上就在捆绑里。
- **逐资源授权**：持有 `[mid, mid_right]` 不会授权开进 `mid_left`。
- **入场前置条件**：必须在方向对应的候车位或对齐位上提出申请，且整车（含余量）在区域外；
  位置不合法的请求被**拒绝而不是入队**。
- **FIFO 且不泄漏**：取消/释放不留队列条目；三轮双向往返后队列恒为空。
- **门禁判据**：`check_movement()` 返回 `PermitCheck(allowed, reason, stop_boundary_hit, detail)`，
  不抛异常；定位无效 ⇒ 保守停车。停止包络随速度与角速度增长。

### 新增/修改文件

新增：`config/resources.yaml`、`src/fleet_core/fleet_core/geometry.py`、
`src/fleet_core/fleet_core/traffic.py`、`tests/test_p4_geometry.py`、`tests/test_p4_traffic.py`、
`scripts/validate_traffic_geometry.py`。

修改（最小插入，锚点断言 + 幂等）：`fleet_core/domain.py`（`ResourceState`、4 个 reason code、
`NodePose`/`DirectionSpec`/`ResourceSpec`/`TrafficConfig`/`PermitCheck`/`ClearanceOutcome`）、
`fleet_core/validation.py`（`validate_traffic_config` / `load_traffic_config`）、`fleet_core/__init__.py`（导出）。

### 最近命令与退出码

```
bash scripts/test_core.sh                                -> 233 passed, exit 0   (此前 177)
.venv/bin/python scripts/validate_traffic_geometry.py    -> 57 checks, PASS, exit 0
/usr/bin/python3 scripts/check_xml_wellformed.py         -> exit 0
/usr/bin/python3 scripts/check_undefined_names.py        -> exit 0
.venv/bin/python -m compileall -q ...                    -> exit 0
```

### 失败 case 与根因证据

本轮写了 56 个新测试，第一次运行 **3 个失败，全部是测试期望写错**（全新的通道处于 `UNKNOWN`，
所以 reason 是 `RESOURCE_UNKNOWN` 而不是 `ROUTE_REQUIRES_PERMIT`），**不是被测代码的缺陷**。
修正方式：`mgr` fixture 定义为"已验证为空的健康通道"，关心 `UNKNOWN` 的两个测试自建 manager。

实现期发现并修掉的**真实逻辑缺陷 1 处**：`check_movement` 最初按整个捆绑判权限，
导致"持有 `west_to_east` 却开进 `mid_left`"被放行。已改为逐资源溯源，并有专门测试。

### NOT_RUN 清单

- ROS 服务面（`AcquirePassage` / `RenewPermit` / `ConfirmClear`）—— **未实现**。
- 门禁 `gate_node.py` 接入许可检查 —— **未实现**。`check_movement()` 是纯函数，尚无调用方。
- 分阶段导航适配器 —— **未实现**。
- **双车窄道实跑（对向 / 排队 / 出口占用 / 掉线）—— 一次都没跑。**
- 因此**不得声称"协同交通通过"或"窄道协同已实现"**。本轮交付的是它的可验证内核。

### 下一步不超过三项

1. P4.2：`fleet_interfaces` + 持有 `CrossingManager` 的协调节点。
2. P4.3：`check_movement()` 接进每车安全门，证明 Nav2 绕不过门，门仍是 `/rXX/cmd_vel` 唯一发布者。
3. P4.4/P4.5：分阶段导航适配器，然后双车验收（交换请求顺序与到达时间）。

### 禁止误称的能力

- 不得称"窄道协同已完成"、"双车交通没有冲突"、"AMCL 正在定位"（后者见上文既有条目）。
- 不得把 `validate_traffic_geometry.py` 的 57 项通过说成"物理验收通过"——它只证明配置与世界文件一致。

---

## P4.3（2026-09-15）：门禁地理围栏 —— 策略层

更新时间：2026-09-15
当前阶段：**P4.3 策略层完成；`gate_node.py` 接线未做（需要 P4.2 提供许可来源）。**

### 已验证能力

- **门不再接受被监管者的声明**：装了 `zone`（`ZoneGuard` 协议）之后，`wants_protected_zone`
  被忽略，判定来自机器人实测位姿 + 朝向 + twist 与保护几何。
- **逐次重算**：位姿与 twist 是参数不是缓存 ⇒ 车开进区域后下一次判定就会拦。
- **速度参与判定**：同一位置，静止放行、0.35 m/s 拒绝；原地转同样按运动处理。
- **崩溃即停车**：`zone` 抛异常 ⇒ STOP + `RESOURCE_UNKNOWN`，detail 写明
  `treated as unknown occupancy, not as permission`。**异常逃出定时器回调是开门，不是停车。**
- **不认识的拒绝码一律变 `NO_PERMIT`**（含将来新增的码）。
- **远离通道的正常行驶不受影响**（围栏不能退化成全局停车）。

### 修改文件

- 新增 `src/fleet_adapter/fleet_adapter/zone_guard.py`（`CrossingZoneGuard`、`map_reason`）。
- 新增 `tests/test_p4_gate.py`（21 项）。
- `fleet_adapter/safety_gate.py`：`ZoneVerdict`、`ZoneGuard` 协议、`SafetyGate.zone`、
  权威判定块（取代旧 flag 路径；旧路径在 `zone is None` 时保留）。
- `fleet_adapter/adapter_base.py`：`AdapterStatus` 增 `yaw` / `angular_radps` / `localization_valid`。
  **`localization_valid` 默认 `False` = fail closed**（见 D-P4-07）。
- `fleet_adapter/fake_adapter.py`：显式声明 `localization_valid=True`。
- `fleet_adapter/__init__.py`：导出。

### 最近命令与退出码

```
bash scripts/test_core.sh                          -> 254 passed, exit 0   (P4.0/P4.1 后为 233)
/usr/bin/python3 scripts/check_undefined_names.py  -> 53 files, exit 0
/usr/bin/python3 scripts/check_ros_params.py       -> 50 files, exit 0
.venv/bin/python -m compileall -q ...              -> exit 0
```

### 失败 case 与根因证据

第一遍 **2 个失败，均为测试写错**：在 `wall_now=0` 取许可、在 `wall_now=100` 询问门，
而 TTL 为 12 s ⇒ 许可确实已过期，门的判断正确（失败信息 reason 即 `PERMIT_EXPIRED`）。
修的是测试（取许可与询问门用同一 wall clock）。**顺带证明 TTL 在整条链路上确实生效。**

实现期发现并修掉的**真实缺口 1 处**：`zone` 调用最初无异常保护；已改为 fail closed + 明确 detail。

### NOT_RUN 清单

- **`gate_node.py` 未构造/安装 `CrossingZoneGuard`** ⇒ 生产路径仍是"无 guard ⇒ 相信调用方 flag"。
  硬前提是 P4.2 的协调节点（目前没有许可来源）。**因此不得声称"窄道已被门禁封锁"。**
- 双车窄道实跑 —— **一次都没跑**。
- `CrossingZoneGuard` 与真实 AMCL 位姿/协方差的接线未做（`localization_valid` 目前无真实生产者）。

### 下一步不超过三项

1. P4.2：`fleet_interfaces` + 协调节点，把许可发给每台车。
2. P4.3 接线：`gate_node.py` 装 guard；**未装 guard 应在启动时报错**，不静默走旧路径。
3. P4.4/P4.5：分阶段导航 + 双车实跑。

### 禁止误称的能力

- 不得称"窄道协同已完成"、"Nav2 已无法绕过门禁"（策略已就位，**链路未接线**）。
- 不得把 254 个纯 Python 测试说成物理验收。

---

## P4.2–P4.5（2026-09-15）：接口、协调器、门禁接线、分阶段穿越

更新时间：2026-09-15
当前阶段：**P4.2 / P4.3 接线 / P4.4 已完成，可构建可运行；P4.5 双车验收未通过。**

### 已验证能力

- `fleet_interfaces`（1 msg / 3 srv）rosidl 生成通过；5 包 colcon 全通。
- **许可通路端到端连通**（双向日志为证）：协调器 `no live permit for r01 … publishing granted=false`
  ↔ 门 `first permit message received: granted=False …`。话题、QoS、类型、命名空间全部正确。
- **启动占用普查成立**：`start-up sweep: all 3 resources verified clear at t=7.9s`；
  车辆未上报时资源保持 UNKNOWN、不可授权。
- **"到期 ≠ 清空"在实跑中触发**：`permit(s) lapsed on ['mid','mid_right'] → UNKNOWN and stay BLOCKED`；
  其后该侧门的拒绝码从 `ROUTE_REQUIRES_PERMIT` 变为 `RESOURCE_UNKNOWN`。
- **门用自己算的几何拒绝**，理由可诊断：
  `unpermitted ['mid'], holdings none (reserve 0.100 m)`。
- 队列被实际使用（`busy_or_queued_samples` 455/486）；所有运行 `simultaneous_occupancy = 0`、
  `unauthorised_entry = 0`。
- 真值通道单次 10803 条、每条 16 transform（两车 × 8 link）。

### 修改/新增文件

新增 `src/fleet_interfaces/**`、`fleet_ros/coordinator_node.py`、`fleet_ros/staged_crossing.py`、
`fleet_adapter/zone_guard.py`、`fleet_bringup/launch/fleet.launch.py`、
`scripts/acceptance_p4.py`、`scripts/acceptance_p4.sh`、`tests/test_p4_mirror.py`、
`tests/test_p4_frames.py`；
修改 `fleet_core/{domain,resources,traffic,validation,geometry,__init__}.py`、
`gate_node.py`、`adapter_base.py`、`fake_adapter.py`、`robot.launch.py`、`build.sh`、
`config/resources.yaml`（`node_reach_tolerance_m` 0.35 → 0.80）。

### 最近命令与退出码

```
bash scripts/build.sh                                        -> 5 packages, exit 0
env -u ROS_DISTRO -u AMENT_PREFIX_PATH bash scripts/test_core.sh -> 274 passed, exit 0
bash scripts/acceptance_p4.sh                                -> both_completed=false（见下）
```

### 失败 case 与根因证据

**P4.5 未通过：两台车一次都没有完成对向穿越**，`both_completed` 在所有运行中均为 `false`。

已定位到"车持有许可、位于对齐点外侧、却完全不动（107 s 零位移）"，门侧判词为
`holdings none`（门侧镜像无许可）。已排除话题/QoS/类型/命名空间。
剩余两种可能，下一次运行的三行日志即可分离：
该轮是否出现过 `permit live for <robot>`（协调器侧）与 `adopted permit …`（门侧）。

同时存在一个独立的、更早的问题：**r01 从出生点去候车位超时**（150 s 未到达），
该轮因此根本没有一次成功 acquire。下一步应先单独诊断这一个 Nav2 goal。

### NOT_RUN 清单

- **双车对向穿越 —— 未通过（0 次成功）。**
- 排队/饥饿/出口占用/通道内掉线 —— 未做。
- 载荷、0.35 m/s 以上、三车 —— 未做。
- `gate_node.py` 的 `traffic_guard` 在**单车**拉起时默认 `false`（无协调器可授许可），
  这是有意的；任何走廊结论都必须来自 `fleet.launch.py` 路径。

### 下一步不超过三项

1. 诊断 r01 到候车位超时（单独发一个 Nav2 goal；门已有逐条判决日志）。
2. 让一次 acquire 成功，读 `permit live for` / `adopted permit` 两行，定位镜像为空的真因。
3. 再跑 P4.5 双向验收。

### 禁止误称的能力

- **不得声称"窄道协同通过"、"无死锁"、"无饥饿"或"Nav2 绕不过门"**：
  没有一次"持许可顺利通过"的对照，无法区分"正确放行"与"一直误拦"。
- 不得把"零同时占用"当作通过——它只说明没被抓到违规，不说明机制生效。
- `check_undefined_names.py` 的结论**依赖环境**（源 ROS 时 Node 成员 23 → 109），
  引用其结论时必须同时给出环境。


### P4.6 — 许可 epoch 归属 + UNKNOWN 复核（2026-09-16）

**读这一段的顺序**：先看 `009_P4_EPOCH_ROOTCAUSE.md`（工作区根），
再看 D-P4-10/11/12。前一份验收报告把症状定位对了（门禁 mirror 空），**根因指错了**。

#### 改了什么

| 文件 | 改动 |
|---|---|
| `src/fleet_interfaces/srv/AcquirePassage.srv` | request **删** `int32 epoch`；response **加** `int32 epoch` |
| `src/fleet_interfaces/srv/RenewPermit.srv` | epoch 注释改成实话（它是作用域标签，不是重启检测器） |
| `fleet_core/traffic.py` | 新增 `ClearanceScan` + `CrossingManager.verified_clear()`；`mark_verified_free()` 同时清账本的 `_unknown` |
| `fleet_core/__init__.py` | 导出 `ClearanceScan` |
| `fleet_ros/coordinator_node.py` | 自己盖章 epoch；`_try_sweep` 改用共享扫描；新增 `_reverify_unknown` / `_auto_reverify` / `_srv_reverify`（`/fleet/reverify`） |
| `fleet_ros/staged_crossing.py` | 不再自行发明 epoch；`self.epoch = int(resp.epoch)` |
| `tests/test_p4_epoch.py` | 新增 12 条 |

#### 根因（一行）

协调者把**请求方填的** epoch 写进租约账本，又用**自己的** epoch 去查
⇒ 它发出的每一份许可对它自己的发布器不可见 ⇒ 线上永远 `granted=false`。

#### 验证

`286 passed`（修前 274）· 守卫 exit 0 · colcon 五包 · srv 自检通过 · 9 项源码断言通过。

#### NOT_RUN

- 双车对向穿越：**修完的第一轮实跑结果见后续报告**；本段写作时尚未完成。
- 通道内过期后的退避路径：未实现（D-P4-12）。
- 门禁判决的计数是**指标**不是测量值（验收采样器自审 `ratio 0.323 → WITHHELD`）。

#### 教训（可跨项目复用）

1. **一个值，两条路径，一条通一条断** ⇒ 别猜哪条对，看哪条和外部证据一致。
   这里是"37 次续期全成功"对"从未打印 `permit live for`"。
2. **归属搞反的输入 = 被监管方自己申报** ⇒ 删掉字段，不要只改读取那一行。
3. **"改完之后测试立刻变红"经常是对的** —— 这次红了 3 次，每次都指向我自己的新缺陷。
4. **一条安全规则两份实现 ⇒ 合并成一份**，否则早晚两个答案。
5. **改安全层里"状态"和"账本"两处记录时，必须一起改**；否则会报成功而什么都没变。


### P4 验收结果：**通过**（2026-09-16）

`reports/p4_acceptance_20260915T170337Z`，`case1_rc=0  case2_rc=0`。

| | case 1（r01 先问） | case 2（r02 先问） |
|---|---|---|
| `both_completed` | **True** | **True** |
| 采样点 | 637 | 648 |
| `simultaneous` / `unauthorised` | **0 / 0** | **0 / 0** |
| 排队采样 / 申请次数 | 164 / r01 103 次、r02 1 次 | 183 / r01 1 次、r02 115 次 |
| 门禁判决 | `OK 3115 / ACCEL_CLAMPED 18` | `OK 3152`（零拒绝） |
| 许可通路对账 | grants 270/260，guard 已装，mismatch **False** | grants 281/297，mismatch **False** |
| `drive_error` | `[]` | `[]` |

四条驱动（case1 r01/r02、case2 r01/r02）**各自六个阶段全部 ok**：
候车 → 对齐 → 申请（12–14 次续期零失败）→ **穿越** → 释放 → **`released [...]`**。

两个 case 的 teardown 都留下了 `teardown verified clean: no process from this project, no gz sim`，
**这是 case 2 可信的前提**。

#### 能说 / 不能说

**能说**：双车对向穿越在两个请求顺序下都成立；1285 采样零同时占用、零未授权进入；
排队与仲裁成立（换顺序换赢家）；许可续期成立；门禁是 `/rXX/cmd_vel` 唯一发布者且 guard 已装。

**不能说**："无死锁/无饥饿"（应为"本轮两个 case 中未出现"）；"Nav2 绕不过门禁"
（缺一次刻意的绕过尝试）；**通道内掉线 / 许可在通道内到期 = NOT_RUN**（`D-P4-12` 未实现）；
出口缓冲作为阻塞条件只由单测覆盖；采样自审仍 `WITHHELD` ⇒ 门禁判决计数是指标。

#### 本轮 10 个缺陷（4–13）与两条通用教训

见工作区 `009_P4_ACCEPTANCE_RESULTS.md` §5。两条可跨项目复用：

1. **能"静默返回空"的仪器，和"机器人没动"无法区分** —— 线程/子进程里的异常必须落进 verdict。
   配套 AST 静态检查 `RUN_PAIR_SCOPE_OK`（扫"既非局部/参数/模块级/内置"的名字）。
2. **"必须走到某处才能满足某几何条件"的节点，余量必须 ≥ 该栈的实测到达误差**，
   不能按完美到达算；推导写进配置并由单测**从配置重算**。

---

## P5.1–P5.4 and P6 (2026-09-16)

Read the passing run `reports/p5_accept_20260916T082659Z` and found six defects, all fixed and
tested. The important one is #18: a fix of mine (`pending.handle.is_cancel_requested`) used a
**server**-side goal-handle attribute from the **client** side, so every tick raised
`AttributeError` and `_tick`'s blanket `except` swallowed it — the cancel still had no
deadline AND five other dispatching steps stopped running, with one ERROR line per tick as the
only trace. A dispatcher that has stopped dispatching must not be able to report healthy, so
tick failures are now counted, traced once, and exposed in the status snapshot.

Also fixed: `in_corridor` was hardcoded `UNKNOWN` (making the contract's `PAYLOAD_HELD` and
`ASSIGNED_NOT_PICKED` recovery rows unreachable in production); `_park` wrote
`CANCEL_NOT_CONFIRMED` for every caller (008's "fail_reason is a constant" shape);
`PAYLOAD_HELD_NEEDS_ATTENTION` was declared and never emitted; a robot holding an undelivered
cargo could be given a second one (observed: r02 held cargo_C and cargo_D at once); and
`CancelTask` was recorded but never transmitted, so one cancel took 306 s to confirm.

Two "the check was not really checking" defects: the Node-member guard's depth depended on the
shell (23 names in a clean shell vs 109 with ROS sourced, so half the check was off where the
project actually runs it), and `_mk_state_cb` filled `position_known` but not
`localization_valid`/`pose_age_s`, so two of three conditions in the new corridor guard could
never fail. Five static guards that existed but were never called are now wired into
`build.sh` via `scripts/check_guards.sh`.

Final acceptance run `reports/p5_accept_20260916T084751Z`: p5-A SUCCEEDED with the full
pick-and-deliver chain, p5-B SUCCEEDED, p5-C parked `NEEDS_ATTENTION` /
`PAYLOAD_HELD_NEEDS_ATTENTION`, p5-D CANCELED in **0.78 s**, `PAYLOAD_HELD_ELSEWHERE: 2`, no
robot holding two cargos, `teardown verified clean`.

Verification: **377 passed** · 5/5 guards · 6 packages · 57 geometry checks · P6 dashboard
HTTP surface **23/23** (`scripts/check_p6_dashboard.sh`) · `docs/ARCHITECTURE.md`,
`docs/LIMITATIONS.md`, `docs/TROUBLESHOOTING.md`, `LICENSE`, `THIRD_PARTY_NOTICES.md`,
`scripts/run_demo.sh`, `scripts/stop_demo.sh` added.

**P5 has acceptance evidence on all six directed scenarios.** The six have been run as
scenarios, and as of round 2 all six have clean verdicts: pre-pickup takeover **7/7**,
no-transfer-while-holding **7/7**, capability filtering **8/8**, restart recovery **13/13**,
low-battery admission **9/9** (`reports/scen3456/low_battery_20260916T132401Z.txt`: r01
charges 0.1363 → 0.8062, `grants=2 releases=2`), charge-pad queueing **11/11**
(`reports/scen3456/charge_queue_20260916T132532Z.txt`: r02 east → `C_right`, r03 west →
`C_left`, r01 waiting with `reachable: ['C_left']`).

The two that were partial in round 1 were blocked by one defect, **D-P5-22**: the charger
allocator chose a pad without reading the robot's position, so half the fleet was sent to the
pad behind the barrier and — because a charge run is *pinned* — refused permanently while the
pad stayed reserved. Fixed in round 2. See `docs/LIMITATIONS.md`, `docs/DECISIONS.md`
D-P5-22, and the workspace reports `009_P5_DIRECTED_SCENARIOS.md` (round 1) and
`009_P5_ROUND2_AND_P7_GATES.md` (round 2).

**P5 still has no *whole-phase* conclusion**, and what is missing is not a scenario but a
class of evidence. The last two holdouts were D-P5-22 (fixed and re-run: both charging
scenarios pass end to end) and D-P5-23 (fixed and re-run: each robot's charge run is now its
**first** serial with **zero** attempts, where before a `FAILED` run always preceded the
successful one). What remains is the batch matrix -- `TEST_AND_ACCEPTANCE` §5's 30 cases plus
the 8 seeded repeats -- which needs a batch runner and a frozen case list that do not exist
yet, and `D-P4-12` / cross-corridor routing, both deliberate non-implementations.

**P6's dashboard has now been driven against a live fleet — 20/20 — and the static check it
passed beforehand was green while being wrong.** `check_p6_dashboard.sh` had only ever run
with no fleet, which is the one state in which the display does not fail. Running it against
a live fleet found three real display defects (a `MutuallyExclusive` callback deadlock that
kept it in `DISCONNECTED` for 284 s; a stale-owner display that stayed `LIVE`; and every
robot rendering as `"?"`).

---

## P5 directed scenarios (2026-09-16)

The six scenarios are driven by `scripts/run_scenarios3456.sh` (cases C–F) and
`scripts/run_scenarios12.py` (cases A–B). Each case gets its **own ledger**, stamped with
the run, because the ledger is durable on purpose and an experiment that inherits last
session's payload custody reports a scheduling failure that is really leftover state.

**Two preconditions had to be built first**, both recorded as decisions:

* D-P5-17 — the launch consumed two of nine budget keys, so a scenario declaring
  `leg_timeout_s` / `cancel_confirm_s` / `max_retries` changed nothing while the run
  reported those numbers as in force. `scenario:=<name>` is now read, applied, printed in
  full, and an unknown key is a hard error. `scripts/check_scenario_budgets.py` checks both
  directions in `build.sh`, with seven controls proving it can fail.
* D-P5-18 — `FaultInject` gained `inject_battery` + `battery_fraction`. The simulated pack
  spends 0.5 Wh/m, so the low threshold is ~160 m of driving away; the injection sets a
  *state*, and every rule downstream still reads the same `BatteryModel`.

**Six product defects found, three of them silent.**

| | defect | fix |
|---|---|---|
| §3.1 | `tasks` had no `kind` column, so every task read back from the ledger looked like a station transfer — and the dispatcher decides `charging_task` from that field. A low robot was refused **its own charge run**, once per tick, while holding a pad it never used. P5's "低电返航" had never been exercised | D-P5-19: persist `kind` + `payload_mode`, read them back, migrate in place |
| §3.2 | `request_id = auto-charge-<rid>-<serial>` with a per-process serial against a durable ledger: the second session's charge run reused a dead row's key, `submit` returned `created=False`, and the caller returned early **after** granting the pad. The pad stayed held all session with no task to release it | D-P5-21: the id carries the epoch, and a failed creation **releases the pad** |
| §3.3 | A leg was dispatched to a robot that could not localise: `_eligible_robots` is the only gate for a **pinned** task, so `Allocator.candidates`' `position_known` check is bypassed. Four charge runs burnt in 155 s | tightening added with a structural test; live re-verification **NOT_RUN** |
| §3.4 | `_reconcile_at_boot` labelled every restart-parked task `CANCEL_NOT_CONFIRMED`. 008's constant-`fail_reason` shape | D-P5-16: `INTERRUPTED_BY_RESTART`, an addition inside CONTRACTS §12's "至少支持" |
| §3.5 | scenario budgets were never forwarded | D-P5-17 |
| §3.6 | `Allocator.allocate` reports `NO_CAPABLE_ROBOT` when the capable robot is merely `RESOURCE_BUSY`. The dispatch decision is right; the **label** is wrong | **fixed in round 2** (D-P5-20): a declared precedence with `NO_CAPABLE_ROBOT` last, the per-robot refusals carried on the `Allocation`, and a `no_allocation` event written once per change of reason. Unit-tested; live re-observation **NOT_RUN** |

**And seven defects in the instruments**, each of which had produced a plausible-looking
verdict: a reused durable ledger read as a capability failure; "three fresh robots" taken as
"the fleet is ready" while Nav2 was still inactive (7–12 s of window, which cost a leg both
its retries); a canned failure explanation printed as evidence beside a PASS; `ledger_boot`
mistaken for a database identity when it is a per-instance id; `chargers.grants` mistaken for
a robot arriving; a test helper that could not see an attribute checked inside an `or`; and
`uv venv --seed` failing for want of network while the check reported `[PASS] venv built`
because the interpreter existed. All seven are written into the report and into
`docs/LIMITATIONS.md` §4.

**Verification.** 456 core tests (31 added in round 2: 14 for D-P5-22, 11 for D-P5-20,
6 for the pad-reachability geometry and the unassignable watchdog) · 6/6 guards · 7 packages ·
4 scenario files, all checked end-to-end by the budget guard.

## P7 relocation (2026-09-16)

`scripts/verify_independence.sh` copies the tree to a temporary path excluding every build
artefact, scans it, builds it from nothing, runs the suite there, compares it against the
frozen manifest, and optionally launches a real fleet from the copy. Frozen inputs:
`docs/FROZEN.md` + `reports/frozen_manifest.json` (185 files hashed).

The scan found three genuine violations before it passed: `scripts/run_scenarios12.py`,
`scripts/acceptance_p4.sh` and `scripts/run_p5.sh` each named an absolute project path, one
of them writing its transcript to a Windows path that exists on this machine only. It also
found that **`build.sh` did not produce everything a launch needs**: the per-robot rendered
models are required by `fleet.launch.py` and nothing rendered them, which was invisible on
this machine because `runtime/models/` had been populated long ago. A procedure was not
relocatable even though the tree was; `build.sh` now renders them.

## P7 section 10 — the release gates (2026-09-16, round 2)

§10 had never been checked as a whole; it was checked item by item by hand, at different
times, by whoever remembered. It is now `scripts/check_p7_gates.py` — eight verdicts, each
quoting the artefact it rests on and reporting PASS / FAIL / **NOT_RUN**, with NOT_RUN
holding the gate open rather than quietly passing — driven by `scripts/run_p7_matrix.sh`.

| §10 | item | verdict |
|---|---|---|
| 10.1 | a fresh path builds **and runs** | **PASS** — `reports/p7/independence_20260916T133118Z.txt`, 8/8 steps |
| 10.2 | dependencies / build command / versions / map source | **PASS** |
| 10.3 | headless and GUI recorded separately | **PASS** |
| 10.4 | GUI off ⇒ the server still decides | **PASS** |
| 10.5 | venv / build / install / log / SQLite / reports / credentials out of Git | **PASS** |
| 10.6 | licences explicit, notices kept | **PASS** |
| 10.7 | README states the real scope | **PASS** |
| 10.8 | GitHub only after the user confirms | **NOT_RUN** — the user gate |

**7 PASS, 0 FAIL, 1 NOT_RUN.** The relocation run: copy the tree to a fresh path → scan →
build from nothing (7 packages, 6/6 guards in the copy) → **456 tests in the copy** → every
recorded hash matches → **a real fleet run launched from the copy**, capability scenario 8/8.

**The first pass found three FAILs, all in `README.md`**, and they are worth naming because a
reader would have been misled by each: the status line still said "开发中（P0）… 没有任何可运行
的机器人" (untrue since P3); the command block still listed `build.sh` / `test_core.sh` /
`run_demo.sh` as "未来接口，实现并验证前不要当作可运行" (all three are implemented and have been
run many times); and the licence was called "待确定" while `LICENSE` said Apache-2.0. A fourth
finding was factual rot in `THIRD_PARTY_NOTICES.md`, which quoted a test count that had since
moved twice. All four are fixed, and the checker is now 7 PASS / 0 FAIL.

**10.8 can never be closed by a script**, and the checker returns exit 5 rather than 0 while it
stands. That is the point: a release gate that closes itself is not a gate.

**What §10 does *not* cover, and must not be read as covering:** the 30-case batch
(`N01–N08`, `F01–F14`, `I01–I08`) and its 8 seeded repeats. Those are §5's matrix, they have
not been run, and §10 passing says nothing about them. `docs/LIMITATIONS.md` remains the
register of everything not executed.
