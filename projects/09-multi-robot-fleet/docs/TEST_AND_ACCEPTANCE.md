# 009 测试、验收与复现协议

测试本身是交付的一部分。不得只交运行视频、单个成功 JSON 或“所有单元测试通过”而声称完成多机器人协同。

## 1. 三种不同结果，永远分开

每 case 同时输出：

- `task_outcome`：SUCCEEDED / FAILED / CANCELED / NEEDS_ATTENTION / NOT_STARTED。
- `safety_outcome`：PASS / FAIL / UNKNOWN。
- `expected_behavior_outcome`：PASS / FAIL / NOT_RUN。

例如：车坏在唯一窄道内，相关任务 NEEDS_ATTENTION、安全停车、资源封锁，是预期故障处理 PASS，但任务并没有成功。不能把它混入“任务成功率”。

`backend` 必须为 fake 或 gazebo_nav2；没有真实物理步进就没有物理安全 PASS。缺真值或采样中断时为 UNKNOWN，最终物理验收不能通过。

## 2. 测试层次

### L1 核心单元测试（P1）

不 import ROS，不启动 Gazebo，用可控假时钟与 fake adapter：

1. 配置字段、单位、非法值、未知站点、重复 ID。
2. request_id 同内容去重、异内容冲突、数据库重启后去重。
3. 能力过滤、电量过滤、距离成本、稳定 tie-break、队列老化。
4. task revision/boot_id/epoch 过旧被拒绝。
5. cancel accepted 后未停不能接管，旧 SUCCEEDED 不完成新任务。
6. 资源同时请求只一人获准、FIFO、重复 acquire、续期、过期。
7. 占用资源 TTL 到期不释放；未知位姿不释放；出口被占不授权。
8. 原子预约 bundle，全成功或全失败，不留下部分资源。
9. 取货/交付原子事务、持货故障不瞬移、不重复领取。
10. 丢 ack 后幂等重发、乱序事件、重复事件消费。
11. 低电预判、充电位容量、持货低电转 attention。
12. sim 暂停不耗电但 wall 超时生效，clock reset 终结当前 run。
13. 崩溃恢复对账、磁盘写失败、SQLite 事务回滚。
14. report 缺失字段/真值/输入哈希时 fail-closed。

要求覆盖这些行为，不规定凑出多少个测试函数。可参数化，计数不能代替覆盖。

### L2 契约与 ROS 集成（P2/P3）

- ROS srv/action/msg 与 Python domain 往返转换不丢失 command key。
- typed fields 与真实 Twist/TwistStamped 链匹配。
- fake/Nav2 adapter 相同接口、不同 backend 明确。
- 每车 topic/action/TF 唯一；map/clock/TF 共享策略符合方案。
- lifecycle 未激活不能发任务，启动 readiness 超时正确退出。
- Gate 最终 cmd_vel 唯一发布者，Nav2 recovery/平滑器没有旁路。
- 测单台控制，只该台关节/odom 响应；另一台不受影响。
- r01 cancel/pause/battery 不影响 r02。

### L3 真正物理仿真（P2～P7）

必须收集 Gazebo 仿真状态。真实车体运动不是修改 model pose，不是 GUI 动画，不是 fake adapter 输出的位置。

## 3. 独立 evaluator

### 3.1 数据来源分离

控制：AMCL/TF、scan、odom、协议状态、已知静态地图与区域配置。

验收：Gazebo 真实车体 pose、车体接触、仿真步进、任务/资源事件和原始输入快照。

真值仅允许初始化、故障注入、独立评价。动态车位置不得由 evaluator 回传给 allocator/Gate/Nav2。源码检查订阅与 import；这属于软件架构约束，不宣称能防恶意插件。

### 3.2 必须判断的事项

- 每个机器人真实 footprint 是否侵入未授权保护区。
- 两车 footprint 是否同时占用容量 1 区域。
- 同一资源 generation 是否同时授权两个 owner。
- 车车、车体/轮子与障碍的非预期接触；轮地正常接触排除。
- 实际速度、停止距离、停止后漂移是否符合阈值。
- 任务最终是否在正确 station 停稳并满足服务时间。
- 同一个 payload_id 是否出现双重持有或重复交付。
- 假故障恢复、假清障：资源未清空却被标 FREE。
- 单任务重复执行、旧任务取消后仍运行、未授权主动移动。

真值采集优先在仿真步级别记录接触；仅低频截图不能证明无碰撞。几何采样建议至少 50 sim Hz，并验证最大采样间隔；快运动时用相邻 pose swept footprint 保守检查。最大间隔 >0.1 sim 秒则该区间评价 UNKNOWN，不能用插值掩盖缺数据。

### 3.3 初始阈值与冻结

以下是 v1 设计验收目标，P2 测出实现能力并记录后冻结；不是已经测量的结果。

| 项 | 初始要求 |
| --- | --- |
| 站点到位 | 真值平面距离≤0.15 m、偏航≤0.20 rad |
| 停稳 | 真值平面速度≤0.03 m/s、角速度≤0.05 rad/s，连续 1 sim 秒 |
| 服务完成 | 停稳后配置时长，默认 2 sim 秒；移动则服务计时重新开始 |
| 通道/资源 | 零双重授权、零未授权侵入、零超容量占用 |
| 非预期碰撞 | 零；接触过滤依据物理实体角色预先定义，不凭力小就忽略 |
| 任务一致性 | 零重复终结、零双持货、零错误站点成功 |
| 健康输入新鲜度 | CONTRACTS 中的阈值；测试超限停止 |
| 正常 case | 全部任务在声明预算内完成，无 safety FAIL/UNKNOWN |
| 故障 case | 达到清单预期，不能把所有结果统一要求任务成功 |

位置余量与停止距离不足时，P2 可以修改模型/布局/控制设计，但不得直接放宽验收使错误通过。若确有阈值设计问题，记录物理测量与影响，提交范围决定，经用户认可后升版阈值，旧结果仍保留。

Gate 为软件保护层，不是硬件急停。实测停车指标必须包含：故障检测延迟、零速度输出延迟、物理停止时间、物理停止距离，不允许只报 publish(0) 的时间。

## 4. P2 专项测量

1. 单车直行、旋转、停止，至少从 0.1/0.2/0.35 m/s 分别停车。
2. 同一 pose 的 odom、AMCL、Gazebo 真值比较，记录误差和方向。
3. 正确 footprint 是否覆盖所有轮子；转弯扫掠不碰墙。
4. 激光是否看见障碍、图像地图与 SDF 实体是否一致。
5. 丢速度输入、丢 adapter、Gate 退出、仿真 pause/resume 的停车行为。
6. 地图/资源图 hash 一致；地图 YAML image 使用相对路径。

输出 `calibration.json` 和 `docs/ENVIRONMENT.md` 的实测章节，再设置保守 a_stop 和保护区边界。不能用网上某款车的参数假装本模型测量。

## 5. 冻结 30 个基础案例

每个 case 有固定场景清单、seed、预算、故障触发条件、预期结果。故障按状态触发而非某个固定 sleep：例如已报告 OCCUPIED 后断开适配器心跳；超时未到触发条件为 PRECONDITION_NOT_REACHED，不算故障测试通过。

| ID | 内容 | 预期关键结果 |
| --- | --- | --- |
| N01 | 单车跨区 visit | 到正确站点、停稳、服务完成 |
| N02 | 单车 logical transfer 往返 | 正确领取/交付，没有实体装卸声明 |
| N03 | 双车各自在本区服务 | 并发运动，控制不串线 |
| N04 | 双车对向、r01 先申请 | r02 候车，r01 完全释放后 r02 通过 |
| N05 | 双车对向、r02 先申请 | 顺序随申请改变，不是固定车优先 |
| N06 | 双车同向批量任务 | 有序使用保护区，无追尾 |
| N07 | 三车、6 个混合区任务 | 分配合理、无重复、所有正常任务完成 |
| N08 | 三车有限任务流、站点/充电位竞争 | 队列有推进、无无限饥饿、容量不超 |
| F01 | 占用通道后插入阻塞物，随后移除 | 停车、保留资源、清障后有界恢复 |
| F02 | 唯一通道永久阻塞 | 明确 BLOCKED/attention，不非法绕墙 |
| F03 | 正确出口缓冲位被占 | 入口不授权，不把对向车放进来 |
| F04 | 取货前低电且能到充电区 | 取消停稳、合法归队、返航模拟充电 |
| F05 | 持货低电 | 保持货物归属、合法停车 attention |
| F06 | 空闲车掉线 | 不再分配，其他车执行可行任务 |
| F07 | 取货前正在运行的 Adapter 掉线 | 本地停车；确认旧命令失效后可接管 |
| F08 | 持货车掉线 | 不把货物瞬移给另一台 |
| F09 | 车在通道内失联 | 停车、资源 UNKNOWN、其他车不进入 |
| F10 | 许可续期中断但车仍发状态 | Gate 按 wall 到期停止，资源不盲目释放 |
| F11 | 中央调度器重启 | 新 epoch 对账，旧任务不重复执行 |
| F12 | Nav2 目标拒绝/服务器退出 | 明确 nav fault，本地停车，无 fake 成功 |
| F13 | 最终 Gate 进程退出 | 下游保护确实生效；否则 FAIL |
| F14 | 定位消息过期/TF 丢失 | 停车封锁相关区域，拒绝真值代驾 |
| I01 | 同请求重复提交 10 次 | 只创建/执行一个任务 |
| I02 | 同 request_id 不同内容 | 拒绝冲突，原任务不变 |
| I03 | 取消后注入旧 goal 成功回调 | 旧结果被忽略，无误完成 |
| I04 | sim 暂停后断通信，再恢复 | wall 保护有效，恢复仍需重新授权 |
| I05 | 运行中 clock reset | 当前 run 有解释地终结，不复用旧状态 |
| I06 | 真值采集器停止/数据缺失 | evaluator UNKNOWN，批测验收失败 |
| I07 | 数据库写故障 | 拒绝新任务、停车、可诊断退出 |
| I08 | 终止本 run 后重新启动 | 无遗留任务、无串线、不杀其他 run |

I03 的旧回调注入在 ROS 集成边界实施，仍使用真实车辆确认不会误动作；不是把整例改为 fake。I01/I02 等既有核心测试也应有真实运行环境的接口测试。

F01 障碍注入不能把实体瞬移到车体内部。先确认空余插入区域再放置；不具备插入条件则测试前置失败。移除仅限本 case 创建的障碍物。

### 批测策略

- P2/P3/P4 各先跑本阶段 2～4 个定向 case，失败先定位，不反复全量刷结果。
- P7 上述 30 基础例都必须执行。使用单 worker，避免 WSL GPU/CPU争抢和 ROS 串线；并行以后另验证。
- N04、N05、N07、N08 再各使用两个额外冻结 seed，增加 8 次；最终物理回归总计至少 38 次。
- case/seed 不能因失败被替换。重跑单独记录，不覆盖首次结果，不只拿最后成功的一次统计。
- 先测 1 例估算整轮 wall 时间和磁盘量；资源不足明确告知，不承诺“半小时一定跑完”。
- 基础 30 + 扩展 8 全部满足各自预期，安全无 FAIL/UNKNOWN，方可标 v1 完成。任何 NOT_RUN 则对应能力尚未验收。

## 6. 事件触发与故障实现细则

- “通信断开”要明确断哪个方向：state/permit/command，不能只把 UI 状态改 OFFLINE。
- “Adapter 退出”只终止该 run 目标进程，Gate 保持运行；Gate 退出另有独立 F13。
- “中央重启”保留 SQLite，不能新建空数据库。
- “数据库写失败”用专用测试数据库与可控故障注入，禁止耗尽用户整个磁盘。
- 电池 force-low 通过公开模拟服务，不能直接改任务完成状态。
- 自动故障触发有 ack、时间和场景状态快照。未实际触发的 case 不算成功。

## 7. 每次运行保存什么

```text
reports/<UTC>-<unique_id>/
  manifest.json
  inputs/                 # 完整配置副本、任务、资源/地图哈希
  events.jsonl
  robot_states.jsonl
  ground_truth.jsonl
  contacts.jsonl
  commands.jsonl          # Gate 前后命令与拒绝原因
  fault_events.jsonl
  processes.json
  stdout/ stderr/
  summary.json
  report.md
```

manifest 必须包含：Git commit（若有）、dirty diff hash、源码/资产/配置 hash、完整命令、依赖版本、backend、seed、schema_version、domain/partition、时间预算、所有测试开关、机器信息、仿真实时因子。

summary 必须包含：case 数与 ID、每例三种 outcome、完成/失败/需人工任务数、重复执行数、碰撞数、未授权进入数、资源冲突数、最长等待、平均/最大任务耗时、路径长度、停车延迟、采样缺失、输入是否被运行修改。

运行修改了输入配置或源码，`inputs_unchanged=false`，该批次不算冻结配置验收。隐藏 env 开关必须全部记录；unknown 默认值不能省略。

## 8. 有意义的性能对比

最终选同一地图、同一 6 个任务、同一速度/电池模型，对比单车与双车：总完成时间、总行驶距离、资源等待占比、能耗代理量、失败数。

不预设双车一定两倍快；唯一通道会限制并发。若双车反而更慢，解释是等待、路径增加、调度开销还是配置错误。报告实测，不做“效率提升 50%”的预填结论。

任务成功率和安全成功率单列；故障预期停车不能提高“运输成功率”。模拟能耗不是硬件续航测量。

## 9. 未来运行命令契约

以下命令要求接手 AI 实现后再在 README 标为可运行。本交接包不包含这些脚本。

```bash
cd ~/projects/009_multi_robot_fleet
bash scripts/doctor.sh
bash scripts/bootstrap.sh --check-only
bash scripts/build.sh
bash scripts/test_core.sh
bash scripts/test_integration.sh

# 真实双车对向让行
bash run_demo.sh --backend gazebo_nav2 --robots 2 --scenario opposing_requests

# 三车任务
bash run_demo.sh --backend gazebo_nav2 --robots 3 --scenario fleet_jobs

# 无界面故障测试，故障注入需显式开启
bash run_demo.sh --backend gazebo_nav2 --robots 2 --scenario comm_loss_in_corridor --headless --allow-fault-injection

# 纯逻辑 demo 要清楚标为 fake
bash run_demo.sh --backend fake --robots 2 --scenario opposing_requests

source scripts/env.sh
python3 scripts/batch.py --manifest config/scenarios/regression_v1.yaml --workers 1 --headless
bash stop_demo.sh --run-id ACTUAL_RUN_ID
```

退出码统一：0=本次声明的验收通过；2=输入/配置错误；3=依赖/启动问题；4=任务/物理验收失败；5=基础设施/采样/预算问题；130=用户中断。预期故障 case 若正确停车可返回0，但 summary 的 task_outcome 不能改成功。

正常 Ctrl+C 必须写 partial report，不把中断当 PASS。所有等待都有期限且能被取消，后台输出可读。

## 10. 复现与发布门

1. 在全新路径构建并运行，不读取 001～008 或旧绝对路径。
2. 依赖清单、构建命令、版本、地图来源完整。
3. 有头/无头分别记录已测试范围，不能从无头成功推断 WSLg 正常。
4. 关闭 GUI 后机器人运动与任务状态仍由服务端决定。
5. `.venv`、build/install/log、SQLite、海量 reports、凭据不进入 Git。
6. 原创资源许可证明确，引用代码按来源保留 notice。
7. README 写清真实完成范围：轮式物理移动 + 逻辑货物、模拟充电、集中式预约；人形/实体交接/LLM 为未实现。
8. 用户确认后再整理 GitHub 对应目录；没有确认不得推送。
