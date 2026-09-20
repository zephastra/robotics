# 009 接口与行为契约

设计版本 v1；以下是要实现的接口，不是对现有源码的描述。P1 先把 Python 数据类型和验证规则落地，P3 再做 ROS 映射。无必要不增加新消息类型；变更先记录兼容性。

## 1. 基本规则

- 长度 m，速度 m/s，角度 rad，电量使用 `[0,1]` 的 fraction，不混用百分数。
- 世界坐标统一 `map`，平面 pose 为 `(x,y,yaw)`；禁止把机器人局部坐标当 map。
- 标识符使用 `[a-z][a-z0-9_]{0,47}`；UUID 型任务/请求 ID 单独验证。
- 配置拒绝未知字段、NaN、Infinity、重复 ID、不存在站点、不合法机器人和负预算。
- 所有外部请求以结构化解析为准，文本不是指令代码，不 eval、不 shell 拼接执行。
- `schema_version: 1`；不兼容变更升版，报告同时记录版本。

## 2. 任务输入

建议 YAML/CLI 经同一个 validator 生成 `TaskRequest`：

```yaml
schema_version: 1
request_id: "11111111-1111-4111-8111-111111111111"
task_id: "22222222-2222-4222-8222-222222222222"
kind: station_transfer
source_station: source_west
destination_station: sink_east
required_capabilities: [navigate, logical_transfer]
payload_mode: logical
payload_id: cargo_001
priority: 10
service_duration_s: 2.0
max_attempts: 2
timeout_sim_s: 180.0
```

含义：到取货区稳定停留，领取逻辑货物，去目标区稳定停留，完成逻辑交付。`payload_mode: physical` 在 v1 必须返回 `UNSUPPORTED_PAYLOAD_MODE`，不能降级 logical 后报成功。

任务类型 v1 只实现：`station_transfer`、`visit_station`、系统生成的 `return_to_charge`。UI 可以中文显示，但内部枚举不能随意同义替换。

去重规则：

1. 同 request_id、相同规范化内容：返回原来的接收结果，不再创建任务。
2. 同 request_id、不同内容：`IDEMPOTENCY_CONFLICT`。
3. 同 task_id 已存在：查询并返回状态或冲突，不创建第二个实体。
4. payload_id 已在使用/已交付：拒绝新的重复领取，除非用户另建合法回程任务并有新的货物状态依据。
5. request_id 不是消息序号；数据库需要唯一约束，不靠内存字典去重。

`SubmitTask` 响应：accepted、task_id、state、reason_code、message、event_seq。accepted 只表示接收，绝不是完成。

## 3. RobotState

最小字段：

| 字段 | 说明 |
| --- | --- |
| schema_version / robot_id / robot_boot_id | 机器人适配器重启必须换 boot_id |
| sequence | 同 boot_id 单调递增，拒绝旧状态覆盖新状态 |
| stamp_sim | 仿真采样时间 |
| pose_map / twist / localization_valid | 实际控制使用的估计状态，不是真值 |
| pose_covariance / pose_age_s | 定位质量与新鲜度 |
| operating_state | OFFLINE / IDLE / EXECUTING / WAITING / CHARGING / FAULT / ESTOP |
| task_id / assignment_revision / leg_id | 当前有效工作归属 |
| capabilities | navigate、logical_transfer、charge 等 |
| battery_fraction / battery_valid | 电量模型输出及有效性 |
| payload_id | 空或已领取的逻辑货物 |
| held_resources / permit_generation | 当前授权，不是占用真值 |
| active_goal_id / nav_state / gate_reason | 分层诊断 |
| stopped_confirmed / fault_code | 不能用“命令零”代替实际停止 |

接收者自己用本地 monotonic 记录最后收到有效消息的时刻；不能跨进程比较独立 monotonic 时间戳。UTC 只用于可读日志，不用于安全计时。

`localization_valid` 初始建议同时要求：TF 可用、pose_age≤0.5 s、位置标准差≤0.10 m、偏航标准差≤0.20 rad。P2 实测后冻结阈值；异常时拒绝开始新 leg，并停车。这个阈值不等于实际误差保证，独立 evaluator 仍检查真值误差。

## 4. 适配器协议

纯 Python 抽象接口：

```python
class RobotAdapter(Protocol):
    def capabilities(self) -> RobotCapabilities: ...
    def state(self) -> RobotState: ...
    def execute_leg(self, command: ExecuteLegCommand) -> CommandAck: ...
    def cancel(self, command_key: CommandKey) -> CancelAck: ...
    def poll_events(self) -> list[AdapterEvent]: ...
    def emergency_stop(self, reason: str) -> StopAck: ...
```

这些方法不能阻塞等待整个导航。ROS action 接收、反馈、结果转换为事件送回状态机。fake 与 nav2 实现遵守同一契约，但 backend 必须明确。

### ExecuteLegCommand

- task_id、assignment_revision、leg_id、command_id。
- expected_robot_boot_id、dispatcher_epoch（服务重启代次）。
- goal_station 或 lane_node_id；目标由可信配置解析，普通任务提交者不能任意传执行器控制参数。
- route_segment_id、resource_bundle_id、permit_generation（非保护区 leg 可以为空）。
- timeout_sim_s、allowed_retries、preconditions。

ROS `/rXX/fleet/execute_leg` action 对应这一命令；内部 Nav2 `/rXX/navigate_to_pose` action 只由该 Adapter 调用。

结果包含：accepted / rejected、终态、reason_code、最终估计 pose、耗时、Nav2 状态、是否确认停止。结果不能只有 success 布尔值。

### 取消、幂等与旧回调

1. 已处理 command_id 不重复发 goal。
2. 更旧 assignment_revision、dispatcher_epoch 或不匹配 boot_id 的请求被拒绝。
3. Nav2 goal accepted 不意味着机器人到达；cancel accepted 不意味着机器人停稳。
4. 取消时进入 CANCELING，等待 action 终态以及实测速度连续达标。
5. 旧 goal 晚到 SUCCEEDED，只记 `STALE_RESULT_IGNORED`，不能完成新 leg。
6. cancel 超时：Gate 停车、标记 `CANCEL_UNCONFIRMED`，保留可能占用的区域与货物归属。
7. 本地没有 ROS 服务器、类型不匹配、无定位，不允许静默改用 fake adapter。

## 5. 任务状态机与货物账本

```text
QUEUED -> ASSIGNED -> TO_SOURCE -> SOURCE_SERVICE
       -> TO_DESTINATION -> DESTINATION_SERVICE -> SUCCEEDED

运行阶段可进入：WAIT_RESOURCE / PAUSED / RECOVERING
取消路径：CANCELING -> CANCELED（满足停止与账本条件）
不可自动安全继续：NEEDS_ATTENTION
确定任务失败：FAILED
```

主 phase 和等待/运行 substate 分开储存，WAIT_RESOURCE 不丢失之前是 TO_SOURCE 还是 TO_DESTINATION。终态不能被迟到回调逆转。

### 领取/交付原子条件

SOURCE_SERVICE 前必须同时满足：

- 机器人属于该任务当前 assignment_revision。
- 对应站点服务位独占授权有效。
- 到正确站点，定位有效，速度连续稳定，服务时间完成。
- payload_id 处于 AT_SOURCE，非别的任务占有。

事务更新 `payload.state = HELD_BY_ROBOT`、holder、task_id、任务 phase，并写事件。进程在事务后崩溃，重启读取同一结果，不再次领取。

DESTINATION_SERVICE 类似，目标站点必须与任务绑定目标一致，事务更新 `DELIVERED`。Nav2 到了别的站点不能交付。

### 故障接管

| 故障时刻 | 可以做什么 | 禁止做什么 |
| --- | --- | --- |
| 未分配 | 换合格机器人 | 重复创建任务 |
| 已分配、未领取货物 | 旧代次失效且确认安全停止/隔离后，增加 revision 再分配 | 仅心跳断开就马上让两台争抢同一货 |
| 已持货 | 标记 NEEDS_ATTENTION，允许其他无关任务继续 | 把 payload_id 改给新车当作实体转移 |
| 通道内掉线 | 封锁资源，等待清障/可信恢复 | TTL 到期自动宣告通道空 |
| 已完成交付但确认消息丢失 | 根据持久账本幂等返回完成 | 再执行一次装卸 |

当中央服务无法确认旧机器人已停止，不能自动接管该任务。安全停车证明可来自恢复后的 Adapter 状态；单机模拟故障的独立控制面可以确认该进程已停且本地 Gate 生效，但必须记录证据，不从心跳丢失推导。

### 低电

- 分配前检查预计路线/等待/服务/到可用充电位的消耗与储备。
- 默认演示模型可用线性消耗：`E = idle_rate*dt + distance_rate*distance + turn_rate*abs(delta_yaw)`；dt 用 sim，参数写配置并标注不是实测电池。
- idle/service/charging 都按同一个模型积分，暂停 sim 不继续消耗。
- threshold 初始可取低电 0.20、紧急 0.08、恢复接单 0.80；实际是演示规则，不代表真实锂电保护。
- 未持货低电：取消当前 leg、确认停止；若可达安全充电位则生成 return_to_charge，原任务按规则归队。
- 持货低电：v1 不自动卸货、不自动把货带到不支持载货的充电位；安全停到可达合法位并 NEEDS_ATTENTION。
- 紧急电量不足且无法证明可到充电位：停稳报警，不能承诺自动返航。
- 低电发生在通道内：剩余能量足够且导航安全时优先清空共享区再执行后续策略；失效/急停则原地停车并封锁，不为“清道”忽略安全。
- CHARGING 需正确站点、站点独占、停稳和充电服务时间；不会真实电气对接，报告写 simulated charging。

## 6. 分配算法 v1

先过滤：在线、状态新鲜、无故障、空闲、能力满足、没有冲突货物、路线连通、电量足够。无合格候选则 WAIT_NO_ELIGIBLE_ROBOT，说明原因，不强行分配。

候选成本以预计时间为单位，避免把米、百分比、秒随意相加：

`cost_s = approach_path_m / nominal_speed + task_path_m / nominal_speed + resource_wait_estimate_s + service_s`

任务按 priority 与等待老化排序；同任务候选成本相同按 robot_id 固定排序。老化规则具体冻结：每等待 30 sim 秒提升一级调度优先级，上限 100；这是有限工作负载下防饥饿政策，不保证无限高优先级任务流下的期限。

第一版不抢占正在持货任务。较新优先任务只能在空闲分配点影响顺序。路径长度取 lane graph 最短路，实际 Nav2 绕障耗时单独记录，不宣称全局最优。

## 7. 资源预约协议

### Resource / ResourceBundle

字段：id、polygon_map、capacity（v1=1）、entry/exit节点、合法等待节点、state、owner、generation、permit expiry、last observed occupancy、queue。

受保护区域外再设“不得越过的停止边界”，必须满足停止距离余量；资源多边形不是只画在机器人中心线。

穿越需要的通道、连接弯道和出口缓冲作为一个 bundle 一次原子申请，不分步占着等。

### 状态机

```text
FREE -> RESERVED -> OCCUPIED -> CLEARING -> FREE
          |             |          |
          +---------- UNKNOWN / BLOCKED
```

FREE 的含义是授权账本无人占、估计状态确认无车、清空证据有效，而不是租约列表为空。

初次启动时资源默认 UNKNOWN；等本次所有配置车辆完成定位和占用核查后，才将有充分清空证据的资源置 FREE。不能在车辆尚未上线时先授权进入它可能占用的区域。

`AcquirePassage`：robot_id、boot_id、task/revision、bundle_id、direction、request_id。服务器拒绝不在合法等待位置/定位失效的入场请求，或将其保留在排队但不给入场许可。

响应 `ResourcePermit`：granted、request_id、bundle_id、owner、generation、dispatcher_epoch、valid_for_wall_ms、allowed_entry、allowed_exit、reason_code。不使用不同进程的原始 monotonic 数值。

必须防迟到许可延长有效期：客户端记录对应请求的本地 monotonic 发出时刻，保守使用 `请求发出时刻 + valid_for_wall_ms` 作为本地到期时刻，而非收到响应后重新计满整个 TTL；服务端有效期不短于所承诺的该请求持续时间。响应到达时本地期限已过就拒绝。每次续期有新的 request_id，只接受仍有效的本次响应，不接受缓存/重放的 permit topic 作为新授权。服务端延迟、网络乱序与重复续期都须有测试。

`RenewPermit`：必须同 owner/boot/revision/generation，续期响应回传当前 epoch；不能仅靠持续收到旧话题消息无限续期。

`ConfirmClear`：Adapter 提交已驶出与定位证据，服务器自己验证最新估计 footprint 完全不交叠保护区且持续稳定，再释放；响应不是盲目信任车辆传一个 clear=true。

### 超时与失联

- 初始建议机器人状态 10 Hz；失联 1.5 wall 秒；许可 2 wall 秒有效、0.5 wall 秒续期。
- Gate 高频检查许可，过期后停止新运动并报告；不能借用“还差一点”继续驶入。
- OCCUPIED 后过期：资源 UNKNOWN/BLOCKED，**保持封锁**。
- RESERVED 但未进入，只有新鲜定位证明整车还在区域外且已停车，才能安全撤销。否则 UNKNOWN。
- 清空未知状态需重新获得全部相关机器人可信状态并完成几何检查，或明确人工清障确认流程；无头批测不能自动点击“已清空”来通过。
- 候选参数 P2/P4 实测冻结；时延预算无法满足时降低速度/扩大停区，不能取消安全检查。

### 公平性

请求按入队序号 FIFO；取消离队有事件；有效请求的年龄不能因为每次轮询重新入队而清零。只要前方释放且请求仍合法，就推进下一位。

v1 单组保护区避免多资源环等待。若出现持续占用，输出 owner/phase/最后定位/许可时间/出口阻塞原因；不能把任何长等待都称为“无死锁”。

## 8. 必须让导航服从预约

正确执行链：

1. 无许可时最多导航到保护区外的候车位。
2. 根据几何校验移动到安全对齐点不能越过停止边界。
3. 获得有效 bundle 许可后才发送跨越 leg。
4. 行进中 Gate 继续检查许可、定位新鲜度、通信健康、急停和区域边界。
5. 驶出到出口缓冲位，继续以同一授权驶向保护区外 release 节点；整车连同余量离开整组区域后才完成 clear。不能在仍处于 bundle 内的缓冲位停车时释放 bundle。

仅用 waypoint 不足以证明 Nav2 不绕行。v1 的隔断确保区域是唯一跨区通路；Gate 用预测 footprint 与禁止边界阻止无许可穿越。若规划器反复尝试违规路径，取消 leg 并返回 `ROUTE_REQUIRES_PERMIT`，不是无限输出被拦截的速度。

Gate 检查保守停止可达区域：当前位置 footprint 加定位余量，再考虑 `v*latency + v²/(2*a_stop)` 和转向扫掠；a_stop 必须从 P2 停车实测取得保守值。简单实现可使用保守外接圆作禁止区域检查，允许通行时再用 polygon 验证，不要写看似精确但无测试的复杂控制器。

若有效定位误差超过设计界限或无法保证停在边界外，应拒绝开始该 leg。进入区域后定位丢失，停车封锁，不用 Gazebo 真值接管导航。

Gate 停止只证明最终速度命令为零，不能证明物理瞬间停止；Evaluator 必须测实际停止时间与距离。Nav2 行为树恢复也必须经过 Gate，不允许它在窄道内无约束倒车/旋转；跨越 leg 配置禁用不适合该区域的恢复行为。

## 9. 时钟、暂停和预算

| 用途 | 时钟 |
| --- | --- |
| ROS 状态 stamp、轨迹、任务服务、能耗 | simulation time |
| 进程存活、许可、心跳超时、启动超时 | 本进程 monotonic wall time |
| 可读报告名称 | UTC + 随机唯一后缀 |

仿真暂停期间任务 sim 预算冻结，但心跳/许可逻辑仍工作。本地 Gate 用 wall timer；不能因为 ROS sim timer 不触发就失去失联保护。

恢复仿真时默认保持零输出，重新确认状态、许可与操作者 resume；不能自动重放暂停前的速度。仿真 reset 或时间倒退时终止当前 episode、停止输出、标 CLOCK_RESET，需要新 run，不混用旧账本。

初始预算：startup 90 wall 秒；Nav2 单 leg 60 sim 秒；普通任务 180 sim 秒；资源等待最多 60 sim 秒，超限转明确 WAIT_TIMEOUT/NEEDS_ATTENTION；重试最多 2 次；cancel 确认 3 wall 秒；恢复最多 1 次。场景可提前声明不同预算，不能运行中悄悄加。

每次 case 另有 wall deadline，按 P2 测得最差实时因子设上限并记录，例如 120 sim 秒场景初始 wall 300 秒。若慢机无法满足，报告 INFRA_TIMEOUT，不能当作控制成功或偷偷少跑案例。

## 10. 本地安全与进程生命周期

- Gate 建议 50 Hz wall timer；cmd 接收超时初始 0.3 wall 秒，超时主动持续输出零。
- dispatch/adapter 失联停车；Gate 自身崩溃也需下游失效保护。
- 必須验证 Gazebo 驱动是否有速度命令超时；若没有，增加小型底层超时保护插件/接收器，或独立监督器在 Gate 退出时暂停世界并使驱动归零，等待重新初始化。不能假定最后速度自动消失。
- 任一保护链未实现，对应故障测试为 FAIL/NOT_RUN，不声称任意进程崩溃安全。
- `stop_demo.sh --run-id X` 只能操作该 run 拥有的 PID/进程组，核对 token/启动标识，防 PID 重用。
- 正常退出：拒绝新任务→取消本 run goals→Gate 零输出→确认停止→保存账本/报告→SIGINT 本 run 子进程→超时后只升级本 run 的终止操作。
- 禁止 `pkill -f python`、`killall gz` 或全局清理 ROS，禁止删除整个 reports 目录。
- 启动 readiness 按 map、TF、scan、clock、Nav2 lifecycle/action、Gate、fleet 状态逐项等待；不以“进程启动了 + sleep 5”代替就绪。

## 11. 持久化、重启与恢复

SQLite 最小表：tasks、assignments、payloads、resources、requests、events、metadata。事务中同时写状态与事件；允许 at-least-once 事件送达，消费者按 event_id 去重。不要宣称网络 exactly-once。

中央重启：

1. 新 dispatcher_epoch；停止派发。
2. 从 durable ledger 恢复未终结任务和资源；已有占用保守标 UNKNOWN。
3. 各车看到 epoch 变化，不继续旧命令；停车并上报本地实际任务/货物/位置。
4. 对账一致、动作取消确认、几何检查完成后，才重新授权与增加 revision。
5. 若对账矛盾，NEEDS_ATTENTION；不能把数据库清空当恢复。

日志损坏/磁盘写失败时拒绝新任务，停车并报告 `PERSISTENCE_FAILURE`。先写事务再发执行命令；重启后同 command_id 幂等重发，不重复实际服务。

## 12. 诊断事件与结果

每事件至少：run_id、case_id、event_id、sequence、sim_time、UTC、source、robot_id、task_id、revision、leg_id、resource_id、from_state、to_state、reason_code、details。

至少支持这些 reason_code：

`INVALID_REQUEST`、`IDEMPOTENCY_CONFLICT`、`NO_ELIGIBLE_ROBOT`、`INSUFFICIENT_ENERGY`、`LOCALIZATION_STALE`、`NAV_SERVER_UNAVAILABLE`、`NAV_FAILED`、`NAV_STALLED`、`GOAL_TIMEOUT`、`WAIT_RESOURCE`、`RESOURCE_TIMEOUT`、`RESOURCE_UNKNOWN`、`EXIT_OCCUPIED`、`PERMIT_EXPIRED`、`ROUTE_REQUIRES_PERMIT`、`STALE_RESULT_IGNORED`、`CANCEL_UNCONFIRMED`、`PAYLOAD_HELD_NEEDS_ATTENTION`、`PERSISTENCE_FAILURE`、`CLOCK_RESET`、`SAFETY_STOP`。

运动问题每次记录：目标/误差、Nav2 action 状态、原始速度、Gate 后速度、Gate reason、最后 scan/odom/TF 年龄、资源 owner/generation、实际轮速或 odom 速度。不要只报一个笼统 `BUDGET_EXHAUSTED`。

## 13. 未来 CLI 接口（开发目标）

```bash
source scripts/env.sh
ros2 run fleet_tools fleet_cli status
ros2 run fleet_tools fleet_cli submit --file config/scenarios/task_example.yaml
ros2 run fleet_tools fleet_cli cancel --task-id TASK_UUID
ros2 run fleet_tools fleet_cli battery force-low --robot r01 --fraction 0.15
ros2 run fleet_tools fleet_cli pause --robot r01
ros2 run fleet_tools fleet_cli resume --robot r01
ros2 run fleet_tools fleet_cli estop --robot all
ros2 run fleet_tools fleet_cli resources
```

fault injection 只在显式 `--allow-fault-injection` 的测试 run 开启；API 默认无该权限。force-low 操作限定模拟电池，不意味着现实充电能力。

CLI mutation 请求都走校验与账本，不直接改 SQLite、进程内变量或 Gazebo pose。取消/暂停报告“已接收/正在停止/已停止”，不能一个 accepted 就显示完成。
