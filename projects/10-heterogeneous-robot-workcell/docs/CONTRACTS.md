# 010 最小接口契约（P2 实现目标）

## 1. 通用规则

schema_version=1；米、秒、弧度、牛顿、千克；四元数顺序按接口显式声明为 xyzw，不允许隐式混用。
拒绝未知字段、重复键、NaN/Infinity、非法实体/数量/路径。输入文本与模型响应不是执行权限。
同时记录 run_id、request_id、task_id、command_id、revision、device_boot_id。

## 2. Order

```json
{
  "schema_version": 1,
  "request_id": "request-001",
  "order_id": "order-001",
  "destination_id": "station_b",
  "items": [{"type": "red_block", "count": 2}, {"type": "blue_cylinder", "count": 1}],
  "priority": 10
}
```

ID 格式/长度、最大行项/数量在 schema 中固定。同 request_id 同内容返回原结果；不同内容拒绝冲突。
用户不能填写控制阈值、任意关节目标、shell命令或仿真坐标。
订单状态：QUEUED→PREPARING_TRAY→KITTING→VERIFYING_KIT→WAITING_TRANSPORT→LOADING→IN_TRANSIT→UNLOADING→VERIFYING_DELIVERY→SUCCEEDED。
任意阶段可进入受约束 WAITING、CANCELING、FAILED、NEEDS_ATTENTION；保留原阶段和具体原因。

## 3. DeviceState / SkillRequest / SkillResult

DeviceState：device_id、boot_id、sequence、sim_stamp、capabilities、operating_state、active_command、
pose及frame、观测年龄、stopped_confirmed、held_payload_id、resources、fault、battery（适用时）。

SkillRequest：command_id、order_id、revision、expected_boot_id、skill、具名语义参数、resource_generation、deadline。
技能白名单候选：OBSERVE、PRESENT_TRAY、PICK_PART、PLACE_PART、VERIFY_KIT、MOVE_TO_STATION、DOCK、
START_TRANSFER、VERIFY_TRANSFER、UNDOCK、VERIFY_DELIVERY、STOP。RETURN_TO_CHARGE 仅扩展预留，不注册未实现的技能。
参数只能引用已注册物品/站点/料位；路径和控制量由受信任适配器生成。

SkillResult：accepted / running / succeeded / failed / canceled、reason_code、证据引用、实测终态、start/end。
重复 command 不重复抓取；旧 revision/boot_id 结果不得改变现任务；取消必须确认停稳或明确冻结/attention。

## 4. 物品、托盘与归属

PartState：物理实体 ID、类别、可见性、估计位姿/不确定性/来源/年龄；独立评测保有私有真值 ID。
TrayState：tray_id、实际观测内容、绑定订单、位置类型、custody、transfer_id。
预期 BOM 和观测 contents 必须分离。订单要求三个零件不代表传感器看到了三个。

货物状态：AT_SOURCE、ON_WORKCELL、TRANSFERRING、ON_AMR、AT_DESTINATION、UNKNOWN。
只有稳定交接确认才能改变唯一归属；跨设备期间用 TRANSFERRING 明确占两端资源，不宣称已在任一端完全到位。
交接中断不得直接重新分配托盘；保持两端停止、保留事务、请求有证据恢复或人工处理。

## 5. TransferTransaction

```text
REQUESTED → BOTH_RESERVED → DOCK_VERIFIED → BOTH_READY
          → TRANSFERRING → RECEIVER_CONFIRMED → COMMITTED → RELEASED
任一异常 → STOPPING → NEEDS_ATTENTION / ABORTED（仅证明未发生转移时）
```

启动条件：双方同 transfer_id/epoch、空余容量、相对高度/横向/偏航/间隙合格、AMR停稳、源端有盘、
接收端无盘、机械臂/人形退出、证据新鲜、停止链健康。任一 UNKNOWN 拒绝启动。
完成条件：接收端确认托盘完全进入、源端已清空、盘速度达标、保持机构到位、两端输送停止。
仅单个光电传感器触发不能同时证明上述所有条件；定义各证据的传感器来源。
标量阈值在 P1/P3 以几何和测量冻结；冻结前不得声称通过。

## 6. 资源与进程失效

资源 FREE/RESERVED/OCCUPIED/CLEARING/UNKNOWN/BLOCKED；默认 UNKNOWN，需要初始化清空证据。
授权带 owner、generation、epoch、ttl；延迟响应不能重新计满 TTL；已占用时超时不释放。
设备失联：命令门和驱动下游保护生效；持盘车不能自动让另一台接管货物。
中央重启保留事务与账本，先对账再派发，不清空数据库冒充恢复。

## 7. 结果与原因

分别输出 task_outcome、physical_result、safety_result、expected_behavior、evidence_complete；字段取值以 TEST_AND_ACCEPTANCE 为准。
正常任务全部必需检查 PASS 且数据完整才能顶层 ACCEPTED；缺项 UNKNOWN/NOT_RUN 则不能 ACCEPTED。
故障测试可以“正确停车”为预期通过，但不能计为配送完成。

原因至少包括：NO_ELIGIBLE_DEVICE、OUT_OF_STOCK、TARGET_NOT_VISIBLE、STALE_OBSERVATION、
NO_FEASIBLE_GRASP、CONTACT_LOST、KIT_MISMATCH、DOCK_OUT_OF_TOLERANCE、RECEIVER_OCCUPIED、
TRANSFER_TIMEOUT、TRANSFER_UNKNOWN、PAYLOAD_LOST、RESOURCE_UNKNOWN、PERMIT_EXPIRED、
LOCALIZATION_INVALID、NAV_FAILED、CANCEL_UNCONFIRMED、DEVICE_OFFLINE、CLOCK_RESET、EVIDENCE_MISSING。
不要把所有失败写成 BUDGET_EXHAUSTED；记录最后有效进展、谁在等谁和首个拒绝原因。
