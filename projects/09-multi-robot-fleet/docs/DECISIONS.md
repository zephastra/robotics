# 009 决策记录

只记录**会影响后续实现**的选择，以及被否掉的替代方案。每条尽量写"如果不这么做会怎样"。

---

## D-P4-01 不建单独的 `config/safety.yaml`（2026-09-15）

**决定**：调度侧阈值（`permit_ttl_s` / `battery_reserve_wh` / `low_battery_wh`）留在
`config/fleet.yaml` 的 `safety:` 段，由 `validate_fleet_config` 读取；交通侧时序与停止模型放在
`config/resources.yaml` 的 `traffic:` 段，由 `validate_traffic_config` 读取。**不新建 safety.yaml。**

**原因**：MASTER_PLAN §6 把 `safety.yaml` 列在目标结构里，但**同一批安全参数存在两处是缺陷，不是结构**。
`fleet.yaml` 已经是这些值的唯一来源并有校验器；再建一个文件必然出现"改了一个没改另一个"的静默不一致，
而这些值直接决定许可什么时候过期。

**代价**：与 MASTER_PLAN 的目录清单不完全一致，需要在 README / 本文件说明。

**被否**：把 `safety:` 从 `fleet.yaml` 搬进新的 `safety.yaml`。会让既有 177 项 P1 测试与
`validate_fleet_config` 的锚点全动，收益只是"更贴计划清单"。

---

## D-P4-02 禁止区域 = "未持有资源的矩形并集"，不另设保护区（2026-09-15）

**决定**：门禁的禁止区域是**该车当前未持有**的每个资源的矩形并集（各自按停止包络 + 定位余量膨胀）。
不引入独立于资源列表的"保护区"配置。

**原因**：独立的保护区列表与资源矩形列表会漂移——有人改了资源矩形忘了改保护区，门禁就放车进未授权区域，
而且**不会有任何报错**。一张表一个真相。

**被否**：单个 `protected_region` 覆盖通道 + 两侧出口缓冲位。看起来更简单，但会让"持有
`[mid, mid_right]`"的车在 `mid_left` 上也被视为合法（落在同一个大区域里），两车于是可以停在一起。
逐资源判定才有 `test_holder_may_not_drive_into_the_other_exit_buffer` 这一条。

---

## D-P4-03 几何图元只用轴对齐矩形（2026-09-15）

**决定**：区域是轴对齐矩形；车身是带朝向的矩形，但**从不分解成一个点**。

**原因**：
- 只判中心点 ⇒ 拒绝太少。CONTRACTS §7 明写"资源多边形不是只画在机器人中心线"。
- 只判外接矩形 ⇒ 拒绝太多。0.60×0.45 的车在 45° 时外接盒 0.74 m，门禁会拦住合法运动。
- 因此外接矩形只做廉价**排除**，命中后用分离轴精确复判。

**验证**：`test_sat_matches_reference_on_a_grid` 在 13448 个位姿上与独立写的精确实现逐例比对；
`test_aabb_alone_would_over_refuse` 断言两者**确实会分歧**，避免有人删掉复核后测试仍然全绿。

---

## D-P4-04 通道矩形比物理缺口宽 0.25 m（2026-09-15）

**决定**：通道矩形 x∈[-1.75, 1.75]；物理缺口（`corridor_marker`）是 x∈[-1.5, 1.5]。

**原因**：把入口喉部算进去。一辆车"横跨缺口口沿"时，从任何实际意义上看都已经进去了；
按物理缺口画矩形会让这类姿态显示为"在外面"。代价是 align 节点必须退到 x=±2.4，
已由 `validate_traffic_geometry.py` 对两个车身尺寸各验一遍（且该脚本会随 `fleet.yaml` 变化重跑）。

---

## D-P4-05 P4 核心保持不 import ROS（2026-09-15）

**决定**：`geometry.py` / `traffic.py` 纯 Python；`tests/test_p4_*.py` 在**没有 ROS**的解释器里跑
（`scripts/test_core.sh` 会在 `ROS_DISTRO` 已设置时直接拒绝运行）。

**原因**：这是项目既有纪律。收益是可验证性：门禁里"错了会静默放行"的算术，全量测试 0.8 秒跑完，
不需要起 Gazebo、不需要等 Nav2 lifecycle。等到 P4.3 接 `gate_node.py` 时，
被接入的仍然是一个已经被 56 个测试钉住的函数。

---

## D-P4-06 停止模型取实测值的悲观端（2026-09-15）

**决定**：`a_stop_mps2 = 0.40`，由 P2 真值实测最差点（0.35 m/s → 0.1381 m）反推
`v²/(2s) = 0.4436` 后**向下取整**；`latency_s = 0.20` 先按偏大取值，等 P4 实测门的
命令到里程计延迟后再改。实测原始值原样记在 `resources.yaml` 的
`stop_model.reported_stop_distance_m` 里，便于复核与重推。

**原因**：这个数偏小 ⇒ 门禁授权它停不下来的运动。所以宁可保守。
`test_stop_envelope_covers_the_measured_stop_distance` 强制三个速度点全部满足
`envelope(v) > measured(v)`，防止有人把 `a_stop` 调大。

---

## D-P4-07 `AdapterStatus.localization_valid` 默认 `False`（2026-09-15）

**决定**：`AdapterStatus` 新增的 `localization_valid` 默认值是 **`False`**（不安全的那一个）。
`FakeAdapter` 显式声明 `True`。

**原因**："适配器忘记上报定位健康" 与 "上报定位健康" 在缺省值上是同一个值。
若默认 `True`，则一个从不检查定位的适配器会因为**字段缺失**而获得进入保护区的资格，
而且没有任何测试会变红。默认 `False` 让"沉默"等价于"不可信"，与门既有的
`HEARTBEAT_LOST` / `STATE_STALE` 处理一致。

**代价**：任何新的适配器在被接入前必须先设置该字段，否则机器人寸步难行。
这是有意的——症状是"不动"，而不是"走了但不该走"。

---

## D-P4-08 门装在 `fleet_adapter`，几何留在 `fleet_core`（2026-09-15）

**决定**：门用 `ZoneGuard` **协议**（结构化类型）接收判定，自己不 import `fleet_core`；
`CrossingZoneGuard` 放在 `fleet_adapter`，由它适配 `CrossingManager`。

**原因**：
- 门必须能解释"我拒绝的原因"，但**不应该知道什么是通道**。让门认识走廊会把交通模型焊进安全层，
  以后加第二个路口就要改门。
- `fleet_adapter/package.xml` 已经有 `<exec_depend>fleet_core</exec_depend>`，方向是允许的；
  门不依赖 `fleet_core` 则保证它可以被单独测试（22 项 P2 门测试仍全过）。
- `tests/test_p1_adapter_no_ros.py` 按 `ast` 检查 ROS 模块名，`fleet_core` 不是 ROS ⇒ 不冲突。

**被否**：把 `CrossingManager` 直接塞进 `SafetyGate`。会让门与交通配置耦合，
且 `SafetyGate` 目前可被 22 个测试在没有任何几何配置的情况下构造，这个性质值得保住。

---

## D-P4-09 不接线 `gate_node.py`，直到 P4.2 存在（2026-09-15）

**决定**：P4.3 只交付策略层；**不在** `gate_node.py` 里构造 `CrossingZoneGuard`。

**原因**：`CrossingZoneGuard` 需要 `CrossingManager` 提供"这台车现在持有什么许可"，
而 `CrossingManager` 属于尚未实现的 P4.2 协调节点。现在硬接线的两个选择都不可接受：
(1) 没有许可来源 ⇒ 车永远出不了通道 ⇒ 打断 P3 已验证的穿越能力；
(2) "接上但默认关掉" ⇒ 看起来有证据，实际是假的。
**宁可在报告里写明策略就位、链路未接，也不制造一个假证据。**

**代价**：生产路径暂时仍是"无 guard ⇒ 相信调用方 flag"。已在
`test_without_a_guard_the_legacy_flag_path_is_still_permissive` 里把这个旧行为明说出来，
避免有人误以为旧路径是围栏。


## D-P4-10 — 许可的 epoch 由协调者盖章；`AcquirePassage.Request` 里没有这个字段

**背景**：`_srv_acquire` 把 `int(req.epoch)` 写进租约账本并存在 robot link 上，
而 `_live_permit` 用 `self._epoch` 去比。驱动硬编码 `self.epoch = 0`，协调者是 1。
结果：协调者发出的**每一份**许可对它自己的发布器都不可见，线上永远是 `granted=false`，
门禁 mirror 永远空，车被自己的门禁钉住，Nav2 在空走廊上超时。同一次运行里 **37 次续期全部成功**
（`book.renew` 比的是 `p.epoch != epoch`，两边都是 0）—— 同一个值，一条路径通一条路径断。

**决定**：

1. `AcquirePassage.srv` 的 **request 删掉** `int32 epoch`，**response 加上** `int32 epoch`。
2. 协调者 `epoch=self._epoch`、`link.epoch = self._epoch`、`resp.epoch = self._epoch`。
3. 驱动 `self.epoch = int(resp.epoch)`；`RenewPermit.Request` / `ConfirmClear.Request` 保留 epoch。

**理由**：epoch 是**协调者**的属性。让被监管方填它，和 P4.3 删掉的 `wants_protected_zone`
是同一类漏洞 —— 被监管方申报的值最多只能把检查**关掉**，不可能让它更严格。
**删掉输入，而不是只改那一行读取**：只改读取，字段还留着，下一个人会再填一次。

**代价 / 边界**：改动接口需要重新生成 rosidl 并重建五包（约 26s）。
`_epoch` 目前硬编码 1 且**从不递增**，所以它**不是重启检测器** —— 租约账本在内存里，
重启本来就丢光所有许可，没有"过期"可言。`RenewPermit.srv` 里原来那句
"echoed back so a client that missed a restart can notice" 与代码不符，已改成实话。

**测试**：`test_the_coordinator_stamps_its_own_epoch_on_a_grant`（源码级——这个缺陷不是算术错，
是**选了哪个值**，只能看代码）、`test_the_acquire_request_has_no_epoch_field_left_to_misuse`（结构级）、
`test_a_grant_is_scoped_to_the_epoch_it_was_stamped_with`。

## D-P4-11 — UNKNOWN 可以被"持续复核"，证据与启动扫描同一套

**背景**：许可过期 ⇒ 资源降到 UNKNOWN（铁律二）⇒ `ConfirmClear` 在 UNKNOWN 上拒绝（设计如此）
⇒ 启动扫描只跑一次 ⇒ **没有任何路径能把它抬回 FREE**。一次失败的穿越永久锁死整条通道
（实测：case 2 开局就是 `mid: UNKNOWN, mid_left: UNKNOWN`，r01 直接被拒）。

**决定**：新增 `CrossingManager.verified_clear(poses, sizes, only=...)` 作为**唯一**的
"已核验净空"实现，启动扫描和持续复核都调它；复核走独立定时器（0.5s）+ `/fleet/reverify` 服务。

放行一个资源当且仅当三条同时成立：

1. **无人持有**（活许可即"有人被授权在里面"，位姿不足以排除占用）；
2. **每台已配置的车都给出位姿**（缺一个，或为 `None` ⇒ `unlocatable` ⇒ 一个都不放行）；
3. **没有任何车的车体与矩形重叠**。

**理由**：证据不是车的自述，是协调者自己从里程计读到的位姿 —— 与启动扫描同强度。
另外，**一条安全规则的两份实现早晚会给出两个答案**，所以合并成一份。

**代价 / 边界**：状态是**逐矩形**的，只有**授权**是逐捆绑原子的。
所以复核可能把 `mid_right` 单独放行而 `mid` 仍 UNKNOWN（那时确实没人在 `mid_right`）。
这**无害**：`acquire` 仍要求捆绑内每个成员都 FREE，孤立的 FREE 成员拿不到任何授权。

**顺带修**：`mark_verified_free()` 过去只改协调者这侧的状态，**没清租约账本自己的 `_unknown` 标记**，
而 `ResourceBook.acquire()` 查的是后者 ⇒ **复核会报"成功"而什么都没变**。
同一件事两个答案。已改为同时 `book.confirm_clear(proof=source)`。
启动扫描从未暴露它，因为它跑的时候还没有东西被标成 unknown。
测试：`test_marking_a_resource_verified_free_also_clears_the_lease_book`。

## D-P4-12 — 通道内过期后的"退避许可"：提议，**未实现**

**问题**：门禁拒绝"驶入我没有许可的矩形"，且**不区分驶入与驶出**。
许可在通道内过期的车因此被冻在原地，**也无法重新申请**（`acquire` 要求站在合法问询点且全区净空）。
复核也救不了 —— 它确实在里面，`mid` 只能保持 UNKNOWN。

**提议**：一份**退避许可** —— 已在矩形内的车可以移动，**当且仅当**指令运动量
**严格减小**它与保护区的重叠。需要定义"重叠减少"的度量，并覆盖
"退避与前进是同一个指令"的边界情况。

**为什么不现在做**：这是对**安全层**的改动，不是补一个分支。需要独立的设计、测试与实跑。
宁可写明"策略层之外还有这条路没走"，也不临时塞一个没验过的逃生口。
**场景标 NOT_RUN**；测试 `test_a_robot_that_lapses_inside_the_bundle_has_no_legal_way_out`
把**当前行为**钉住，别人"顺手改进"门禁时会先撞上它。

### D-P4-12 补记：现行行为是**解释**，不是合同要求（2026-09-16）

合同的边界条款只说"封锁资源，等待清障/可信恢复"，并禁止从心跳丢失推导"机器人已停止"。
**合同里没有"退避许可"这个机制** —— 那是我们这一侧的读法。所以：

- 现在的失败行为（**封锁整条资源 + 把任务标 NEEDS_ATTENTION**）**可以**说成是对合同
  边界条款的保守读法：资源不交给未经核验的一方，任务不假装还在进行。
- **不可以说**"按合同要求实现了通道内过期处理"，也不可以说"这是合同允许的唯一做法"。
  合同对这个情形没有给出机制，我们**选择了**fail-closed。
- 登记位置：本节 + `docs/LIMITATIONS.md` 第 2 节。报告里引用时按**解释**（interpretation）
  措辞，不写成"实现了合同要求"。
## D-P5-13 — the cancel confirmation budget is 15 s, not the contract's 3 s

**Conflict, recorded rather than resolved.** `CONTRACTS.md` §4 item 6 allows 3 wall seconds
for a cancel to be confirmed, and says that on timeout the gate stops the robot and the task
is marked `CANCEL_UNCONFIRMED`. This stack cannot meet 3 s:
`nav2_adapter_node.CANCEL_SETTLE_S = 10.0`, because after Nav2 stops, the adapter waits for
the measured speed to stay settled for a continuous window before it will call the robot
stopped at all.

**What was NOT done.**

- The budget was not quietly set to 15 s with no record. It is a declared node parameter
  (`cancel_confirm_s`), it is documented here and in `docs/LIMITATIONS.md`, and the deviation
  is asserted by a test.
- The adapter's settle window was **not** shortened to make the contract's number reachable.
  That would make `stopped_confirmed` easier to obtain, which is a safety-relevant relaxation
  — the one thing the contract's §4 item 3 exists to prevent.
- The contract number was **not** adopted literally. A 3 s budget would mark *every* cancel
  `CANCEL_UNCONFIRMED`, which satisfies the letter of the rule and inverts its intent.

**Resolution.** `cancel_confirm_s` defaults to 15 s (the adapter's own settle window plus a
margin). `tests/test_p5_custody.py` asserts the budget stays larger than the settle window,
so restoring 3 s by accident turns a test red.

**Provenance.** Found by reading the numbers of the 08:26Z run, which passed: `p5-D` reached
`CANCELED`, but in **306 s**, against 25 s and 153 s in earlier runs. The variance was the
missing feature — `_srv_cancel` recorded the request in the ledger and told nobody, so the
leg ran to its own natural end (aborts and retries included) before the dispatcher could
finalise. A cancel that is recorded but never transmitted is not a cancel.

---

## D-P5-14 — one robot, one cargo: a loaded robot is not eligible for a new pickup

`CONTRACTS.md` §5 forbids handing a `payload_id` to another robot "as if it moved". It does
not say whether a single robot may hold two logical cargos, because in a logical-payload
model there is no mechanical reason why not.

**Observed.** In the 07:57Z run, `r02` was parked by a fault while carrying `cargo_C`, was
re-allocated, picked `cargo_D`, and ended up holding **both**. Every ledger row was
individually consistent, the status output looked plausible, and `cargo_C` was never
delivered. Nothing anywhere reported a problem.

**Chosen reading, and why it is the conservative one.** A robot holding an undelivered cargo
is not eligible for a task that would load a second one. The alternative reading —
"logical payloads, so stacking is fine" — is the one that lets the run pass, which is exactly
the interpretation the handoff's boundary clause warns against choosing. The refusal is
counted under `PAYLOAD_HELD_ELSEWHERE`, and the state is visible in the status output as
`cargo_C HELD holder=r02`.

**Knock-on, and why it is correct.** This makes a faulted, loaded robot unavailable for the
rest of the session until a human resolves it. That is the intended consequence: the contract
says such a task is marked `NEEDS_ATTENTION` and its payload stays with its holder. There is
no automatic path back, by design, and `D-P4-12` (a retreat permit for a robot stopped inside
the corridor) is the related unimplemented half.

---

## D-P5-15 — `PAYLOAD_HELD_NEEDS_ATTENTION` is emitted, and a park reason is never a constant

`CONTRACTS.md` §12 lists `PAYLOAD_HELD_NEEDS_ATTENTION` as a reason code and nothing emitted
it. Meanwhile `_park` wrote `CANCEL_NOT_CONFIRMED` for **every** caller: a navigation timeout,
a blocked protected region and a charging timeout all produced the same machine-readable
reason, with the real explanation only in a log line.

This is the same shape that cost 008 a week — a `fail_reason` column that was a constant,
which made the whole failure report untrustworthy. `_park` now takes the reason from its
caller, and a task parked while its robot still holds cargo records
`PAYLOAD_HELD_NEEDS_ATTENTION`, which says "the work stopped and the goods are still on the
robot" — a different statement from "somebody else has it", and the difference decides
whether an operator looks for a missing cargo or a stuck robot.

A test asserts the constant is gone from `_park` and that every call site supplies a reason,
so this cannot come back as a cosmetic refactor.

---

## D-P5-16 — `INTERRUPTED_BY_RESTART`: a parked task says why it was parked

**Found while writing the restart scenario, not while writing the code.**
`_reconcile_at_boot` moves every task left open by a dead task service to
`NEEDS_ATTENTION`, and recorded `ReasonCode.CANCEL_NOT_CONFIRMED` for all of them. Nothing
had been cancelled. `_park` had already been fixed for exactly this complaint (D-P5-15);
the reconcile path had the same defect and nobody looked, because it only runs at boot.

**Decision.** Add `ReasonCode.INTERRUPTED_BY_RESTART` and use it there.

**Why not reuse an existing code.** `CANCEL_UNCONFIRMED` is also wrong (there was no
cancel). `PERSISTENCE_FAILURE` is for a corrupt ledger or a failed write, which is a
different event with a different operator response — a restart is routine, a corrupt
database is not. `NOT_ASSIGNED` describes a task nobody took, which is the opposite of
what happened. The project has already recorded the principle once, in the
`ORPHANED_ASSIGNMENT` comment: *a code that is merely "close enough" is worse than a new
one*, because 008 lost a week to a `fail_reason` column that was a constant.

**Provenance of the code name.** `CONTRACTS.md` §12 says "至少支持这些 reason_code" — at
least support these. So this is an addition *within* the contract, not a deviation from
it, and it is registered here rather than presented as one of the contract's own names.

**Pinned by.** `tests/test_p5_restart_labels.py` reads the AST of `_reconcile_at_boot` and
requires `INTERRUPTED_BY_RESTART`, requires `CANCEL_NOT_CONFIRMED` to be absent from it,
and requires the cancel-deadline path to still carry a cancel label — so the two causes
cannot swap instead of being fixed.

---

## D-P5-17 — a scenario's declared budgets are consumed, and the guard checks both ways

**The problem.** `CONTRACTS.md` §9 lets a scenario declare its own budgets up front and
forbids adding them at runtime. It does not say the launch has to honour them, and it did
not: `fleet.launch.py` forwarded only `tick_hz` and `pose_timeout_s`, so a scenario naming
`leg_timeout_s`, `cancel_confirm_s`, `max_retries` or a starting state of charge changed
nothing at all — while the run's own report would have said those budgets were in force.
That is the failure mode this project is most prone to: a value believed to be connected
and not connected.

**Decision.**

1. `fleet.launch.py` gains `scenario:=<name|path>`. It reads `config/scenarios/<name>.yaml`,
   applies the `budgets:` block to the nodes it names, applies the optional per-robot
   `robots:` block, and **prints the effective value of every key** so a report can quote
   what was actually in force.
2. An unknown section or key is a hard error, not a shrug. A budget that is silently
   ignored makes the run claim numbers that were never applied.
3. `scripts/check_scenario_budgets.py` is wired into `build.sh`. It reads
   `SCENARIO_NODES` out of the launch file and checks BOTH directions: a key a scenario
   names that the launch does not forward fails, and a key the launch forwards that the
   node never `declare_parameter`s fails. It also refuses NaN, Infinity and non-positive
   durations.
4. The table's first element is the **executable**, not the node name. Writing the node
   name (`fleet_coordinator` instead of `coordinator_node`) made the guard report that
   three executables did not exist — a guard failing on its own vocabulary, reported as a
   project fault.

**Falsifiability.** `_p009/p5b/controls.sh` runs seven controls against the guard: five
bad scenario files, one broken launch table, and a restore. All seven behave. A guard that
cannot fail is not a guard, and this project has already shipped two of those.

---

## D-P5-18 — the battery override injects a STATE, and is gated by its own flag

**Why it was needed.** The simulated pack spends 0.5 Wh/m against a 100 Wh capacity, so
reaching the 20% low threshold naturally needs about 160 m of driving — roughly thirty legs.
Without a way to set the state of charge, the low-battery and charge-queue scenarios cannot
be run at all.

**Decision.** `FaultInject.srv` gains `bool inject_battery` and `float64 battery_fraction`.

**Why a separate boolean rather than "a negative fraction means unset".** A sentinel inside
a numeric field is a second meaning for a number, and the failure mode is asymmetric and
severe: a caller that forgets to set the field would **empty the pack**, which stops the
robot and parks its task. State of charge 0.0 is a legitimate state; "not requested" is not
a state of charge at all. `inject_battery` defaults to `false`, so every existing caller of
`inject_fault` — the CLI included — is unaffected.

**What this is not.** It is not a bypass of the policy under test. The low/critical bands,
the allocator's finishability test and the charge admission all read the same `BatteryModel`
afterwards; only the starting charge changed. A scenario that injects 15% exercises the
shipped admission rules, not a special case written for the test.

**Undo.** `duration_s` keeps its ordinary meaning here: `> 0` restores the previous charge
after that many wall seconds, `<= 0` lasts for the session. The restore is done by the
adapter's own timer and is logged, because a state that changes by itself with no line in
the log is indistinguishable from a bug.

**Pinned by.** `tests/test_p5_scenarios.py` reads the adapter's AST and requires the early
`return` on `inject_battery` to be the first statement of `_apply_battery_injection`, so
removing the gate is a test failure rather than a silent behaviour change.

---

## D-P5-19 — the ledger must persist the task KIND, not default it on read

**Found by running the low-battery scenario, and it is the most serious defect of this
round because nothing failed.**

`tasks` had no `kind` column, and `_row_to_task` rebuilt a `TaskSpec` from the row
without one, so `kind` fell back to `STATION_TRANSFER`. The dispatcher decides
`charging_task = task.spec.kind.value == "return_to_charge"` from exactly that object,
and `_eligible_robots` uses it to decide whether the low-battery admission rule applies.
The observed sequence:

1. `_create_charge_task` created `auto-charge-r01-1` and logged
   `r01 low (15.0 Wh) -> auto-charge-r01-1 to C_left`. The C_left pad was granted to r01.
2. On the next tick `_assign_ready` asked `_eligible_robots` about that task, which now
   looked like an ordinary transfer, so the low-battery rule applied **to the charge
   task itself** and refused r01 with `INSUFFICIENT_BATTERY`.
3. That happened once per tick for the rest of the run: the counter went 182 -> 275 over
   46 s at 2 Hz, which is one per tick.
4. r01 never moved. It stayed at its spawn with `operating_state=IDLE` and an empty
   `task_id`, while the status output said `chargers.occupant = {C_left: r01}` and
   `grants = 1`.

So the fleet *looked* like it was charging — a pad held, a counter moving, a plausible
page — and nothing was. P5's requirement that a low-battery robot returns to a pad had
**never actually been exercised**, in this round or in the earlier "P5 end-to-end" run.

**Second defect in the same place.** `TaskSpec.fingerprint()` includes `kind`, so the
stored fingerprint and the reconstructed spec's fingerprint disagreed: an idempotent
replay of a charge task would have raised `ConflictError` against the row's own
fingerprint. `payload_mode` had the identical problem and is persisted too.

**Decision.**

1. `tasks` gains `kind` and `payload_mode`, written at insert and read back in
   `_row_to_task`.
2. `Ledger._migrate()` adds missing columns to an existing file rather than refusing to
   open it. CONTRACTS section 11: clearing the database is not recovery. It records what
   it added in `meta.migrated_columns`.
3. The migration cannot recover what was never written. Rows predating it are relabelled
   `station_transfer`, which is **wrong for a charge row**, and that is stated rather
   than hidden — see `docs/LIMITATIONS.md`.

**Why no unit test caught it.** `Ledger.submit` returns the in-memory `Task` it just
built, so every test that submits and inspects the return value sees the right `kind`.
The field only disappears on the way *back out* of the database, and the only callers
that read back are the dispatcher's `open_tasks()` loop and the status snapshot.
`tests/test_p5_ledger_kind.py` now asserts the round trip, the fingerprint round trip,
and the in-place migration.

---

## D-P5-20 — a refusal that names the wrong cause: `NO_CAPABLE_ROBOT` for a busy robot

**Observed live, registered, then fixed as its own change.** The fix is at the end of this
entry. Keeping the observation verbatim, because it is the reason the fix is shaped the way
it is: while the only INSPECT-capable robot was executing another task, `Allocator.allocate`
returned `NO_CAPABLE_ROBOT` and the task service counted ten of them. No robot was
missing a capability; the capable robot was `RESOURCE_BUSY`.

`allocate()` picks its single reason by scanning `rejected` for `INSUFFICIENT_BATTERY`
and otherwise falling back to `NO_CAPABLE_ROBOT`. `RESOURCE_BUSY` is collected in
`rejected` and then never surfaced.

**Why it was not changed in this round.** It is an allocator semantics change with
existing tests on the current behaviour, and it is a *reporting* defect rather than a
safety one: the task is still not dispatched, which is the correct outcome. Changing it
mid-round would have invalidated the live scenario evidence already collected.

**The standing rule it belongs to.** D-P5-15 and D-P5-16 both say a label that is merely
close enough is worse than a new one, because it is the label an operator reads. This
one is in that family and was registered as unfinished: an operator seeing
`NO_CAPABLE_ROBOT` will look for a missing capability that exists.

**Fixed.** Three changes, and the third is the one that keeps it fixed:

1. **The precedence is declared, not implied.** `allocator.REASON_PRECEDENCE` orders the
   refusals most-actionable first — `RESOURCE_BUSY`, `INSUFFICIENT_BATTERY`,
   `RESOURCE_UNKNOWN`, `NO_CAPABLE_ROBOT` — and `reported_reason(rejected)` walks it.
   `NO_CAPABLE_ROBOT` is last because it is the only one that needs a human to change
   something; the old if-chain happened to give `INSUFFICIENT_BATTERY` priority and
   everything else fell through to it.
2. **The evidence rides on the result.** `Allocation` carries
   `rejected: tuple[(robot_id, ReasonCode)]` and a human-readable `detail`, populated on
   success as well as on failure: "who was considered and why not" is what makes a choice
   auditable, and an auditor cannot reconstruct it from the winner. An offline robot is now
   rejected as `RESOURCE_UNKNOWN` rather than `NO_CAPABLE_ROBOT` — it has the capability,
   not a link, and this is the same defect through a different input.
3. **The dispatcher writes them down.** `_note_no_allocation` emits a `no_allocation` event
   carrying the full rejected list, **once per change of reason** rather than once per tick:
   at 2 Hz a per-tick event would write the same line twice a second for as long as the task
   waits and bury the one line that matters. The task appears in the snapshot's
   `awaiting_allocation` list meanwhile.

**Tests.** `tests/test_p1_allocator_reason.py` (11 tests): the exact case above; a busy
robot outranking an incapable one; an offline robot not reported as incapable; the battery
case that already worked, kept as a regression; the rejected list surviving on success; the
precedence pinned as a declaration with `NO_CAPABLE_ROBOT` last; and two structural checks
that `allocate` reports through `reported_reason` and that the dispatcher actually calls
`_note_no_allocation`.

---

## D-P5-21 — a charge run that cannot be created must not keep the pad

**Found by reading the evidence of a PASSING run.** The charge-queue scenario reported
9/9, and its own dump said:

```
chargers: occupant={'C_left': 'r01', 'C_right': 'r02'}  queue=[r03]  grants=2  releases=0
r01 / r02 / r03: op=IDLE, at their spawns
task auto-charge-r01-1: FAILED          task auto-charge-r01-4: NEEDS_ATTENTION
                                        (INTERRUPTED_BY_RESTART)
```

Two pads held, nobody going anywhere, and the charge tasks that were supposed to consume
them belonged to a previous session. The mechanism:

1. `_create_charge_task` built `request_id = f"auto-charge-{rid}-{serial}"` from a serial
   that restarts at 1 in every process, and `request_id` is the ledger's idempotency key.
2. The ledger is durable on purpose, so a row with that id from an earlier run made
   `ledger.submit` return `(existing, created=False)`.
3. The code did `if not created: return` — **after** `chargers.request()` had already
   granted the pad. The pad then stayed granted for the session, owned by a robot that had
   no task to release it.
4. Nothing reported it. The status page showed a plausible `occupant`.

**Fix, both halves, because either alone leaves the defect.**

* The generated id includes the task service's epoch, so a new generation cannot collide
  with a dead row. `charge_request_id()` lives in `fleet_core.charging` so it is testable
  without ROS.
* If `created` is still False, the pad is **released** and `charge_not_created` is emitted.
  A granted resource must have an owner that will release it; a silent `return` between
  grant and release is the leak, and the leak is the harm.

**Why the scenario passed anyway, and why that is a finding about the instrument.**
`E6` asserted "a pad was actually occupied" from `chargers.grants` alone. A counter that
says a pad was admitted is not a robot that arrived. The scenario now also asserts that a
charge task reached `EXECUTING`/`SUCCEEDED` and that a robot was observed **inside the
pad's radius**. Both are `PASS` in the run that follows this fix; neither existed before,
so the earlier 9/9 was true about queueing and silent about charging.

**The generalisation, which is now the third instance.** A durable store plus an identity
that is unique only within one process produces a silent no-op:
`kind` (D-P5-19), `request_id` (here), and the park reason (D-P5-16) are the same shape
three times. The rule that would have caught all three: **an identity that is written to
a durable store must be unique for the lifetime of that store, not for the lifetime of
the process that generated it.**

---

## D-P5-22 — the charger allocator ignores where the robot is, and a pinned task makes
## that permanent

**Registered as a diagnosis, then fixed in the same round.** The diagnosis below is what
made the fix small: it named the decision (which pad), the input the decision was missing
(the robot's position) and the mechanism that made the mistake permanent (pinning). See the
closing section for what changed and the run that shows it.

**Diagnosis, from evidence rather than from instrumenting the dispatcher.**

1. The failing run reported `ROUTE_REQUIRES_PERMIT: 2087` and both charge tasks stuck at
   `ACCEPTED` with `robot=''`, while `chargers` showed `occupant={C_left: r02,
   C_right: r03}`, `grants=2`, `releases=0`, and every robot still at its spawn.
2. `grep -rn ROUTE_REQUIRES_PERMIT src/` shows the task service has exactly **one**
   `_refuse(ReasonCode.ROUTE_REQUIRES_PERMIT)` — `task_service_node.py:1114`, inside
   `_eligible_robots`. (The other two hits in the tree are `coordinator_node.py:494`, a
   `msg.reason_code` default, and `traffic.py:313`, a `PermitCheck.reason`.) So the counter
   identifies the branch on its own: no probe was needed to find it.
3. That branch calls `approach_direction` → `corridor_direction(from_x, to_x, tcfg)`, which
   returns a direction **iff the sign of x changes** — the barrier runs along x = 0.
4. The run's own log says which pad each robot was sent to:

   ```
   r02 low (13.0 Wh) -> auto-charge-r02-<epoch>-1 to C_left
   r03 low (11.0 Wh) -> auto-charge-r03-<epoch>-2 to C_right
   ```

   `r02` spawns at (6.0, −2.0) — east. `C_left` is (−5.0, −3.8) — west.
   `corridor_direction(6.0, −5.0)` = `east_to_west` ≠ `None` ⇒ refused. `r03` spawns at
   (−6.0, 0.6) — west, sent to `C_right` (5.0, −3.8) — east ⇒ refused the same way.

**Root cause.** `ChargerAllocator.request(rid, now)` with `preferred=None` builds its
candidate order from `sorted(cfg.chargers)`, which is always `["C_left", "C_right"]`. **It
never reads the robot's position.** An east-side robot is therefore routinely granted the
west pad — and because the resulting charge task is *pinned*, the dispatcher will never
choose a different robot for it, so the route refusal is permanent, the task never leaves
`ACCEPTED`, and the pad is never released.

**This also explains why scenario D passed its charging half and E could not.** D's single
low robot is `r01` at (−6.0, −2.0), west, and the allocator sent it to `C_left` — west.
`corridor_direction(−6.0, −5.0)` = `None`, the gate passes, and the charge run executed.
The difference between the two scenarios was never the charging logic; it was which side of
the barrier the first pad in sorted order happens to be on.

**The family it belongs to.** Same shape as D-P5-21: a resource is granted to an owner that
can never release it. That is now the fifth instance, and the first caused by geometry
rather than by an identity.

**Intended fix.** Pad selection must be reachability-aware: prefer a charger on the
robot's own side of the barrier, because the alternative requires a corridor permit that a
system-generated charge run has no way to obtain. `NO_SAFE_CHARGER` already exists and is
currently reachable only from the CRITICAL branch; "no pad I can legally reach" is the
honest answer there. Independently, a pinned task its own robot is refused for should be
surfaced rather than looping — a pad held for a session by a robot that can never use it
must not be reportable as `occupant` without comment.

**Fixed.** The split is *who owns the policy and who owns the geometry*:

1. `legs.reachable_chargers(cfg, tcfg, from_x)` answers "which pads can a robot standing here
   actually be dispatched to". It calls the **same** `corridor_direction` the dispatcher's
   gate calls, deliberately: two implementations of "can this robot get there" would
   eventually disagree, and the disagreement would surface as a task the allocator thinks is
   dispatchable and the gate refuses. A test asserts the two agree over a grid of positions
   for exactly that reason. A pad with no pose anywhere is excluded too — it cannot produce a
   goal, so granting it would reserve a pad and then refuse the whole task with
   `UNKNOWN_STATION`.
2. `ChargerAllocator.request(..., reachable=...)`: an unreachable pad is not "busy", it is not
   a candidate. When nothing is a candidate the answer is `NO_SAFE_CHARGER` and **no queue
   entry** — a robot with no usable pad must not hold a place at pads it cannot drive to. A
   stated `preferred` that is not admissible is ignored: the preference is a wish, the
   reachable set is a fact.
3. `_watch_unassignable`: a pinned run whose own robot has been ineligible for
   `pinned_grace_s` is parked with **the refusal that blocked it** (never a constant) and
   **releases the pad**. Only `PERMANENT_REFUSALS` are parked — a stale pose, a busy robot or
   a battery rule is a wait, and `ACCEPTED` means "waiting"; parking a wait would turn a
   recoverable condition into an operator call-out.
4. `_create_charge_task` asks for a pad only once the robot reports a position. Without one
   the pad is chosen on a guess, and a guess that lands on the far side is the bug again.

**The regression this fix could have introduced, and the test for it.** Queue matching used
to be `q.preferred in (None, charger_id)`, so a robot queued with no preference matched
**every** pad. Adding a reachable set while leaving that predicate alone would have put a
west-side robot at the head of the east pad's queue and blocked the only robot that could
use it — a deadlock created by fixing a different bug. `_Queued` now carries its own
`reachable` set and `_admits` checks both halves; `test_a_queued_robot_does_not_block_a_pad_it_cannot_reach`
is the regression test.

**Tests.** `tests/test_p5_reachable_pads.py` (14 tests), including the four-line offline
diagnosis of the original bug, the allocator/gate agreement grid, the queue regression, and
two structural checks that `_create_charge_task` passes `reachable=` and that `_tick` calls
the watchdog at all.

---

## D-P5-23 — a charge run can be created before the robot's Nav2 is active, and Nav2's
## refusals spend the run's attempts

**Observed live, registered, then fixed.** The design is at the end of this entry; the
observation is kept verbatim because it is what the fix had to answer.

Both charging scenarios run end to end, and both show the same prelude in their own logs.
Scenario D, `reports/scen3456_low_battery.log`:

```
[bt_navigator-14] [r01.bt_navigator]: Action server is inactive. Rejecting the goal.
[task_service_node-47] auto-charge-r01-<epoch>-1: leg to_charger-0 failed
                       (NAV_FAILED: 2 attempts rejected); attempt 1 of 2
...
[bt_navigator-14] [r01.bt_navigator]: Action server is inactive. Rejecting the goal.
[task_service_node-47] auto-charge-r01-<epoch>-2: leg to_charger-0 failed
                       (NAV_FAILED: 2 attempts rejected); attempt 1 of 2
```

`auto-charge-...-1` ends `FAILED` with `attempts=2`; `-2` then succeeds and the robot
charges from 0.1363 to 0.8062. Scenario E shows the same shape.

**Root cause.** `_watch_battery` runs from the task service's first tick, so a low robot gets
a charge run while `bt_navigator` is still transitioning to ACTIVE (measured: 13–16 s after
the state topic). The goal reaches Nav2, Nav2 correctly rejects it, and the rejection
consumes one of the run's two attempts — a rule that is deliberate
(`_poll_pending` refuses to re-issue silently, or the dispatcher spins on the same refusal).
Two rejections kill the run. The dispatcher then creates a fresh charge run, which succeeds.

**Why this is registered rather than patched.** The correct fix is a readiness input the
dispatcher does not currently receive: `CoreRobotState.nav_state` exists but does not carry
Nav2's lifecycle state, so the dispatcher cannot tell "not up yet" from "up and failing". The
alternative — not counting an immediately-rejected goal as an attempt — undoes a decision
that was made on purpose, and doing either late in a round would invalidate the live evidence
just collected.

**What it costs today.** A charge run is recorded `FAILED` in a healthy run, which is a wrong
entry in the task statistics; two attempts are spent per boot; and the pad accounting shows
`grants=2` for one robot's single charge. It cannot cause a permanent stall, because a
terminal charge run lets `_has_charge_task` return false and a new one is created.

**The family, now three deep.** "Not yet up" is treated as a fault in three places: the
dispatcher's own eligibility gate (§3.3 of the status document, fixed by requiring
`position_known`), the adapter's pose check, and Nav2's lifecycle (this entry). The first was
found by a live run, the second by a live run, the third by reading the log of a run that
passed.

**Fixed.** The split is *who can tell* and *who acts on it*:

1. **The adapter asks instead of assuming.** `navigate_to_pose`'s action server exists while
   its node is still INACTIVE, so `wait_for_server()` returns true and `send_goal_async()`
   resolves with `accepted=False` -- from the client, "not up yet" and "busy" are the same
   event. The adapter now queries `bt_navigator/get_state`, waits (bounded by
   `nav2_ready_timeout_s`, 30 s ≈ twice the measured bring-up) for ACTIVE **before** the
   retry loop starts, and gives an attempt back when a refusal is explained by readiness.
2. **"Cannot tell" is not "not ready".** `fleet_adapter.nav2_readiness` decides when to ask
   and what an answer is worth, with no ROS import, so it is testable with a fake clock.
   `should_wait_for_recovery(None) is False` is the rule that keeps the change safe: an
   unreachable lifecycle service leaves the previous behaviour exactly as it was. A readiness
   gate that blocks when it cannot see would be a new failure mode rather than a fix.
3. **The probe never waits on its own future.** `_execute` runs inside an action callback on
   a MultiThreadedExecutor, and the service response is delivered by that same executor --
   so the probe is two-phase (issue, harvest later) and takes a lock, because three threads
   call it and two concurrent `call_async` would leave neither answer harvestable. This is
   the P6 deadlock one layer down, and the adapter's own docstring already forbade it.
4. **The dispatcher stops spending attempts on a wait.** `NAV2_NOT_ACTIVE` is a new reason
   code (an addition inside CONTRACTS §12's "至少支持这些", like D-P5-15's
   `INTERRUPTED_BY_RESTART`), `WAIT_NOT_A_FAILURE` names the class in one readable place, and
   `_handle_leg_result` re-asks without incrementing `attempts` -- **bounded** by
   `wait_budget_per_task`, because an unbounded "not yet" is a task that never resolves,
   which is the spin `_poll_pending` deliberately refuses to do. `run.waits` counts the
   deferrals, so a wait that happened is diagnosable rather than invisible.
5. **`nav_state` now says which case it was.** The field used to report only whether the
   action *client* existed -- true while the server's node was INACTIVE, which is the one
   case the field exists to describe ("Nav2 is fine but the gate is refusing" vs "Nav2 gave
   up"). Both the state message and the leg result now carry
   `NAV2_NOT_ACTIVE` / `NO_NAV2_ACTION_SERVER` / `""`.

**A guard caught a real rename.** The first version stored the clock as `self._clock`, and
`check_undefined_names.py` refused it: `rclpy.node.Node` owns `_clock`, so a class in a
deliberately ROS-free package must not use that name. The public parameter stayed `clock`;
the attribute became `_now_fn`. That is the same guard that caught `self.clients` after it
had already cost two three-robot runs, doing its job before the fact this time.

**Tests.** `tests/test_p5_nav2_readiness.py` (19 tests). The pure half pins the rules that
matter: `None` is not `False`; an answer is reused until the TTL expires; a *failed* ask
becomes "unknown" rather than "inactive"; an ask in flight erases nothing; ACTIVE is decided
by state id (3) rather than by truthiness. The structural half pins the wiring, because
`fleet_ros` imports rclpy and cannot be imported by a test: readiness is asked *before* the
goal is sent, an attempt can be given back, the probe contains no spin/sleep/wait_for, the
wait is bounded and cancellable, `nav_state` no longer reports the client's existence, and
the dispatcher decides the wait **before** counting an attempt.

**Live verification, both charging scenarios re-run after the fix.**
`reports/scen3456/low_battery_20260916T135059Z.txt` (**9/9**): the ledger holds **one** charge
task, `auto-charge-r01-<epoch>-1`, `attempts=0`, `SUCCEEDED`; r01 charges 0.1376 → 0.8045 and
ends 0.26 m from `C_left`; `chargers: grants=1 releases=1`. Before the fix the same scenario
held `-1 FAILED attempts=2` **and** `-2 SUCCEEDED`, with `grants=2 releases=2` for one robot's
single charge. The boot refusal counters also lost their `LOCALIZATION_STALE` flood
(`{"ROUTE_REQUIRES_PERMIT": 1}` alone).

`reports/scen3456/charge_queue_20260916T135208Z.txt` (**11/11**): `auto-charge-r01-<epoch>-1`
and `auto-charge-r02-<epoch>-2`, both `EXECUTING attempts=0 reason=OK`, each heading for the
pad on its own side; no `FAILED` charge task anywhere in the ledger. Before the fix the same
scenario had three failed charge runs (`-1`, `-2`, `-4`) before `-5` executed.

**What this does and does not establish.** It establishes that a bring-up race no longer
costs a charge run or an attempt: the run that used to die is now the run that succeeds. It
does **not** establish anything about readiness on a machine where the lifecycle service is
unreachable -- `None` still means "carry on as before", and no run here has removed that
service. And the queue/capacity assertions rest on 2 occupancy samples in this run because the
instrument stops as soon as a robot reaches a pad, which now happens sooner; the
165-sample run `charge_queue_20260916T122243Z.txt` remains the stronger capacity evidence.

## D-P8-03 — the latch is geometric, judged from a pose 1.87 m from truth

**Follows `D-P8-02`.** That entry established the latch and the code path that makes it permanent.
This one is the geometry, because the gate now publishes the pose it judged from.

**The latch, with the pose.** `reports/batch_20260917T092455Z/N01`:

    t=94.4  mode=NORMAL  pose=[3.4917, -0.2515, -1.7905]  cmd=(0.35, -0.0788)
    t=94.5  mode=STOP    pose=[3.4878, -0.2685, -1.7943]  cmd=(0.0, 0.0)
            detail: speed 0.380 m/s -> stopping envelope 0.256 m + localization margin 0.100 m,
                    then footprint margin 0.150 m;
                    unpermitted ['mid_right'], holdings none  (reserve 0.356 m)

The refusal is not spurious *in the gate's own frame*: `mid_right` is `x[2.15,2.85] y[-1.75,-1.05]`,
expanded by 0.356 + 0.150, and a footprint at `(3.488, -0.269)` with yaw -1.79 rad can reach it.
The named resource, the reserve and the margins all come out of the gate's own sentence, so this is
a measurement and not a reconstruction.

**The two criteria are not the same criterion.**

    gate refusal      _unpermitted_hits   rect + reserve(speed) + footprint_margin
    clearance proof   confirm_clear       rect +                  footprint_margin

`reserve` is speed-dependent -- 0.356 m at 0.38 m/s, 0.100 m at rest. So the passage driver's
"cleared" is strictly weaker than the gate's "clear", and the difference is exactly the term the
gate prints. Measured: the driver's `confirm_clear` stage reported success in the run whose gate
began refusing at that instant. **This part is a defect regardless of which fix is chosen**: two
components that must agree about one boundary compute it from two different expressions.

**And the pose it judged from was 1.87 m from truth.** At the end of the run the gate still
believed `(3.389, -0.656)`; the simulator's truth for r01 was `(5.223, -1.024)`. The gap is
**1.87 m**, against round 5's p95 |claim - truth| = **1.8588 m** on a different case. Round 5
recorded that as consistent with `map -> odom` being nearly constant, i.e. that AMCL barely
corrects. If that is what it is, then the gate refused on a pose whose error is about the width of
the boundary it was testing.

**The decision: register, measure the clock offset, then choose.** Two candidate fixes with
different safety properties:

    (a) frame / odometry-scoped: if the 1.87 m is a near-constant offset, this is a calibration and
        provenance question -- what may a gate conclude from odometry it knows is offset?
    (b) retreat allowance: if the error is pose-specific, a robot inside a boundary must be allowed
        to move AWAY from it. That is the same thing `D-P4-12` names for a robot stopped inside the
        corridor ("the related unimplemented half"), generalised to the exit buffer's boundary.

**The measurement was then made, in the same round and without a new run.** The recorder logs its
own start on absolute time (`recorder-stdout.log`) and `runtime/events.jsonl` carries absolute
timestamps, so the crossing's harvest locates the latch on the recorder's axis -- the same trick
round 7 used. At the latch:

    TRUE  (simulator world)  4.403, -1.490
    GATE  (its map frame)    3.488, -0.269
    gap                      1.526 m

**And the refusal is geometrically correct in the gate's own frame.** Re-derived by hand from the
gate's published pose and printed margins: footprint 0.60 x 0.45 at yaw -1.7943 gives box corners
(3.639,-0.612) (3.201,-0.510) (3.337,0.075) (3.775,-0.027); `mid_right` expanded by the printed
0.506 m is x[1.644,3.356] y[-2.256,-0.544]; the corner **(3.201,-0.510) lies inside it**, so
`box_overlap` was right to fire. The robot's TRUE pose is 1.553 m from `mid_right` and no part of
its footprint comes within 0.506 m of the expanded rectangle.

**So: right in its frame, wrong in reality, because the pose it judged from was 1.526 m from the
truth.** That is round 5's p95 |claim - truth| = 1.8588 m again, and the documented behaviour of
`map -> odom` (nearly constant; AMCL barely corrects).

**Which decides the order of the fixes.** (a) FIRST: the pose the gate is allowed to conclude from.
`map -> odom` being nearly constant means the 1.5 m is a **constant offset**, so it is measurable
directly -- compute `map -> odom` and `world -> odom` over one run and compare -- rather than
inferable from a boundary argument. (b) SECOND, and still worth having: the retreat allowance, as
defence in depth. A gate that can refuse every motion for ever is a liveness hazard *whatever* its
pose error is, and this run is the proof: 815 s with `mode=NORMAL` occurring zero times.
**A retreat rule chosen BEFORE this measurement would have mitigated a 1.5 m localisation error by
loosening a boundary test, and the register would have recorded it as a fix.**

**What this does and does not establish.** It establishes the refusal's geometry, the named
resource, the reserve, that the two criteria differ structurally, and that the pose behind it was
1.87 m from truth at the end of the run. It does NOT establish that the 1.87 m held at the latch,
because the two clocks are not aligned here -- and the analysis script that tried to read the pose
out of the probe line was itself wrong (it split on the wrong token and reported "no pose in this
line" while the pose was printed on it). The pose above was read by hand, twice, from the log.

**The defect, measured.** In `reports/batch_20260917T084732Z/N01`, with `scripts/probe_gate_state.py`
subscribed:

    t=105.7  mode=NORMAL  reason=OK              cmd=(0.35, -0.0288)  measured=0.35
    t=105.7  mode=STOP    reason=PERMIT_EXPIRED  cmd=(0.0, 0.0)       zero_at=87.2638
    ... 815 s later, still: mode=STOP reason=PERMIT_EXPIRED cmd=(0.0, 0.0)

After the first `PERMIT_EXPIRED`, `mode=NORMAL` occurs **zero** times. The gate keeps publishing a
zeroed command for the rest of the run.

**The code path.** `traffic.py:624` writes `"permit expired without clearance"` into the resource's
block reason; `traffic.py:302` computes `lapsed` partly by testing `"expired" in
self._block_reason.get(n)`; `traffic.py:308` turns a lapsed permit into
`ReasonCode.PERMIT_EXPIRED`, **which is not a refusal** -- see `D-P12-02`: `check_movement` returns
its only `allowed=True` before the mark is consulted, and `lapsed` selects a *label* on a refusal the
geometric test had already decided. The sentence in this paragraph that calls `PERMIT_EXPIRED` "a
refusal" is wrong; the refusal is the overlap, and what never clears is the label. The behavioural
deadlock described below is therefore the absent retreat allowance, not the string. The mark is
cleared only on the
release / confirm-clear / reverify paths. So a permit that lapses without one of those leaves a
mark whose own text keeps re-arming the refusal.

**Why it read as a navigation fault for two rounds.** The crossing is the last thing that holds a
permit. The moment it lapses the robot is stopped for good, and the next thing the dispatcher tries
is the next leg -- which in N01 is the 5 m drive inside the east half. Rounds 6 and 7 investigated
that drive because that is where the failure surfaced. The truth stream did say the robot never
moved (372 s in round 6, 788 s in round 7), which was the clue that the fault was upstream of
Nav2; what was missing was the gate's own verdict, which nothing recorded.

**Why it was invisible.** `gate_node.py` has three paths that end in a zeroed command and only one
of them ever logs, and `_published_zero` latches after the first. The gate's verdict goes to
`<ns>/gate_state` at 20 Hz. `scripts/probe_gate_state.py` (added this round) subscribes to it;
`D-P8-01` (also this round) fixed the topic so the leg executor sees it too.

**The decision: register, not patch.** Three candidate changes are on the table and they are not
equivalent -- (a) clear the block mark once the robot is provably outside every protected region;
(b) do not let a lapsed permit refuse a robot whose stopping boundary touches no unpermitted
resource, which is the same idea as round 6's `split_by_passage` change applied to the gate;
(c) treat the mark as scoped to the permit's own bundle rather than to the resource for ever. They
have different safety properties. The field that distinguishes them is the gate's `detail` string,
which this round's own probe truncated to 80 characters -- so the honest state is *the mechanism is
known, the instance is not*, and the next run will say which rectangle the gate named. A
safety-layer behaviour change chosen on a guess is the one thing this project has repeatedly
refused to do.

**What this does and does not establish.** It establishes the mechanism, the latch time, the fact
that nothing clears it, and that it explains every N01 failure. It does not establish that fixing
the latch makes N01 pass: the leg after the crossing is also the leg whose start is at the edge of
an exit buffer, so a scoped fix may still refuse it for a legitimate reason -- and that would be a
different finding, not a bug.

**The defect.** The safety gate publishes its verdict as JSON at 20 Hz. Its own default topic is
`gate_state`; `robot.launch.py` does not override it, so it publishes on `/r01/gate_state`.
`nav2_adapter_node` declared its own default as `fleet/gate_state`, and `fleet.launch.py`
confirmed it, so it subscribed to `/r01/fleet/gate_state`. Measured in a live fleet:

    /r01/fleet/gate_state   publishers 0, subscribers 2   (nav2_adapter, gate_state_probe)
    /r01/gate_state         publishers 1 (safety_gate), subscribers 0

**Why this is a defect and not a naming preference.** Four other consumers already use
`gate_state`: `scripts/acceptance_p4.py` and `scripts/probe_odom_truth.py` subscribe to
`/{r}/gate_state`, `scripts/check_fleet_isolation.py` lists `gate_state` in `REQUIRED_TOPICS`, and
`stop_distance_calibrator.py` defaults to it. The adapter was the only dissenter, so the direction
of the fix is measured, not chosen. `acceptance_p4.py` is deliberately NOT touched: the P4 gate
verdict must keep the evidence it was based on.

**The cost, measured rather than inferred.** `_on_gate_state` sets one field, `_gate_reason`, and
its docstring is explicit that nothing there may influence motion. The field is copied into the
published `RobotState.gate_reason`. So the cost is an empty diagnostic field on every
`RobotState` this fleet has ever published -- real, but not a control-path fault, and **not an
explanation of the stalled base**.

**Why it survived four phases.** Each of the five consumers was internally consistent. No tool in
this project can see a subscription with no publisher: DDS does not error on it, `ros2 topic
info` was never run against it, and both `tests/test_p4_gate_wiring.py` and
`scripts/check_guards.sh` check the gate's own payload and the guard list, not the agreement of
topic names across processes. The check added here does the one thing none of them did: it
collects every `gate_state_topic` value in the tree and requires them to be equal.

**What this does and does not establish.** It establishes that the executor now subscribes to the
topic the gate publishes on, and that the tree cannot drift again without failing a test. It does
**not** establish that the gate's verdict explains the stalled base: the probe that records that
verdict was added in the same round and its output is reported separately.

**The defect.** `_poll_crossings` decided whether a finished crossing belonged to a task that was
being stopped by reading `task.state is TaskState.CANCEL_REQUESTED`. `TaskState` has no such
member (SUBMITTED, ACCEPTED, ASSIGNED, EXECUTING, SUCCEEDED, FAILED, CANCELED, NEEDS_ATTENTION,
NOT_STARTED), so the expression raised `AttributeError` whenever it was evaluated. Because
`task.state.terminal` precedes it in an `or` and short-circuits for a finished task, it was
evaluated exactly when a task was still active -- i.e. on the tick that harvests a crossing.

**The cost.** One tick per crossing, counted by `_tick`'s blanket handler with the rest of that
tick's dispatch steps skipped, and `self.crossed[key]` and the `crossing_complete` event never
written. Measured once per run in both round-6 and round-7 N01. The crossing still completed
because the next `_crossing_dir` answer was `None` on its own -- the robot's position had changed
enough that the leg no longer looked like a crossing. That is luck, not design: with a lagging
odometry estimate the same leg would have been crossed a second time.

**The decision.** Ask the question of the thing that holds the answer: `task.cancel_requested`,
the ledger field that CONTRACTS section 4 item 6 leaves true from the request until the
confirmation, while the state stays `EXECUTING`. No new state is invented, and the enum is not
extended -- a state that exists only to be compared against is what a field is for.

**Why two rounds of tests missed it.** `tests/test_p5_crossing_from_tasks.py` asserted
`"CANCEL_REQUESTED" in poll` -- a substring assertion over the source, pinning the spelling of a
name whose only possible run-time effect is to raise. `check_undefined_names.py` cannot see it
either: `TaskState` is defined, and an attribute access on a defined name is not an undefined
name. Both checks are the right shape for the faults they were built for; neither can falsify an
attribute that does not exist.

**The correction.** The assertion now names the predicate. And a check was added for the class
rather than the instance: every `TaskState.<member>` read anywhere in the module must exist on
the enum. That is the general form of the fault, so it is the version worth keeping.

**What this does and does not establish.** It establishes that the harvest no longer throws, and
that no other `TaskState` member is read that does not exist (the new check would fail if one
were). It does not establish that a crossing which finishes under a *cancel* is handled
correctly end to end -- no run has cancelled a task mid-crossing, and the `crossing_reaped` path
that this predicate feeds is still only structurally pinned.

## D-P10-01 — one boundary: the refusal and the clearance proof were two expressions of it

`_unpermitted_hits` (the gate's refusal) grew each rectangle by
`reserve(speed) + footprint_margin`; `confirm_clear` (the clearance proof) grew it by
`footprint_margin` alone. `reserve` is speed-dependent -- 0.323 m at the 0.35 m/s the stop model
was fitted to, 0.100 m at rest -- so the passage driver's "cleared" was strictly weaker than the
gate's "clear", and `ConfirmClear` could return `cleared=True` for a pose the gate refuses in the
same instant. It did: live, the gate published `unpermitted ['mid_right'], holdings none
(reserve 0.356 m)` while the same robot's `confirm_clear` stage had just reported success.

Worse, the release node's placement was derived from the weaker expression:

```
documented derivation   buffer_outer 2.85 + margin 0.15 + half_extent 0.30 + tolerance 0.80 = 4.10
the enforced boundary   buffer_outer 2.85 + reserve 0.323 + margin 0.15 + half_extent 0.30
                        + tolerance 0.80                                                    = 4.423
```

so `release_east`/`release_west` at 4.2 satisfied a boundary that was not the one being enforced.

**Decision.** `CrossingManager.protected_region(names, speed_mps=...)` is the single definition.
`stop_boundary` and `_unpermitted_hits` call it at the CURRENT speed; `confirm_clear` calls
`clearance_region`, which is that same function at the FASTEST speed the stop model was measured
at (`StopModel.max_measured_speed_mps`, derived by the validator from the largest key of
`reported_stop_distance_m`). The release nodes and their `pad_release_*` floor markers moved
4.2 -> 4.5 (the marker moves too because `validate_traffic_geometry.py` requires a node that
names a pad to sit at that pad's centre; the marker has no collision geometry).

**The bound, stated rather than hidden.** `envelope_m` grows with speed, so the containment is
monotone only up to the measured speed: above it the refusal boundary is larger again and
`cleared` stops implying `not refused`. That bound is only respected if the gate's own clamp is
inside it, and the two numbers live in different files (0.35 in `robot.launch.py`, 0.35 as the
largest measurement key). `tests/test_p10_one_boundary.py` ties them, asserts the containment
holds across `[0, max_measured]`, and asserts it **fails** at 0.6 m/s -- so a future edit cannot
quietly widen the claim.

**Nothing was loosened.** The refusal keeps the stopping envelope (CONTRACTS section 8); the
clearance proof got stricter; the nodes moved further from the rectangles.

**What this does not establish.** That the crossing now completes. With the pose error of
D-P10-02 still present, the stricter proof turns a silent 830-second freeze into a loud
`CLEARANCE_NOT_PROVEN` at the last stage, which is better evidence and worse news.

## D-P10-02 — the 1.5 m is accumulated odometry drift, and AMCL is already at 0.11 m

One instrumented run, one process, one clock: the gate's published pose, the raw odometry, AMCL's
belief and the simulator's truth sampled together at 2 s
(`reports/batch_20260917T095811Z/N01/pose_track.jsonl`, the probe's own rows).

| measured | value |
|---|---|
| `\|gate - (spawn (+) odom)\|` | mean **0.12 mm**, p95 0.10 mm, max 17.50 mm |
| `err = f(distance travelled)` | **`err = 0.0809 x path - 0.069`**, R^2 = **0.923** |
| error, first quarter mean | 0.281 m |
| error, last quarter mean | 1.617 m |
| at one instant | `\|amcl - truth\|` = **0.1144 m**, `\|gate - truth\|` = **1.8579 m** |
| at the latch | path 21.04 m, `\|gate - truth\|` 1.797 m |

Three conclusions, each of which kills a hypothesis:

* **Not a frame bug.** The gate's pose IS `spawn (+) odom` to within one message at full speed.
  So the resource rectangles and the gate's pose are in the same frame, and the "odom origin at
  spawn" family is not the answer this time.
* **Not a constant.** The error is ~8.1% of distance travelled. A constant can be measured once
  and subtracted; a rate cannot, and `map -> odom` being "nearly constant" (0.077 m over 3.400 m)
  is not the same statement as the error being constant -- the drift is in `odom -> base`, which
  AMCL is correcting and the gate is not reading.
* **A better pose is already on the machine.** AMCL agrees with truth to 11 cm at an instant when
  the gate is 1.86 m out. The gate composes raw odometry with the spawn deliberately
  (`geometry.compose_2d`'s own docstring says so), for the stated reason that a TF lookup may not
  refresh while a robot stands still. That reason is real and it costs 1.86 m.

**Decision recorded, not applied.** The fix belongs at the pose SOURCE. It is a safety-layer
change with its own staleness semantics (what the gate may conclude when the localised pose goes
stale is a new failure mode to design, not a parameter to change), and D-P10-01 has just changed
the boundary -- one variable at a time. Registered here with the numbers so the next round does
not have to re-measure it.

## D-P10-03 — the truth stream's slot index is not an identity

`fleet_recorder` reads the world's `dynamic_pose/info` as `transforms[slot * LINKS_PER_MODEL]`,
i.e. it assumes every model contributes exactly eight links and that the model order is fixed.
Measured, from the streams themselves:

* **8 is right for r01** and the eight transforms belong to one model: slot 0 moves with the
  robot, slots 1-7 are static offsets `(0,0)`, `(0,0)`, `(+/-0.260, +/-0.205)` and `(0.200, 0)` --
  the robot's own links.
* **The width varies within a run.** `reports/batch_20260917T082919Z/N01` carried 8, 16 and 24
  transforms at different times, so `range(width // 8)` yields a different number of slots at
  different times; slot 1 exists only while the width is at least 16.
* **In the two-robot case N03 one robot is absent.** The stream carried 8 transforms -- one
  model, starting on r01's spawn -- while **r02's own odometry shows 9.902 m of travel**. r02 has
  no truth slot at all, and the second slot an earlier N03 saw was a stationary offset at
  `(0.200, 0.000)`, not a robot. This is why the 20 blocked two-robot runs cannot be judged.

`judge.attribute_truth` already refuses to guess: a slot is accepted as a robot only if it STARTS
on that robot's spawn, slots that start nowhere near a spawn are reported by position, and a
robot with no slot is named -> the truth-based checks are `NOT_RUN` rather than answered from the
estimate. So the honest verdict was never the defect; the missing pose is.

**The probe, and the answer.** `scripts/probe_truth_slots.py` reports, index by index, which
entity is where. Run against the real two-robot world: 16 transforms, index 0 starting on r01's
spawn and moving 10.243 m, **index 1 starting on r02's spawn and moving 10.287 m**, and indices
2-15 the same seven local link offsets repeated per model. So the layout is *model frames first,
then each model's links* -- and r02's frame is at index 1, not index 8.

**Fixed.** `fleet_recorder._on_truth` now records **every** transform index and performs no index
arithmetic at all, so the identity path no longer contains an assumption. Identity is decided
downstream by `attribute_truth`, which accepts a slot as a robot only when it STARTED on that
robot's spawn, reports the other fourteen by position, refuses a non-decisive assignment, and
counts continuity violations -- i.e. it can *verify* a layout instead of trusting one. The audit
key `links_per_model` was removed: it asserted the link count as a fact and the fact was wrong.
`judge`'s message now names up to four non-robot slots and counts the rest, so a sixteen-slot
stream does not bury the sentence that matters.

**Validated live.** N03, the same two-robot case that could not be judged:

| | before | after |
|---|---|---|
| safety verdict | `UNKNOWN` | **`PASS`** |
| batch exit | 5 ("could not establish") | **0** ("every declared outcome held, and safety was judged from truth") |

`tests/test_p10_truth_slots.py` reproduces the defect from the measured layout: it asserts that
the old slice (indices 0 and 8) **cannot** attribute two robots and names the `(0.200, 0.000)`
slot that was read in r02's place, then asserts the full stream can. That test would have failed
before the fix.

**What this does and does not establish.** It establishes that the two-robot safety verdict is now
obtainable and that it held on one run. It does not establish anything about the 19 other blocked
two-robot runs, which are blocked on capabilities that still do not exist, and it does not make
the truth channel name its models -- the `Pose_V -> TFMessage` bridge still drops the name, so
every future layout change lands in `attribute_truth` rather than in a bridge.

## D-P11-01 — the localiser switch is verified live, and it exposed a deadlock the old pose was hiding

**Context.** `D-P10-02` measured the gate's pose error as a RATE: `|gate − truth|` grew with
distance travelled (8.1%, R² 0.923) while `|amcl − truth|` stayed at 0.11 m. The gate was
therefore switched to conclude from the localised pose, behind an explicit parameter
(`pose_source`), with a fail-closed rule when the localised pose is unusable. This registers
what happened when it was run.

**What the change does, measured rather than asserted.** `robot.launch.py` sets
`pose_source = "localiser" if start_nav2 else "spawn_odom"`, and the gate's own start-up line
in the run confirmed `pose source localiser`. `GateNode._trusted_pose` returns
`self._localiser_pose` when the verdict is usable, and otherwise returns the composed pose
**with `usable=False`** so the traffic layer refuses with `LOCALIZATION_STALE` — the composed
pose is never substituted, because bounding motion with a pose whose error is a rate is the
defect, not the fallback. The fail-closed path is visible at t=19.081 s in the run:
`mode=STOP reason=POSITION_UNKNOWN detail="localization invalid"`, one tick before the first
`amcl_pose` arrived at t=19.236 s.

**What it exposed.** The run then froze at t≈47.5 s with
`mode=STOP reason=RESOURCE_UNKNOWN`, and Nav2 reported `status=6 Failed to make progress`
(32 aborts, `spin`/`backup` timing out too) — the failure rounds 6–9 chased as a navigation
problem. The gate's own detail named the cause:

    unpermitted ['mid'], holdings none (reserve 0.323 m)

`mid` is the corridor at x ∈ [−1.75, 1.75], y ∈ [−0.65, 0.65]. The robot's true pose at that
moment was (−2.391, 0.194) — at `align_west`, which is at (−2.40, 0.00) and is one of only two
poses from which `acquire` may be called. Its footprint reached to within 0.35 m of the
corridor, and the gateway's envelope at that speed is 0.223 (stop) + 0.100 (localisation
margin) + 0.150 (footprint margin) = 0.473 m. **`align_west` was inside the refusal boundary.**

**Why the old pose hid it.** With the composed pose the gate saw the robot at (−2.869, 0.336) —
0.498 m from the truth, and 0.744 m from the corridor instead of 0.266 m. A robot whose
stopping distance reaches the corridor was allowed to drive deeper into it, because its
position was wrong by more than the margin. The deadlock was always there; the wrong pose was
the thing that made it invisible. This is the general shape: **a pose error that moves the
robot away from a boundary turns a deadlock into an apparent success.**

**Why it is a deadlock and not a tight fit.** The handshake may only be opened from `wait_*` or
`align_*`. The robot reached `align_west`, where it was refused *while moving* and allowed
*while stopped* — and a stopped robot's stopping envelope never shrinks, so the refusal could
never clear. Nothing in the system changes state to break it: the coordinator's sweep marks
resources FREE in the **coordinator's** manager, `ResourcePermit` carries no resource states,
and `GateNode._on_permit` deliberately ignores refusals ("a refusal carries no authority to
adopt"), so the gate's own manager can only reach a non-UNKNOWN state through a *grant* or by
observing the robot inside the region it is trying to enter.

**The decision: move the asking points; do not widen the boundary.** The refusal boundary is
the property that a robot whose stopping distance reaches the corridor may not proceed, and it
must hold at every speed the gate may authorise. Weakening it to admit the asking point would
trade a deadlock for the exact failure it exists to prevent. `align_west`/`align_east` move from
|x| = 2.40 to 3.35, derived rather than chosen: the boundary reaches |x| = 2.523 at
`max_measured_speed_mps` (0.35 m/s), and a robot counted as "at" a node may be
`node_reach_tolerance_m` = 0.80 m from it, so the whole legal asking ball must clear the
boundary — 2.523 + 0.80 = 3.323, rounded out to 3.35.

**The check that keeps it true.** `scripts/check_asking_points.py` asks the traffic layer
itself (`CrossingManager.check_movement`, the same call the safety gate makes) whether each
named node is a pose a robot may hold and arrive at with no permit; it re-derives no geometry,
because a check that re-derives the thing it checks can only agree with itself. The same
invariant is a test, `tests/test_p11_asking_points.py`, so it runs in `build.sh` and a later
edit to the stop model, the margins or the corridor rectangle fails the suite rather than a
simulation run. The test also asserts the *negative*: `exit_*` must remain unreachable without a
grant, so the invariant cannot be satisfied by widening the boundary.

**What this does NOT establish.** That the crossing completes: the geometry change has not yet
been run end-to-end (the run is in flight). That `wait_*` was ever broken — it was not; it is
separated from the corridor in y. And that the deadlock is gone in general: only that these two
nodes are no longer inside the boundary, at the measured speeds, with the configured tolerance.

## D-P11-02 — closing the localiser loop: the escape hatch could never become live

**Context.** `D-P11-01` put the gate on the localised pose. The first run afterwards froze
completely, and the probe named why: `pose_source: localiser` on 34/34 samples but
`pose_usable: False` on 33 of them, `localiser_messages: 1` for a 640 s run, `localiser_moved_m:
null` on every row, and **0.000 m of odometry travel** — the robot never moved at all.

**The defect.** `LocaliserPolicy` says a stale estimate is still usable while the robot has
moved no more than `max_moved_m` since it arrived — written precisely so that a stopped robot
does not freeze the gate. That needs the odometry pose from the estimate's instant, which
`_on_localiser` captured as `self._odom_raw`; live, that was `None`, because the estimate
arrived before the gate's first odometry sample. So `_moved_since_estimate()` returned `None`
for ever and the hatch never opened.

**Why it was permanent rather than momentary.** `nav2_params.yaml` sets `update_min_d: 0.25`
and `update_min_a: 0.2`, so AMCL updates — and publishes `amcl_pose` — only when the robot has
*moved*. Refusing motion therefore stops the localiser, which keeps the refusal live:

    gate refuses (stale localiser) -> the robot cannot move -> AMCL never updates
      -> amcl_pose never arrives -> the pose stays stale -> the gate refuses

That is the liveness hazard the hatch exists to prevent, and the hatch was wired to a value
that only a lucky message ordering could supply. `D-P9-*` already recorded the general form
("a gate that can permanently forbid all motion is itself a liveness hazard"); this is it
arriving through a different door.

**The fix.** `_on_odom` captures the arrival snapshot lazily when it was missed: if an estimate
has arrived and no snapshot was taken, the first odometry sample becomes the reference. The
window left unmeasured is one odometry period — 20 ms at the recorder's 50 Hz, 7 mm at the
0.35 m/s limit — which is why it is filled from odometry rather than estimated.

**Verified live.** The next run: `pose_usable: True` on **2872 of 2873** samples, and the
verdict strings show the hatch working:
*"localised pose is 6.699 s old but the robot has moved only 0.000 m since it arrived (<= 0.100
m), so it still describes where the robot is"*. Odometry travel went from **0.000 m to 34.999 m**.

**And the point of the whole change, measured.** The pose the gate *acted on* versus the pose it
*reported* (the composed one), against simulator truth, over 2873 samples:

| pose | median | p95 | max |
|---|---|---|---|
| the pose the gate REPORTED (`spawn (+) odom`) | 1.5516 m | 2.6825 m | 2.7610 m |
| **the pose the gate ACTED ON (localiser)** | **0.1866 m** | **0.3305 m** | 0.4322 m |

**8.3x better at the median, 8.1x at p95.** `D-P10-02` said the composed pose's error is a rate
and that the machine already carried a 0.11 m pose; this is that prediction confirmed under
load, on a route where the composed pose reached 2.76 m.

**The test that would have caught it.** `tests/test_p11_pose_source.py`: the policy *does* open
the hatch for a stationary robot; an unknown displacement is refused *and says so*; and
`_on_odom` has an AST-checked assignment to `_localiser_odom_at_arrival`, because the check is
"this assignment exists in this function" and a substring search would be satisfied by the
explanatory comment above it.

## D-P11-03 — the coordinator still judges from composed odometry, and its tolerance is smaller than that pose's error

**Context.** With `D-P11-01` and `D-P11-02` in place, N01 ran end to end. Task 1 **SUCCEEDED**,
`attempts 0`, and the crossing completed all six stages:

    stage_wait    reached wait_west     15.905 s
    stage_align   reached align_west    12.543 s      <- the node D-P11-01 moved
    acquire       granted west_to_east for 12.00 s after 1 request(s)
    stage_cross   reached exit_east     22.044 s
    stage_release reached release_east   9.434 s
    confirm_clear released ['mid', 'mid_right']

Task 2 (`east_to_west`, the same robot later in the same session) failed, and the reason is not
navigation:

    acquire: PRECONDITION_NOT_REACHED
    "not at a legal asking point: wait_east 1.252 m, align_east 2.375 m
     (pose x=1.273 y=1.153). Tolerance 0.80 m is set from the stack's measured arrival error"

Nav2 reported arrival at both `wait_east` and `align_east` (status 4, SUCCEEDED). The
coordinator's view of the robot was 1.25 m from a node Nav2 said it had reached.

**The cause, stated in the code.** `AcquirePassage.srv` says the callers' pose fields are
diagnostic: "The coordinator judges the request against the pose it has itself observed from the
robot's odometry". `coordinator_node.py` composes that observation as
`link.pose = compose_2d(link.spawn, odom)`. **That is the pose whose error is a rate** — measured
this run at **1.5516 m median, 2.6825 m p95, 2.7610 m max**.

`at_node` uses `node_reach_tolerance_m = 0.80 m`. **The tolerance is smaller than the median
error of the pose it is applied to**, so the check cannot be satisfied reliably once odometry has
accumulated: task 1 passed because it ran first (little drift), task 2 failed with ~2.7 m of it.

**The decision: register, do not patch.** Fixing it means giving the coordinator a pose source,
which is a safety-layer change of the same shape as `D-P11-01` but in the *supervisor* rather
than the supervised component — and `CONTRACTS` section 7's refusal to trust a caller-reported
pose is a deliberate property that must survive the change. The honest sequence is: decide what
the coordinator may conclude from when its localiser is stale, make it wrong-proof, then switch.
Doing it while the gate's own stale policy is one run old would stack two safety changes.

**What this does NOT establish.** That task 2 fails *only* for this reason: it also sits behind
the round-8 latch (`PERMIT_EXPIRED`, 34 rows this run, now transient rather than permanent) and
the request was retried 317 times over 158 s before the stage timed out. And it does not
establish that switching the coordinator's pose source fixes it — that is a prediction, not a
measurement.

## D-P12-01 — the coordinator concludes from a pose source of its own, and one implementation serves both components

**Context.** `D-P11-03` measured the coordinator's pose: `compose_2d(spawn, odom)`, 1.5516 m median
and 2.6825 m p95 from ground truth on N01, compared against `node_reach_tolerance_m = 0.80 m`. Task 2
of that run was refused at `acquire` -- "not at a legal asking point: wait_east 1.252 m" -- while Nav2
had reported arrival. The same pose also feeds three other judgements, so all four inherited it: the
start-up occupancy sweep, the automatic `_reverify_unknown` pass that lifts a bricked corridor out of
UNKNOWN, `acquire`'s asking-point precondition, and `confirm_clear`.

**The decision: give it a pose source, and do not write a second copy.** The gate already had this
machinery (`D-P10-02`, `D-P11-02`). The coordinator gets the same three outcomes from the same code --
`fleet_core.pose_source.PoseSource`, new, holding the bookkeeping that `D-P11-02` was a bug in --
rather than a second implementation of it. Two components that must agree about where a robot is may
not each keep their own copy of the answer; that is the same argument as `D-P10-01`, where the refusal
and the clearance proof were found computing two different boundaries.

`PoseSource` keeps the composed pose and the localised pose, records the odometry sample from the
instant an estimate arrived (**including lazily, when the estimate beats odometry to the node**, which
is `D-P11-02`), and answers one question: `trusted(now)` -> the pose to act on, whether it may, which
source, and the reason. `pose` is `None` whenever `usable` is False, deliberately: a caller handed a
pose alongside `usable=False` will eventually use it, and using the composed pose after refusing on the
localiser is the 1.86 m this mechanism exists to remove.

**`CONTRACTS` section 7 survives.** The coordinator still judges from a pose it observed ITSELF, on
`/{robot}/amcl_pose`; the pose fields on `AcquirePassage` remain diagnostic. A robot still cannot
assert its way past the entry precondition -- it can now only be wrong about where it is by 0.19 m
instead of 1.55 m.

**Wired, and self-reporting.** `fleet.launch.py` sets `pose_source: "localiser"` for the coordinator
under the same reasoning `robot.launch.py` uses for the gate: this file always starts Nav2. The node
announces its choice at start-up -- observed in the run:

    [fleet_coordinator]: pose source: 'localiser'; localiser on '/<robot>/amcl_pose', usable while
      age <= 2.00 s, or while the robot has moved <= 0.100 m since the estimate arrived. The safety
      gate makes the same decision on the same subject; see fleet_core.pose_source.

**Two things this run showed that the change caused, and one it did not.**

* The start-up sweep now needs a *localised* pose, and at start-up there is none: measured,
  `start-up sweep: ['r01'] have no fresh pose; resources stay UNKNOWN`. That is the designed
  fail-closed behaviour rather than a defect -- a resource is not granted because nobody has looked
  yet -- and the automatic re-verify lifted all three within the same run
  (`promoted ['mid','mid_left','mid_right'] from UNKNOWN to FREE on fresh poses`). But it is a real
  delay in clearability that `spawn_odom` did not have, and it is registered.

* The run then failed *earlier* than the previous one and for a different reason: the crossing
  driver's first goal was rejected three times ("another navigator is still processing one", 9.0 s),
  while `nav2_adapter_node` was also having goals rejected and the controller had been aborting
  `Failed to make progress` for ~150 s beforehand. **The cause is not attributed**: the run shows
  contention over the single `NavigateToPose` server between the leg executor and the crossing
  driver, which is a pre-existing shape that did not bite in the previous run, and one run cannot
  separate that from an effect of this change.

**What this does NOT establish.** That the coordinator's judgements are now correct -- only that they
are made from a pose measured 8.3x closer to truth. That `confirm_clear`, the sweep and the re-verify
behave better: none of them has been re-measured. And that the goal rejection above is unrelated to
this change.

## D-P12-02 — correcting D-P8-02: the "expired" mark chooses a label, it does not cause a refusal

**Where the claim lives.** Not under a `D-P8-02` heading -- that number appears in
`009_P8_GATE_LATCH_ROOTCAUSE.md`'s list of registers, but its content sits inside the `D-P8-03` block,
whose code-path paragraph ends "... `ReasonCode.PERMIT_EXPIRED`, which is a refusal (`allowed=False`)".
The paragraph carries the correction inline as well, so a reader who arrives at it is not left with the
wrong inference. That the claim lost its own heading is worth noting by itself: a register entry that
cannot be found by its number is a register entry that gets re-derived.

**What was registered, and why it is wrong.** `D-P8-02` recorded that the mark written at
`traffic.py` on a lapsed permit -- `_block_reason[name] = "permit expired without clearance"` -- is
tested by `any("expired" in _block_reason.get(n) for n in hits)`, and concluded that **the mark's own
text re-arms the refusal**, so the refusal could never clear. Read as a description of *behaviour*
that is false.

**What the code does.** `check_movement` computes the geometric test first and returns the only
allowance before the mark is consulted:

    hits = self._unpermitted_hits(unheld, corners, speed_mps=speed)
    if not hits:
        return PermitCheck(allowed=True, reason=ReasonCode.OK)      # the only `allowed=True`
    ...
    lapsed = (deadline is not None and wall_now >= deadline) or any(
        "expired" in (self._block_reason.get(n) or "") for n in hits)
    if granted is not None and set(hits) <= set(granted):
        reason = ReasonCode.PERMIT_EXPIRED if lapsed else ReasonCode.PERMIT_STALE

Every path after that early return has `allowed=False`. **`lapsed` selects a label on a refusal the
geometry had already decided.** The mark cannot admit or refuse anything; it can only make the reason
say "your permit expired" -- for ever, and for a refusal that may have nothing to do with an expiry.

**Why the distinction matters.** It changes what the fix is. If the mark re-armed the refusal, the fix
would be a string or a data structure. It does not, so the fix is geometric: in the round-8 freeze the
robot's footprint plus its stopping envelope overlapped a resource it did not hold, and **any motion,
including a motion away, is refused** because the test is on the predicted stopping envelope in every
direction. That is `D-P4-12`'s general form -- a robot stopped inside the passage has no legal exit --
and it is a question about retreat, not about text.

**What remains a real defect here, at the right size.** The label is still wrong to keep: a refusal
that persists for the rest of a session while reporting an expiry that happened once is a diagnostic
that lies, and this project's registers already say labels must not be trusted
(`fail_reason` in 008). Registered as a labelling defect, downgraded from a liveness defect.

## D-P12-03 — the escape hatch is still self-sustaining: the displacement freezes above the threshold

**Context.** `D-P11-02` closed one form of the localiser loop: the arrival snapshot could be `None`, so
the "the robot has not moved" hatch could never open. This is the same loop one regime further out,
found in the N01 repeat run (209 probe rows):

    "localised pose is 13.503 s old but the robot has moved only 0.000 m since it arrived
     (<= 0.100 m), so it still describes where the robot is"      <- the hatch, working
    "localised pose is 2.000 s old (> 2.00 s) and the robot has moved 0.262 m since it arrived
     (> 0.100 m), so it no longer bounds where the robot is"
    ... 550+ s of rows identical except the age, every one reading "...has moved 0.283 m..."

**The mechanism.** The quantity the hatch tests is the displacement since the estimate arrived. The
robot stopped, so odometry stopped, so that quantity is **frozen at 0.283 m** and can never fall back
under 0.100 m. Meanwhile `amcl_pose` is motion-gated (`update_min_d: 0.25`, `update_min_a: 0.2`: see
`nav2_params.yaml`), so a robot that is not permitted to move never receives a fresher estimate
either. The refusal sustains itself: refusing forbids the motion that would refresh the estimate, and
the displacement cannot decay without it.

**And the two constants are below the sensor's own error.** Measured across runs, AMCL's error against
ground truth is p50 0.101 m and p95 0.247-0.348 m, while:

    localization_margin_m   0.10 m   the only allowance the stop boundary makes for position error
    max_moved_m             0.10 m   the hatch's threshold

So the gate demands knowledge finer than the localiser provides, and then refuses when it does not get
it. Neither constant is wrong by itself; they are wrong together, and `fleet_core.pose_source`'s own
docstring already said so about the first one ("0.10 m is not a bound on a 0.348 m error").

**Why raising the threshold alone would be unsound.** The gate's position uncertainty is
`localiser error + displacement since the estimate`. The boundary adds a fixed 0.10 m for that whole
quantity. Lifting `max_moved_m` to 0.35 m to admit the observed case would let the gate act on a
position it is up to ~0.63 m wrong about while its boundary accounts for 0.10 m -- trading a liveness
failure for the soundness failure the margin exists to prevent. The two numbers have to move together,
and the uncertainty has to **enter the boundary as a computed quantity** rather than sit in a constant
that nobody re-derived when the pose source changed.

**The decision: register, do not tune.** The shape of the fix is now stated, the numbers it must
satisfy are measured, and both live in files that the frozen manifest tracks -- so the change is a
derived one that a reader can check, not a threshold moved until the run passes. Doing it tonight,
after two other safety-relevant changes in the same session, would stack a third.

**What this does NOT establish.** Whether the loop is what stopped the robot in that run: the observed
rows are consistent with it, but the run also had the goal rejection below and two tasks that never
started, so the gate was never asked to authorise the 0.283 m of motion. The loop is a property of the
policy as written and is demonstrated by the verdict strings; that it was the binding constraint in
that particular run is a separate claim, and it is not made.

## D-P14-01 — 被杀的驱动既不撤销目标、也不留报告：改为驱动自己停

**症状**：N01 连续两次在 `stage_wait` 死掉，报 `goal to wait_west was not accepted`（9.0 s，
两次序列相同），而导航本身没坏。Nav2 侧的原话是
`Requested navigation from navigate_to_pose while another navigator is processing`。

**实测真因**（`reports/batch_20260917T122048Z`，三个独立写入方一致）：

    t=673.6  穿越驱动 #1 启动（pid 4975）
    t=678.8  bt_navigator：开始导航 → wait_west          （目标被接受，正常在跑）
    t=854.5  派发器到 180 s 预算，杀掉驱动 #1
    t=854.5  驱动 #2 启动（同一个 command_id）
    t=862.7  驱动 #2 的目标被拒："另一个导航器还在处理"
    t=871.8  驱动 #1 那个被遗弃的目标才自己失败退出（error 105）
    t=872.6  驱动 #2 已经用完 3 次尝试并放弃

⇒ **杀进程不会撤销导航目标。** 重试撞上孤儿目标，而它的耐心是
**3 次 × 3 秒 = 9 秒**，孤儿活了 **9.123 秒**（重试在 9.026 秒放弃）⇒ **差 0.097 秒**，
两次都一样 ⇒ 这是竞争，不是抖动。**并且第一份报告根本不存在** —— SIGTERM 默认动作结束进程，
`main` 没走到 `write_text`。

**两条教训，形状不同**：
* **耐心是时长，不是次数**。次数是错的单位：延迟由调用方选，所以"三次尝试"关于服务器会说谎。
* **释放者必须是驱动自己**，而且在派发器还肯等的时候。

**实现**（`fleet_core.crossing_release.ReleasePolicy` + `staged_crossing`）：
* 驱动自己的运行截止 = `timeout_s - 15 s`（余量 > 实测孤儿 9.123 s）；派发器 180 s 的杀只是兜底。
* `busy_budget_s = 60 s`（时长制），不是 3 次。
* SIGTERM/SIGINT 安装处理器，**只设标志**（任何 rclpy 调用都在 spinning 线程上），
  `drive_to`/`acquire`/`confirm_clear` 的循环检查它 → **撤销目标** → 写报告 → 退出 4。
* 报告的 `stopped_by` 区分 `signal` / `run_deadline` / `busy` / `nav2_absent`：
  四者给读者的下一步动作不同，所以不给同一个"失败"。

**实测（2026-09-18）**：驱动在 **162.6 s**（自己 165 s 预算）停住，报
`stopped_by: run_deadline`，报告落盘，派发器**没有杀过它**。修前该文件不存在。

**已知小缺陷**：`stopped_detail` 的出口当时没传 `t0` ⇒ `elapsed_s` 为 0（文字里有 162.6 s）。
已修，并立规矩：`_set_stage` 的每个出口都要带 `t0`，否则结构化字段与句子会不一致。

## D-P14-02 — 撤退许可是**两半**，`D-P4-12` 只写了一半（2026-09-18）

`D-P4-12` 的提案是：已在矩形内的车可以移动，**当且仅当**指令运动严格减小它与保护区的重叠。

读代码时发现这只是一半。`check_movement` 的输入是**当前位姿 + 指令 twist**，
输出是"这一刻允许不允许"。**如果没有人下撤退指令，再宽的许可也不会让车动。**
所以第二半是**派发器里的撤退发出方**：发现某台车停在它无法合法离开的区域里时，
必须**发出**一段"后退 N 米"的目标。

**两半必须一起做，且顺序有讲究**：
* 只做第二半（发撤退指令）而门禁仍拒绝 ⇒ 指令被零化，看起来像导航故障。
* 只做第一半（放宽许可）而没人发指令 ⇒ 车原地不动，看起来像"修了没用"。
* **先做第二半**是对的：它是**策略层**改动，可测、可回放；第一半是安全层改动，要独立设计与实跑。

**同时记下一条测不准**：`scripts/probe_gate_state.py` 只搬门禁的**组合位姿**，
所以本轮同一行里 `|gate-truth| = 0.650 m` 而 `|amcl-truth| = 0.072 m`。
门禁发布两个事实（`pose` 与 `pose_trusted`），仪器只搬一个 ⇒ **缺的那个看起来"不存在"**
（`LESSONS.md` 第 15 条，两轮后重犯）。**在做第二半之前必须先把 `pose_usable` 搬进仪器**，
否则"门禁按哪个位姿判断"在数据里不可见。

## D-P15-01 — 拒绝会停掉定位器，所以要能主动要一份"不动"的估计

**实测（2026-09-18，N01，516 样本，`_p009/p14/_run/pose_track.jsonl`）**：

| 量 | 值 |
|---|---|
| `pose_usable` | True 463 / **False 52** |
| `\|amcl − 真值\|` | p50 0.1021 · p90 0.2201 · **p95 0.2515** · max 0.3385 |
| 拒绝原话 | `localised pose is 2.251 s old (> 2.00 s) and the robot has moved 0.280 m since it arrived (> 0.100 m)` |
| `localiser_moved_m`（被拒时） | p50 **0.2583** · p95 0.3117 · max 0.3397，**冻结不动** —— 常量即车已停 |
| `localiser_age_s`（被拒时） | p50 **316 s** · max **813 s** |

**循环**：拒绝运动 ⇒ AMCL 不发布（`update_min_d: 0.25` 是运动门控）⇒ 年龄无界增长、
位移再也不变 ⇒ 两个判据永远同时不成立 ⇒ **拒绝自己维持自己**。逃逸舱之所以打不开，
是因为车是在**走过 0.100 m 之后**才停的，而停下的车的里程计再也不会动、也就永远降不回阈值以下。

**治因，不是调阈值**：这份 AMCL 提供正确入口（`nav2_amcl/nav2_amcl/amcl_node.hpp`）：

```
// Let amcl update samples without requiring motion
nav2::ServiceServer<std_srvs::srv::Empty>::SharedPtr nomotion_update_srv_;
// Request an AMCL update even though the robot hasn't moved
void nomotionUpdateCallback(...);
```

**实现**：
* `fleet_core.pose_source.needs_refresh(age_s, policy, since_last_request_s, min_interval_s)`
  —— 纯函数，**不含任何位移判据**（提问不能让位姿更不准，所以没什么可设阈值；
  而任何位移判据都会把同一个冻结在下一层重演）。
* 门禁 `_ask_for_an_estimate()`：每 tick 问一次策略，需要时 `call_async(Empty)`，**间隔 2 s**。
* **服务名从图里发现**（`get_service_names_and_types` 里名字含 `nomotion` 的），**不硬编码**：
  同一个控制在旧 nav2 是 topic、在这个 build 是 service；名字写错会静默无所作为，
  而"静默无所作为"与"定位器无话可说"读起来一模一样。
* 找不到服务时打 **`error`** 并写进 payload（`nomotion_service: null`）——**"我没能问"
  必须和"我问了但没有变化"可区分**。
* **顺带修一个真缺陷**：那条"位姿不可用"的 `warning` 原来是**每 tick 打**（20 Hz × 355 s ≈ 7100 行），
  改成**理由变化时 + 至少间隔 `unusable_log_period_s`（10 s）**。
  **重复陈述一个事实的日志，会把它后面那个新事实埋掉。**

**实测结果（2026-09-17T18:15:39Z，N01）**：

* 服务名被**从图里发现**为 **`/r01/request_nomotion_update`**，门禁自己打印了
  `asked the localiser for an estimate without motion on '/r01/request_nomotion_update'`。
* **`N01 seed 1: task=SUCCEEDED · safety=PASS · behaviour=PASS` ⇒ `batch exit 0`** ——
  这个用例历史上第一次。
* 两段穿越各六步全过、**续期 14 / 13 次**、`confirm_clear` 分别释放 `['mid','mid_right']`
  与 `['mid','mid_left']`、**0 次杀进程**。
* `pose_usable` **True 3776 / False 24**（上一轮 463/52）；`localiser_age_s`
  **p50 0.63 / max 2.39**（上一轮 p50 0.80 / **max 813**）。
* "位姿不可用"警告 **65 行**（上一轮同一段时间约 7000 行）。
* `|gate_trusted − 真值|` p50 0.1967 / p95 0.3653 —— **仍大于 `localization_margin_m` 的 0.10**，
  所以 `D-P15-02` 的健全性缺口没有被这次成功掩盖。

**⚠️ 一次成功 ≠ 稳健。** 本轮**只跑了一次、无争用、无排队、无取消**。可以说"这个用例通过了一次"，
**不可以说"穿越稳健"或"位姿问题已解决"**。而"刷新救回了死锁"这一因果**尚未证明**：
本轮的机器人从未进入上一轮那种完全停死状态，所以两轮的差异里既有这一改动、也有运行形态。

**本轮自己留下的仪器缺口（已修）**：只记录第一次请求 ⇒ "问了几次"不可观测；且探针的白名单
带 `pose_usable` 却没带那三个新字段 ⇒ **同一个事实被丢了两次**。

## D-P15-02 — 两条 safety 余量必须**一起**从实测里重推（未实现，耦合已写清）

**为什么必须重推（实测，不是印象）**：

| 常数 | 今天 | 它必须覆盖的量（实测） | 差距 |
|---|---|---|---|
| `stop.localization_margin_m` | **0.10** | 定位误差 **p95 = 0.2515**（p99 0.3085，max 0.3385） | **小 2.5 倍** |
| `localiser_move_margin_m`（`max_moved_m`） | **0.10** | 被拒时的冻结位移 **p50 = 0.2583**、max 0.3397 | **小 2.6 倍** |

⇒ ①**边界是不健全的**：它用**只赔 0.10 m 的**假设去认证一个可能有 0.25 m 误差的位姿；
②**逃逸舱打不开**：阈值比观测到的每一个冻结值都小。

**为什么不能只改一个数字 —— 耦合**：

1. `localization_margin_m` 是**拒绝边界**的一项（`traffic.protected_region`）。
   它从 0.10 抬到 0.2515 ⇒ 边界沿 |x| **外移约 0.1515 m**。
2. 申请点 `align_*` 是**由那条边界推导出来的**（`D-P11-04`：`2.523 + 0.80 = 3.323` → 取 3.35），
   而 `tests/test_p11_asking_points.py` 的余量**只有 0.027 m**（3.35 − 3.323）。
   ⇒ **边界一动，那个测试就红，于是必须重推 `align_*` 与地面的 `pad_release_*` 标记。**
3. 而 `max_moved_m` 与 `localization_margin_m` **耦合**：陈旧估计的总不确定度是
   `定位误差 + 陈旧位移`。若允许陈旧到 `d_max`，边界就必须多赔 `d_max`。
   取 `d_max = p95 = 0.2515` ⇒ `localization_margin_m ≥ 0.503`（今天的 5 倍），
   边界再外移 0.40 m —— **这会把申请点推到走廊外很远，本身就该被质疑**。

**所以顺序是**：**先做 `D-P15-01`（治因）**，让陈旧位移回到接近 0，从而**不需要**为陈旧多赔；
**再**把 `localization_margin_m` 抬到实测 p95 并**同时重推申请点与地面标记**；
`max_moved_m` 则**由 `localization_margin_m` 导出**，而不是第二个独立常数。

**⚠️ 不许做的事**：只把 `localization_margin_m` 从 0.10 改成 0.2515 然后看着测试变绿 ——
那既动了安全边界、又可能悄悄放宽申请点不变式。**这两半必须在同一次改动里、带着重推的节点一起落。**

# 009 decisions

## D-P16-01 — the crossing cases are runnable; arrival order is a predicate, not a sleep

`crossing_scenarios` was the register's largest entry (13 runs, then 10) and its content had
become scenario plumbing plus one missing idea. Both are now in:

* the four scenario files, `config/scenarios/n04_face_to_face_r01_first.yaml`,
  `n05_face_to_face_r02_first.yaml`, `n06_two_west_queue.yaml`,
  `n07_three_robots_six_tasks.yaml`;
* **arrival order as a predicate.** `fleet_core.wait_for` evaluates a condition against the
  fleet snapshot until it holds. A case writes `wait_for: {robot_crossing: r01}` on r02's
  submission and r02 then asks second *by construction*, rather than by winning a race. The
  rejected alternatives and why: sequential submission alone is not order (two requests sent
  back to back reach their robots whenever each frees up, so the case would pass or fail by
  luck and nothing in the record would say which), and a sleep is the same shape as the
  "sleep then kill" trigger this project refuses elsewhere (a fixed sleep is a guess, wrong in
  both directions).

The claims are then checked from the task service's event stream rather than the snapshot,
because the snapshot is last-writer state and "who went first" is gone by the end of the run:
`crossing_exclusive` refuses any overlap of two robots' occupancies, `crossing_order` requires
the first occupancy to be the declared robot's. Round 8's latch was an example of the same
file being the only record of something the status map had already forgotten.

**Neither check has a threshold in it.** An overlap is `start_b < end_a and start_a < end_b`
between two timestamps from one file, so there is no number anyone can widen later. An
occupancy with no recorded release is treated as NEVER RELEASED (it runs to infinity for the
overlap test) -- the conservative reading, since the property is that the passage held one
robot at a time and no recorded release means we cannot prove the robot left.

## D-P16-02 — a batch case cannot pin a task to a robot, so the geometry has to do it

`SubmitTask.srv` has no pin field and `pinned_robot` is set only for the charge runs the task
service generates itself. The allocator chooses the cheapest candidate by travel distance at
the robot's own speed, with battery as a hard filter first, so **with two idle robots a task
always goes to the nearer one.**

That has a consequence for the crossing cases that is not obvious and cost an hour to work
out: **a task whose pick is on the far side of the barrier will be given to the robot already
on that side, so no crossing happens at all.** N04/N05/N06 are therefore written with the pick
on each robot's own side and the drop on the far side, which makes the SECOND leg the
crossing. Two further constraints follow from the same place: the diagonal station pairs
(`S_left_a <-> S_right_b`, `S_left_b <-> S_right_a`) are the only pairs whose straight line
passes through the corridor, so a same-side pair silently plans no crossing at all and the
robot would be sent at the wall; and the order between two robots has to be arranged with
`wait_for`, because there is no way to say "this robot first" to the allocator.
# D-P16-03 -- a permit queue is not part of the driver's movement budget

Round 16's N04 made a queue on purpose -- its claim is section 5's "r02 waits" -- and the run that
produced the queue also showed the budget was wrong. From r02's own report
(`reports/batch_20260917T191716Z`):

    stage_wait     38.924 s   reached wait_east (nav status 4)
    stage_align    26.570 s   reached align_east (nav status 4)
    acquire        42.868 s   granted east_to_west for 12.00s after 86 request(s)
    stage_cross    51.653 s   reached exit_west (nav status 4)
    stage_release   2.620 s   FAILED -- "reached this driver's own budget during stage_release"

and r01, the robot ahead of it: `acquire ok=True 0.003 granted west_to_east after 1 request(s)`,
`confirm_clear released ['mid', 'mid_right']`, task SUCCEEDED.

**The crossing protocol was correct.** r01 was granted on its first request; r02 was refused 85
times while r01 held the corridor and then granted. What failed is arithmetic: `crossing_timeout_s`
is 180 s, the driver's own deadline is 165 s (180 minus round 14's 15 s release margin), and
**42.9 s of those 165 went on queueing**. The robot ran out of budget 2.6 s into its last stage
with the permit in its hand.

### The fix

* `ReleasePolicy.hard_deadline` is `started_at + timeout_s - release_margin_s`, and **nothing may
  move it outward** -- not even the queue allowance. The margin belongs to the dispatcher: a kill
  leaves an uncancelled Nav2 goal behind and the next driver collides with it (round 14 lost that
  race by 0.097 s).
* `run_deadline` is `hard_deadline - queue_allowance_s`, i.e. the allowance is held back rather
  than granted, so a driver that never queues is unaffected and a driver that queues gets it back.
* `after_queue(deadline, queued_s, ...)` credits `min(queued_s, queue_allowance_s)`, capped again
  at the hard deadline.
* `queue_allowance_s` is **declared by the scenario** (`crossing_queue_s`), because CONTRACTS
  section 9 says a scenario that expects a queue should say how long it is prepared to wait --
  the words were already in `n01_crossing.yaml`'s budget comment. n04-n07 declare 90 s and raise
  `crossing_timeout_s` to 300 s; n01-n03 declare nothing and behave exactly as before, and the
  seventeen round-14 tests keep their numbers because the default allowance is zero.

`tests/test_p16c_queue_budget.py` asserts the credit **and the cap**, because the cap is the
property that matters: whatever the queue, the driver must still stop before its dispatcher.

# D-P16-04 -- `crossing_exclusive` measured the wrong interval, and is not asserted

The version written this round reconstructed each robot's occupancy from `crossing_started` to
`crossing_complete` / `crossing_failed`. On N04 it reported:

    corridor exclusivity: r01 and r02 were inside the passage at the same time:
      r01 [670.9, 801.9] vs r02 [687.9, 856.9]

**That is correct queueing, not a violation.** `crossing_started` is emitted when the passage
driver PROCESS is launched, and the driver then runs `wait -> align -> acquire -> cross ->
release`. So the interval runs from process launch, and it necessarily includes the time the
robot spends waiting for the permit -- which, in a case whose whole subject is waiting, is the
longest part of it. r02 was legitimately queued from 687.9 and was not granted until 42.9 s into
its own acquire stage.

The interval that the claim is about -- the permit held, or the body physically inside the
corridor -- is **not recoverable from what is recorded**:

* the events carry `t = time.time()`;
* the driver's stages carry `elapsed_s` measured with `time.monotonic()`, a different clock
  domain with an arbitrary epoch, and the report has no absolute start.

Deriving a base by adding `elapsed_s` to the `crossing_started` timestamp was tried and rejected:
it puts a process-spawn delay of unknown size into the answer, and round 9's lesson is exactly
that a derived time base produces confident numbers about the wrong thing.

### What is asserted instead, and what is left open

* `crossing_order` **is** asserted, and is unaffected: it compares two timestamps from the same
  file, which needs no base. N04 measured it holding (`['r01', 'r02']` as declared).
* `crossing_exclusive` is **not declared by any case**. This is an unasserted claim, not a passing
  one, and it is recorded here so that a later reader does not find the key's absence and assume it
  was never wanted.
* The check's failure message now says what interval it reports and that an overlap of episodes is
  what correct queueing looks like, so a future reader cannot mistake it for a safety finding.
* **The replacement, in preference order:** have the coordinator emit `passage_granted` /
  `passage_released` events when it grants and clears a bundle -- the instrument belongs to the
  component that owns the resource -- or add absolute wall timestamps to the driver's stages. The
  first is better: it is one event pair, it needs no clock reconciliation, and the coordinator
  already owns every fact the claim depends on.

The acquire stage's request count per robot IS recorded (`crossing_acquire_requests`): 1 for r01
and 86 for r02 on N04, which is the direct evidence that the corridor made somebody wait.

## D-P17-01 — the trigger vocabulary: `when <predicate> do <action>`

**What was missing.** `docs/LIMITATIONS.md`'s `batch_driver_extended` entry said the runner "needs
either a bespoke multi-phase driver or an event trigger the runner does not implement", and
TEST_AND_ACCEPTANCE section 5 states the rule behind that: 故障按状态触发而非某个固定 sleep ...
超时未到触发条件为 PRECONDITION_NOT_REACHED，不算故障测试通过.

**What was built.** `fleet_core/trigger.py` enumerates the actions; `steps:` in the manifest
carries them; `scripts/batch.py` fires them inside its observation loop; and
`scripts/check_batch_manifest.py` parses the actions, the actions' required fields, the node names
the launch files actually start, and the behaviour checks `batch.py` implements — so a manifest
cannot name something that does not exist and be told it passed.

Three properties were kept deliberately:

* **One predicate vocabulary.** `when:` is evaluated by `fleet_core.wait_for`, against the same
  snapshot the rest of the case reads. A second vocabulary would drift, and the drift would look
  like a case waiting for something nobody checks.
* **Steps run BEFORE the loop's exit condition.** Those two can become true in the same tick, and
  a case that finished without its fault having been delivered would be an ordinary run wearing a
  fault case's name.
* **A fault is addressed by argv, not by node name.** Measured with `ps` on a live fleet:
  `python3 .../lib/fleet_ros/nav2_adapter_node --ros-args -r __node:=nav2_adapter -r __ns:=/r01`.
  A node's NAME is a runtime registration and does not appear in `ps`; its argv does, and the
  namespace is part of it. So `-r __ns:=/r01` addresses one robot's nodes and not the other's.

**What moved.** F06, F07, F08, F09, F11, F12, F13, F14 and N08 (three seeds) — eleven runs. That
retires `robot_kill_trigger`, `nav2_fault_injection` and `localization_fault_injection`, and
narrows `batch_driver_extended` to F03, I03 and I08.

**What did NOT move, and why.** `restart_process` restarts a node from its OWN argv, read out of
`/proc/<pid>` before the kill, because the argv carries the generated `--params-file /tmp/launch_params_*`
and nothing else reproduces it; a restart that invented its own argv would start a node with
default parameters and report that the fleet recovered. Section 6's 中央重启保留 SQLite is
satisfied by the argv coming back unchanged, and not by anything the runner chooses.

## D-P17-02 — F05 was filed under the wrong prerequisite

F05's title is 持货低电 and its expected result is 保持货物归属、合法停车 attention. It was
registered under `batch_driver_extended` from the start, which put it in the same bucket as the
cases that were missing a trigger.

**It is not a trigger problem.** Wiring the trigger showed that nothing in the tower implements
the rule at all. CONTRACTS section 5 states it in two lines — 未持货低电 gets its leg cancelled and
a charge run; 持货低电 does not auto-unload and is parked at a legal reachable position with
NEEDS_ATTENTION — and while the first half exists (`_service_battery` generates the charge run),
the second does not:

* `task_service_node._service_battery` starts with `if state.executing or not self._fresh(...):
  continue`, so a robot that is mid-task is never even considered for charging;
* `nav2_adapter_node` simulates the battery (`self.battery.simulate`) and reports it
  (`battery_fraction`, `battery_wh`) and takes no decision from it;
* `gate_node` does not read the battery at all.

An injected `FaultInject` sets the STATE of charge and nothing else, which is the right design
("This injects a STATE, not a decision") — but it means that with no decision anywhere above it, a
robot forced to critical mid-leg simply drives on.

F05 therefore keeps a `requires:`, on a key that names the missing policy
(`low_battery_while_executing`) rather than the missing vocabulary. It is a behaviour gap, so it
belongs with the stage-1 defects and not with the stage-2 capabilities.

## D-P17-03 — two cases run with a clause their manifest cannot decide

F13's claim is 下游保护确实生效；否则 FAIL and F14's is 停车封锁相关区域，拒绝真值代驾. In both
cases the runner can decide part of it and not the rest, and both now say so in their own `note:`
rather than leaving a reader to infer that "the case passed" covers everything the title claims:

| case | asserted | NOT asserted |
|---|---|---|
| F13 | the task cannot succeed with the gate gone; the truth-based safety properties hold | that nothing downstream of the gate publishes `cmd_vel` — that needs the command stream (`commands.jsonl`, section 7), which this runner does not read |
| F14 | the task is parked; safety from truth | that the controller did not make do with a substitute pose — `err_gate_truth` is recorded in the judge's summary and asserted nowhere |

This is the same shape as D-P16-04 (an interval the instrument could not decide) and it is recorded
for the same reason: **unasserted is not passing**, and the difference has to be written where the
reader is already looking. Both cases therefore count as RUN, not as PROVEN, until a check exists.

## D-P17-04 — two fault-entry shapes, and only one of them is checked (`faults:` vs `steps:`)

`faults:` predates `steps:` and is narrower in all three directions: it accepts only
`kind: battery_inject`, only `when: immediate`, and it runs before anything is submitted. It is
still what F04 uses, and F04 is a passing case, so it was not rewritten in the round that
introduced the replacement.

The cost is that `scripts/check_batch_manifest.py` validates `steps:` and does not look at
`faults:` at all, so a `faults:` entry naming a trigger the runner does not implement is only
caught at run time, as `CaseFailed` → NOT_RUN. Registered as debt: when someone next touches F04,
`faults:` should become a `steps:` entry with no `when:`, and the guard should then refuse the
old key rather than ignore it.

## D-P17-05 — 导航反馈回调必须按 rclpy 真实的调用形式写（一个参数、`msg.feedback`）

`send_goal_async(goal, feedback_callback=...)` 的回调被 rclpy 以**一个参数**调用
（`rclpy/action/client.py`: `await_or_execute(self._feedback_callbacks[goal_uuid], feedback_msg)`），
而收到的是 **ActionFeedback 包装消息**，payload 在 `msg.feedback`。round 17 的仪器两半都写错了
（`def _on_feedback(self, _handle, feedback)` 且直接读 `msg.distance_remaining`）。后果：每个驱动在
第一条反馈到达时抛 `TypeError`，报告写成 `stages: [], stopped_by: exception` —— **读起来和"车拒绝
移动"一模一样**，而且 safety 反而 PASS（车一步没动）。四个用例因此白跑，其中 N05 那次 PASS 是
空转出来的。

**为什么七道守卫 + 712 个测试都没看见**：语法合法；名字有定义（`check_undefined_names.py` 查的是
**名字**，这个是**参数个数**）；单元测试从不用 feedback 调它（**只有 ROS 执行器会调**）；
子串断言对坏代码也绿（`feedback` 在定义、注册、注释里都有）。
**而两处的正确写法本来就在这个仓库里**：`scripts/nav_goal.py:65` 与 `:76`。

⇒ 新增守卫 `scripts/check_ros_callback_arity.py`（已接进 `check_guards.sh` ⇒ `build.sh`）：
把 `send_goal_async` / `create_subscription` / `create_timer` / `create_service` 的每一个回调
**解析到它的定义**，再按 ROS 真正传参的个数检查（78 处 resolved / 12 处 skipped，每个 skip 都写明
理由，不是"假定没问题"）。**教训：名字存在但签名不对时，所有基于名字的检查都会放过它**
（`LESSONS.md` 第 7 条的新变体）。

## D-P17-06 — AMCL 的更新步必须 ≤ 门禁的逃逸舱（`update_min_d` 0.25 → 0.10）

`amcl.update_min_d`（车要走多远才重发 `amcl_pose`）与门禁的 `max_moved_m`
（= 节点的 `localiser_move_margin_m`，"估计到达后车还能走多远、这次估计仍兜得住它"）是**两个文件里
的两个数**，由两个子系统拥有，而它们必须满足一条不等式。当天不成立时，门禁**不是偶尔**、而是
**每个更新区间都必然**拒绝一次运动。

stock 的 `update_min_d: 0.25` > 逃逸舱 `0.10`。实测（`batch_20260918T061032Z`，N05 三种子）：

| | `no-motion` 刷新 | 门禁拒绝 | Nav2 abort(105) |
|---|---|---|---|
| **通过那次** | 220 | 256 | **0** |
| 失败两次 | 395 / 405 | 227 / 262 | **17 / 18** |

代价不在拒绝本身，而在 Nav2 的反应：`SimpleProgressChecker` 是 `required_movement_radius: 0.5` /
`movement_time_allowance: 10.0`，被门禁按住 10 s 就变成 `Failed to make progress`，行为服务器清两层
costmap 后重规划 —— **68 s 的接近段变成 194 s**。这也是 `D-P15-01` 的同一族：自我维持的拒绝，
只是这次的触发者是"估计不够新"。

**改法**：`update_min_d` → `0.10`，即**不能跨过逃逸舱的最大步长**，不是新的调参偏好。
必须在 `scripts/gen_nav2_params.py` 里改：`config/nav2_params.yaml` 自带 "Do not hand edit"，
且 `gen_nav2_params.py --check` 会失败 —— 手改会被下一次重生成静默回滚（与"安装脚本覆盖就地补丁"
同一类陷阱）。

**新增守卫 `scripts/check_pose_freshness.py`**（已进 `build.sh`）：从生成的 Nav2 参数与两个节点的
`declare_parameter` 默认值里读这两个数，`update_min_d > max_moved_m` 就红，并说明后果与两条出路。

**⛔ 这个决策只覆盖"两个数不再互相矛盾"。** 逃逸舱本身仍是 0.10 对定位 p95 0.2515 m ——
那是 `D-P15-02`，**仍未做**。接近段的实跑效果要由之后的 N05 三种子来判，本决策不预设它会绿。

**读旧条目时注意**：本决策之前的条目里写的 `update_min_d: 0.25`（例如 `D-P15-01` 与 `D-P12-03` 段）
描述的是 **2026-09-18 之前**的状态。那些**测量**依然有效（它们是在 0.25 之下量到的）；变的只是这个值本身，
而 D-P17-06 正是它们指向的那条循环的另一个触发器。

## D-P17-07 — 判不了 ≠ 通过：真值槽位在"还没上报"时被读成了原点（2026-09-18）

**症状**：N07 三个种子全是 `safety=UNKNOWN`，而 `no_simultaneous_occupancy` 与
`no_unauthorised_entry` **两条都是 NOT_RUN**。judge 自己写得很清楚：

```
no slot starts on the spawn of ['r02', 'r03']: slot 1 sits at (0.000, 0.000) and moved
10.558 m in total; slot 2 sits at (0.000, 0.000) and moved 6.000 m in total; ...
```

**根因**：`_slot_tracks` 取"某个槽位第一次出现的那条样本的位置"当作它的**起点**。而世界的位姿数组是
**随着模型上线逐渐变宽**的 —— 实测 N07 seed 1（`batch_20260918T080259Z`）的宽度序列是
**0（241 条）· 8（7 条）· 24（12147 条）**；在那 7 条 width-8 的样本里，**只有槽位 0 有位置，槽位 1–7 全是 (0, 0)**。
其中两个后来证明是 r02 与 r03 ⇒ 它们的"起点"被钉在原点 ⇒ `attribute_truth`（规则是
**槽位只有在"从该机器人的出生点出发"时才算那台机器人**）两个都匹配不上 ⇒ 所有基于真值的检查 NOT_RUN。

**为什么两车用例从没暴露**：两车的宽度只有 0 与 16，**中间没有那个可以毒化起点的数组**。
但它在 **N06** 上出现过 —— N06 报 `robots_absent: ['r02']`，**只按 3 台里的 2 台判**。

**修法**：`first` = 槽位**第一次上报"离开原点"的位置**；在此之前它只是"还没上报"。
**跳过原点是可靠的，不是图方便**：没有机器人能出生在原点 —— `scripts/validate_assets.py`
证明 `config/spawns.yaml` 里每个出生点都避开墙、垫子与受保护走廊，而 (0,0) 正在走廊喉部。
**永远没离开原点的槽位 `first = None`**，这是一个关于录制的事实，**故意不等于**"这个槽位从原点出发"
—— 那正是旧版本断言的东西。序列化输出 `start: None`，不再把它变成一个数字。

**结果（用现有录制重判，不需要仿真器）**：

| 用例 | 归因 | no_simultaneous_occupancy | no_unauthorised_entry |
|---|---|---|---|
| N07 seed1 | **PASS**（原 NOT_RUN） | **FAIL** | PASS |
| N07 seed2 | **PASS** | **FAIL** | **FAIL** |
| N07 seed3 | **PASS** | PASS | PASS |
| N05 ×3 | PASS | PASS | PASS |
| 015759Z/N06 | PASS（仍 2/3） | PASS | PASS |
| 015759Z/N07 / seed2 | PASS | PASS / **FAIL** | PASS / **FAIL** |

⇒ **N07 的 UNKNOWN 是仪器故障，不是系统的属性。修好之后它变成真实判决：2/3 个种子真的不合格。**
⇒ 而 **N05 的三种子仍然是 PASS/PASS/PASS**，所以上一轮那个绿色结果**没有被这个修复推翻**。

**⚠️ 但 N05/N06 的既有 batch 记录仍是旧值**：`batch.py` 在跑的时候用当时的 judge，所以
**要拿到权威记录必须重跑**（或接受"重判值"并标注）。
**⛔ 还不能说**：N06 的 PASS 仍然只覆盖 **3 台里的 2 台**（`absent: ['r02']`）—— 那是一条**独立**的待查项。


---

## D-P17-08 — 三车录制里"真值冻结"的真身：**真值流只递出一帧，然后无限重复它**（2026-09-18，2026-09-19 两次更正）

**⛔ 2026-09-19 第二次更正 —— 上一节的推理链里有一个读错坐标的错误。**

上一节我写：冻结的那一帧里 `model1` 在 `(-0.26,-0.205)`（距出生点 5.80 m）、
`model2` 在 `(0.2,0.0)`（距出生点 6.14 m），因此"这是一个跑到中途的快照"。
**这是错的。** 那两个数是**刚体内部的连杆偏置**，不是世界坐标：

| 槽位 | 值 | 它到底是什么 |
|---|---|---|
| 0 / 1 / 2 | `(-6.0,-2.0)` / `(-6.0,0.6)` / `(6.0,-2.0)` | **三个模型帧**（世界坐标，出生点） |
| 3 / 4 | `(0,0)` | 模型内部空偏置 |
| 5–8 | `(±0.26, ±0.205)` | **四个轮角**（车体坐标） |
| 9 | `(0.2, 0.0)` | 某个连杆（车体坐标，与出生点无关） |

我把**轮子相对车体的偏置**当成了**车的世界位置**去算距离。
这正是本项目反复犯的"字段存在 ≠ 它是你要的那个"（`D-P17-03` / `D-P17-07` / `D-P10-03`）。
**"从出生点走了 5.8 m" 这个说法作废。**

**重新逐帧数过之后的实测**（判据 = 槽位 0/1/2 的"相邻去重"计数）：

| 运行 | 车数 | 槽位 0 的相邻去重计数 | 槽位 0 首 → 末 | 判定 |
|---|---|---|---|---|
| `N07` T103152Z | 3 | **1** | `(-6.0,-2.0)` → `(-6.0,-2.0)` | 一帧都没变 |
| `N07_seed2` T103152Z | 3 | **1** | 同上 | 一帧都没变 |
| `N08_seed2` | 3 | **1**（38679 个样本） | 同上 | 一帧都没变 |
| `N05` T015759Z（对照） | 2 | **4921** | `(-6.0,-2.0)` → `(5.082,2.586)` | **正常在动** |
| `N06` T015759Z（对照） | 2 | — | `(-6.0,-2.0)` → `(5.01,2.634)` | **正常在动** |

**同一个 N07 运行里的 odom（出生点相对）累计路程**：`r01 37.15 m`、`r02 31.17 m`、`r03 26.28 m`。
**车确实开了 26–37 米。真值流说它们一毫米都没动。**

⇒ **机理修正为**：三车时真值流**只递出一帧，之后永远重复那一帧**。
这**不是**"队列排空太慢拿到陈旧值" —— 一个真在被排空的队列**迟早会前进**，
而这里是 8738 个连续样本**完全相同**。**发布端在第一次之后就没有再产生新的位姿。**

⇒ **两车的到达率（52.65 Hz）与三车（0.13–2.75 Hz）的 400 倍差距仍然成立**，
但它是**症状**，不是机理。真正的机理是**三车时 `SceneBroadcaster` → `ros_gz_bridge`
这条链路只成功递出了一帧**，`truth_messages` 记的那些数是**零星漏过来的**。

**✅ 因此仍然成立的结论**：
* ✅ N07 三车的 `safety = PASS/PASS/PASS` **不可引用** —— 真值里三台车一步没动，
  对真值的检查**只能在真值里发生**。
* ✅ 三车真值通道**目前不可用**，任何三车用例在它修好之前**没有可引用的安全判决**。
* ⛔ 不能说"发布端被 CPU 饿死" —— **未测**；`samples` 仍稳在 20 Hz，说明订阅者有余力，
  但发布端为什么只出一帧**本轮没有探针**，属于**未归因**。
* ⛔ 不能说深度 2000 修好了它 —— **只在 2 车验过**，而 2 车本来就没坏。

**下一步（未做）**：给 `SceneBroadcaster` 的输出加一个**独立探针**
（`gz topic -e /world/warehouse/dynamic_pose/info` 在 3 车跑时计数），
把"发布端没发"和"桥没桥过去"分开。**在那之前，3 车结论一律不引用。**


**怎么发现的**：`D-P17-07` 修好归因之后，N07 三个种子从 `UNKNOWN` 变成了**真实判决**
（`safety = PASS/PASS/PASS`，`batch_20260918T103152Z`）。但把那份录制**当成地图看**就会发现：

| 量 | 值 |
|---|---|
| 样本数 | 9141 |
| 含真值的样本 | 8738 |
| **某个刚体位移过的样本** | **0** |
| 槽位数 | **24**（三车 × 八连杆，**消息是完整的**） |

**24 个槽位、8738 条样本、零位移。** 三台车在整段录制里**没有一个动过**。
把三个模型帧的坐标打出来，它们就**钉在各自的出生点**：`(-6.0,-2.0)`、`(-6.0,0.6)`、`(6.0,-2.0)`。
而**同一份录制里的 odom 是在动的**（r02 走了 10.6 m）。

⇒ **`safety = PASS` 是对着一个不动的世界判出来的。** 这一条**推翻了本项目此前**对 N07 的读法：
那三行 PASS **不能**当作安全通过，因为 `no_unauthorised_entry_from_truth` 要发现的那件事
（车驶进了它没有权限的走廊），在"真值里车从没动过"的前提下**根本不可能被看见**。

**⛔ 这一段在 2026-09-19 被自己的后续测量推翻了。** 当时的推理是：宽度 24、
`truth_slots_recorded = 24`、`truth_messages` 非零 ⇒ "世界在发，停的是送达"。
**这一步推理是错的**，因为 `truth_messages` 非零**不等于**它有量级：
它在 N07 是 **60**，而同一份录制有 **9141 个样本**。

**更正后的实测**（同一批 batch，逐份读 recorder 审计）：

| 运行 | 车数 | 模型数 | `truth_messages` | 样本 | **消息/s** | 样本/s | 时长 s |
|---|---|---|---|---|---|---|---|
| `N07` T103152Z | 3 | 3 | **60** | 9144 | **0.13** | 20.01 | 457 |
| `N07_seed2` T103152Z | 3 | 3 | **641** | 9958 | **1.29** | 20.01 | 498 |
| `N07_seed3` T103152Z | 3 | 3 | **570** | 12319 | **0.93** | 20.00 | 616 |
| `N05_seed1` T015759Z（对照，2 车） | 2 | 2 | **22547** | 8548 | **52.65** | 19.96 | 428 |

**三车真值到达率 0.13–1.29 Hz，两车对照 52.65 Hz —— 差 40 到 400 倍。**
一个 `KEEP_LAST:10` 的队列**只能丢掉已经到达的消息**；它**不可能**把 52 Hz 的输入
削成 0.13 Hz，因为削完剩下的量级仍然是"每个采样区间里有若干条"。
**到达率本身就是 0.13 Hz ⇒ 发布端没在发。**

**所以真身是**：三台车时，世界的真值发布（SceneBroadcaster → `ros_gz_bridge`
→ `tf2_msgs/TFMessage`）**几乎停摆**，而不是订阅端掉队。
载体是"三车"这件事本身 —— 2 车 52.65 Hz、3 车 0.13 Hz，而 `samples` 始终 ~20 Hz，
**说明瓶颈不在订阅者、也不在 CPU（它还有余力 20 Hz 采样）。**

**✅ 修复仍然有效，但机理要改写**：把订阅深度从 10 改成 2000
（`recorder.truth_qos()`）**确实把三车从那 0.13 Hz 的死状态里救了出来** ——
但在 `qosverify_20260919T011913Z`（**那是 2 车跑**，见 D-P17-09 的更正）
测到的 4889 条 / 452 样本，与**同样 2 车**的 `N05` 52.65 Hz **量级相当**。
⇒ 深度 2000 在 2 车配置下与默认值**无可测差异**，它在 3 车配置下是否必要
**尚未单独验证**（需要在 **3 车**上对比 depth 10 与 depth 2000，本轮的修复跑全是 2 车）。

**⛔ 现在能说的和不能说的**：
* ✅ 能说：三车录制的真值**到达率**比两车低 40–400 倍，这是**发布端**的问题，不是队列。
* ✅ 能说：N07 三车的 `safety = PASS/PASS/PASS` **不可引用** —— 真值里三台车**一步没动**，
  而对真值的检查**只能在真值里发生**（`no_unauthorised_entry_from_truth` 要发现的事，
  在"车从没动过"的前提下**不可能被看见**）。同一次录制里 odom 累计走了 **94.6 m**。
* ⛔ 不能说：真值通道"已解决"。**深度 2000 只在 2 车上验过**，3 车未验。
* ⛔ 不能说：`safety=PASS/PASS/PASS` 是通过 · "三车判得对" · "§10 已关闭" · "批测通过"。

**实测的判据 —— `samples / truth_messages`**（本机全部 batch，98 份录制）：

| 配置 | samples per truth message | 真值活性 |
|---|---|---|
| **1 车**（F07/F11/F12/F13/F14） | **0.3 – 0.5** | 活（真值**比采样还密**） |
| **2 车**（N05 ×7 / N06 / F06 / F08 / F09） | **0.3 – 0.5** | 活 |
| **3 车**（N07 ×11 / N08 ×3） | **7.2 – 152.3** | **饿死 / 冻结** |

**两个区间没有任何重叠。** 这个比值一算出来就是结论：不是"三车用例特殊"，
是**三台车把队列打爆了**。1–2 车完全不掉队，第 3 台一上线就掉两个数量级。

**根因**：`recorder.py` 的真值订阅写的是
`self.create_subscription(TFMessage, truth_topic, self._on_truth, 10)`
—— 第 4 个参数是**深度**，默认 QoS 是 `RELIABLE / KEEP_LAST / depth 10`。
而世界以 **`max_step_size 0.001`（1 ms）**步进，每台车 8 条连杆都在动，
**两条 5 Hz 采样之间产生的位姿条数远超 10** ⇒ 队列一满，只留最后几条、其余全丢
⇒ recorder 拿到的是**陈旧的位姿** ⇒ 模型帧读起来像钉在出生点。

**队列而不是"发布停了"的物证**：N07（`T015759Z`）那份**饿死但没死透**的录制里，
相邻两次采样之间**最大位移是 9.5666 m**（`N07_seed2` 是 10.62 m）。
刚体不可能在两个 200 ms 采样之间走 9.5 米 —— **能一次递出陈旧值、隔很久再递一个新值的，只有队列。**

**修法**：给真值订阅一个**显式的深历史 profile**（`recorder.truth_qos()`，`TRUTH_QOS_DEPTH = 2000`），
并把 `truth_qos_depth` 写进 recorder 的审计，这样一份录制**自己说得出它是用哪个 profile 听的**。
**只动 recorder，世界/桥/控制链路一律不碰** —— 一个会改变被观察系统的见证者，就不再是见证者。
`RELIABLE` 保留：桥是按可靠发的，深度才是那个错的旋钮。

**新增守卫 `scripts/check_truth_liveness.py`（第 10 道，接进 `check_guards.sh`）**：
它按特征把录制判成 `live` / `frozen` / `unknown` 三种，并把判据（样本数、动过的样本数、槽位数、
最大跨越）全部打出来。**`frozen` 故意不等于"没数据"**：没数据会让真值类检查全 `NOT_RUN`（诚实），
冻结会让它们 `PASS`（不诚实）。

**⚠️ 守卫的两种模式，以及为什么必须有**：
- `--all`（**取证模式**）：查全部录制。**退出 1**，实测历史上有 **22 份 frozen / 98 份**（72 live、4 unknown）。
- 默认（**门禁模式**）：**只查审计里带 `truth_qos_depth` 的录制**，即修复代码产出的那些；
  修复前的**跳过并打印跳过数量**。干净树上还没有这样的录制 ⇒ **退出 0**（构建不该因为"还没跑过仿真"而红）。

**为什么门禁不查历史**：那 22 份是**修复存在之前的证据**，拿它们把今天的构建判红，
是在惩罚错的对象，而且**会训练读者忽略这道守卫** —— 那正是守卫的死法。

**✅ 这个修复已被一次三车活跑证明有效**（`reports/qosverify_20260919T011913Z`，2026-09-19）：

| | 修复前（三车） | 修复后（三车） |
|---|---|---|
| `truth_messages` | 60 – 1711 | **4889** |
| `samples / truth_messages` | **7.2 – 152.3** | **0.09** |
| 有位移的样本 | **0 / 9141** | **133 / 448** |
| 最大单步位移 | 9.5666 m（物理上不可能） | **0.0483 m**（一步 200 ms 的正常值） |
| 判决 | FROZEN | **live** |

审计里记下了 `truth_qos_depth = 2000`。比值从 152 掉到 **0.09**，即真值现在**比采样快 11 倍**送达，
而不是被饿死两个数量级。**真值通道的送达问题已解决。**

**⛔ 修复有效 ≠ 三车的判决现在可信。** 修复前产出的**任何**三车录制，其真值一律不可用
（N07 的 `safety=PASS/PASS/PASS` 仍然不是通过）。修复之后的三车录制**通道是好的**，
但用例**还没有在修复后的录制上重跑过** ⇒ **到今天为止，没有任何一份三车的安全判决是可引用的。**

**⛔ 仍然不能说**：N07 的 `safety=PASS/PASS/PASS` 是通过 · "三车判得对" · "§10 已关闭" · "批测通过"。

---

## D-P17-09 — 【已撤回】`spawn_r02` 段错误与"模型没进世界"的推论（2026-09-19）

**⛔ 这一条整条撤回。它的结论来自一个把 2 车跑当 3 车读的错误。**

**怎么写出来的**：证明 `D-P17-08` 的修复时跑了一次活跑（`reports/qosverify_20260919T011913Z`，
452 样本、448 条真值、**133 个样本有位移**）—— 通道**确实活了**。但那份录制只有 **16 个槽位**，
而三模型的世界在本项目里从来是 **24**。于是写下："少了一台车，`spawn_r02` 段错误。"

**错在哪**：**那次跑根本不是三车。** `fleet.launch.py` 的 `robots` 参数
**默认值是 `r01,r02`**（`fleet.launch.py:332`，注释写明"Two is the P4 acceptance set"），
而那次验证命令**没有传 `robots:=`**。**2 车 × 8 连杆 = 16 槽位 —— 完全正确，没有缺车。**
守卫的算术一直是对的（`slots // LINKS_PER_MODEL` 对比 recorder 的 `robots`）；
**错的是喂给它的"应有车数"**：那次活的审计里 `robots = ['r01','r02','r03']`，
而**世界里只有两台**，于是它按 3 车去比 16 槽位，报 SHORT。

**而三车的真实录制里，三台车都在**：`N07` T103152Z 的
`truth_slots_recorded = 24`、`truth_slots_recorded 24` 与逐槽打印的
**24 个槽位 / 3 个模型**（`model index per slot = [0, 1, 2]`），
三个刚体帧**各自钉在自己的出生点**：`(-6.0,-2.0)`、`(-6.0,0.6)`、`(6.0,-2.0)`。
**没有缺车。**

**那个段错误本身**：`[ERROR] [create-49]: process has died [pid 1674, exit code -11, ... -name r02 ...]`
是**真实见过的日志**，但它是**另一次跑**（round 17 早期）留下的，**与本推论无关**。
本轮**未能复现**：一次同构的三车启动里，
`spawn_r01` 与 `spawn_r02` 都打印 "Entity creation successful." 并 **cleanly 退出**
（`process has finished cleanly`，**没有 -11**），r03 则因为**默认 `robots:=` 不含它**
而根本没被启动。⇒ **`ros_gz_sim/create` 的段错误仍然是未复现、未定因的偶发**，
**不构成本轮任何结论的依据。**

**⚠️ 但有一个真问题被这条错误结论**顺带**发现了**，它才是该记下来的：

`robots:=` 的**默认值（`r01,r02`）与用例声明（`runners: [r01,r02,r03]`）不是同一个来源。**
N07 与 N08 的用例都声明 `runners: [r01, r02, r03]`（`config/scenarios/regression_v1.yaml`）。
如果某次调用**漏传** `robots:=`，launch 会**安静地只起两台**，
而 recorder 的审计、judge 的 `robots_judged`（从 odom 拿）**都会显示出 r03**，
真值里**只有 16 槽位** ⇒ **一个看起来正常、数字合理、却少了一台车的录制。**
**这正是本项目一直在防的那类失败**，而且它是**静默**的。

**修法（守卫层，保留）**：`check_truth_liveness.py` 把**槽位数当作一等事实**，
算 `models_in_world = slots // LINKS_PER_MODEL`，与 recorder 审计里的 `robots` 比对，
不足报 **SHORT** 并**退出 1**；规则写成**算术**而非硬编码 24。
**它现在报的 SHORT 是真的**（2 车世界 vs 3 车声明），只是**不是段错误的证据**。

**⛔ 待办**：`robots:=` 必须由 **`batch.py` 从用例的 `runners` 派生并显式传入**
（已确认 `batch.py:195` 写的是 `robots:={','.join(self.runs)}`），
并加一条**静态守卫**：用例的 `runners` 与 `robots:=` 的取值**必须同源**。
本轮**未做**。

---

## D-P17-10 — `check_guards.sh` 十道守卫**一道也拦不住构建**：`set -uo pipefail` 从来没打开过 pipefail（2026-09-19）

**怎么发现的**：把 `check_truth_liveness.py` 修成"发现 SHORT 就 `exit 1`"之后，
**守卫自己单跑返回 1**（已验证），**但从 `check_guards.sh` 里跑就变成 0**。
顺着查下去发现的是**比那一条守卫大得多的问题**。

**两个独立缺陷，叠在一起让整道门失效：**

### 缺陷一：`set -uo pipefail` 不是 `set -o pipefail`

`scripts/check_guards.sh:27` 写的是

```bash
set -uo pipefail
```

**这一行不开启 pipefail。** `set` 的参数被解析成 `-u`、`o`、`pipefail` 三项：
`-u` 生效，`o` 被当成一个**位置参数**，`pipefail` 被当成**普通参数忽略**。
正确写法只有一种：`set -u -o pipefail`（或分成两行）。

实测（本机，2026-09-19）：

| 调用方式 | 真实退出码 | 观察到的 |
|---|---|---|
| `/usr/bin/python3 -c 'sys.exit(1)'` | 1 | **1** ✅ |
| `/usr/bin/python3 -c 'sys.exit(3)'` | 3 | **3** ✅ |
| `/usr/bin/python3 script.py`（`sys.exit(7)`） | 7 | **7** ✅ |
| `raise SystemExit(4)` | 4 | **4** ✅ |
| `( exit 5 )` 子 shell | 5 | **5** ✅ |
| `false` | 1 | **1** ✅ |
| **`python3 script.py 2>&1 \| sed 's/^/  /'`** | 7 | **0** ❌ |

**⇒ 退出码本身传递是好的，唯独"经过管道"就丢了。**
而 `guard()` 恰恰**每一条守卫都经过管道**（`| sed` 加缩进）。

### 缺陷二：`guard()` 的形状把状态交给了 `sed`

```bash
if "$@" 2>&1 | sed 's/^/  /'; then
  return 0
fi
```

管道取的是**最后一个命令**（`sed`）的退出码，**`sed` 永远成功** ⇒ `if` 永远为真。
所以 **`guard()` 永远 `return 0`，`FAILED` 永远是 0。**

**⇒ 十道守卫的结论从来没有影响过 `check_guards.sh` 的退出码。**
"build 通过"这句话此前**只证明了 colcon 编译过**，**不证明任何一道守卫通过**。
本项目此前看到过的每一行 `[FAIL]`，都是**守卫自己打印的文本**，
**不是**这个 runner 判定出来的 —— runner 从不判定。

**修法（两处都改，缺一不可）**：

```bash
set -u -o pipefail                      # 缺陷一
```

```bash
guard() {
  ...
  local rc
  set +e
  "$@" 2>&1 | sed 's/^/  /'
  rc=${PIPESTATUS[0]}                   # 缺陷二：取守卫自己的码，不取 sed 的
  set -e 2>/dev/null || true
  if [ "${rc:-1}" -eq 0 ]; then return 0; fi
  printf '  [FAIL] %s (exit %s)\n' "$label" "$rc"
  FAILED=1
  return 1
}
```

`"${rc:-1}"`：**取不到状态按失败算**，不按成功算。
这正是本项目反复吃的那类默认值错误（"缺失"被当成"没问题"）。

**自检（写进流程，不是靠人记得）**：修完必须**注入一个必失败的守卫**（`exit 7`）
跑一次，确认 runner **报 FAIL 且退出非 0**；再恢复跑一次确认退出 0。
**一个不能被证明会失败的检查，等于没有检查。**

**⛔ 这条的影响范围**：`D-P17-08` 关于"守卫已能拦住冻结真值"的说法**在此之前是不成立的**
—— 守卫能**发现**，但**拦不住**。修完才成立。


## D-P17-11 — 三车真值通道：**只递一帧**，以及它今天第一次真的活过来了（2026-09-19）

**这一条合并了两件事**：把 `D-P17-08` 的机理**第二次**修正到实测上，
以及记录**三车真值第一次被观测到是活的**。

### 一、机理（第三次、也是目前唯一站得住的版本）

判据不再是"到达率"，而是**槽位 0 的"相邻去重"计数** ——
槽位 0 是模型帧、世界坐标；**槽位 5–9 是连杆偏置，换算成距离会得到假的"车走了 5.8 m"**
（这正是上一节犯的错）。

| 运行 | 车数 | 槽位 0 相邻去重 | 首 → 末 | 判定 |
|---|---|---|---|---|
| `N07` T103152Z | 3 | **1** / 8738 样本 | 钉在出生点 | 一帧都没变 |
| `N07_seed2` T103152Z | 3 | **1** / 9609 | 钉在出生点 | 一帧都没变 |
| `N07_seed3` T103152Z | 3 | **1** / 12110 | 钉在出生点 | 一帧都没变 |
| `N08_seed2` | 3 | **1** / 38679 | 钉在出生点 | 一帧都没变 |
| `N05` T015759Z（对照） | 2 | **4921** / 8236 | `(-6,-2)` → `(5.082,2.586)` | 正常 |
| `N06` T015759Z（对照） | 2 | 3921 / 5755 | `(-6,-2)` → `(5.01,2.634)` | 正常 |

**同一份 `N07` 录制里的 odom（出生点相对）**：`r01 37.15 m`、`r02 31.17 m`、`r03 26.28 m`。
**车确实开了 26–37 米；仲裁者说它们一毫米没动。**

⇒ **"队列陈旧"和"发布端饿死"两种说法都不成立。**
一个在被排空的队列**迟早会前进**；一个饿死的发布端**不会把同一帧重复 8738 次**。
真正的现象是：**三车时链路只成功递出一帧，之后永远重复那一帧。**

### 二、gz 侧探针（独立证据，机理仍未归因到那一层）

在 3 车跑动期间用 `gz topic -i` 反复探 `/world/warehouse/dynamic_pose/info`：

```
Publishers [Address, Message Type]:
  tcp://169.254.83.107:44741, gz.msgs.Pose_V
No subscribers on topic [/world/warehouse/dynamic_pose/info]
```

**世界在发；桥在 gz 侧没有订阅者。** 而在同一次跑的 14 次活体轮询里，
**订阅者只出现了 3 次**（poll 13/15/16），其余 11 次缺席：

```
poll  3 -   4 -   5 -   6 -   7 -   8 -   11 -   12 -
poll 13 SUB 14 -  15 SUB  16 SUB  17 -  18 -
```

⇒ **桥的 gz 侧订阅在反复断开重连。** 这是与"只递一帧"**并列**的观测，
不合并进机理 —— **根因在 `ros_gz_bridge` 还是 gz 传输层，本轮没有分离**。

### 三、★ 三车真值第一次是活的

`reports/batch_20260919T021226Z/N07_seed2`：

| 量 | 值 |
|---|---|
| 车数 | 3（`robots: [r01,r02,r03]`） |
| `truth_messages` | **7095**（上一批同一用例是 **252**，再上一批是 **0**） |
| 槽位宽度 | **24**（三模型全在） |
| 槽位 0 相邻去重 | **686** |
| 槽位 0 轨迹 | `(4.914,-1.634)` → `(-6.0,-2.0)` |

**这是本项目第一次观测到三车真值流有真实轨迹。**
`batch_20260919T015913Z/N07_seed2` 也已经 `LIVE`（distinct=23），
说明这不是孤立的一次。**使能条件尚未单独确认** —— 候选是
"深度 2000（`recorder.truth_qos()`）+ `robots:=` 由用例 `runners` 派生"
这套改动的组合，但**没有做 A/B**，所以只能说"改完之后活了一次以上"。

### ⛔ 能说与不能说

* ✅ 能说：三车真值此前**只递一帧**（不是队列、不是饿死），证据是槽位 0 的去重计数。
* ✅ 能说：同一份录制里 odom 走了 26–37 m，所以"车没动"是**测量假象**。
* ✅ 能说：桥的 gz 侧订阅**在 3 车里反复断开**（14 次活体轮询只 3 次在）。
* ✅ 能说：改完 `truth_qos` + `robots:=` 之后，**三车真值至少活了两次**。
* ⛔ 不能说：修好了。**没有一个失败的注入用例**，也没有 A/B；**下次可能又冻**。
* ⛔ 不能说：`batch_20260918T103152Z` 的三车判决可以引用了 —— 那份录制**仍然是冻的**。
* ⛔ 不能说：闸门就没问题了 —— 三车真值活了**只解决"看得见"**，不解决规划/预算是非。

### 待办（未做）

1. **A/B 确认使能条件**：三车固定，只切 `TRUTH_QOS_DEPTH`（10 vs 2000），各跑 3 次。
2. **给 gz 侧订阅加观测**：把 `gz topic -i` 的订阅者计数写进 recorder 审计，
   这样"桥掉了"是**录制自己说得出**的事，而不是要人手工轮询。
3. **`check_truth_liveness.py` 的判据要改用槽位 0 去重计数**，而不是"动过的样本比例"：
   前者对单帧重复是 1，对真运动是数千；后者在 24 槽位下会被连杆偏置干扰。

## D-P17-12 — 门禁真正据以行动的位姿，现在记下来了；**它推翻了我们关于"安全余量"的前提**（2026-09-19）

**这是第 5 项（"给安全门装位姿记录仪"）的落地记录，也是第 6/7 项的前置。**
前置的结论是 §"②-b 卡在仪器上"：**「门禁据以行动的位姿」没有任何录制记下来**。
现在有了。

### 一、仪器：`_pose_fields()`，33 个键，写进每一条样本

`src/fleet_ros/fleet_ros/gate_node.py` 的 `_pose_fields()` 把"位姿那一半"收成**一个 dict**，
理由写在 docstring 里：**不能让一个 payload 把"来源 A 的年龄"和"来源 B 的位置"并排报出来**
—— 那个形状读起来像"一个事实"，其实是**两个**。

落到 `samples-<CASE>.jsonl` 的 `gate.<robot>` 里，一共 **33 个键**，其中位姿相关的 11 个：

| 键 | 含义 |
|---|---|
| `pose_source` | 这次据以行动的是**哪个**来源（`localiser` / `spawn_odom`） |
| `pose_usable` | 这个来源**现在能不能用**（门禁会不会据此下达动作） |
| `pose_trusted` | 实际采用的位姿 |
| `spawn` | 出生点（用于分离帧口径） |
| `pose` | 合成位姿（`spawn ⊕ odom`），**仅上报，不用于决定** |
| `localiser` | AMCL 位姿，**仅上报，不用于决定** |
| `localiser_age_s` | AMCL 位姿的年龄 |
| `localiser_moved_m` | 该位姿到达之后机器人移动了多少 |
| `localiser_messages` | 收到过几条 |
| `localiser_verdict` | **判不了的那句话**（始终带数字） |
| `pose_disagreement_m` | 两个来源之间的距离 |

`pose_disagreement_m` 存在的理由写在代码注释里：**"the disagreement is the measurement
that motivated the switch, and it would be invisible if only the chosen pose were published."**
这正是本项目那条硬教训（"记录证据时要同时记下它是什么、哪个帧、哪个来源"）的代码化。

### 二、★ 仪器第一次读数，就否掉了余量推导的前提

**第一条读数：`pose_source = localiser`，全部运行、全部样本。**

⚠️ 这**推翻了此前的一个推理前提**。`gate_node.py:177` 的**参数默认值是
`POSE_SOURCE_SPAWN_ODOM`**，但 launch 把它覆盖成了 `localiser`
（`fleet.launch.py:214`）。**照默认值推理得出的"门禁用的是合成位姿"是错的。**

**第二条读数（★ 这条是本节的重点）：声明的不合格原因，压倒性的是"位姿从没到过"，而不是"位姿太旧"。**

`judge_localiser` 只有三个出口，`usable=False` 有**两种不同的形状**：

| 出口 | 条件 | 出现次数（N07_seed3） |
|---|---|---|
| `usable=True` | `age ≤ 2.0 s` | 56 |
| `usable=True` | age 超了**但** `moved ≤ 0.10 m` | 1329 |
| **`usable=False`（形状 A）** | **`age_s is None` —— 一条 `amcl_pose` 都没到过** | **44608** |
| `usable=False`（形状 B） | `age > 2.0 s` **且** `moved > 0.10 m` | 0 |

实测：

| 录制 | 不合格占比 | 主导原因 | `localiser_age_s` |
|---|---|---|---|
| `T022011Z/F01` | **1521/1521 = 100%** | no localised pose has arrived | `n/a`（从没到过） |
| `T022011Z/F05` | **2076/2076 = 100%** | 同上 | `n/a` |
| `T021520Z/N05_seed2` | **2288/2288 = 100%** | 同上 | `n/a` |
| `T015913Z/N07_seed3` | 49446/59457 = 83% | 同上（44608） | p50 **67.99 s** · p95 **360.90 s** · max 363.32 s |

**⇒ 后果，直说：**

1. **余量（`max_moved_m = 0.10`、"0.60/0.62 m"那套）描述的是一个 0 次触发的事件。**
   主导的失败形状是**"没有任何消息"**，而**"位移多少"对一个没到的消息没有定义**。
   拿位移去定余量，**定的是一个不发生的故障的余量**。
2. **N07_seed3 里位姿到了的那部分，年龄 p50 = 68 s、p95 = 361 s**，对 `max_age_s = 2.0 s`
   是**两个数量级**。也就是说即使消息到了，也**几乎全程不合格**。
3. **`pose_usable=False` 全时段，但车照样在跑、任务照样在判。**
   这不是 bug —— `_trusted_pose()` 的 docstring 写明：不合格时**故意不拿合成位姿顶替**，
   而是让交通层以 `LOCALIZATION_STALE` 拒绝。**"拒绝"本身就是被测的行为。**
   ⚠️ 但因此：**F01–F08 的 `pose_usable=100% False` 不能读成"这些用例坏了"**，
   也不能读成"定位没问题" —— 它是**注入故障下的预期拒绝**。

### 三、这改变了 ②-b 的定义（未改代码）

原计划（第 7 项）：*"按新参数重测定位误差，再定余量（`align_*`/`release_*`/`max_moved_m`/SDF 标记一起动）"*。
**现在必须先回答一个问题**，否则余量改的还是一个不发生的故障：

> **`amcl_pose` 为什么在注入用例里整场不到、在三车用例里到了却陈旧到 68–361 s？**

在这一点查清之前，**把 `max_moved_m` 从 0.10 调大或调小都不是修复**：
它只会改变**形状 B（0 次发生）**的阈值，而**形状 A（主导）根本不是阈值问题**。

⚠️ 与 `D-P15-01`/`D-P17-06` 的关系：那一条说 `amcl_pose` 是**运动门控**的，
判据 `update_min_d 0.25 > max_moved_m 0.10` 构造上互不相容。
**本条不推翻它**，但指出：**"运动门控"解释不了 `age_s is None`（一条都没到）**。
门控会让消息**变稀**，不会让它**不存在**。⇒ 形状 A 另有原因，**尚未查**。

### ⛔ 这一节能说与不能说

* ✅ 能说：门禁的位姿、来源、可用性、判不了的原因，**现在每条样本都记下来了**（33 键）。
* ✅ 能说：门禁实际用的是 **`localiser`（AMCL）**，不是 `spawn_odom` 默认值。
* ✅ 能说：不合格的**主导原因是"位姿从没到过"**，不是"位移太大"。
* ✅ 能说：`max_moved_m` 那套余量的**目标事件在实测里发生 0 次**。
* ⛔ 不能说：定位坏了。**先要分离"注入故障导致的预期拒绝"与"通道本身的缺陷"。**
* ⛔ 不能说：余量已修好 / 余量是错的 —— **余量改动的方向取决于"形状 A 的根因"，那条没查。**
* ⛔ 不能说：F01–F08 通过。**它们在这一节里只提供了仪器读数，没有提供判决。**

### 待办

1. **查形状 A**：`amcl_pose` 在 `f_fault_triggers` 下为什么整场不发布
   （候选：注入的是同一个 TF/定位链路；`amcl` 起没起；topic 名/命名空间）。
2. **查形状 B**：三车里 `age` 为什么能到 68–361 s（候选：AMCL 更新被运动门控停住）。
3. 两条都查清**之后**再动 `max_moved_m` / `align_*` / `release_*`。

## D-P17-13 — 两个"没通过"不是一回事：`FAIL` 与 `NOT_RUN` 各有各的含义（2026-09-19）

**这一条回答两个只读问题**（第 3、4 项），两个问题的共同形状都是
**"同一个字段/检查名，两种不同的'不通过'"**。

### 一、`driver_claims_corroborated_by_truth`：N07 是 FAIL、A/B 组是 NOT_RUN

**判据只有一行**（`judge.py`）：

```python
verdict = FAIL if contradicted else (NOT_RUN if (unchecked or not checked) else PASS)
```

| 判决 | 触发条件 | 含义 |
|---|---|---|
| **`FAIL`** | `contradicted` 非空 | **有**司机声称 `complete=True`，而真值**不同意** ⇒ 真矛盾 |
| **`NOT_RUN`** | `unchecked or not checked` | **一条声称都没收到** ⇒ **这个检查没有做** |
| `PASS` | 有声称、且全部被真值证实 | 做了，且通过 |

**为什么 A/B 组必然是 `NOT_RUN`**：声称的来源是 `runtime/crossings/*.json`，
由 `harvest_crossing_claims()` 收集 —— **只有穿越用例才产生这个文件**。
`batch.py` 的 docstring 把话说死了：

> *"Non-crossing cases have no such report, so `driver_claims_corroborated_by_truth`
> stays `NOT_RUN` for them, which is the correct answer: it was not performed."*

⚠️ **而且这本身是一个已经修掉的缺陷**，注释里留着现场：

> *"`checked == 0` happened on every case of the regression batch ... and the formula below
> used to answer PASS, with the detail line next to it reading '0 corroborated'.
> **A check that looked at nothing has not passed; it has not been performed.**"*

⇒ **`NOT_RUN` 是诚实答案，不是退化。** 而 **N07 的 `FAIL` 是一个真矛盾，值得追**
（N07 是穿越用例，所以它有声称、且真值不同意）。

### 二、N08 seed1 的 `truth_attribution` 与 `judged` 并不自相矛盾

两个字段回答的是**两个不同的问题**：

| 字段 | 来源 | 回答的问题 |
|---|---|---|
| `robots_judged` | **用例/清单里声明的** `runners` | 这个用例**是关于哪些车的** |
| `truth_attribution` | `attribution["ok"]` | 真值里的槽位**能不能被认到这些车上** |

实测 N08 三个种子：

| 用例 | samples | `truth_has_heading` | `truth_attribution` | margin | 槽位映射 |
|---|---|---|---|---|---|
| **N08** (seed1) | 7856 | **False** | **NOT_RUN** | — | — |
| N08_seed2 | 38960 | True | PASS | 2.6 m | `{0:r01, 1:r03, 2:r02}` |
| N08_seed3 | 7711 | True | PASS | 2.6 m | `{0:r01, 1:r03, 2:r02}` |

seed1 的 detail 一句话说清：**`"the recording contains no truth positions at all"`**
—— **零个真值位置**。所以判官说"认不出来"，同时仍然报告"这个用例覆盖三台车"。
**这不是矛盾，是"用例范围"与"证据可得性"的分离。**

而且它**反过来印证了 `D-P17-08`/`D-P17-11` 的冻结**：seed1 有 7856 条样本却
`truth_has_heading=False` ⇒ **它就是那批"冻的/空的"录制之一**；
seed2 / seed3 是活的，所以能给出映射。

⚠️ **一个已知未查的小异常**：N08_seed3 的 `continuity_violations = 1`（共 7711 样本）。
该检查的用途写在代码里：**"A reordered stream would otherwise swap two trajectories with
no other symptom."** 单次计数 = 某一帧里两台车距离最近槽位不再各自唯一。
**映射没换（`ok=True`、mapping 与 seed2 相同）**，所以读作**擦肩**，不是**互换**。
**未查**它具体是哪一帧、是不是真值通道抖动的那一帧。

### 三、`retreat_policy.py` 有策略、没发出方（第 8 项的一部分）—— 诚实的"没做完"

`D-P14-02` 是"撤退指令的**发出方**（只写了一半）"。实测：

| 部分 | 状态 |
|---|---|
| 策略（`needs_retreat` / `retreat_command` / `progress`） | ✅ 已实现 |
| 测试（`tests/test_p14_retreat_policy.py`） | ✅ 通过 |
| **调用方（`fleet_ros` 里谁来发这条命令）** | ❌ **全仓零调用点** |

模块自己的 docstring 就把缺口写在第 1 行：

> *"The retreat command emitter: **the missing half of `D-P4-12`**."*
> *"**If nobody issues a retreat command, the widest permit in the world moves nothing.**"*

⇒ **这是"函数存在但没人调用"的第 N 次**（与 `D-P17-05` 的回调、`D-P10-03` 同形）。
**在发出方接上之前，撤退能力在成因上等于不存在。**

### ✅✅ 同时确认已完成的（第 8 项的其余部分）

| 项 | 证据 |
|---|---|
| **`D-P17-02` 持货低电** | `fleet_core/loaded_battery_policy.py` 存在；`task_service_node.py:1447 _service_loaded_battery()` 调用 `decide_loaded_battery(...)`；记录在两处上报（`:497`、`:1846`）；`task_service_node.py:1394` 写明策略 **"carrying cargo -- do NOT auto-unload. Park at a legal, reachable position"** |
| **阶段 2 六个能力** | `docs/LIMITATIONS.md` 七个 prereq 全部标 **IMPLEMENTED (round 17, stage 2); retired from this list** |
| **11 个阻塞用例** | 全部解锁 ⇒ **`cases: 38  runnable: 38  blocked: 0`** |
| **`KNOWN_ACTIONS`** | 18 个动作，`_BASE_ACTIONS`(4) ∪ `_STAGE2_ACTIONS`(13+)，由 `test_stage2_capabilities.py` 断言一致性 |

### ⛔ 这一节能说与不能说

* ✅ 能说：`FAIL` = 有声称且被真值否掉；`NOT_RUN` = 没有声称，检查没做。
* ✅ 能说：A/B 组的 NOT_RUN 是**设计如此**，并且曾是一个"空检查报 PASS"的缺陷、现已修。
* ✅ 能说：`judged` 与 `truth_attribution` 是正交的两个量，seed1 不同时不矛盾。
* ✅ 能说：`D-P17-02` 与阶段 2 六个能力**已完成**，11 个用例**已解锁**。
* ✅ 能说：`retreat_policy` 的**发出方缺口仍然存在**。
* ⛔ 不能说：N07 的 `driver_claims` FAIL 已被处理 —— **它还没查**。
* ⛔ 不能说：`continuity_violations=1` 已解释完 —— **只解释了判据，没定位那一帧。**

## D-P17-14 — 判活用的判据**漏判**了冻结：`moved` 说"活"，而槽位 0 说"根本没动"（2026-09-19）

**这是 `D-P17-11` 待办第 3 条的实际后果，而且它比"待办"严重 —— 它是一个漏判。**

### 一、实测：11 个录制全判 `live`，其中 5 个的槽位 0 是 `distinct = 1` 到 `3`

`scripts/check_truth_liveness.py --root reports/batch_20260919T022011Z` 的判决：

```
recordings: 11   live: 11   FROZEN: 0   unknown: 0   short: 0
RC=0
```

但把**槽位 0 的相邻去重计数**单独拿出来看（`LINKS_PER_MODEL = 8`，
所以槽位 0–7 = 模型 0，8–15 = 模型 1，16–23 = 模型 2）：

| 用例 | 槽位0 | 槽位1 | 槽位2 | 槽位1–7 | 守卫判决 | 真相 |
|---|---|---|---|---|---|---|
| F01 | **1014** | 1014 | 1021 | 628–769 | live | **真的活** |
| F05 | **1** | 1017 | 1017 | 1017 | live | ⚠️ **模型 0 的身体冻住** |
| F07 | **1** | 1025 | 1025 | 1025 | live | ⚠️ 同上 |
| F11 | **1** | 990 | 990 | 990 | live | ⚠️ 同上 |
| F06 | **1** | **1** | 957 | 957 | live | ⚠️ **两个身体冻住** |
| F09 | **1** | **1** | 967 | 967 | live | ⚠️ **两个身体冻住** |

**F01 的健康形状**（对照片）：槽位 0/1/2 **全都动**，槽位 0 的轨迹是
`(4.914,-1.634) → (-6.0,-2.0)` —— 有真正的位移。

**F05 的冻结形状**：槽位 0 的**首末都是 `(-6.0,-2.0)`**，2232 个样本一次没变；
而槽位 1–7（**同一个模型的连杆**）动了 1017 次。

### 二、★ 为什么这个形状以前看不见 —— 轮子在转，身体不动

`LINKS_PER_MODEL = 8` 把 24 个槽位切成 3 个模型 × 8 个连杆。
**槽位 0 是模型的世界位姿（root body），槽位 1–7 是模型内部的连杆偏置。**

**轮子的转角来自 `/joint_states`，与模型的世界位姿是两条独立的流。**
⇒ **身体位姿冻住的时候，轮子照样在转**，于是：

* 守卫的 `live` 判据用的是 **`moved`**（"有多少样本里有*任何*槽位动过"），
  24 个槽位的轮子提供了**充足**的 `moved` 样本 ⇒ **判成 `live`**。
* 而 `slot0_distinct` 才是"这个模型在真值里到底动没动"。

**这两件事不一样，守卫现在用的是错的那一件。**
`D-P17-11` 的待办第 3 条原文已经写明了应该换成槽位 0 去重计数 ——
**本条给出"为什么必须换"的实测证据**：不换的话，**5 个冻结录制会被判成活。**

### 三、与 `D-P17-08` 的关系：同一个病，两种表现

`D-P17-08` 记的是**整条流只递一帧**（`slot0_distinct = 1` 且**全部**槽位 = 1）。
本条记的是**局部冻结**：**槽位 0 冻、槽位 1–7 不冻**。

⇒ **`D-P17-08` 的判据（"全部槽位都=1"）也漏判本条这种形状。**
判据必须逐模型看**槽位 0**，而不是看整条流的"动过比例"。

### 四、这条对已记录的判决意味着什么

⚠️ **F05 / F06 / F07 / F09 / F11 这 5 个用例的真值通道在本次批次里是（部分）冻的。**
它们的 `safety` 判决**建立在"一个只在里程计里移动过的世界"上** ——
这正是 `D-P17-08` 那句 "*Do not read those verdicts as passes.*"

**⛔ 因此：本次批次（`batch_20260919T022011Z`）里 F05/F06/F07/F09/F11 的任何
真值派生判决都不可引用**，直到通道确认是活的。

⚠️ **但 `pose_usable` 那条读数不受影响**（`D-P17-12`）：
那个数据来自**门禁自己的 payload**（`gate.r01`），**与真值通道无关**。
⇒ `D-P17-12` 的结论**不受本条影响**，可以保留。

### ⛔ 能说与不能说

* ✅ 能说：守卫的 `live` 判据是 `moved`，**会漏判**"槽位 0 冻、连杆不冻"这种形状。
* ✅ 能说：实测 11 个录制里 **5 个**的槽位 0 是 `distinct ∈ {1,1,1,1,3}`，守卫全判 `live`。
* ✅ 能说：`slot0_distinct` 与 `moved` 是**两个不同的量**，前者才是"模型动没动"。
* ✅ 能说：`LINKS_PER_MODEL = 8` ⇒ 槽位 0 是模型 root body，槽位 1–7 是连杆。
* ⛔ 不能说：这 5 个用例失败了 —— **还没查它们为什么冻**（注入故障恰好命中定位/TF 链路？）。
* ⛔ 不能说：守卫已经修好 —— **判据的替换是待办，本条只证明它必须换。**
* ⛔ 不能说：整批的 safety 判决可以引用 —— **有 5 个的真值通道不干净。**

### 待办（本条新增，优先级高于原第 3 条）

1. **把守卫的 `live` 判据换成"每个模型的槽位 0 去重计数"**，
   而不是"整条流里动过的样本比例"。判据应是**逐模型**的。
2. **给守卫加自检**：喂一份"槽位 0 冻、连杆动"的合成录制，**必须判 FROZEN**。
   （本项目硬教训第 2 条：给检查器写自检。）
3. **查这 5 个用例为什么冻结** —— 尤其是 F05/F06/F07/F09/F11 与 F01–F04/F08/F10
   的差别是什么（注入的故障类型不同？）。

## D-P17-15 — 判活判据已改成**逐模型**槽位 0 去重计数，并**第一次给它配了自检**（2026-09-19）

`D-P17-14` 证明了旧判据会**漏判**"某台车身体冻住、轮子照转"。
本条记**修法、实测前后对比、以及自检**。

### 一、修的是什么

旧判据（`LIVE_FRACTION = 0.01`）：

```python
if moved / truth_bearing < LIVE_FRACTION:   # moved = "有任何一个槽位动过的样本数"
    return "frozen"
return "live"
```

**"任何一个槽位"就是漏洞**：24 个槽位里，槽位 1–7 是**连杆偏置**，
其轮角来自 **`/joint_states`** —— **与模型世界位姿是两条独立的流**。
⇒ **身体冻了，轮子还在转，`moved` 照样高，判成 `live`。**

新判据：**逐模型看它的槽位 0**（模型 m 的根槽位 = `m * LINKS_PER_MODEL`，
即 0 / 8 / 16），判定三分：

| 判决 | 条件 |
|---|---|
| `frozen` | **没有**任何一个模型活着 |
| `partial` | **有的活、有的没活** |
| `live` | **全部**模型都活着 |
| `unknown` | 真值样本 < `MIN_SAMPLES`(50) |
| （回退） | 一个根槽位都没见过 ⇒ 退回旧的 `moved` 比例，而不是凭空造一个判决 |

`ROOT_ALIVE_MIN_DISTINCT = 2`（= "至少变过一次"）。
`partial` 是**新增的第三态**：既不是干净跑、也不是全冻，
**折进任何一边都会把一个真实缺陷藏起来**。

### 二、实测前后对比（同一批 `batch_20260919T022011Z`，同一批文件）

| | 修前 | 修后 |
|---|---|---|
| F01–F04 | live | **live** |
| F05 | live | **FROZEN** |
| F06 | live | **PARTIAL** |
| F07 | live | **FROZEN** |
| F08–F12 | live | **PARTIAL** |
| 汇总 | `live: 11  FROZEN: 0` | **`live: 4  FROZEN: 2  PARTIAL: 6`** |

**修前 11/11 判活，修后 4 个活、8 个不合格。** 退出码从 `0` 变成 `1`。

扩到整仓：`recordings: 22  live: 12  FROZEN: 3  PARTIAL: 6  unknown: 1  short: 1`，`GUARDS_RC=1`。

### 三、★ 自检（本项目硬教训第 2 条的落实）

**这个判据是在"沉默的方向"上错的** —— 它对真实数据说 `live`，
而真实数据**正是被误判的那个东西**。
⇒ **在真实数据上跑出多少绿都不会暴露它。** 所以必须给**合成的**、答案已知的输入。

`scripts/check_truth_liveness.py --self-test` 按构造生成录制并断言自己的判据：

| 用例 | 构造 | 期望 | 实测 |
|---|---|---|---|
| 1 | 三个身体都动 | `live` | ✅ |
| **2** | **一个身体钉住、其余动、轮子转** | **`partial`** | ✅ ← **这就是 `D-P17-14` 的漏判，现在被抓住** |
| 3 | 三个身体全钉住、轮子转 | `frozen` | ✅ |
| 4 | 三个身体全钉住、轮子也停 | `frozen` | ✅ |
| 5 | 完全没有真值 | `unknown` | ✅ |

用例 2 **在旧判据下会得 `live`**（`moved` 被转动的轮子顶满）——
**它是这条缺陷的回归测试。**

**而且它已经接进 `scripts/check_guards.sh`，并且排在真值检查之前**：

```
guard "the truth-liveness criterion itself (self-test)"   "$PY" scripts/check_truth_liveness.py --self-test
guard "no recording carries a frozen truth stream"   "$PY" scripts/check_truth_liveness.py
```

⇒ **判据自己坏了，会在它"通过"任何东西之前先把构建拦下来。**
守卫数从 **11 → 12**。

### 四、这对本批 38 次的后果（重要）

⚠️ **本批多数录制是 `PARTIAL` 或 `FROZEN`。**
⇒ 它们的**真值派生 safety 判决不可引用** —— 判官读的是一个
**部分静止的世界**，而录制没有解释它为什么静止。

✅ **但 `D-P17-12` 的读数不受影响**：那批数据（`pose_source` / `pose_usable` /
`localiser_verdict`）来自**门禁自己的 payload**（`gate.<robot>`），
**与真值通道是两条独立的流**。`D-P17-12` 保留。

### ⛔ 能说与不能说

* ✅ 能说：旧判据漏判"单模型冻结"，机制是轮子来自另一条流。
* ✅ 能说：新判据是逐模型的，并在实测上把 `11 live` 改成 `4 live / 2 FROZEN / 6 PARTIAL`。
* ✅ 能说：新判据有自检，5 个用例 + 1 个回归用例，已接进 `check_guards.sh`。
* ⛔ 不能说：本批 38 次通过了 —— **守卫现在是 `exit 1`**，因为录制确实不合格。
* ⛔ 不能说：`partial` 的**原因**已查清 —— **只证明了判据能看见它，没查它为什么发生。**
* ⛔ 不能说：N04–N08 的三车判决可以引用 —— **它们仍然在被判不合格的那一类里。**

### 待办

1. **查 `partial`/`frozen` 的成因**：为什么这么多用例的某个模型身体冻住？
   （与注入的故障类型有关？与三车/两车有关？与 `robots:=` 派生有关？）
2. 成因查清后，**重跑受影响的用例**，再谈 safety 判决。

## D-P17-16 — 单台车的用例，真值流里却有**三台车**：世界里的旧模型没被清掉（2026-09-19）

**这是本批 38 次里大面积 `NOT_RUN` 的成因**，而且是**量出来的，不是推的**。

### 一、观测：`creates=1` 而 `truth_widths = [0, 8, 16, 24]`

F01 是**单台车**用例（`runners: [r01]`）。它自己的 stdout 说得很清楚：

```
009 launch: spawn pose for r01 from config/spawns.yaml: (-6.0, -2.0) yaw 0.0
[create-20] [INFO] [spawn_r01]: Entity creation successful.
```

**只有一次 `create`。** 而同一用例的录制审计：

| 字段 | 值 |
|---|---|
| `robots` | **`['r01']`** |
| `truth_widths` | **`[0, 8, 16, 24]`** |
| `truth_slots_recorded` | **24** |
| `truth_messages` | 12998（104.9 s ⇒ **~124 Hz**） |

**`LINKS_PER_MODEL = 8` ⇒ 24 个槽位 = 3 个模型体。**
⇒ **一个只生了 1 台车的世界，在真值流里报了 3 台车。**

逐用例对照（各自的 stdout）：

| 用例 | `Entity creation successful` 次数 | 声明的车 |
|---|---|---|
| F01 | **1** | r01 |
| F02 | 1 | r01 |
| F03 | 2 | r01, r02 |
| F04 | 1 | r01 |
| F05 | 1 | r01 |
| F06 | 2 | r01, r02 |

**每一个用例的真值流都是 24 槽位**，与"这个用例跑几台车"**无关**。

### 二、★ 后果：判官找不到"从 r01 出生点出发的槽位"

F01 的 `truth_attribution`：

```
verdict: NOT_RUN
detail: no slot starts on the spawn of ['r01']: slot 0 sits at (4.914, -1.634) and
        moved 12.000 m in total; slot 1 sits at (-4.919, -0.661) and moved 1.339 m;
        slot 2 sits at (-5.095, 2.508) and moved 6.000 m; slot 3 sits at (0.260, 0.205);
        and 20 further slot(s) that did not start on any spawn.
not_robot_slots: [0..23]   ← 全部 24 个
margin_m: None
```

槽位轨迹：

| 槽位 | 起点 | 终点 | 行程 |
|---|---|---|---|
| **0** | `(4.914,-1.634)` | **`(-6.0,-2.0)`** | 12.0 m |
| 1 | `(-4.919,-0.661)` | `(-4.919,-0.661)` | 1.339 m |
| 2 | `(-5.095,2.508)` | `(0,0)` | 6.0 m |
| 3–7 | 连杆偏置（`0.26`,`0.205`, …） | | ≤0.52 m |

**★ 注意槽位 0 的终点正好是 r01 的出生点 `(-6.0,-2.0)`。**
⇒ 槽位 0 是**上一个用例留下的那台车**，它跑完停在了 r01 的出生点上；
而**这个用例新生的 r01 反而没有独立的槽位**（或者说，它的轨迹与旧体混在了一起）。

⇒ 判官**正确地**拒绝：**没有任何槽位"从 r01 的出生点出发"**。
`truth_attribution = NOT_RUN` ⇒ **所有真值派生的检查一起 `NOT_RUN`**
（`no_simultaneous_occupancy_from_truth`、`no_unauthorised_entry_from_truth`、
`pose_estimate_error`、`estimate_and_truth_agree_on_containment` …）。
**这就是本批大面积 `NOT_RUN` 的形状。**

### 三、机制：`pkill` 杀了仿真器，但桥订阅的是一个"报全世界模型"的话题

真值桥的接线（`world.launch.py`）：

```python
arguments=[f"/world/{wn}/dynamic_pose/info"
           "@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V]"]
```

**`dynamic_pose/info` 报的是世界里*所有*模型**，不是"本用例声明的那些车"。

而 `scripts/batch.py:231-232` 在用例之间确实做了清理：

```python
for pattern in ("gz sim", "fleet_ros/lib/fleet_ros", "nav2_", "amcl"):
    subprocess.run(["pkill", "-f", pattern], ...)
```

**但全仓没有任何地方删除世界里的实体**（`grep remove/delete/delete_entity` 只有注释命中）。

⇒ **旧模型体在同一个 `gz sim` 进程或同一份世界状态里留存下来**，
而新用例的真值流把它们**连同新生的车一起报出来**。

⚠️ **旁证**：`truth_messages = 12998 / 104.9 s ≈ 124 Hz`。
一个正常的 3 体流应该是每个模型一条 TF，量级远低。**124 Hz 是"体比应有的多"的第二个症状。**

### 四、与 `D-P17-14`/`D-P17-15` 的关系（两件不同的事）

| | `D-P17-14` / `D-P17-15` | **本条** |
|---|---|---|
| 症状 | 槽位 0 的**去重计数**很低（1–3） | **槽位数 = 24，但用例只要 1–2 台** |
| 判据 | 逐模型看槽位 0 动没动 | 槽位数 vs 用例声明的车数 |
| 后果 | 真值"看起来在动但没动" | 判官**认不出**哪台是哪台 ⇒ 全 NOT_RUN |

**两条都是"录制的世界与用例的世界不一致"**，只是不一致的方式不同。
⚠️ **两者可能同源**：如果旧体停在原地、新体在动，
那么"槽位 0 冻住"完全可能**就是旧体占了槽位 0**。
**这条没有查**，不能断言。

### ⛔ 能说与不能说

* ✅ 能说：单台车用例的真值流报 3 个模型体（`truth_widths = [0,8,16,24]`）。
* ✅ 能说：判官因此把**全部 24 个槽位**列为 `not_robot_slots`，`truth_attribution = NOT_RUN`。
* ✅ 能说：这是**正确**的拒绝 —— 它没有拿猜的映射去判。
* ✅ 能说：全仓没有删除世界实体的代码路径。
* ⛔ 不能说：旧体就是根因 —— **`pkill` 与"实体残留"之间没有直接观测**，
  本轮只观测到"槽位数与用例不符"这个**结果**。
* ⛔ 不能说：`truth_messages` 的 124 Hz 已解释 —— **只指出它是旁证。**
* ⛔ 不能说：`D-P17-14` 的冻结与本条同源 —— **没查。**

### 待办（优先级最高）

1. **给真值桥的订阅加一条"世界里有几个模型"的观测**，
   并把它和用例声明的车数**写进同一份审计**（这样不一致是**录制自己说得出**的）。
2. **查 `pkill "gz sim"` 之后实体的实际存活情况**（在两次用例之间直接探 gz 侧）。
3. 决定修法：**每个用例重建世界**，还是**按用例声明的车数过滤真值流**。
4. **在此之前，本批 38 次的任何真值派生判决都不可引用。**


## D-P17-17 — D-P14-02's retreat emitter is wired: the policy had no call site

`fleet_core/retreat_policy.py` was complete, documented, and unit-tested by
`tests/test_p14_retreat_policy.py`, and it had **zero call sites**. `D-P17-13` recorded
this; `D-P14-02` had already written down what it costs:

> If nobody issues a retreat command, the widest permit in the world moves nothing.

The refusal zeroes the velocity, so a robot that has lapsed inside a region it cannot
legally leave sits there for ever. The project observed that repeatedly and read it as a
navigation fault.

**Where the emitter now sits, and why exactly there.** Immediately after `acquire`
succeeds and before `stage_cross`:

* before `acquire` there is nothing to retreat *from* — a robot at its wait node holds no
  permit and is not inside a rectangle it needs one for (`resources.yaml` states the
  alignment nodes are placed far enough back that a stopped robot's footprint plus margin
  clears every rectangle);
* after `stage_cross` is too late — `stage_cross` drives to the exit pad, which the same
  file says in capitals is *"still INSIDE the bundle"*, so a robot that cannot get to
  *that* pad is precisely the robot this policy is for.

It is called only from the `stage_cross` failure path, not from `stage_release`: the
release node is outside every rectangle, so a robot that fails to reach it is inside
nothing.

**Why landing the emitter alone is safe.** The retreat is sent as an **ordinary Nav2
goal**. It passes through the robot's own safety gate, which holds its own geometry and
can refuse it. So this stage can only ever produce a refusal or a legal motion — never
unauthorised motion. If the gate refuses, the *permit* half of `D-P14-02` is what is
missing, and the report says so in those words, under a new stop reason
`STOP_RETREAT_REFUSED = "retreat_refused"`. Reusing `run_deadline` would have sent the
reader to this driver's budget when the answer is a design gap.

**A pose source had to be added.** `staged_crossing.py` previously had **none** — it drove
by issuing goals to named nodes and never asked where the robot was. That is fine for
driving and useless for a question that is entirely about position. So a subscriber was
added to `amcl_pose` **relative to the robot's namespace**, i.e. the same topic and the
same source the gate trusts for its own pose. Deliberately the same source: `D-P10-02`
records what a second localisation chain costs, where a composed `spawn ⊕ odom` pose
carried an error that was a **rate**, 0.0809 × distance travelled (R² 0.92).

The subscription is created **lazily**, on the first failure that needs it, so the
ordinary crossing path pays nothing. When no message has arrived, `_latest_pose` stays
`None` and the stage reports **"no localised pose has arrived"** rather than pretending the
robot is outside every rectangle — the same distinction `D-P17-12` measured as the
dominant reason the gate cannot trust a pose (44 608 / 49 446 samples in N07_seed3).

**The `recoveries=8` guard, carried over.** `retreat_policy.progress` is consulted before a
**second** retreat is issued. A retreat measured not to reduce the overlap is not
re-issued. That is the same unbounded retry loop the project spent a round chasing under
the name `recoveries=8`, and the fix then was the same: notice, and stop.

**Report fields added**, so "we looked and the robot was fine" is separable from "we never
looked": `retreat_pose_messages`, `retreat_overlap_m`, `granted_resources`.

## D-P17-18 — `self.movement_stages` was read for days and never assigned

`staged_crossing.py` read `self.movement_stages` at the time-out branch:

```python
if self.expected_passage_s > 0.0 and self.movement_stages > 0:
    stage_expected_s = self.expected_passage_s / self.movement_stages
```

**No line anywhere in the repository assigned it.** `grep -rn movement_stages src/` returned
exactly two hits: both are this read.

It was invisible to every instrument this project has, and the reason is the `and`:

* **Python short-circuits.** The left operand is `self.expected_passage_s > 0.0`, so the
  attribute is only touched when a scenario declares an expected passage.
* **All 143 recorded crossing reports declare `expected_passage_s` 0.0** (checked across
  every `runtime/crossings/*.json` in the repository). The branch has never been entered.
* The test suite never set it either, a live run never reached it, `py_compile` and linters
  accept `self.x` as legal syntax, and in review it reads like a field set in `__init__`.

**It was armed, not harmless.** `D-P17-08`'s reserve check and `D-P17-09`'s INFRA_TIMEOUT
split both *need* a scenario to declare `expected_passage_s`. The first one that did would
have raised `AttributeError` **inside the time-out branch** — and the driver's blanket
handler reports that as `stopped_by: "exception"`, i.e. *"the driver is broken"*, when
`CONTRACTS` §9 wants *"the box was slow"*. The two want opposite next actions.

Confirmed from the one recorded run that did reach the branch
(`batch_20260917T110748Z/N01/…/p7-N01-s1-1-to_pick-0.log`): it printed `stage_wait: timed
out after 180.0s` and fell through to the `nav_status: "TIMEOUT"` return **without
crashing** — proof the guard was not taken, which is exactly what the short-circuit
predicts.

**Fix.** A module constant `MOVEMENT_STAGES = 4` (the driver moves through `stage_wait`,
`stage_align`, `stage_cross`, `stage_release`, and `expected_passage_s` describes the
*passage*). A named constant rather than an attribute, so the count cannot go missing
again.

**New guard: `scripts/check_self_attrs.py`.** AST-based, not a grep — a grep for
`self.movement_stages` finds the *read* and says nothing about whether an assignment exists
elsewhere in the file or in a sibling class. It reports every `self.<name>` a class reads
that nothing in its file assigns, annotates, or defines.

Deliberately **not** flagged, because a guard that reports 40 lines on a clean tree is a
guard that gets switched off:
* `@dataclass` fields — `@dataclass` assigns them from annotations, so no `self.x = …`
  line exists even for a correct field;
* class-level constants (`self.ACCEL`);
* `rclpy.node.Node` members (`NODE_ALLOW`, kept short — every entry is a permanent blind
  spot);
* any class inheriting from a named stdlib base whose attributes the guard cannot see
  (`UNKNOWN_BASE_ALLOW`: `BaseHTTPRequestHandler`, `Thread`, `Enum`, …). Such a class is
  **skipped entirely** rather than analysed with a hole in it.

Self-tested with 11 cases (`--self-test`), including **the exact D-P17-18 shape** and three
legitimate sources that must *not* be flagged. Wired into `check_guards.sh` ahead of the
build (guards **12 → 14**).

## D-P17-19 — `_park_candidates` did not exist, and that is why F05 could not happen

The same shape a third time, and this one is inside the branch `D-P17-02` reported as
"fully wired".

```python
if decision.action == ACTION_PARK_AND_ATTEND and state.position_known:
    park = legal_park_point(
        (state.x, state.y, 0.0),
        self._park_candidates(),                              # defined nowhere
        footprint_radius_m=float(self.cfg.footprint_margin_m or 0.30),
        reachable=self._reachable_park_candidates(state),      # defined nowhere
    )
```

`grep -rn "_park_candidates" src/` returns **two hits, both of them these call sites.**
There is no `def _park_candidates` and no `def _reachable_park_candidates` anywhere.

**Two independent guards had to hold for the `AttributeError` to fire, and both were closed
on every recorded run:**

1. `decision.action == ACTION_PARK_AND_ATTEND` — the **loaded** half of `CONTRACTS` §5.
   Every recorded `loaded_low_battery*` event in `reports/` carries `carrying: null`,
   `carried_by: null`, and `action: "cancel_and_charge"`: the **unloaded** half. The loaded
   half has never once been selected.
2. `state.position_known` — and it is inside an `and` too, so even with the right action the
   call would not be touched without a position fix.

**This is the answer to F05.** F05's expected result is *keep custody, park legally, raise
attention* — `expect: {task: NEEDS_ATTENTION, safety: PASS, behavior: PASS}` — and its
trigger is deliberately `when: {robot_carrying: r01}`, the **carrying** condition rather
than **executing**, precisely so it exercises the loaded half. `decide()` already selects
`ACTION_PARK_AND_ATTEND` correctly for that input. What was missing was the geometry: which
points a loaded robot may stop on, and which of those it can still reach.

**Fix.** Both methods implemented on `TaskService`:

* `_park_candidates()` — the configured **stations**, minus the **chargers**. Stations
  because they are the points the rest of the system already names and the dispatcher can
  already route to; inventing a grid here would create a second, unreviewed notion of where
  a robot may stand. Chargers excluded **from the list** rather than filtered later, because
  `CONTRACTS` §5 forbids driving a loaded robot to a pad — a candidate rejected only at the
  end is one edit away from being offered. Whether each point clears a protected rectangle
  stays `legal_park_point`'s question, answered by the same `_clears_every_rect`; this method
  deliberately does not re-implement it, since two copies of a clearance rule is how the gate
  and the driver came to disagree by exactly the stopping reserve (`D-P10-01`).
* `_reachable_park_candidates(state)` — the subset the battery can still pay for, using the
  **same** `can_reach_charger` the `no_safe_charger` path already uses, so "reachable" means
  one thing in the file. Returning the whole list when the battery model cannot answer would
  make `legal_park_point`'s reachability branch dead code.

**Tests**: `tests/test_p17_park_candidates.py` (9). The load-bearing one is
`test_the_two_methods_are_actually_defined` — an **AST** lookup, because a grep for
`_park_candidates` in that file returns the *call sites* and would pass.

**Follow-up recorded, not done.** The reachable set is computed once, at decision time, and
never re-checked while the robot drives to the parking point. `_reachable_park_candidates`
answers "can it afford this now", not "will it still be able to afford it on arrival". The
`no_safe_charger` path has the same shape. Whether that needs a re-check is **unresolved**,
and it is left open rather than guessed at.


## D-P17-20 — the truth message holds THREE models, and none of them is "model m = robot m"

`D-P17-14` fixed the liveness criterion by making it **per model**: a model is alive
if **its root slot** (`m * LINKS_PER_MODEL`) advanced. The reasoning recorded there
was that a whole-stream test "could not see a PER-MODEL freeze", and that a model
whose body froze kept scoring moved because slots 1..7 carry wheel link offsets fed
by `/joint_states`. That reasoning is right about the short-circuit. It is **wrong
about what a model index is**, and this entry corrects it.

### What the truth message actually contains

`/world/warehouse/dynamic_pose/info` (gz `Pose_V`, bridged to `tf2_msgs/TFMessage`)
carries **every model in the world**, always the same three, in all 33 recordings of
`batch_20260919T022011Z`. Measured on `F02` (a **one-robot** case, `robots: [r01]`):

```
model 0: slots 0..7    root=(6.000, -2.000)   x-range=(-6.000,  6.000)  y-range=(-2.000, 0.600)
model 1: slots 8..15   root=(-0.260,-0.205)   x-range=(-0.260,  0.260)  y-range=(-0.205, 0.205)
model 2: slots 16..23  root=(0.200,  0.000)   x-range=(-0.260,  0.260)  y-range=(-0.205, 0.205)
```

* **model 1 and model 2 have the same footprint** (0.52 x 0.41 m) at different origins.
  Neither is a robot: `F02` asked for exactly one robot, `r01`, and only one body in the
  world ever travelled. They are the **payload** and the **pad** — fixtures that the case
  places and hands over, not vehicles.
* **model 0 is the body that moves** in every one of the 33 recordings.
* The world's declared models are 24 static props (`barrier_north`, `wall_east`,
  `pad_spawn_east`, …). None of them appears in the truth stream. The three bodies in the
  stream are the **spawned** entities, and there are always exactly three regardless of
  how many robots the case asked for.

### The consequence: `dead_models` is not a defect

Every `PARTIAL` verdict in the batch is explained by this and by nothing else:

| case | robots declared | `root_distinct_by_model` | reading under the old text |
|---|---|---|---|
| F01 | 1 | `{0: 1026, 1: 845, 2: 744}` | all three "moving" |
| N05 | 2 | `{0: 21, 1: 975, 2: 1}` | "model 2 pinned" |
| N01 | 1 | `{0: 1, 1: 1015, 2: 1}` | "model 0 pinned" |

Note `N05` and `N01` disagree about **which** index is pinned, on the same world with the
same three fixtures. A genuine per-model freeze of a *vehicle* would pin the same vehicle.
What is pinned is whichever fixture the case did not put on a robot — the payload sits in
the pad, and the handover is a fixed translation, so the pad's own position never changes.

**So `PARTIAL` in these recordings is the guard correctly reporting that a fixture's world
pose is constant. It is not evidence of a frozen robot, and `D-P17-14`'s alert text saying a
PARTIAL recording "is not a clean run" is not supported for a fixture.** The alert is being
corrected to say what is measured.

### What is actually still broken, and it is the same thing as before

Model **0** — the robot body — does move, so the batch's central claim survives. But read
the counts:

| case | model 0 distinct root poses |
|---|---|
| F01 / F02 | 1026 / 1236 (of ~1640 / ~1851 samples) |
| N05_seed2 / N04_seed3 / N03 | 979 / 1045 / 934 |
| **F05 / F07 / F13 / I02 / I08 / N02** | **1** (frozen) |
| N01 | **1** — and yet `N01` passed its own case |
| I01 / I04 / I05 / I06 / I07 | 17 / 21 / 14 / 6 / 12 |
| F11 / I03 | 3 / 36 |

Two separate facts, and the second is the one that matters:

1. **Six recordings have a genuinely frozen robot body** (`slot0_distinct == 1`), and five
   of them are fault-injection cases whose whole point is that the robot keeps working.
2. **`N01` is the alarming one.** It has `slot0_distinct == 1` — the robot's world pose never
   changed for 2086 samples — while the case's own grading passed. Combined with
   `F02`'s trajectory showing model 0 alternating between exactly **two** states
   (`(6.000,-2.000,yaw 3.1420)` and `(4.914,-1.634,yaw 1.5090)`) and settling on 1235 distinct
   values over 1851 samples, the honest description is that **the robot body's truth arrives
   at a very low, bursty rate**, sometimes not at all. In `N01` it never arrived.

That is the same fault `D-P17-14` was written about, now measured on the robot body rather
than on an aggregate: the three-model stream is delivered at a low rate that collapses to
one frame for some runs. `D-P17-16`'s open item — that no truth-derived verdict from this
batch is quotable — **still stands**, and now for a sharper reason.

### What the guard says now

`scripts/check_truth_liveness.py` gained a per-model breakdown in its evidence and its
verdicts were split three ways in `D-P17-14`. This entry does not change the criterion; it
corrects the **interpretation** the guard prints, and adds `models_present_per_sample` so
that "how many bodies did this sample even carry" is visible next to the verdict.

### Not claimed

* ⛔ Not claimed: model 0 is `robots[0]` in the case's `runners` list. Model 0 moves whenever
  any robot moves; with one robot declared, the mapping from index to robot name is **not
  established by these recordings**. `judge.attribute_truth` decides identity from where a
  slot *started*, and `recorder-*.json` says so in `truth_note` — that attribution is the
  thing that must be used, not the index.
* ⛔ Not claimed: model 1 / model 2 are a payload and a pad **by name**. The claim is
  narrower and is enough: they have the same footprint, they never travel, and their count
  does not change with the number of declared robots.

## D-P17-21 — the liveness guard's three stale tests pinned a criterion that was replaced

`tests/test_p17_truth_liveness.py` failed three cases after `D-P17-14`:

```
FAILED test_live_verdict
FAILED test_short_when_world_has_fewer_models_than_robots
FAILED test_matching_recording_passes_the_guard
```

None was a regression. All three built a synthetic recording that walks **slot 0 only**,
which under the new per-model criterion leaves models 1 and 2 pinned — so the honest verdict
is `partial`, and the guards were right.

This matters more than three red lines: **a deliberately corrected guard whose tests assert
the old behaviour is indistinguishable from a regression**, and the next reader's cheapest
fix is to revert the guard. The tests were updated to the new semantics and **extended**,
because the new criterion needs a case the old one did not have:

* `test_live_verdict` now moves **every** model's root. Without it the correction could be
  satisfied by a guard that never says `live` — which would fail every real run.
* a new `test_partial_when_only_one_models_body_moves` states the D-P17-14 correction as a
  test, asserting `alive_models == [0]` and `dead_models == [1, 2]`.
* `test_matching_recording_passes_the_guard` was asserting `exit 0` on a recording it had
  never read the verdict of; it now asserts the recording passes *for the stated reason*.

**A trap in the assertions themselves.** The first version of the fix asserted
`"PARTIAL" not in res.stdout` — and failed, because the guard's **summary line always reads**
`FROZEN: 0   PARTIAL: 0`. A substring test on a word that appears in a zero-count summary is
a test that cannot pass. The assertions now match the per-recording markers (`[PARTIAL]`,
`[FROZEN]`, `<== SHORT`) and check the summary counts explicitly. This is the same class of
error as `D-P17-18`: the check was **plausible and wrong**, and only running it showed that.

`11 passed`. Wired into the same file the guard is tested from, so the two cannot drift.

## D-P17-22 — `_park_candidates` follow-up: two methods, and the reachability one is only half an answer

`D-P17-19` recorded as follow-up that `_reachable_park_candidates` is computed once at
decision time and never re-checked. Investigating it produced a second, separate gap worth
writing down, because the two look like one item and are not:

* **`_park_candidates` is a set of points.** It answers *where a loaded robot may legally
  stand* and nothing else. It is deliberately not filtered on distance.
* **`_reachable_park_candidates` is a filter on that set**, using the **same**
  `can_reach_charger` the `no_safe_charger` path uses, so "reachable" means one thing in the
  file.

The gap: the filter answers *"can it afford this now"*, and the decision is then acted on
over the **minutes** it takes to drive there. Between the two, battery drains. The
`no_safe_charger` path has the identical shape. Whether it needs a re-check is **unresolved**
— it is left open rather than guessed at, because guessing here would mean inventing a
re-check interval with no measurement behind it, and the project has already paid for
thresholds chosen without a measurement (`D-P10-01`, `D-P17-06`).

What is *decided* is the shape of the fix if one is needed: it must re-ask
`legal_park_point` with a fresh battery state rather than clamp a distance in the caller,
since two copies of a clearance rule is how the gate and the driver came to disagree by
exactly the stopping reserve.

## D-P17-23 — the biggest three-robot defect is one edge with three publishers, and the fix is to stop bridging it

### What was measured, before anything was changed

Three robots brought up together and driven for 30 s each, three identical attempts:

| attempt | crashed nodes | TF_OLD_DATA | "Clearing TF buffer" | `Static cache is empty` |
|---|---|---|---|---|
| 1 | 0 | 0 | 0 | 0 |
| 2 | **6** — amcl ×3, controller_server ×2, planner_server ×1 | 6 | **4,045** | 8 |
| 3 | **4** — amcl ×3, controller_server ×1 | 5 | **10,448** | 4 |

A full stack, from `N01`'s earlier recording:

```
[amcl-10] terminate called after throwing an instance of 'tf2::LookupException'
[amcl-10]   what():  Static cache is empty, when looking up transform
                     from frame [r01/laser_link/scan] to frame [r01/odom]
  #10 tf2_ros::MessageFilter<sensor_msgs::msg::LaserScan>   #9 tf2_ros::Buffer::waitForTransform
   #8 tf2::BufferCore::lookupTransform                      #6 __cxa_throw
   #5 std::terminate()                                      #1 raise
[amcl-10] Aborted (Signal sent by tkill() 72421 1000)
[ERROR] [amcl-10]: process has died [pid 72421, exit code -6, ...]
```

One attempt earlier in the day also produced, verbatim:

```
[r01.local_costmap.local_costmap]: Timed out waiting for transform from r01/base_link
  to r01/odom to become available, tf error: Invalid frame ID "r01/odom" passed to
  canTransform argument target_frame - frame does not exist
```

### The mechanism

The edge `odom -> base_footprint` was produced by bridging gz's own pose stream:

```
/<ns>/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V
```

That bridge is instantiated **once per robot**. TF is one shared tree, so all of them write
into the single global `/tf`. That gives one logical edge **three independent publishers**,
each with its own gz node, its own subscription, and its own callback queue, each stamping
from the same gz clock at the moment its own callback happens to run.

tf2 requires **arrival order to match stamp order per child frame**. When it is violated the
message is not merely dropped — tf2 discards its entire buffer:

```
[tf2_buffer]: Detected jump back in time. Clearing TF buffer.
```

The `odom` frame then does not exist for that consumer, and the next
`tf2_ros::MessageFilter` to ask for it throws `tf2::LookupException`. **Nothing in nav2
catches it**, so the process calls `std::terminate()` and aborts on signal 6.

The victim is whoever happens to be holding the stale request when the buffer is cleared —
which is why amcl, `controller_server` and `planner_server` all took turns.

### Confirming it was this edge and not something else

`/tf` on one robot carries exactly two dynamic edges, measured directly:

```
map        -> r01/odom              n=80     (AMCL)
r01/odom   -> r01/base_footprint    n=159    (the gz bridge)
```

The URDF has **no `odom` link** (`base_footprint` is the root; `base_footprint -> base_link`
is a fixed joint, published by `robot_state_publisher` as static), so the gz bridge was the
only possible source of that edge. The truth channel is bridged on its own topic
(`/world/warehouse/dynamic_pose/info`), not `/tf`. Two candidates eliminated.

`TF_OLD_DATA` attribution was **100 % `r03/base_footprint`** — the robot that starts last
and whose clock therefore lags — which is the signature of a per-child-frame ordering race
rather than a clock problem (`/clock` and the TF stamps were verified to agree to 0.025 s).

### The fix

Do not bridge the gz transform stream at all. `/<ns>/odom` was already bridged as
`nav_msgs/Odometry`, and that message carries the pose, the child frame and the stamp. A new
node, `fleet_ros/odom_to_tf`, subscribes to the bridged odometry and publishes the one edge
`<ns>/odom -> <ns>/base_footprint` onto the absolute `/tf`.

One robot, one publisher, one edge, one clock. Frames are read from the **message header**
(gz's DiffDrive is configured with `frame_id __ROBOT__/odom` and
`child_frame_id __ROBOT__/base_footprint`), so the launch file cannot drift away from the
model SDF. `map -> odom` remains AMCL's and is untouched.

### After

Identical three attempts, same machine, same 30 s of driving:

| attempt | crashed nodes | TF_OLD_DATA | `Static cache is empty` | `/tf` ordering violations |
|---|---|---|---|---|
| 1 | **0** | 0 | 0 | 0 |
| 2 | **0** | 0 | 0 | 0 |
| 3 | **0** | **0** | **0** | 0 |

Measured live on `/tf` with three robots:

```
        parent -> child                  count   viol
r01/odom -> r01/base_footprint              45      0
r02/odom -> r02/base_footprint              10      0
r03/odom -> r03/base_footprint              32      0
```

`viol = 0` on every edge. Before the fix `r03/base_footprint` alone had **11,681**
out-of-order arrivals in a single run.

### Why a guard, and why this guard

**Nothing in `scripts/check_*.py` could see this**, and the reason is structural: every
existing guard reasons about **one robot at a time**. The defect only exists in the
relationship between robots — three publishers, one topic. It is occurrence #8 of
*"断言比现实窄"*.

`scripts/check_tf_single_publisher.py` is registered as guard **15** in
`scripts/check_guards.sh` and asserts:

1. `_bridge_arguments` does not bridge any gz transform stream — parsed from the **AST of
   the argument list**, not by searching the file text, because a text search flags the
   comment that documents the fix.
2. `odom_to_tf` is launched, per robot, with an absolute `/tf` and `use_sim_time`.
3. The odometry bridge it consumes is still present.
4. `tf2_msgs` is declared in `package.xml` and `scripts/odom_to_tf` is in
   `install(PROGRAMS ...)` — this package installs executables that way, **not** via
   `setup.py`'s `console_scripts`. Getting that wrong builds clean and produces a node the
   launch file cannot start.

It is **proven to fail**: with the old bridge line restored in a copied tree, the guard exits
**1**; on the real tree it exits **0**. Both directions measured, because `D-P17-10` is the
lesson that *"一个不能被证明会失败的检查，等于没有检查"*.

### What this does NOT fix

Two things were exposed underneath, and neither is this defect:

* **`amcl_pose` still does not arrive on a three-robot bringup.** With the crash gone, r02's
  lifecycle manager now fails a different way:
  ```
  [r02.lifecycle_manager] Failed to change state for node: planner_server.
    Exception: planner_server/get_state service client: async_send_request failed.
  [r02.lifecycle_manager] Failed to bring up all requested nodes. Aborting bringup.
  ```
  That is a **service-discovery timeout under load**, not a transform problem. The three
  robots are launched concurrently (only a 3 s spawn delay), and r02's bridge does not start
  until ~7 s after r01's on this CPU-only machine.
* **`Detected jump back in time` still fires during bringup** (182 / 1,371 in the post-fix
  runs), now from amcl and `controller_server` at `initTransforms` time — i.e. **10 s before
  the odometry edge exists**, while the world is still materialising three Nav2 stacks.

The safety gate behaved correctly throughout: with no localised pose it logged
`localised pose unusable, refusing rather than composing odometry: no localised pose has
arrived, so there is no position to bound motion with` and refused motion. That is the gate
doing its job, and it is the honest reason item ④ is **not yet closed** — the crash that
stopped `amcl_pose` is fixed, but the bringup that would deliver it does not complete yet.

## D-P17-24 — a three-robot bringup never completed, and the timeout that said so was lying

### The observation

Six three-robot bringups, none reached "Managed nodes are active" within 150 s. The
lifecycle manager always failed the same way:

```
[r01.lifecycle_manager] Failed to change state for node: map_server.
  Exception: map_server/get_state service client: async_send_request failed.
[r01.lifecycle_manager] Failed to bring up all requested nodes. Aborting bringup.
```

### The message is not what it says

`async_send_request failed` reads like a failed send. It is not one. It is thrown here:

```cpp
// nav2_ros_common/include/nav2_ros_common/service_client.hpp:111
auto future_result = client_->async_send_request(request);
if (spin_until_complete(future_result, timeout) != rclcpp::FutureReturnCode::SUCCESS) {
  client_->remove_pending_request(future_result);
  throw std::runtime_error(service_name_ + " service client: async_send_request failed");
}
```

The request WAS sent. The **reply did not arrive before `timeout`**. And `spin_until_complete`
resolves to

```cpp
return rclcpp::spin_until_future_complete(node_base_interface_, future, timeout);
```

whose timeout is **wall time**. Sixth occurrence of *"字段存在 ≠ 它是你要的那个"* in this
project: the name of the error describes a different fault from the one it reports.

### Which timeout, and what value it had

```cpp
// nav2_lifecycle_manager/src/lifecycle_manager.cpp
double service_timeout_s = nav2::declare_or_get_parameter(node, "service_timeout", 5.0);

bool LifecycleManager::changeStateForNode(const std::string & node_name, std::uint8_t transition) {
  message(transition_label_map_[transition] + node_name);
  if (!node_map_[node_name]->change_state(transition, std::chrono::milliseconds(-1), service_timeout_) ||
      !(node_map_[node_name]->get_state(service_timeout_) == transition_state_map_[transition])) {
    RCLCPP_ERROR(get_logger(), "Failed to change state for node: %s", node_name.c_str());
    return false;
  }
```

So both halves of every state change are bounded by one parameter, **`service_timeout`,
default 5.0 s** — and the launcher **never set it**. It set `bond_timeout: 4.0` and left this
at the library default.

The arithmetic agrees: the manager logged `Configuring map_server` at 434.10 and the failure
at 439.63 — **5.53 s later**, against a 5.0 s timeout.

### Why it is fine for one robot and fatal for three

Three robots each start eight managed Nav2 nodes, plus gz emulating three robots, on a
CPU-only host. `map_server` for r02 and r03 did not even reach "lifecycle node launched"
until ~20 s after r01's manager began asking for it:

```
405.796 -> 425.845   r01 manager polls "Waiting for service map_server/get_state" 11 times,
                      every 2 s
426.112              r01 map_server finally launches       (20 s late)
434.102              Configuring map_server   -- succeeds
439.630              Aborting bringup
439.799              r02 map_server launches                (4 s after r01 gave up)
443.894              r03 map_server launches                (8 s after r01 gave up)
```

### The cost of getting this wrong is evidence, not a red test

An aborted bringup does not produce a failing verdict. It produces a case with no fresh state
at all, which `batch.py` records as `NOT_RUN`. So a host-speed problem **destroys the evidence
for every case in the run** instead of being reported as the infrastructure fault it is —
the opposite of what CONTRACTS section 9 asks for.

### The change

`nav2_service_timeout` is declared in both launch files, forwarded from `fleet.launch.py`
into each robot's include, and passed to the lifecycle manager as `service_timeout`. It is
forwarded **explicitly** rather than left to `robot.launch.py`'s own default, because a fleet
run is the only configuration that starts several Nav2 stacks at once, so it is exactly the
run whose timeout must be this value — the same lesson as `D-P17-09`.

Guarded by `scripts/check_nav2_lifecycle_timeout.py` (guard 16), which asserts the parameter
is present, that it comes from the launch argument rather than a literal, that both files
declare it, that the fleet launch forwards it, and that its default sits inside
`[10 s, 120 s]`. The ceiling is generous on purpose: a genuinely dead node is caught by the
BOND (`bond_timeout`, 4.0 s), not by this timeout, so this one only has to outlast a slow
reply.

### Result

| configuration | reached "Managed nodes are active" |
|---|---|
| stock `service_timeout` (5.0 s), 6 attempts | **0 / 6** |
| `nav2_service_timeout:=20.0`, run 1 | **yes, 35 s** |
| `nav2_service_timeout:=20.0`, run 2 | **yes, 37 s** |
| `nav2_service_timeout:=20.0`, run 3 | **yes, 32 s** |

All three clean runs: `LookupException = 0`, `exit code -6 = 0`, `Aborting bringup = 0`,
`TF_OLD_DATA = 0`, `Detected jump back in time = 0`.

## D-P17-25 — the teardown missed three process classes, and one of the metrics used to check it could only ever read 1

### How this was found, and why it matters more than the leak

While measuring `D-P17-24`, the first three-attempt run looked like a partial success and then
a regression: attempt 1 reached active with no crash, attempts 2 and 3 crashed four nodes each.
The obvious reading was *"the TF defect is only partly fixed; a second out-of-order source
remains"* — and that reading was written down before it was checked.

It was wrong. The process table showed **three generations of orphans still alive**:

| PPID | elapsed | survivors |
|---|---|---|
| 1521 | 06:32 | 3 × `gz_bridge`, `truth_bridge`, `clock_bridge`, 3 × `robot_state_publisher`, 3 × `static_transform_publisher` |
| 8357 | 03:00 | the same set again |
| 5933 | 03:43 | the same set again |

`load average: 20.24` on a host with 1 GiB in use. Three live `clock_bridge` publishers on
`/clock`, and three generations of `robot_state_publisher` writing the same frames to one
`/tf`. Multiple publishers on `/clock` is not a leak but a **correctness fault**: every
consumer's clock jumps, and tf2's response to a backwards jump is to discard its whole buffer.

So attempts 2 and 3 were crashing on attempt 1's garbage. **`D-P17-23`'s conclusion stands, and
the "second independent source" hypothesis it was briefly extended to explain is retracted.**
Re-run with a correct teardown: **3 of 3 reached active, 0 crashes, 0 buffer clears.**

### The two defects

**The sweep patterns did not match the processes.** The teardown swept for
`("gz sim", "fleet_ros/lib/fleet_ros", "nav2_", "amcl")`. Three of the classes the tree
actually spawns contain **none of those words**, because the node name in a ROS command line
is an argument (`-r __node:=...`), not the executable:

```
/opt/ros/lyrical/lib/ros_gz_bridge/parameter_bridge /clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock --ros-args -r __node:=clock_bridge
/opt/ros/lyrical/lib/robot_state_publisher/robot_state_publisher --ros-args -r __node:=robot_state_publisher -r __ns:=/r01
/opt/ros/lyrical/lib/tf2_ros/static_transform_publisher --frame-id r01/laser_link --child-frame-id r01/laser_link/scan
```

The process that publishes `/clock` is called `parameter_bridge`. Searching for "clock" would
not find it either; searching for "parameter_bridge" does.

**The group kill was gated on the launcher being alive.**

```python
if self.proc is not None and self.proc.poll() is None:   # <-- the hole
    os.killpg(os.getpgid(self.proc.pid), signal.SIGINT)
```

A tree whose `ros2 launch` parent has already exited still has live children. That is exactly
the case the condition skips, and it is the case that leaks. The pgid is now captured and
signalled unconditionally; a dead group raises `ProcessLookupError` and that is the quiet,
normal outcome.

**Also: the leftover count could only ever read 1.** It used `pgrep -c -f`, which is fine, but
the probe written to judge the same cleanup used `ps -eo args | grep -Ec "<pattern>"` — and
**grep does not exclude itself**, so the grep process's own command line (which contains every
pattern) was counted. That metric printed `1` before and after a cleanup that provably removed
78 processes. Eighth occurrence of *"字段存在 ≠ 它是你要的那个"*: a number that is always the
same size is not a measurement. `Fleet.leftovers()` now reads `/proc/<pid>/cmdline` and skips
its own pid, and a test requires it to read **0** on an idle host and **non-zero** for a real
process, because a counter that can only read one value is useless in either direction.

### The change

`Fleet.LEFTOVER_PATTERNS` is a class constant covering `fleet_bringup/fleet.launch.py`,
`gz sim`, `parameter_bridge`, `robot_state_publisher`, `static_transform_publisher`,
`nav2_`, `amcl`, `fleet_ros/lib/fleet_ros`. It is a constant rather than a literal inside
`stop()` so that a test can require coverage.

`tests/test_p17_teardown_and_timeout.py` asserts coverage **against nine real command lines
copied from the process table**, not against a list of words — because the failure mode is
precisely a word that is not in the command line. It also asserts, structurally, that no
`os.killpg` call in `Fleet.stop` sits inside an `if <...>.poll() is None:` branch.

The first version of that coverage test asserted `"clock" in patterns` and failed. The test
was wrong, not the code: `clock_bridge` IS a `parameter_bridge`, so the pattern that covers
one covers the other. Asserting on real command lines is what made that visible.




## D-P17-27 — a case pointed a robot at the buffer it did not need, and the guard judged history

Three separate things, all found by running the batch rather than reading it.

**F03 named the wrong exit buffer.** The case is "the correct exit buffer is occupied", and
it drove `r02` to `(-2.5, -1.4)` -- `exit_west`. But `r01` is submitted to pick at
`S_left_a` (west) and drop at `S_right_b` (east), so the buffer it needs after leaving the
gap is the FAR-side one, `exit_east` (`config/resources.yaml:144-145`, and
`exit_node: exit_east` for this direction at `:156`). Two consequences, both measured: the
fault asked `r02` -- spawned at `(6.0, -2.0)`, on the east -- to cross the whole map, and
the run reported `Failed to make progress` (`error_code 105`), so the fault was never
delivered and the case was honestly recorded `NOT_RUN`. A fault that cannot be delivered is
not a test of anything. The target is now `(2.5, -1.4)`.

**`despawn_obstacle` hung.** `ros2 run ros_gz_sim delete_entity` was killed by its own 30 s
timeout on F01's second step, while `gz service` on the same world answered at once -- the
path `pause_world` uses and that works. The world's own `remove` service is now the primary
path and the CLI is the fallback, and the note says which one answered so the two are not
confused. Measured after the change: F01 `rc=0`, both steps `ok`, and
`every declared outcome held`.

**A second clock reader was added and then WITHDRAWN, because it produced a confident wrong
number.** `/clock` over ROS stops publishing while the world is paused, and I05 pauses the
world before asking for a jump, so `jump_clock` refused for want of a reading. A
`gz topic -e -t /clock` fallback was added; it returned `1789832190.000 s -> 1789832191.000 s`
-- a pair of WALL-CLOCK stamps, because the first `sec:` line of that dump belongs to a
header, not to the Clock message. The independent check that settled it: the recorder's own
samples carry `t` of 23.5 / 76.3 / 857.7 for the same kind of run. A confident wrong number
in a record is worse than a refusal, so the reader is gone and the reason is in the
docstring. The real fix is different and is in `set_world_paused`: the pause samples the
clock **before** it freezes the world, and because a clock cannot move during a freeze that
sample IS the value at jump time. `jump_clock` falls back to it and says which of the two it
used, because a remembered number and a read number are not the same kind of evidence.
I05's step order was deliberately NOT changed: its own comment says the pause is the
mechanism it shares with I04, and reordering would have traded a measurability bug for a
semantics change.

**The liveness guard judged history.** It walks every `samples-*.jsonl` under `reports/`, so
when five labels were re-run after their recordings froze -- and the re-runs came back live
-- the guard stayed red on the superseded recordings. Its own comment states the intent it
was violating: *"Guard mode judges only recordings the fix could have affected, so a
non-zero exit is a real regression."* It now judges ONE recording per label, the newest, and
lists the older ones as **superseded**: counted, printed, kept on disk, and not fatal. A
re-run is the fix for a frozen recording, so the guard must not fail because the fix was
applied. Measured: 10 superseded, and the fatal set went from 5 labels to 4, all of them
still awaiting a re-run.

Separately, five pre-`tf2`-fix batches were moved to `reports/_invalid/` with a README
naming the cutoff and asserting that archiving changes nothing true. Not deleted.


## D-P17-28 — two instrument defects inflated `safety=FAIL`, one deflated the evidence behind `PASS`, and the third was mine

Seven runs of this generation ended with `safety` other than PASS, six of them on
`no_unauthorised_entry_from_truth` and one on `no_simultaneous_occupancy_from_truth`. Before
treating any of them as a finding about the fleet, each was taken apart sample by sample.
Two instrument defects and one missing rule fell out, and a fourth defect was introduced by
my own fix and caught the same evening.

**(a) A slot that has not been placed yet reads exactly `(0, 0, 0)`, and `mid` contains the
origin.** `_slot_tracks` documents this at length -- *"the world widens its pose array as
models come up"* -- and already skips the origin when deciding where a slot STARTED, citing
`scripts/validate_assets.py` to prove no robot can start there because `(0, 0)` is inside
the corridor's throat. The two TRUTH-based containment loops never got the same treatment,
so two unplaced robots read as "two robots inside the same region" of a capacity-1 corridor,
and an unplaced robot read as occupying a region its book called free. Measured:
`N07_seed2`'s 12 simultaneous-occupancy samples are EXACTLY its two robot slots' 12 leading
zeros (`t` = 23.351..23.901, both poses `(0.0, 0.0, 0.0)`), and the freshly re-run `N08`'s 6
unauthorised samples are all pre-placement. Both verdicts were false positives.

The rule applied is per slot: skip a slot only for samples BEFORE its first placement.
Skipping every `(0, 0)` would have been simpler and wrong -- the corridor's throat contains
the origin, so a robot crossing it passes through the origin and that sample must stay
countable. Not hiding a violation mattered more than a tidy predicate.

**(b) `owner is None` was treated as "unowned".** The book has three states and
`traffic.py` sets the INITIAL state to `UNKNOWN`, not `FREE` ("until the ...", CONTRACTS
section 7), while `domain.py` defines `UNKNOWN` as *"no trustworthy occupancy knowledge ->
block"*. The check's own PASS text promises *"no robot was inside a region its own book
called unowned"*, and `UNKNOWN` is not "unowned", it is "unknown" -- the implementation was
broader than the property it claims to test. It mattered: `F02`'s 1949 hits are **1450
`UNKNOWN`** followed by 499 `FREE`, which is one continuous episode (`UNKNOWN` from
`t`=88.2 to 160.6, then `FREE` to the end at 185.5), not one uniform fault. The verdict is
unchanged -- CONTRACTS section 7 makes `UNKNOWN` block entry, so neither state authorises
occupancy -- but the two counts are now reported apart, and a number nobody could read
became two numbers that can be read.

**(c) A PASS with zero exposure is not evidence, and the judge already knew that.** Fixing
(a) turned two `N08` verdicts from FAIL to PASS, and the PASS was vacuous: `N08`'s three
robots travelled at most 4.6 m in 433 s and **no robot's footprint ever overlapped a
protected region** (nearest approach 2.78 m against a region grown by `footprint_margin_m`).
Meanwhile `driver_claims_corroborated_by_truth` is already reported NOT_RUN when there are
no claims, for exactly this reason. Both occupancy properties now return NOT_RUN, with the
reason stated, when there is no exposure. Without this rule the fix for (a) would have
produced a worse outcome than the defect it removed.

**(d) My own patch printed one verdict and returned another.** The fix for (c) added the
zero-exposure reason to the DETAIL of `no_simultaneous_occupancy_from_truth` and left the
verdict expression untouched, so four checks printed *"NOT_RUN rather than PASS: a property
that was never exercised cannot be evidence that it holds"* while returning PASS. Caught by
reading the re-judged output and noticing the two disagreed -- the same class of defect this
project keeps finding in its own instruments. The verdict and the detail now share one
condition. A second round of the same patch had dropped the geometry-mode note from the new
NOT_RUN branches, which `tests/test_eval_judge.py` caught; the note is unconditional because
a PASS under the conservative footprint test means something stronger than usual.

**What the 38 runs look like once the instrument is honest.** Re-judging every batch offline
(the judge needs no simulator) and diffing against the stored verdicts changes 23 labels;
the protected-region property redistributes as: **11 runs PASS with real exposure** (F01,
F03, F05, F09, N01, N04 x3, N05 x3), **5 runs FAIL** (F02, N06, N07, N07_seed2, N07_seed3),
and **22 runs NOT_RUN** because no robot ever entered a protected region at all. The
headline is the third number: **over half of this generation never exercised the property
that the `safety` column was read as evidence for.** A table that shows `safety=PASS` for a
recording in which the fleet never went near the corridor was never saying what it looked
like it was saying.

**The size of a violation is not comparable across runs, and the liveness guard cannot see
that.** `truth_messages` counts messages, not whether their content changes. Folding each
failure run into DISTINCT truth observations: `N07_seed2` 7 samples become 2 (`t`=199.4 and
275.0), `N07_seed3` 12 become 1, `N07` 100 become 2, `N06` 401 become 381, `F02` 1949 become
1450 `UNKNOWN` plus **one** observation held for 24.8 s. `N07_seed2`'s truth updated **29
times in 600 s**, worst hold 120 s, while the guard called the recording live. So "X
samples" here measures how long a stale frame was held, not how often the fleet erred.

**What still stands, and the leading suspect.** The predicate and the margin are SHARED: the
judge builds `Region(..., margin_m=cfg.footprint_margin_m)` and `traffic.py` uses the same
`footprint_margin_m` (0.15) with the same `box_overlap`, and both use `L, W = 0.60, 0.45`.
So "the two sides disagree about what 'inside' means" is refuted by the source. In the same
runs `pose_estimate_error` reports p95 of 0.75-1.35 m against a 0.35 m limit, which is a
sufficient mechanism for a fleet that believes it has left a region while its footprint
still overlaps: the reservation logic is answering a question about a pose it does not have.
**Two observations resist that explanation**: `r03 in mid @ (-0.204, -0.106)` and
`r01 in mid @ (1.441, -0.12)` sit well inside a 3.5 x 1.3 m corridor while the book says
FREE, and no ~1 m localisation error turns "in the middle of the corridor" into "outside
it". Those two need their own answer and are the next thing to chase.

**Not done, deliberately: the shadow verdicts have not been written back.** Re-judging wrote
to a separate directory; no `judge-*.json` or `summary.json` was overwritten, because
rewriting evidence is a bigger decision than fixing the instrument that reads it. `F03`'s
corrected target `(2.50, -1.40)` is still unreachable (`error_code:105 Failed to make
progress`), so its fault remains undeliverable and it correctly records NOT_RUN; the
recording itself is healthy (`slot0_distinct=2281`).


## D-P17-29 — the five "unowned region occupied" verdicts are one finding, and it is the pose estimate

Five runs of this generation failed `no_unauthorised_entry_from_truth` with counts of 7, 12,
100, 401 and 1949 samples. Read as sample counts they look like five separate incidents of
increasing severity. Reconstructing the reservation book sample by sample, and asking the
containment question twice per sample -- once of the world's truth, once of the odometry the
fleet actually acted on -- collapses them into one shape and one cause.

**The book's own timeline makes the three-corridor cases a handover gap, not an entry.**
`N07`'s `mid`: `RESERVED owner=p7-N07-s1-5, queue=1` from t=613.73, then **`FREE, queue=1`
at t=699.40**, then `RESERVED owner=p7-N07-s1-4` at t=699.61. The failing samples are that
**0.21 s** window. `N07_seed2` is the same at `mid` (0.35 s, t=274.99 to 275.34) and
`N07_seed3` at `mid_left` (0.60 s, t=446.76 to 447.36). In all three the robot was inside
the region **before** the release and **after** the re-grant -- it held the permit for the
previous window and the next holder's grant lands within the same second. Nobody drove into
an unowned region; the judge sampled the gap between two reservations of the same region.

`N06` is a different shape again: `mid_left` has **two state changes in the whole run**
(`UNKNOWN` then `FREE`) and is **never reserved at all**. Its 381 folded observations are 20
seconds of r01 driving east past the west pad, whose footprint plus `footprint_margin_m`
(0.15) clips the corner of the pad's grown rectangle by 0.031 m at closest approach
(0.344 m to the corner against a 0.375 m half-diagonal for the 0.60 x 0.45 body).

`F02` is the only one that is a release with no re-grant: `mid` goes `RESERVED(f02-run)` at
t=59.99, `UNKNOWN` at 88.17, `FREE` at 160.62, and is **never reserved again** while r01
stays inside for the remaining 97 s of the run.

**What they share is the reason the region was released.** For **every** failing sample in
**all five** runs, the truth places the robot inside the region and the recorded odometry --
the pose the fleet acted on -- places it outside. 100% of 100, 4, 12, 401 and 1949. And
`domain.py` defines the state the judge found as `FREE = "ledger empty AND geometrically
verified clear"`: the fleet declared the region *verified clear* while its own robot was in
it, because it verified against a pose it did not have.

**The residual is real, and this retires an earlier attribution.** `pose_error()` now goes
through `detect_odom_frame()` and `to_map()`, so the reported claim-versus-truth numbers are
frame-corrected; the previous note that "p95 ~ 5.96 m is a frame-convention artefact" no
longer explains them. Regressing the error on cumulative distance travelled, the method
`D-P10-02` used to find the old `spawn x odom` rate error:

| run | robot | distance | err p50 | R(err~distance) | slope |
|---|---|---|---|---|---|
| F02 (single robot, healthy) | r01 | 11.64 m | 0.70 m | +0.839 | 0.0396 |
| N07_seed2 | r02 | 22.12 m | 5.15 m | +0.775 | 0.2366 |
| N07_seed3 | r02 | 16.47 m | 6.93 m | +0.894 | 0.3365 |
| N07_seed2 / seed3 | r01 | 26.8 / 20.8 m | 3.70 / 1.58 m | -0.07 | ~constant 1.9 m offset |

So the healthy single-robot run shows the benign 4 cm per metre, while the worst three-robot
robots show 24-34 cm per metre -- six to eight times worse, and the same *kind* of error.
And **two symptoms coexist inside one run**: a distance-driven drift (R = +0.8..0.9) on one
robot and a roughly constant 1.9 m offset (R ~ 0) on another. `min = 0.00` for every robot
says there is a moment when the estimate is exactly right, so this is accumulated error after
a correction, not a fixed composition mistake. Error grows with the number of robots: 0.70 m
median on one robot, ~1 m on two, p50 6.93 / p95 8.13 m on three.

**What follows for the next step.** The five verdicts are not five defects and not a
reservation-logic bug; they are the downstream consequence of a pose-estimate defect, and the
causal chain is complete and measured end to end: the book releases on a "verified clear"
test, the test uses a pose that is metres wrong, the release precedes the truth leaving, and
the truth-based property then sees a footprint inside a region the book calls free. So the
work belongs in the estimator (AMCL, the odometry chain, the TF edges), **not** in the traffic
logic -- changing the traffic logic would treat the symptom.

Two things must NOT be said on the strength of this. The failures are not explained away:
`F02` released a region and never took it back for 97 s with its own robot inside, and the
`no_simultaneous_occupancy` PASSes across all of it came from the robots happening to be far
apart, which is luck rather than a guarantee. Nothing here shows the mutual-exclusion property
holds -- it shows only that this generation never exercised it.


## D-P17-29 addendum — the fleet's OWN pose, the guard that checks the wrong condition, and a fifth meeting with the same trap

D-P17-29 above used `containment_from_odom()` as a stand-in for the fleet's belief. That is
wrong and the entry should not be read that way: both launch files set `pose_source` to
`localiser` (`fleet.launch.py:214`, `robot.launch.py:552`), so the gate acts on AMCL's
localised pose, while `containment_from_odom` reads the sample's `odom` field -- the
odometry path. (The function's own docstring says "the estimate the controller acts on",
which is not accurate under `pose_source=localiser`; recorded as a wording defect, not
changed here.) The fleet's actual pose IS in the samples, as `claim.robots[rXX].x/y/yaw`
alongside `localization_valid` and `fresh`.

**Its frame had to be settled before it could be compared with anything.** At the first
sample carrying a claim (t = 8-24 s, robots still near their spawns): `N06` r01 reads
`(-6.0, -2.0)` and `N07_seed2` r03 reads `(-6.0, 0.6)` -- exactly their spawns, so `claim` is a
MAP-frame pose. Robots that have not reported yet read `(0.0, 0.0)`.

So the comparison is valid, and it is unambiguous. On the failing samples -- the region's
`owner` is None and TRUTH places the robot inside it -- the fleet's OWN map-frame pose places
it **outside**, in every single one, with `localization_valid = True` and `fresh = True`
throughout:

| run | failing samples | fleet says inside | fleet says outside | |self - truth| p50 |
|---|---|---|---|---|
| N07_seed2 | 4 | 0 | 4 | 5.458 m |
| N07_seed3 | 12 | 0 | 12 | 8.064 m |
| N07 | 100 | 0 | 100 | 7.459 m |
| N06 | 401 | 0 | 401 | 0.948 m |
| F02 | 1949 | 0 | 1949 | 0.697 m |

Two regimes: `F02` and `N06` are sub-metre (0.70 / 0.95 m), enough to flip a decision at a
0.15 m-margin boundary; `N07` x3 are 5.5-8.1 m, which is not a boundary question at all. And
the validity flag is True for all of them, so **`localization_valid` does not mean the pose is
usable** -- the fleet certifies an 8 m error.

**A fifth meeting with the same trap.** `claim.x/y = (0, 0)` means "not reported yet", it comes
with `localization_valid: true` on the same row, and `(0, 0)` lies INSIDE `mid`. Any check that
reads a claim pose without skipping the unset zeros will see robots parked in the corridor.
This is the fifth occurrence of this shape in the project (`D-P03`, `D-P10-02`, `D-P17-03`,
`D-P17-07`, and defect (a) of this round in the truth slots). The pattern is not that the
project forgets once; it is that every new *source* of poses re-introduces it, because
"no value" and "the origin" are the same two numbers.

**A guard that checks the wrong condition.** `LocaliserPolicy` calls a pose usable when
`age <= max_age_s` (2.0 s) OR the robot has not moved more than `max_moved_m` (0.100 m).
`scripts/check_pose_freshness.py` validates only `update_min_d <= max_moved_m` -- the DISTANCE
condition -- and has no test at all for the age condition. The failing runs fire on the age
condition: the gate logs `localised pose is 2.011 s old (> 2.00 s) and the robot has moved
0.124 m since it arrived`, 83 times in `N07`, 28 in `N07_seed3`, 11 in `N07_seed2`, 1 in
`F02`. And `max_age_s = 2.0` is documented as "2.3x the measured p95 age while moving
(0.853 s)" -- a figure measured in a configuration that this one exceeds. So the bound's
calibration does not hold for three robots, and nothing checks that it does.


## D-P17-30 — the guard suite stopped at its first failure, one usability condition had no guard at all, and one case aimed a robot at a place it cannot legally arrive

### The suite reported less than it checked

`check_guards.sh` runs seventeen guards and prints one `[FAIL]` line per failing guard, then
exits 1 if any failed. It was running **thirteen** of them. `guard()` ended with `return 1`
on failure and, on its first invocation, executed `set -e 2>/dev/null || true` to "restore" a
mode the file never had -- its header says `set -u -o pipefail`. `set -e` is global, so the
first failing guard killed the whole script: everything after it never ran, the FAIL list
under-reported, and the closing "all static guards passed" line never printed. Measured
2026-09-20: 13 sections, and the four skipped ones included `check_tf_single_publisher.py` and
`check_nav2_lifecycle_timeout.py`, the two most recently added and the only two that reason
about more than one robot.

So since the frozen-truth guard started failing, two fleet-level guards have been silently
absent from every "the guards are green except one" report. This is `D-P17-10` a third time
-- a check that reports less than it did -- and it is the reason the failure count could not
be trusted: "one guard failed" meant "one failed and four never ran".

Fixed by not touching `set -e` at all and returning 0 from `guard()` while setting `FAILED`.
Measured after: 17 of 17 sections, two FAILs named, and the closing line printed.

### The localiser age bound was the half of the usability rule nobody checked

`LocaliserPolicy` calls a pose usable when `age <= max_age_s` (2.0 s, from
`localiser_timeout_s`) **OR** the robot has moved no more than `max_moved_m` (0.100 m).
`check_pose_freshness.py` asserts the second condition -- `update_min_d <= max_moved_m` -- and
said "OK" without ever mentioning the first. A green result from it was read as "the pose is
safe", which it never claimed, and the condition that actually fires in real runs is the age
one: the gate logs `localised pose is 2.011 s old (> 2.00 s) and the robot has moved 0.124 m
since it arrived`, 83 times in N07, 28 in N07_seed3, 11 in N07_seed2, 1 in F02.

`scripts/check_localiser_age.py` now measures it from the recordings, and re-derives the bound
rather than quoting the comment that justifies it: `pose_source.py` says 2.0 s "is 2.3x the
measured p95 age while moving (0.853 s) and holds a fresh pose on 98.3% of moving samples".
Pooled over 38 recordings and 149,692 moving samples:

| bound | fraction of moving samples accepted |
|---|---|
| 0.50 s | 58.1% |
| 1.00 s | 83.8% |
| **2.00 s (declared)** | **96.3%** |
| 3.00 s | 98.2% |
| 5.00 s | 98.3% |

Pooled p95 while moving is **1.802 s**, so 2.0 s is **1.11x** its own justification, not 2.3x,
and the quoted **98.3% belongs to a 5 s bound**. The guard fails on the real tree, and its
self-test proves it can fail. `F14` is an outlier worth its own look: p50 63.9 s, max 128.9 s
of age while the robot was moving.

### F03's target is not a legal arrival point, and the contract says so

F03 wants a body ON the exit buffer. The manifest's comment records that the target was moved
from `exit_west` to `exit_east` on 2026-09-19 because `exit_west` measured as unreachable --
and the re-run to `exit_east` timed out the same way, `Failed to make progress`, error_code
105. The reason is not navigation and never was: `config/resources.yaml` records that
**arriving** at a pad is refused (`RESOURCE_UNKNOWN` on `mid`) while a robot **standing** there
is allowed, so the permit can never be requested and -- because a stopped robot's stopping
envelope never shrinks -- the deadlock is permanent. The file says in as many words that the
driver "reported it as `nav2 status=6 Failed to make progress`, which reads as a navigation
fault and is not one."

So `drive_to_pose` cannot occupy a pad, and F03's gap is a missing LEGAL ARRIVAL POINT rather
than a missing action. It now declares `requires: [pad_occupation_by_crossing]`, with its
`submit`/`steps`/`truth`/`expect` kept under `intended_*` and the key registered in
`docs/LIMITATIONS.md`. Pointing `drive_to_pose` at `align_east` (a legal asking point) would
make the run green and stop testing what the title says, so that is explicitly ruled out in
the register.

### `docs/LIMITATIONS.md` over-claimed in three places

"IMPLEMENTED" described the POLICY files, not the actuator: eight of the eighteen declared step
actions appear only in `fleet_core.trigger` / `fleet_core.stage2_capabilities` and nowhere in
`scripts/batch.py:fire_step`, so a case naming one cannot deliver its fault. The register also
said `drive_to_pose` meant "a robot can now be driven to the exit buffer" (it cannot, above),
and its `obstacle_injection` entry still ended "**Not yet run**: neither F01 nor F02 has
produced a verdict" -- both have, and F01 records `every declared outcome held`. All three are
corrected in place, with the correction placed BEFORE the entries rather than after, because a
register whose first paragraph over-claims is worse than one with gaps.

### The 38 verdicts were rewritten, once, with a copy taken first

Every verdict in `reports/` was produced by the pre-correction judge, so the tree mixed two
instruments. `scripts/rejudge_writeback.py` re-judges each recording, writes `judge-<label>.json`
with the runner's own serializer, and recomputes `safety_outcome` by calling
`batch.safety_from_judgement` -- imported, not copied, so the derivation cannot drift. 110 files
were backed up first; 33 verdicts changed; each rewritten `summary.json` gained a `rejudged`
block naming the date, the tool and the reason, and each of the 16 affected `report.md` files
got a banner saying its table is a snapshot from the old judge. The distribution after:

    safety   UNKNOWN 22   PASS 11   FAIL 5      (was PASS 33 / FAIL 4 / UNKNOWN 1)
    quotable safety PASS (fault delivered AND recording live): 10

The 22 UNKNOWNs are the point: those runs passed a property they never exercised. Reported in
`_p009/p17/p17_rejudged_38.md`.
