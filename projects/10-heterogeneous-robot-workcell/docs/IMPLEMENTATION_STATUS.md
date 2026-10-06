# 当前实现状态

## 当前状态 · 2026-10-04 · 翻转保持单机构正负例通过，运输未验

- 用户“okay go ahead”批准继续证据驱动路线：先量无接触的实际表面间隙与持续时间，再选择最小合理机构；压持不是已证明唯一必要方案。不下载/安装/购买/推送，不改001～009。
- 两份测量均exit1：第一份raw convex distance不合理负值不引用为可靠间隙；解析复验初始化外13事件均2ms，下平面分离界最大5.419µm（非整盘精确最大离地高度）。这次载荷门未拒绝，但托盘保持7.617mm FAIL、deck yaw0.010375rad超原±0.01门，仍未到达。不能混淆接触采样、滑移和甲板偏航。
- 新v6直升鞋物理FAIL：遮挡相机、双侧接触丢失、滑移10.668mm，拒绝集成并保留资产/失败。新独立v7向外翻转鞋：闭合/释放/视觉/原保持/停止六项PASS（exit0/55.228s，保持0.043855mm）；保持释放位故障exit0/41.597s，actual_closed_contact=false、无运输授权。这只是初始化静止单机构正负例，不是Nav2/实际供盘或订单。
- 最新源码正例`p4-hinged-retainer-current-positive-20261004-01` exit0/37.996540s，负例`p4-hinged-retainer-current-negative-20261004-01` exit0/41.233532s：预期缺接触字段明确为expected_no_closed_contact，actual_closed_contact=false、无运输授权。初始视觉UNKNOWN拒绝闭合已有短测试边界检查，但该遮挡分支自身物理注入NOT_RUN。单机构尚未接入导航/货权/卸盘，deck yaw修复/第二车/3D保护均NOT_RUN。
- 最终全量exit0：1105 passed/1 skipped，176.75s，runtime/pytest-hinged-retainer-20261004.log；所有物理/pytest均退出，无后台续跑承诺。冻结37资产/135源码/110原阈值PASS。旧v5 SHA未变，v6失败与全部原失败保留。
- P4-BELT-03 PARTIAL、010未完成。下一步：甲板偏航诊断和新包络/真实保持互锁（含UNKNOWN注入）→初始化移动重验→真正连续链与3D/恢复/P5/P6/P7。详见history/SEPARATION_AND_RETAINER_20261004.md。下述RUNNING/待批准条目都是历史，不代表现状。

## 当前状态 · 2026-10-04 · 已证实短暂离地，接触柔顺候选

- 精确快照`p4-retained-support-snapshot-20261004-01` exit1/254.805s：保持3.238592mm、货物停止0.254241mm/s及姿态/车停/数量PASS，但sim37.352时托盘与所有deck roller无接触（rows=[]），只剩零件→托盘接触；命令age0.144s有效。不是只看邻近采样猜视觉误报，禁止屏蔽UNKNOWN。
- 零步检查：原implicit-fast integrator=3、Newton solver=2、100iterations、2ms timestep；原deck/tray solref=[.02,1]。不再把“换implicit-fast”当未做的修复。
- 新opt-in只改首物理步前c_tray_floor/c_deck_roller*接触time constant .02→.01s，保持原damping ratio=1、几何/摩擦/步长/策略/所有验收不变，完整列出变更geom与before/after，qualified=False。7项候选纯测试PASS（0.33s）；最新全量1087/1skip早于本增量。
- 唯一worker `p4-retained-deck-solref01-20261004-01` PID690/start_ticks1088885/session84072/720s上限，90sim接收站诊断；此前快照worker已退出。最终待证据。冻结35/131/110，V1与P4-BELT-03均未完成。
- 下一步：接触候选物理结果/失败保持→证据驱动修正或实际连续装载重验→3D/中断恢复/P5/P6/P7。不改001～009，不推送GitHub。

## 当前状态 · 2026-10-04 · 全量通过，精确支持快照运行

- 全量已exit0：`runtime/pytest-retained-multiccd-20261004.log`，1087 passed/1 skipped，166.70s。冻结35资产/131源码/110原阈值检查通过。此前pytest RUNNING信息过期。
- 唯一物理worker `p4-retained-support-snapshot-20261004-01` PID33688/session19578/480s上限；45sim上限、0.08m/s/multiCCD/原低加速度同条件，只增原相机时刻的接触力与行对快照，不改UNKNOWN门/任何阈值。最终待证据。
- P4-BELT-03 PARTIAL，V1未完成；真实连续新候选Nav2/卸盘、3D保护/恢复/P5/P6/P7未验收。下一步原时刻支持诊断→证据驱动修正→实际连续装载重验。

## 当前状态 · 2026-10-04 · 限速保持通过，瞬时支持仍阻塞

- 所有物理worker已退出。`p4-retained-multiccd-speed08-20261004-01` exit1/208.200s：0.08m/s候选保持4.672091mm PASS、全部货物停止0.195561mm/s PASS、车停止/姿态/计数PASS；但sim30.498支持UNKNOWN锁存，距目标2.856501m，Nav2/到达FAIL。邻近sim30.402/30.502/30.602支持均deck，尚不能断言是触觉仪器还是短暂离地。
- 新增相机采样原时刻的support_rows/托盘接触geom/dist/dim/接触力/指令age诊断快照，仅独立评测。支持UNKNOWN门不屏蔽、不宽限、不自动恢复，原物理与所有判据不改。此新增仪器尚未物理复测。
- 当前只运行完整pytest（session30383），日志`runtime/pytest-retained-multiccd-20261004.log`；不与任何物理探针并发。最终结果待日志。冻结35资产/131源码/110原阈值；相关纯测试12PASS。P4-BELT-03 PARTIAL，V1未完成。
- 下一步最多三项：全量结束后精确时刻支持诊断；据原始接触力选择数值/机构修正而非屏蔽门；移动通过后重验真实供盘装载/Nav2/停靠卸盘，再3D/恢复/P5/P6/P7。下述活动状态均为历史。

## 当前状态 · 2026-10-04 · 接收站距离保持仍失败，速度单变量对照

- 下述活动信息已过期。短程multiCCD+retained候选14/14PASS保留；接收站距离`p4-retained-multiccd-receiver-20261004-01` exit1/124.777s，载荷UNKNOWN撤销运动，车辆/货物停止PASS，保持7.963mm FAIL，无到达。首次5mm越界sim12.5先于support UNKNOWN sim15.736，不允许说只是视觉误报。
- 同条件只加记录的`p4-retained-multiccd-contact-diagnostic-20261004-01` exit1/128.893s，保持11.193mm FAIL、cargo速度1.450mm/s PASS；近拒绝时指令age0.07～0.176s仍<原0.5s。增加实际接触对/距离/dim与指令age，只用于独立诊断，不馈入控制。再次失败不刷实际千秒长链。
- 唯一物理worker `p4-retained-multiccd-speed08-20261004-01` PID30607/session63449/720s上限，仅将命令线速度上限0.15→0.08m/s；加减速度、原内部goal、所有安全/停止/保持门与multiCCD物理一致。参数通过ROS显式overrides并记入run，不覆盖默认文件。10项相关纯短测PASS；本轮全量待物理退出。
- P4-BELT-03 PARTIAL/V1未完成，实际新候选供盘装盘/卸盘、3D/恢复/P5/P6/P7未通过。下一步最多三项：单变量速度对照及接触解释；有移动证据后重验真实供盘装盘链；负例/恢复与全量验证。

## 当前状态 · 2026-10-04 · 多点接触候选重验

- 更新：初始化短程`p4-retained-multiccd-short-20261004-01`已exit0/113.345s，14/14PASS。到达70.697mm，货物保守停止速度0.079900mm/s、漂移0.008038mm、托盘deck-relative保持1.111mm，均过原门。现唯一worker为`p4-retained-multiccd-receiver-20261004-01` PID26566/session65417/720s上限，验证初始化载荷接收站距离；下述短程活动信息过期。

- P4-BELT-03仍PARTIAL，010 V1未完成。下述历史活动PID均已退出。第三真实连续链供盘/装载成功，但导航姿态/保持/货物停止失败的证据全部保留，不回填旧报告。
- 安装版MuJoCo 3.3.6的MULTICCD位实际为0（关闭）；baseline蓝柱每帧仅1个地板支持接触。独立初始化候选启用位16，几何、摩擦系数、2ms步长、原100Hz策略与验收阈值不变。`p4-cargo-rest-multiccd-20261004-01` exit0/70.170s：每帧5个支持接触，蓝柱静止保守点速度0.119680mm/s，托盘/两红件均<0.009mm/s，全部低于原10mm/s。只证明静止诊断，不是带载订单。椭圆锥与impratio10均FAIL且更差，未接入主链。
- opt-in `--multi-ccd`仅在首个物理步前配置；原默认资产不变。相关纯测试19项PASS；冻结35资产/130源码/110原阈值PASS。最近完整1067 passed/1 skipped早于新增量，待物理结束后复验。
- 当前唯一物理worker：`p4-retained-multiccd-short-20261004-01`，PID25668/session90733/480s上限；初始化带载短程Nav2+低加速度候选，最终待证据。没有运行001～009、下载或推送GitHub。
- 下一步最多三项：短程及接收站距离验证全部货物停止/保持；重验实际H供盘→真实三件装载→Nav2/精确停靠/卸盘独立裁判；之后3D保护/中断恢复/P5/P6/P7。不能因静止PASS宣称完成。

## 历史状态（活动信息过期）

## 当前状态 · 2026-10-04 · 姿态保护与静止货物诊断

- P4-BELT-03 PARTIAL，010 V1未完成。第三实际连续链`p5-continuous-loaded-nav-v5-20261004-03`已exit1/989.970s：供盘/三件装载/源→甲板通过，Nav2实际运输4.209659m；`c_deck_yaw=-0.010001610rad`跨越原±0.01rad门。此次锁存运动但保留制动，未SIM冻结；车辆停稳PASS（速度5.23408e-6m/s、漂移6.45723e-7m）， cargo停止FAIL、相对甲板保持FAIL（29.816mm > 原5mm）。原第二轮下层原因仍缺失，本轮具体原因不能回填旧报告。独立裁判`p5-continuous-loaded-nav-judge-20261004-03`exit1；不卸盘、不释放源/甲板占用资源。
- 独立`retained-motion`控制候选只降低命令速度/加速度，内部到达目标收紧为80mm以满足原120mm技能门；不改原验收阈值。`p4-retained-motion-short-20261004-03`初始化短程：Nav2/到达72.615mm/车停止/姿态/视觉/支持/轮控归还通过，保持2.405mm通过；全部货物停止FAIL（21.142mm/s）。这是初始化诊断，不是真实连续链。首轮初始化IndexError、第二轮不匹配的90°摆放导致视觉UNKNOWN，全部失败保留，不作为控制效果证据。
- 静止无ROS/无导航基线`p4-cargo-rest-20261004-01`exit1/73.074s：蓝柱仍21.573mm/s，其他货物≤3.024mm/s；实际接触dim6、原系数已含扭转/滚动摩擦，不能靠再加condim修复。单独椭圆锥`p4-cargo-rest-elliptic-20261004-01`exit1/74.091s，蓝柱61.694mm/s、更差，不接入主链。当前唯一物理worker `p4-cargo-rest-elliptic-ratio10-20261004-01`，PID23572/session49155/180s上限，仅数值接触候选；原几何、摩擦系数、2ms步长/100Hz神经策略不变。
- 最近全量1067 passed/1 skipped（174.23s）早于随后state/控制候选与诊断增量；82项相关短测PASS（6.94s），最新还需全量复验。冻结35资产/128源码/110原阈值PASS；原v5资产SHA aa3e9085bf08236193537418f5cb1d32d6254f5cf0b2b7991f4796e82a2a8ac5不变。没有改001～009或推送。
- 下一步最多三项：确认静止蓝柱数值接触根因/候选；正例重新验证实际连续导航停靠卸盘与负例；再推进3D保护/显式恢复、P5/P6/P7。不能因车辆刹住、单项或静止候选PASS称010完成。

## 历史续作 · 2026-10-04 · 下述活动信息已过期

- P4-BELT-03 仍 PARTIAL，010 V1 未完成。新增 `continuous_nav_session.py` 只作为已有 plant/model/data/WorldOwner 的 guest：保留原 H 零速度策略和机械臂保持，只有接收站 MOVE 接入 Nav2；DOCK/UNDOCK 不被替换。车载 RGB-D、原载荷许可门、确认停止后的轮控归还及独立货物速度日志一并接入。
- 首轮 `p5-continuous-loaded-nav-v5-20261004-01` exit1 / 945.766s：H供盘PASS、实际三件装盘/两红一蓝视觉PASS、源端到甲板转移与前五技能SUCCEEDED。Nav2未启动，因代码错误地拿原spawn先验核对加载结束的source dock，`DECLARED_SOURCE_LOCALIZATION_PRIOR_UNCONFIRMED`。保留失败与全部检查点，不称导航或订单通过。
- 修正只改先验来源：使用初始化时由夹具导出的source dock声明，原5mm/0.01rad前置核对不放宽。ROS worker接收声明的`--start`与`--goal`；没有把运行中的本体/货物真值发送给AMCL或视觉。失败分支单独尝试原轮速制动，不能自动重授权/释放占用资源。
- 第二轮`p5-continuous-loaded-nav-v5-20261004-02`已exit1，1013.529s。实际供盘/三件装载及前五技能通过，真实Nav2搬运4.164397m，距接近目标尚0.459134m时在sim171.604冻结`CONTROL_UNAVAILABLE`；旧报告未保留下层原因，不能断言具体关节根因。无卸盘、无轮控归还，占用货物/资源保留。
- 独立重判`p5-continuous-loaded-nav-judge-deck-frame-20261004-02` exit1。旧19.040mm是托盘相对车体变化，不是原G-2托盘相对甲板滑移；甲板有柔顺自由度，旧记录缺deck-frame，保持判据为缺证据FAIL而非实测滑移超限。旧第一次裁判与原报告均保留，不补造数据。
- 新增TransportPostureGate：原姿态界限不变，失败撤销运动并锁存，但继续原轮速制动；不再主动在control内抛异常禁用制动。新增原因链/关节原值/甲板相对日志与失败时停止自有Nav2进程的退出路径。73项相关短测PASS（8.99s），冻结33资产/125源码/110原阈值检查通过，非本轮全量。
- 短長程诊断`p4-loaded-nav-receiver-diagnostic-20261004-01`已exit1/308.300s：初始化托盘瞬时甲板支持UNKNOWN导致LOAD_ENVELOPE_UNKNOWN锁存；计数仍RESOLVED，未复现旧control冻结；旧退出路径等待仍在执行的Nav2超时，已修。第二初始化诊断`p4-loaded-nav-source-retention-diagnostic-20261004-02`已exit1/158.376s，bridge STATE_STREAM_STOPPED_EARLY（最后sim20.452），未复现姿态失败；这轮退出前没有新增制动证据，不宣称机械停止。诊断均已退出。
- 完整回归`runtime/pytest-continuous-nav-20261004.log`：1067 passed/1 skipped，174.23s，exit0；它早于随后新增的state调度修正。新增纯StatePublishSchedule：模拟时间慢于渲染时按墙钟补发**新推进的真实状态**，严禁重复/倒退sim时间、虚构clock或额外physics；bridge仍5s watchdog、载荷仍0.5s。新增RGB-D/control/checkpoint峰值计时与初始化探针异常后的有界制动；80项相关短测PASS（9.60s），冻结33资产/126源码/110原阈值PASS，最新增量尚待完整回归。
- 当前唯一物理worker：`p5-continuous-loaded-nav-v5-20261004-03`，PID14092/session49690，1800s墙钟上限；完整实际供盘/装盘后的连续Nav2复测，最终结果未出。启动标识以bounded manifest为准。没有修改001～009或推送GitHub。
- 下一门仍为该连续候选实际到达/停稳/精确停靠/卸盘及独立裁判，随后3D保护、载荷中断/显式恢复与P5/P6/P7；不能把Nav2组件接通视为完整订单。详见history/CONTINUOUS_NAV_20261004.md。

## 当前状态 · 2026-10-03 · 车载 RGB-D 与居中带载 Nav2

- P4-BELT-03 仍 PARTIAL，010 V1 **未完成**。本轮只完成独立初始化载荷的车载感知/短程导航候选，不能代替连续真实供盘装盘或完整订单。
- 车载斜视 RGB-D 在正常、初始移位场景定位/两红一蓝计数通过，遮挡和缺深度返回 UNKNOWN。原 6mm 形状及 20mm 定位门未改；最新无多重采样候选 `p4-vehicle-rgbd-optics-20261003-06` exit0。垂直视角和高采样失败均保留。
- `p4-vehicle-rgbd-motion-20261003-06` exit0，78.853s，8执行项PASS：实际行驶0.118544m，托盘相对滑移0.00070735m，车辆停止速度2.08389e-6m/s/漂移2.95630e-8m。载荷时效门仍0.5s；不是硬实时性能保证，制动阶段仍可能出现慢帧。
- 行驶许可与制动权限已分开：运动中载荷UNKNOWN撤销行驶并锁存任务，新的图像不自动恢复；原轮速制动可继续推进物理。`p4-vehicle-rgbd-motion-20261003-05`任务FAIL但实际制动PASS保留；`p4-vehicle-rgbd-missing-20261003-01` exit1、任务FAIL/负例安全PASS，零行驶指令、无安全冻结、车辆停稳。它们不是任务成功，也不是订单恢复完成。
- 宽载荷Nav2 `p4-initialized-load-nav2-20261003-01` exit1：9节点ACTIVE但动作超时，另有数组报告序列化错误（已补修/短测/原子检查点，旧run不补造丢失真值）。零步诊断证明初始宽足迹与 `p5_arm_pedestal_geom` 投影重叠22.192mm；没有删障碍或改旧配置。
- 独立居中候选许可域收紧为托盘中心y±60mm（包含测量不确定度）；目录球半径、机构余量、0.5s时效和原验收门不变，越界就拒绝。`p4-centered-clearance-20261003-01`仅零步投影诊断，无静态扫描高度重叠，不冒充动态防撞。
- `p4-centered-load-nav2-20261003-01` exit0，97.791s：初始化载荷同一世界Nav2/实际到达/车辆停稳/RGB-D数量及甲板支持/控制权归还11执行项PASS；到达误差0.124910m，停止速度2.19802e-6m/s、漂移3.84533e-8m。这是执行探针判决；独立载荷保持、载荷停稳与3D碰撞裁判尚未补齐。
- 本轮完整回归`runtime/pytest-vehicle-loaded-nav-20261003.log`：1022 passed/1 skipped，169.20s，exit0；冻结33资产/122源码/110原阈值检查通过。全部本轮物理探针和pytest已退出。此前997项保留但不替代本轮结果。
- 下一门：在同一实际人形供盘、RGB-D/PCL装盘后的自由托盘链中接入上述Nav2与精确停靠；独立3D/货物停止守卫；继续P4故障/显式恢复、P5完整订单、P6双车/库存及P7的36实例。没有修改001～009或推送GitHub。详细证据见 `history/VEHICLE_LOAD_NAV_20261003.md`。

## 上一检查点 · 2026-10-03 · 共同世界真实 Nav2 接入

- P4-BELT-03 仍 PARTIAL；010 V1 未完成。此前连续真实供盘、三件装载、物流与货权诊断 PASS 保留，不冒充完整订单。
- `p4-shared-world-nav2-empty-v5-20261003-02` 实际 exit0：空载同一 v5 世界，9 个 Nav2 节点 ACTIVE、唯一 /cmd_vel 发布者 collision_monitor、83 个非零速度消息，车侧收到59条命令；实际到达误差0.142082m，停止速度2.48648e-8m/s、漂移1.72805e-8m，无安全冻结。只证明空载短程接通，不是带载/完整订单。
- 新增几何审计、独立配置生成及最小启动器；完整车载机构的 x 包络延至车体前方2.51m，不能使用旧裸车0.26m半径。静态地图是初始化测绘先验，**不是SLAM/3D防撞**；载荷包络仍NOT_COMMISSIONED。
- 首轮 `p4-shared-world-nav2-empty-v5-20261003-01` exit1 保留：启动器未展开行为树包路径，修正为 ament 安装包解析，不修改旧配置。
- 重启前全量回归已落盘：`runtime/pytest-shared-nav-20261003.log`，972 passed / 1 skipped，179.94s，exit0。重启后检查原29资产/112源码/110阈值冻结及独立profile生成一致；没有残留010物理探针。该回归早于以下载荷包络与车轮交接增量。
- 第三轮 `p4-shared-world-nav2-empty-v5-20261003-03` 已exit0（75.825s wall）。独立裁判 `p4-shared-world-nav2-empty-judge-20261003-03` exit0，12/12 PASS，停稳窗口显式记录；空载 Nav2 物理探针全部退出，下方活动信息全部为历史。
- 重启后完成载荷保守包络候选（14项测试/生成与--check通过），以初始化甲板支撑区及托盘/零件目录球半径推导，包含所有朝向；**运行中RGB-D包络门未接线，真实载荷保持/Nav2仍NOT_RUN**。
- `p4-shared-nav-parking-handoff-20261003-01`实际exit0，98.446s wall：停车→原始轮速伺服供Nav2独占→确认物理停止/关闭IPC与导航进程→归还停车，8执行判据PASS，到达0.135566m，停止速度1.2574e-8m/s/漂移1.0066e-8m；归还后同owner继续1s无冻结。独立空载导航裁判`p4-shared-nav-parking-judge-20261003-01` exit0，12/12 PASS（独立裁判只覆盖导航，交接本身以实际执行报告和控制器身份测试为证，不冒充带载交接）。
- 车轮交接+旧停车+拓扑21项测试PASS；新增“COMPLETED后发生异常”负例先复现误PASS，再修复，独立裁判16项PASS。旧运行保留，新异常路径不留COMPLETED。以上物理探针全部退出。
- 最新增量全量已结束：`runtime/pytest-loaded-nav-authority-20261003.log`，997 passed/1 skipped，269.65s，exit0；31资产/115源码/110原阈值冻结检查通过。所有本轮物理探针和pytest已退出，无后台续跑承诺。
- 下一步最多三项：初始化前车载RGB-D候选与遮挡/过期拒绝；3D机构碰撞观测；接入连续装载后的真实Nav2运输及精确停靠。双车有限库存、故障恢复、完整订单和36实例/P7尚未完成。
- 本轮详细证据与边界：`history/SHARED_NAV_COMMISSIONING_20261003.md`。

## 历史运行索引 · 此处活动状态已过期，以顶部及实际报告为准

- 本轮运行全部已退出：连续正例exit0（924.748s wall）、全量938 passed/1 skipped（161.79s）与更严格供盘负例exit1均结束。负例独立裁判 `p5-policy-supply-rejected-judge-20261003-01` exit0，13/13安全PASS：未装盘、未运输、未提交货权，原SUPPLY_UNCONFIRMED锁存SIM冻结；任务FAILED保留，机械停止/恢复NOT_RUN。没有本轮后台仿真，旧PID/session条目仅作历史；P4-BELT-03仍PARTIAL，下一门为共同世界真实Nav2载荷运输及运行中中断/恢复，不更换仿真器或阈值。
- 最新全量回归已exit0：`runtime/pytest-policy-continuous-20261003.log`，938 passed / 1 skipped，161.79s。当前单物理worker为更严格供盘负例 `p5-policy-supply-rejected-v5-20261003-01`（session16689，600s wall；PID572由新run记录/启动标识核验，不用旧PID判所有权），结果待出。连续正例和全量pytest均已退出；下文活动条目属历史。
- 连续候选已exit0：`p5-continuous-policy-transactions-v5-20261003-01`，924.748 s wall，接收停止窗口结束sim190.418 s。同世界H真实供盘→在线RGB-D/PCL三件装盘→11物流技能→两段货权→接收核验，12项顶层诊断判据PASS、接收/货权七项PASS。加载后底盘/甲板yaw均0；接收速度界0.000614647 m/s、漂移0.0000132293 m，阈值未改。源端/甲板FREE，接收仍OCCUPIED。原摔倒对接阻塞已在此实际连续链上解决。Nav2/订单/双车/有限库存/故障恢复/36实例仍NOT_RUN，P4-BELT-03保持PARTIAL，V1未完成。PID572已退出；当前仅全量pytest session16562，900s deadline，日志runtime/pytest-policy-continuous-20261003.log，最终结果待出。此前活跃进程条目均为历史。
- 站稳候选对照已完成：`p4-post-supply-policy-hold-20261003-01` exit0，446.939 s wall；供盘10项PASS，随后原网络零速度保持90.002 s，末态高度1.014637 m/倾角0.15143°，未冻结。它只验证供盘后的站稳；当前单物理worker为 `p5-continuous-policy-transactions-v5-20261003-01`，PID572/1800 s，验证同世界真实供盘→RGB-D/PCL装盘→物理物流→两段货权→接收核验。最终结果待报告，V1不标完成；禁止覆盖运行使用的helper。下文PID673已退出。
- 装盘接触诊断已exit1（935.488 s wall）。实际快照确定H基座降至0.35856 m、左脚与n_chassis接触、车体被抬至0.09900 m且车轮无地面接触；不是正常轮式对接误差，供盘后连续人形监督缺失。新本体失稳permit沿用原0.65 m/35°守卫、仅SIM冻结；真实源端冠面残差替代旧lateral=0缓存仪器。35项相关短测通过。详见 history/POST_SUPPLY_STABILITY_20261003.md。
- 静态保持对照已exit1，371.218 s wall：不装盘的原H2供盘后，在sim118.454 s触发BODY_FALL（高度0.74916 m、倾角35.0763°），最终写者实际锁存SIM冻结。机械臂装盘不是失稳的必要条件。当前单物理worker：`p4-post-supply-policy-hold-20261003-01`，PID673，1200 s wall；同90 s预算，退出第一周期选择原网络零速度控制。候选尚未通过，不称修复完成；禁止覆盖使用中的helper。
- 共同世界ROS IO短测 `p4-shared-world-ros-io-20261003-01` exit0，43.2795 s wall：同model/data、201状态发送/184接收、184扫描/clock/轮式odom、解码拒绝0；启动前17帧差额明确保留。无速度发布者，不是Nav2或停止验收。详见 history/SHARED_WORLD_NAV_IO_20261003.md。
- 小偏航初态-0.02 rad的10 s停车诊断可回正（约1.96e-7 rad），末态跨机构接触只见车轮-地面；装盘初态校准读得left=right=+1，不支持“轮向符号错”这一假设。它们不能反证实际加载后接触/控制失效，因此使用上述实际场景诊断。
- 最新已结束全量回归：`runtime/pytest-heading-supply-20261003.log`，899 passed / 1 skipped，352.84 s，exit0；对应新增共同世界导航IO/ROS lease之前的源码，不替代这些新文件的全量验证。新增导航时效/无命中射线/控制互斥短测14 passed；ROS共同世界候选未实际接入Nav2。
- `p5-continuous-heading-transactions-v5-20261003-01` 已exit1：H供盘10项（含原六阶段和异常接触）全部PASS，10项装盘判据全部PASS；首次MOVE成功，但DOCK_OUT_OF_TOLERANCE（甲板yaw -0.206224 rad）拒绝，全链FAIL。加载后底盘yaw -0.127208、甲板相对yaw -0.0000139 rad；不是甲板相对底盘自由转动导致这段大偏航。首因优先生效，未再拿空接收端感知错误覆盖DOCK失败。当前短诊断 `p4-heading-small-yaw-contact-20261003-01` 180 s墙钟/10 s仿真，记录轮命令与跨机构接触，不再直接刷连续长链。
- 新停车候选已完成同条件初始-0.18 rad对照：原轮制动终态底盘-0.196559 rad、甲板相对-0.00000757 rad；差速反馈及实际安装路径复测都回到数值接近零，约90 s wall/30 s sim。只证明初始偏航停车恢复，不是H连续供盘或载货Nav2成功。候选以显式`--heading-feedback`启用，旧默认不变；输送阶段也持续通过原轮速伺服制动。
- `p5-supply-rejected-heading-v5-20261003-01` 实际exit1：人形失败、没有任何装盘phase/运输row/货权提交，最终写者SUPPLY_UNCONFIRMED冻结，保留FAILED任务结局。独立安全裁判-01误把冻结dict当bool而FAIL，原误报保留；修类型后-02对同一原始证据13/13 PASS。这是已录物理证据重判，不是又跑了一次；仅仿真冻结，机械停止和恢复NOT_RUN。
- 供盘helper现加入原`tray_task`六阶段裁判和原脚接触/异常碰撞采样仪器，状态必须六阶段全PASS；不以人类summary字符串当枚举，不另造脚角度阈值。上述连续heading run已实际通过该10项，但仍是受限H2重验，不自动等于原全部9条H2机构判据；更严格helper负例尚未重新物理验证。采样契约从全文件限制改为仅检查supply函数，避免站姿控制合法tick条件导致测试误报；8项相关测试通过。
- `p5-continuous-geometry-transactions-v5-20261003-01` 已 exit1：H供盘前置PASS、10项真实装盘判据全PASS（含完整碰撞几何落格），但 DOCK_OUT_OF_TOLERANCE，甲板世界偏航 -0.187305 rad；初始 MOVE 已成功，DOCK 拒绝后没有输送。后续错误地观察空接收端产生 RECEIVER_PCL_PLANE_UNKNOWN，已补首个物流失败即时退出/保留原因的3项契约测试，不改对接阈值。独立停车偏航诊断正在执行，不能尚未测量就断定是底盘或甲板偏航。
- 供盘预期失败独立裁判11项、航向反馈数学7项、首因优先3项短测共21 passed。航向反馈仅为未接入主链的候选控制，不是Nav2、不是已修成功。`p4-heading-hold-baseline-20261003-01` 单worker/300 s墙钟上限，仅初始偏航故障停车诊断。
- 上述连续 run 使用1800 s有界预算；实际没有耗尽预算，而是对接先失败。完整落格与不覆盖阶段检查点已经落盘，全链判决FAIL，不沿用此前RUNNING。
- `p5-humanoid-loaded-transactions-v5-20261003-01` exit 124 / WALL_TIMEOUT，900 s watchdog 终止；供盘8/8、空盘确认后实际三件装盘和首次 MOVE_TO_STATION 已推进，但无完整最终报告，不能记全链通过。保存 run_manifest、stdout 和已落盘 H 子报告。由组合链实际耗时为新 run 选择 1800 s 有界墙钟预算，不改任一冻结速度、空间或时间判据。
- 新持续站姿计算仅在 stand_weight=arm_weight=1 且 stand_target 有效时，跳过已被反馈律完整覆盖的行走网络输出；不足全权重时回退原控制器。16 项 fake 状态向量对照与回退测试通过，当前真实连续运行重验。该 Runtime 后续只允许站姿保持、不再接回步行策略；不能拿本次优化声明行走性能。
- 装件中心落在料格不足以证明完整物体落格：新增独立碰撞 geom 姿态/全包络 judge，沿用原 10 mm footprint 容差；非支持几何 UNKNOWN，圆柱包围盒为保守值。5 项数学短测通过，物理包含结果以当前 run 为准。阶段检查点互不覆盖且不声称订单完成。合计本轮相关短测 39 passed。
- `p5-humanoid-visual-loading-v5-20261003-01` exit 1（719.00 s wall）：H 前置 8/8 PASS、抬盘 0.126354 m，视觉源端送料/原停稳确认 PASS，三件实际抓放/终态计数 PASS；但初始空盘观测因机械臂遮挡 UNKNOWN，顶层正确 FAIL。发现准入代码仅记失败却继续装盘，已修：至多两次真实退臂重观测，仍不确认则关闭许可冻结、禁止抓放。不掩码遮挡、不改 75 mm 高度/6 mm 尺寸门；23 项聚合/空盘/源端/控制采样契约通过，物理修复待当前复测。
- 人形 IK 只在 Runtime 实际接收目标的 tick % 5 == 0 求解，避免 4/5 未使用计算；阶段时长/轨迹公式/物理时间步不变。新增结构测试，不以性能优化替代物理验收。
- `p4-loaded-transactions-pcl-v5-20261003-02` exit 0，334.50 s wall / 101.526 s sim：在线原生 PCL、实际三件装盘、11/11 物流技能、两段账本物理确认、接收视觉两红一蓝三格、真实支撑/原停稳窗口/缺深度 UNKNOWN 全部 PASS。最终源端和甲板经实际清空证据 FREE；接收缓冲区仍 OCCUPIED、归属 loaded-diagnostic。接收停稳保守界 0.000530730 m/s、漂移 0.000003578 m。仍为初始化托盘供应、理想本体状态单车，不是连续 H/完整 Nav2/双车订单。
- v5 -01 虽物理交接与原生 PCL 正常，接收轮廓混入独立把手片导致 51.445 mm 尺寸差/UNKNOWN，顶层 FAIL 原样保留。新增像素连通片方法选择长壁轮廓，原 6 mm 尺寸门不变；录制帧重析 5/5 PASS、读者短测 9 passed，再由上述新在线 -02 验证，未覆盖旧报告。
- 真实事务 v4 复测 `p4-loaded-transactions-v4-20261003-02` exit 1：已经上车，源端不再接触、甲板确实支撑，但载货甲板停止速度保守界 0.208773 m/s（原限 0.01），停稳 FAIL。账本进入 NEEDS_ATTENTION，货权 TRANSFERRING，两端 OCCUPIED，未误释放。前次 -01 为许可闭包计数变量被技能序列覆盖的接口错误，已更名并加结构短测，失败报告保留。
- 新独立 v5 在 v4 第一台车甲板加 14 个分段被动支撑辊；嵌套于真实甲板、随车运动，原执行器/接触参数/阈值和旧世界保留。静态 12/12、原稳定性正负例 2/2 PASS；不是载荷停稳证据。新增父坐标系测试与在线 PCL 拒绝/许可结构测试 9 passed。
- `p4-loaded-receiver-v4-20261003-02` 已退出 exit 0：实际三件装盘、11/11 物流技能、接收端实际停稳与支撑、独立 RGB-D 两红一蓝三格计数及缺深度 UNKNOWN 全通过。304.35 s wall / 97.83 s sim。接收停稳界 0.001044 m/s、漂移 0.000004085 m。仅单车初始化供盘诊断；H 连续供盘、Nav2、资源事务、全订单仍不由此证明。
- 已实际编译并运行原生 PCL 1.15.1：`p4-pcl-recorded-floor-20261003-01` 2/2 PASS，VoxelGrid＋RANSAC 平面分割、缺云 UNKNOWN。该报告只验证录制帧；当前在线探针才验证每次观察的实时平面。零件颜色/轮廓/计数仍为受限 NumPy 几何，不宣称全部感知是 PCL。
- 新资源授权交接保持 OCCUPIED，不先伪造 FREE 再启转移；资源/货权/物理事务会话纯契约 122 passed。不得用纯测试替代上述真实交接判据。

### 较早记录（以下 RUNNING/NOT_RUN 已被顶部实测状态更新）

- 最新接收端强化验收：`p4-loaded-receiver-20261003-01` exit 1。11 个技能成功且三件真实接触托盘，但接收段停稳 FAIL（速度保守界 0.180842 m/s，原限 0.01），视觉轴对齐模板报 UNKNOWN；不接受原空间子项作为完整交付。
- 已修顶层 profile：要求运输时，运输 FAIL/UNKNOWN/NOT_RUN 必须阻塞顶层 PASS；新增 7 项短测。实际 `transfer()` 入口已加入四故障全局 writer 物理复测，`p4-global-transfer-writer-20261003-01` 24/24 PASS，仅仿真冻结、不是机械停止。
- 独立候选 v4 在 v3 接收排增加 38 个分段被动支撑辊；旧 v3 不变、执行器不变、阈值不变。首版静态审计把支撑辊误算成主辊，报告 FAIL 保留；修正后静态 12/12 PASS，原稳定性正/负例 2/2 PASS。满载 `p4-loaded-receiver-v4-20261003-01` exit 1：停稳和接触 PASS，但视觉模板 FAIL。离线真实帧上沿＋各零件自身轮廓重析 `p4-receiver-recorded-rim-20261003-02` 5/5 PASS；在线新源码 `p4-loaded-receiver-v4-20261003-02` 当前 RUNNING。
- 接收端视觉增加点云最小面积矩形拟合及托盘自身坐标分格；只用 RGB-D、相机标定、已知目录尺寸/接收工装高度，不读取货物/托盘运行真值。受限颜色/几何方法不是 PCL，旋转定位及缺帧短测通过；物理相机结果以复测为准。
- 新 `PhysicalTransferSession` 复用原资源/货权账本：运动前预留并确认接驳两端占用、货权进入 TRANSFERRING；缺接收/源清空/停稳确认走 NEEDS_ATTENTION，超时不释放占用资源。8 项纯契约短测通过。实际载荷链接入仍 NOT_RUN，不用纯测试冒充事务物理验收。

- 续作实测：三个料位单红件分别 13/13 PASS；候选 v3 同世界连续装两红块＋一蓝圆柱，三次真实抬起、三格与真实支撑、独立 RGB-D 计数均通过，9/9 PASS，`reports/p4-multi-loading-bias-comp-20261003-01`。托盘在独立初始化装盘位，仍不是连续人形供盘/完整订单。
- 多件过程已保留所有失败：低采样圆柱尺寸不足、腕部遮挡、后侧观察姿态触发 IK 初值局部解，以及闭爪错误保持未到位姿态。最终版本用 640×480 采样、后侧退臂、标准姿态备选 IK 初值、保持最后目标、有上限到位等待和机器人模型偏置补偿；不修改原物理/尺寸判据。理想双指触觉不确认时锁闭仿真，不再空手继续。
- 真正 transfer 循环原来仍直接 mj_step，已改为走共享 writer 的 commit；终态保持也走许可门。新输送提交/守卫/终态短测 21 passed（0.27s）。旧默认未附 writer 的行为保留；不能说历史全部路径已强制迁移。
- 最新全量 771 passed / 1 skipped、192.99s、exit 0，`runtime/pytest-world-owner-20261003.log`：对应多件/transfer 后续补丁之前的源码检查点，不替代最新代码回归。新增计数/库存/输送提交测试后的最终全量仍 NOT_RUN。
- `p4-loaded-continuous-20261003-01` 已退出，exit 0，310.17s wall / 96.158s sim：实际三件装盘之后，同一托盘连续经过接盘/运输/卸盘等 11 技能，11/11 SUCCEEDED；终点三个零件仍在三格，运输空间诊断 PASS。未重置世界/货物。仍是旧本体状态物流控制；Nav2、货权事务、接收端独立视觉、终态三件实际接触/速度窗口、人形连续供盘 NOT_RUN，不能标完整订单。原始 PASS 范围只是装盘＋额外运输空间子项，后续需强化顶层必需项聚合。

### 本轮较早检查点（进度以本节顶部最新条目为准）

- 本轮新增：共享最终写者 `WorldOwner` 的四类故障诊断 24/24 PASS；真实 T800 `Runtime.step` 入口复测 24/24 PASS。只能声明显式仿真冻结，不是机械停止、事务恢复或完整订单。H3 新增可选 `--global-writer`，其完整连续长测 NOT_RUN。
- 实际运输托盘：候选 v3 初始化到声明装盘站，Panda 用 RGB-D 识别红件，物理夹持、抬起、放入第 1 格；独立 RGB-D 计数与缺帧 UNKNOWN，共 13/13 PASS，`reports/p4-real-tray-count-20261003-03`。不是固定夹具，也不是人形供盘后的连续订单；蓝色圆柱、多件连续装盘、其它料位验收另列。
- 新计数读者无订单/物体 geom 标签输入。先按颜色分离再聚类，缺帧、碎片、超过声明内容高度的遮挡返回 UNKNOWN；固定相机、声明静止装盘位和受限红/蓝件是假设，不是通用视觉/PCL。初始 Panda 挡盘引发 UNKNOWN，已用真实退臂而非放宽门槛处理；失败报告保留。
- 本轮短测试：全局写者、人形入口、读者与 H3 兼容共 42 passed（exit 0）。本轮最终全量回归尚待执行；下列 752 项是 2026-10-02 旧源码结果，不能替代本轮。
- ACTIVE_TASK 仍为 P4-BELT-03/PARTIAL。未修改 001～009，未推送 GitHub。V1 未完成；下一门是同一托盘的连续供盘→视觉装盘→物流交付及运行中中断/恢复，随后 P5/P6/P7。

### 上轮检查点 · 2026-10-02（不替代本轮验收）

- 最新：W2 分段支撑 v2 四故障/显式重新授权恢复 27/27 PASS、机械停止 PASS；集成 v3 源辊道支撑静态 12/12、整世界保姿及高接口三件载荷取消通过原门槛。原世界失败保留，甲板/接收段停止与全局最终写者仍待验。详见 `docs/history/STOP_DIAGNOSIS_20261002.md`。
- 用户已授权接手 010；011 暂停推进，001～009 未修改、未停止；没有推送 GitHub。
- ACTIVE_TASK：P4-BELT-03；PARTIAL，完整 V1 **未完成**。
- 最终回归：**752 passed / 1 skipped，177.74 s，exit 0**；日志 `runtime/pytest-stop-final-20261002.log`。冻结 22 资产 / 80 源码 / 110 阈值；独立逐文件核验确认旧 17 资产哈希不变、110 阈值完整 JSON 不变。本轮有界进程已退出。此前 740/749 项通过是较早源码结果，保留原日志。
- 用户已明确批准独立集成候选世界及重新验收。新增 `world_p5_candidate.xml` / `world_p5_candidate_v2.xml`，旧冻结资产和阈值保留；不是订单完成。
- 运行中许可门新增严格证据校验、闭锁与显式恢复接口；**原 W2** 四种中断安全冻结 13/13 PASS、机械停稳 FAIL/整体诊断 FAIL。获批支撑候选已按新报告通过停稳，不覆盖旧失败。订单事务自动恢复 NOT_RUN；共同世界最终写者的全局冻结仍待接入。
- 候选 v1 静态可达但动态稳定性 FAIL：料台与 Panda 初始穿透 36.071 mm。v2 只将料台及初始零件的横向偏移从 0.30 增到 0.50 m，独立静态审计 12/12 PASS，原 P3 稳定性复测 PASS 且失稳负例正确拒绝；不是机械停稳或订单。
- 本轮安全版本全量日志：735 passed / 1 skipped，258.96 s；后增加候选测试，最终全量结果须看本节后续记录。此时不能把候选完整全链、P5/P6/P7标完成。详情见 `docs/history/SAFETY_AND_CANDIDATE_20261002.md`。
- 修复重复卸盘驱动、卸载后无条件跑带、非法方向/预算仍运动，以及冠面高度写死为 W2 高度的问题。冻结几何和验收阈值未改。
- 第一次修复虽被旧探针判为 PASS，退出后托盘底面降到 0.362048 m，不在冠面，**不接受**；新增独立判据将该报告判 FAIL。
- 第二版卸载停止位置由接收排内冠距推导；退出后底面 0.480747 m，在冠面；重复 4 s/20 s 命令均零运动。独立诊断 11/11 PASS，**仅是空盘 W2 诊断**。
- 本轮全量回归：705 passed / 1 skipped（284.07 s），日志 `runtime/pytest-continuation-20261002.log`。覆盖终态零运动、旋转包络、三件载荷保持、制动峰值、独立最终入区拒绝负例。上一轮 684 passed / 1 skipped 是历史结果。
- 高接口 H3 当前代码正例 18 PASS / 1 NOT_RUN；负例安全验收 13 PASS / 6 NOT_RUN（托盘仍在源端、交接未提交）。正例最终底面 0.805011 m、在冠面。整盘占地进入接收区未验证，不由冠面支撑替代。
- 本轮续作：`p4-loaded-retention-20261002-01` 的三件自由零件短程保持 11/11 专项 PASS。正常 0.350067 m 行程、净滑移 1.666 mm、含制动峰值 3.881 mm；驱动故障净滑移 272.769 mm。只是 W2 初始化载荷诊断，不是机械臂装盘或高接口满载交付。
- 修复旋转物理包络：按真实 geom 姿态测量 box/sphere/capsule/cylinder，排除不碰撞视觉 geom。`p4-h3-contained-20261002-01` 旧 H3 PASS 但最终尾沿 12.196825 m < 接收冠起点 12.203700 m，纵向入区不通过。
- 第二版停止位置考虑物理 geom 方向无关包络与既定停止漂移预算，未改世界几何和阈值。高接口复测 `p4-h3-contained-20261002-02` 旧 H3 行 18 PASS / 1 NOT_RUN，独立 `p4-receiver-envelope-20261002-02` 8/8 PASS；最终纵向尾沿 12.243893 m、前沿 12.883963 m，位于接收冠区间 [12.203700, 13.243700] m。不是横向全包含、满载交付或订单。
- P4 未完成：中断恢复、运行中共享区互锁、完整接收占地与高接口满载交付。P5/P6/P7 仍未验收。
- 全链前置缺口：固定工装并非实际运输托盘；单车物流控制使用仿真本体状态，不能当成 Nav2 全链；第二车未具备运输甲板；PCL/部署建图待完成。
- 上一条的“第二车未具备甲板”是旧世界边界；新候选已经复制完整机构，但第二车实际运行/运输/跨车互锁尚未验收。库存和第二托盘供给仍未交付。
- 历史报告和第一次修复失败证据保留。详情、命令、边界见 `docs/history/TAKEOVER_20261002.md`；交接由生成器派生。
- 本轮续作详情见 `docs/history/CONTINUATION_20261002.md`；长测已退出，无本轮后台仿真。冻结为 17 资产 / 73 源码 / 110 阈值。

## 历史状态（以下截至 2026-09-29，不覆盖上述最新状态）

更新：**2026-09-29**（第十四轮至第二十九轮：**证据核查推翻 `tray_held`** + **方案 B 候选世界建成** + **H1/H2/H3 全部 PASS** + **H3 账本守卫修补**）。

> ⚠️ **本文件的"当前状态"到 2026-09-29**。更细的逐项状态以 `docs/TASK_BOARD.md` 为准（**权威**），
> 判决链以 `docs/DECISIONS.md` 为准（**D024–D115，92 条无缺号**）。领取记录只在手写的 `docs/CLAIMS.md`。

## 历史状态（2026-09-29）

**主线已按用户指导文档改向**：不再是"蹲得更深"，而是**让已有 W5 物流链真正收到一次人形供的托盘**。

| 层 | 状态 | 证据 |
|---|---|---|
| P1（含 H 链） | **完成**（H 链在候选世界里 **H1/H2/H3 全部 PASS**） | `reports/p1-gate-08`、`p1-h-seq-10`、`p4-h1-w5-01`、`p4-h2-w5-03`、`p4-h3-w5-01` |
| P2 核心 | 完成 | 变异证明 86 个，见下文历史 |
| P3 四项 | **全 DONE** | `p3-nav-05/-06`、`p3-vision-02`、`config/p3_freeze.json` |
| W2–W5 | **DONE**（W5 范围 = 单车物流闭环） | `w2-logistic-05`、`w3-mechanism-07`、`w4-skills-05`、`w5-loop-07` |
| **人形真实供盘 → 链把同一个托盘送进接收端** | ✅ **完成（H3 PASS：正例 18 判行 + 1 NOT_RUN、负例 13 判行 + 6 NOT_RUN）** | **`reports/p4-h3-w5-01`**（正例，含 `acceptance.md`/`diagnostics`）＋ **`reports/p4-h3-w5-neg-01`**（集成负例） |
| P4 蹲姿 | **已被降级为对照项**（不再当主线；H 链**只在站姿下成立**） | `reports/p4-armik-05`：诚实门下 `D_MAX = None` |

### ★ 三条必须知道的新结论
1. **`tray_held` 不是抓取验收**（`D100`）：审计实测——**命令位姿**的手-柄距 **0.0010 mm**，而
   **机器人实际的手**在 **187.7 mm** 外，差额全是伺服跟踪误差（手臂被托盘**物理挡住**）。
   P4 的托盘**从来没被抓过**（自由体、与手无任何约束）。`probe_p4_armik.py` 已拆成
   `tray_held_commanded` + 由仿真状态量的 `tray_held`。旧报告已加 `COMMANDED_ONLY.md` 标记。
2. **人形供盘的能力工程里已经存在**：`experiments/probe_h.py` + `src/humanoid007/runtime.py`
   的 `GRASP` 手指屈曲姿态，`reports/p1-h-seq-10` = 双手接触、升 **+0.1263 m**、双手持 **14.0 s**、
   退出托盘只动 **0.07 mm**，且 `free_base=True` / `equalities=0`（**真实接触，非 weld**）。
   ⛔ 但它是 `DIAGNOSTIC_ONLY`：**声明的固定工装位姿、无视觉**，且跑在**未合并的 007 世界**。
3. **H3 现在把「供盘」和「物流链」接起来了，而接法是可核对的**（`reports/p4-h3-w5-01`）：
   人形把托盘放到源端辊道并释放，**同一个 `c_payload`（body 124 / 0.098 kg，首尾同 id）** 被链取走，
   经 11 条命令送到接收端（x **4.448 → 12.596**），两段转账走完契约的 8 个阶段、
   **custody 只在 `COMMITTED` 从源端转到接收端**。判据是**四条实测的契约**：
   一个世界一个 `MjData`（plant 作 guest，**拒绝 reset** ⇒「零运行 qpos 写入」是结构性的）、
   同一个托盘实体、**下游比松手晚 16.888 s 才动**（时间顺序，不是「两件事都发生了」）、托盘上零等式约束。
   ⛔ 但**「人形把托盘交给了车」不能说** —— 交付的是「放到了源端辊道上」，物权转移是**链的** TRANSFER；
   而且**站位是声明的固定工位、无视觉、只在站姿下成立**。

### 布局：用户已选方案 B（`D101`）
`assets/world_w5_h085.xml`（建造器 `experiments/build_w5_h085_world.py`，有 `--check`）：
**一个统一的 `dz = +0.325 m`、47 个锚点**，把整个物流接口（源端辊排 + **AMT 甲板** + **接收端辊排与导向件**）
从冠高 0.485 抬到 **0.810**，托盘起点 **0.8500**（＝ H 链已验证的握持高度）。
**源世界 `world_w2_logistic.xml` 逐位未变**。⛔ **候选世界不能引用旧 W5/W3/W4 的 PASS。**

### 人形供盘链：H1 → H2 → H3 全部完成
1. **H1**（`reports/p4-h1-w5-01`）：双手抓持、升 **+0.1259 m**、持 **13.9 s**、放回、释放、退出只动 0.04 mm。
2. **H2**（`reports/p4-h2-w5-03`）：把托盘放到**停止的源端辊道**上（`c_fixed_roller_0_0`/`_0_1`）、释放、退出。
   阻断曾经是**手宽 109 mm 对辊端余量 50 mm**，由 **CHANGE 4**（源端辊半长 0.250 → 0.200，**扫出来的**）解除。
3. **H3**（`reports/p4-h3-w5-01` ＋ `reports/p4-h3-w5-neg-01`）：**把 H 链接进 W5 物流链** ——
   人形放到源端辊道并释放之后，**同一个 `c_payload`** 被链经 MOVE/DOCK/TRANSFER/VERIFY/UNDOCK 送到**接收端**，两段转账走完契约阶段、**custody 在 COMMITTED 转移**。
   正例 **`PASS: all 18 judged H3 rows [positive arm]`**（1 NOT_RUN；11 条链命令全 `SUCCEEDED`、托盘 x 4.448 → 12.596、异常碰撞 0 对）；
   集成负例 **`PASS: all 13 judged H3 rows [negative arm]`**（6 NOT_RUN；人形判 FAIL、链的 transfer/verify **全部拒绝**、托盘留在源端）。
   ★ **两臂都判的交叉行 `ledger_agrees_with_physics` 都是 PASS**——账本的 custody 与**实测支撑**一致；负例里 `xfer_source_to_deck` 停在 `TRANSFERRING`（`REFUSED_NO_PHYSICAL_DELIVERY`），**没有把物权提交到 `AT_DESTINATION`**。
   世界 `assets/world_w5_h085_loop.xml`（候选 B ＋ 把人形站位写进关键帧）；**候选 B 逐位未变**。
4. ⚠️ **H3 不是生产级工件线**：`scope = DIAGNOSTIC_ONLY`、**无视觉**、站位是声明的固定工位、**只在站姿下成立**（`D_MAX = None`）。

---

## 历史：2026-09-23（P2 核心收口）

五个核心模块已实现，**变异证明 86 个 = 85 `CAUGHT` + 1 声明冗余、`rc=0`**，
两次运行逐字节相同；恢复后全量套件 **514 passed, 1 skipped**。

## 已完成

- P0 文档与独立目录保留；本机基础环境核查（ENVIRONMENT.md）；独立 Python 3.12 / MuJoCo / MNN 环境。
- 复制资产保留许可证，86 文件来源哈希一致；**MNN 可执行栈声明已修**（`patchelf`，现哈希 `174dc8fd…`，
  原始 `10a5cf73…` 在 `runtime/mnn_original/`；**重建环境必须重做**，`experiments/fix_mnn_stack.py`）。
- T800 站立、旧物体抬升、EGL 相机输出；三段固定滚筒组件实验；ROS/Nav2 模块导入。

### H —— 人形取放 V1 托盘

- **`P1-H-03`：V1 轻空托盘已建立**。`assets/objects/tray_v1.xml`（地板 + 四面壁 + 两块隔板形成三个料位 +
  两条提手），**声明质量 0.098 kg**（对照 007 旧件的 0.210 kg）；**抓持接口刻意逐字不变**（有测试断言）。
  世界由 `experiments/make_world.py` 生成到 `assets/world_tray_v1.xml`（有 `--check` 幂等自检）。
- **`P1-H-04`：退出阶段改为两段式，六阶段在 V1 托盘上首次全部通过。** 旧行为是"退出立刻命令默认姿态"，
  速率限制器在关节空间插值，张开的双手扫过托盘（`reports/p1-h-seq-07`：12.0 mm）。新的 `ExitPath`
  先沿**笛卡尔直线**把手走到默认手部位姿，再**显式斜坡**回默认关节姿态 —— 此时手已离开托盘空间。
  **判据一行未改**，而它的两半各自都被真实运行证明会失败（关节直跳那半败在托盘位移 12.0 mm，
  只走笛卡尔那半败在姿态 0.284 rad）。权威记录 `reports/p1-h-seq-08`：六阶段全 PASS、
  退出位移 **0.07 mm**（限值 5 mm）、姿态稳定到 0.0001 rad。
- ⚠️ **`full_H_acceptance = DIAGNOSTIC_PASS`，不是 V1 验收**：在**声明的**固定工装位姿下跑出，
  **无视觉、无位姿扰动**。取证那条路径仍属 P1 之后。

### A —— 固定臂抓放

- **`P1-A-04`（DONE）**：模型 = Franka Emika Panda（Menagerie `822c2d8f877d`，Apache-2.0，
  80 文件 / 34.9 MB，逐文件哈希在 `manifest-panda.json`）。装置 `experiments/arm_rig.py` 生成
  `assets/world_arm_a.xml`；探针 `experiments/probe_arm.py`；判据 `experiments/evaluate_arm.py`。
- **权威 `reports/p1-a-p05` = PASS 14/14**：`nominal` 三档摩擦全部 `DONE`、`grip_never_closes` 三档全部
  `GRIP_FAILED`；6 个不同零件初位；**包络外 9 个 run 照跑照报但不注册**（1 停住 / 8 保守）。
- ⚠️ **包络 = 8 个枚举点，不是半区间**（沿闭合轴 +4 mm 过、**+5/+6 mm 在 μ=1.00 失败**、+8 mm 又过 ——
  平行夹爪不自定心，反被棱角收住）。**不是生产抓取**：单技能、替身台面、无视觉、无位姿扰动、单零件。
- 这一项花了四个真实缺陷：① Jacobian 列按位置取 `[:, :7]` 而 payload 的 freejoint 排在最前；
  ② 抓取点在手指坐标系里取值（回调 `(0, 0.0055, 0.0445)` 而非 `(0, 0, 0.1029)`）；
  ③ 指缝用世界 y 差分而闭合轴是手指局部 y（报出 **−15.78 mm**）；
  ④ 探针 `mj_resetDataKeyframe` 把 9 值 home 关键帧补到 nq=16，补的部分落在 payload 的 freejoint 上
  ⇒ **零件开局躺在世界原点**。另：指尖扎进台面 12 mm，台面摩擦卡住夹爪 ⇒ **μ=1.0 FAIL 而 μ=0.6 PASS**。
  已把"零件必须足够高"写成**派生断言**（`payload_rest_z() >= min_grasp_frame_z()`）。

### C —— 车载接驳

- **`P1-C-05`（DONE）**：**权威 `reports/p1-c-crown-02` = PASS 16/16**。真 V1 托盘在接收端被接收并上车，
  互锁窗口**逐摩擦实测**，**声音性与紧性都由运行证明**：108 次运行（58 准入 / 50 拒绝），
  58 个准入全部到 DONE，50 个被拒用例开闸后 **0 个会成功**。**偏航上限 4° 是传感器量程而非机械公差。**
  （`reports/p1-c-deck-22` 的第一步 PASS 已被完整的源端→车载→接收端转移 + 卸载取代；
  判据口径变更见 `DECISIONS.md` D028。）
- ⚠️ **PASS 可脱离 `scope_claim` 引用**：甲板是替身、机构实验、**非真 AMR 对接**、偏移是初始条件。
- ⚠️ **`RECV_FIRST_CROWN_X` 的固定距离断言在装配后不再成立**（见下）。

### N —— 同世界轮式导航

- **`P1-N-07`（DONE）**：`experiments/make_map_n.py` 从世界几何派生静态地图，
  `make_nav2_params.py` 从 nav2 stock 派生参数（**每处偏离 stock 都具名并写明理由**），
  `probe_n_nav.py` 编排 sim + 桥 + 观察者 + Nav2，`evaluate_n_nav.py` 用真值判 32 项。
- **权威 `reports/p1-n-nav-07` = PASS 32/32，真值到站 0.230 m**（限 0.25 m，**阈值一行未改**）。
- **`P1-N-08`（DONE）：误差在 AMCL 内部产生，已归因并用受控 A/B 修好。** 先分开"谁的错"
  （`analyse_n_amcl.py` 把误差分解成沿/垂轨迹两个分量）：N-07 的误差是**沿轨迹超前 0.234 m**、
  垂直只差 0.061 m。**再量输入**：**同一次运行内** AMCL 位置中位 **0.256 m** vs 轮式里程计
  **0.005 m**（差 47.6 倍）⇒ 误差在 AMCL 内部。**机制**：stock `alpha1..5 = 0.2` 假设里程计有
  ~10% 行程噪声，本底盘实测 0.005 m / 3.7 m ⇒ 每次更新注入 σ≈0.05 m，15 次 ≈0.19 m 随机游走；
  而墙在 4 m 外、似然面太平、`recovery_alpha_* = 0.0` 又无恢复。**A/B（只改一个变量：0.2 → 0.02）**：
  AMCL 位置中位 0.256 → **0.070 m**；回到验收：到站 **0.431 → 0.230 m**。
  修正前的 FAIL 完整保留在 `reports/p1-n-nav-06`，两份构成 A/B。
- ⚠️ **未证明**：五个 alpha 里哪个起作用；未试"把似然面变尖"（`z_rand`/`max_beams`）这条替代路径；
  **0.02 是按这台自制差速代身推的**，AMR 选型后必须重标；**只有一个目标点，没有重复性**；
  地图是世界几何先验**不是 SLAM**（`slam_exercised: false`）。

### ⭐ 共同世界加载门 `P1-GATE-07`（`D039`；摩擦那条已由 `D040` 更正）

- **权威 `reports/p1-gate-08` = PASS 15/15，12 个探针全部可失败。** 产物 `assets/world_p1_cell.xml`
  （生成物，勿手改，**装配后** sha256 `c9e4b7478d1a3390…`；装配前的 `58ef7aed28ab…` 已被取代）·
  构建器 `experiments/merge_world.py`（`--check` 幂等）· 判据 `experiments/evaluate_gate.py` ·
  测试 `tests/test_p1_gate.py` 16 条 · **套件 230 passed**。
- 一个 `MjModel` 装四角色：nq149 nv145 nu116 nbody140 ngeom310 njnt125 neq1 nkey1 nsensor15 nmesh115。
  **跨角色自由度泄漏 0**、**跨角色接触 0**、3 s 沉降后各角色末速 ≤0.055。
- **V1 装配已实施**：`c_deck` 的父节点 = `n_base_link`，11 个 C 的 body 随底盘移动（12 个名字重定位），
  夹具与货物留在世界层。`dock_offset = +3.943e-06 m`，甲板世界 x 6.33372。
  **底盘停在哪是初始条件，不是被驱动的接近** ⇒ C 的"固定夹具、甲板出迎"这条主张不是循环论证。
- **合并且用 `MjSpec.attach(child, prefix, site=<worldbody site>)`**。`attach` 会带上子模型的
  `<keyframe>`（⇒ 编译硬失败）和 `meshdir`（⇒ 路径被前缀两次），两者都要在 attach 前手动清掉。
- **自由关节的 qpos 是绝对世界位姿**，必须按工位平移；且**未命名关节会被按名字查漏**
  （人形底座 free joint 无名字 ⇒ 保留**零四元数** ⇒ 脚陷地面 **1.0165 m**、静止态 220 接触，
  **并伪装成"A 的机械臂伺服失灵"**）。⇒ 自由关节一律取 `qpos0`；现已在合并世界命名 `h_base_free`。
- **求解器不是原因**：六变量扫描（tolerance / iterations / ls_iterations / cone）**逐位相同**。
- **`D040` 更正：`D039` 的"地面摩擦耦合"不成立。** MuJoCo 在 `priority` 相同时取两 geom 摩擦的
  **逐分量最大值**（实测 N 轮子 1.6 vs 地面 0.25 ⇒ 接触 1.6）。实测扫描：地面 `0.001…0.95` 与 0.25
  **逐位相同**；`1.20` 偏 5.385 m、`5.00`（**敏感性对照**）偏 4.556 m ⇒ **可接受区间 [0, 1.00]**，
  边界 = 接触 geom 中最低者（此处 1.0，余量 0.75）。**取四者最低 = 最安全。**判据每次重测这条边界。
- ⚠️ **真正被丢掉的耦合是 "N 自己的场地"**（围墙+三根柱子不在，cell 墙 **1.00** vs N 自己 **0.90**）。
  另：② 求解器容差四者不同（非原因）；④ **人形不是准静态**（零力矩 3 s 下沉 1.582 m，
  靠它自己的 `stand.yaml` 站立律才保住 → `home_hold_ctrl` 一处写）。
- 🟡 **V1 装配的配方实测通过但**（就 C 的验收而言）**未在装配世界里重跑**。拆法：**底盘**
  `deck`+`deck_roller_0..7`+`pusher_carriage`+`pusher_blade`（11 body）；**世界**
  `fixed_roller_*`(24)+`recv_roller_*`(14)+`payload`。三条硬约束：① `attach` **拒绝自由关节下移**；
  ② 同源加载两份 ⇒ `repeated default class name`（`spec.default.name` 可写 ⇒ **不需要 weld**）；
  ③ **`MjsBody.parent` 只读** ⇒ 拆分必须在 attach **之前**。

## P2 —— 订单与资源核心（纯 Python，无 ROS / 无物理）

- **五个模块**：`src/workcell/schema.py`（严格校验）、`ledger.py`（订单状态机）、`resources.py`（资源预约）、
  `transfer.py`（交接事务）、`safety.py`（命令门 + 五字段判决）。
  **关键约定**：订单状态 `QUEUED→…→SUCCEEDED` 加受限态 `WAITING/CANCELING/FAILED/NEEDS_ATTENTION`（**保留前序阶段与原因**）；
  技能白名单 12 项（`RETURN_TO_CHARGE` **保留但不注册**）；资源默认 **UNKNOWN**（要清场证据才能用）；
  判决五字段**分开**，顶层 `ACCEPTED` 要求必需检查全 PASS **且**数据完整。
- **★ 两套词汇表不能互换**：检查的三态（`PASS/FAIL/UNKNOWN/NOT_APPLICABLE`）
  **不是**任务结局（`SUCCEEDED/FAILED/NEEDS_ATTENTION/CANCELLED`）。
  本轮 `safety.build()` 正是把前者当后者用，判决里出现了 `'PASS' not in [task outcomes]`（见 `D044` 缺陷 1）。
- **权威证据 = 变异证明 `experiments/mutate_p2_core.py`**：**86 个变异，85 `CAUGHT` + 1 `REDUNDANT (declared)`，`rc=0`**；
  两次运行输出 **逐字节相同**（sha256 `04b77c58…`）。**恢复后全量套件 514 passed, 1 skipped**。
  含 5 个必需 P2 测试文件：`test_p2_schema/ledger/resources/transfer/safety.py`（safety 46 条）。
- **harness 的三条纪律**（本次全部被真实触发）：① 锚点必须**恰好一次**（0 次 = 静默空操作却仍报状态；
  2 次以上 = 一次改坏多条）；② 空 `expect` 列表 = **声明该守卫冗余**，须干净存活（已记录 2 处）；
  ③ 变异后模块必须仍能 `ast.parse`（用 `if False:` 会改变缩进 ⇒ 全红 ⇒ **因错误的原因读成 CAUGHT**，已改为反转条件）。
- ★ **本轮修掉两个真实缺陷**：① `task_outcome` 写成三态值（**两套词汇表混用**）；② `audit()` 里
  `if mode != 'HOLDING' and (v != 0.0 or w != 0.0):` 是**永不执行的死守卫**——枚举 168 个状态无一满足
  （`step()` 在非 HOLDING 下总是返回 `0.0, 0.0`）。修法：抽出 `_check_mode_agrees_with_state(mode)`，
  **把 `mode` 作参数传入**（这才是让检查可失败的关键）。**这是"死代码"族第 2 例**（第 1 例见 `resources.mark_cleared`）。
- **`D041` 欠账的第一步已完成（`D045`，2026-09-23）**：`experiments/probe_c_in_cell.py` 在**装配世界**里跑 C 的输送机构，两臂对照，`reports/p2-c-in-cell-02` = **8/8 PASS**。托盘从 x 4.613720 前进 **+2.3787 m**、**上了甲板**、受支撑区间内 z 波动 ≤ 4.3 mm / 倾角 ≤ 3.448°；执行器全 0 的对照臂只漂 0.0007 m。⚠️ 甲板滑移/接收辊排/推杆**全部置零**，所以托盘**必然跑出甲板远端、必然到不了接收端** —— **这是声明的结果，不是缺陷**；**不是 C 的验收**。★ 新观测：上甲板前 |yaw| ≤ 0.474°，上甲板后增到 ~10.7°。**已归因（`D046`，`reports/p2-c-yaw-attrib-01`）：不是装配造成的 —— C 自己的单角色世界做同一件事偏航 10.023°，合并世界 10.707°，差 +0.68°。★ 肇因是侧向导轨：关掉后掉到 1.277°（约 8 倍）。**已排除辊面摩擦（逐位相同）、求解器（三世界逐位相同）、底盘晃动（行程 [0,0]）。**残余 +0.68° 仍未归因。**
- ⚠️ **边界**：这是**五个纯 Python 模块**的契约证明，**没有 ROS、没有物理、没有真实机器人**参与；
  变异只证明"这 86 条性质有测试盯着"，**不是**"代码里没有别的问题"。

## P3 —— 正式世界、视觉（**本节已被下面那节取代，保留作历史**）

- **`P3-WORLD-01` DONE（`D050`）**：`assets/world_p3_cell.xml`（sha256 `30289f69ae15bc7c…`），
  五实例 `h/a/c/n/n2`（**AMR 两台**，实例 1 载甲板）。`reports/p3-world-01/gate.json` = **10/10**，
  **6/6 探针可失败**。隔离：**20 个有序对泄漏 0**，声明装配 c-on-n 的 11 个 body 全部跟着走（例外被使用）。
  V1 装配未被破坏（dock_offset **+3.943e-06**）。
  ⚠️ 两实例在**两个推导工位**、**实例 2 无甲板** ⇒ 不是最终 V1 布局。
- **`P3-VISION-03` PARTIAL（`D051`）**：`experiments/probe_p3_vision.py` = **6/8**。
  已成立：RGB-D（0.783–2.337 m）、**相机约定经标定**（`forward = 局部 −z`）、台面分离、
  **颜色落到 geom 并回读验证**、观测带帧/时间/时效/内参、每条判据可失败。
  缺口：**蓝件未被分离**（2 个连通域，第 2 个是 1215 px 灰 blob @ y≈−0.017），红件 x,y 误差 8.6 mm。
- **（历史）`P3-NAV-02` / `P3-FREEZE-04` 当时未开始**。NAV-02 是**重建**（`pillar_a` 在这个世界里不存在），
  **不能继承** `p1-n-nav-07`。门槛 `D049`：每条执行链一个权威命令门，P3 用 `workcell.safety.CommandGate`。

## P3 —— 四项全部 DONE（正式世界 / 导航 / 视觉 / 冻结）

- **`P3-WORLD-01` DONE（`D050` + `D052`）**：`assets/world_p3_cell.xml`（sha256 `89ffeab89407e601…`），
  五实例，`reports/p3-world-01/gate.json` = **11/11**，**7/7 探针可失败**。
  ★ **`D052` 修了一个此前没有任何判据覆盖的缺陷：生成的"单元格墙"根本没有围住单元格**
  （实测西边 0.0%、南边 0.0%、北边 50.2%、东边重叠 151.5%）。已补"围合 + 名字与几何一致"判据与探针。
- **`P3-VISION-03` DONE（`D057`）**：`experiments/probe_p3_vision.py`，权威证据 **`reports/p3-vision-02/report.json` = 13/13 PASS，9/9 探针触发**（两次运行非计时字段逐字节相同）。
  实测：约定得分 **14.97 mm**（红件自身标签像素 x,y **9.00 mm** / 顶面 0.00 mm + 台面轮廓 **5.97 mm**）、**两个零件 8.6 / 9.7 mm** 定位、相机约定**24 候选唯一**（次优 294.96 mm = 14× 容差）、逐帧 **366 ms** 中 **359 ms 是渲染**。
  ★ 上一轮"蓝件被臂的手挡住"**被推翻**：真因是 **`<keyframe>` 把后加自由关节的 qpos 补零 ⇒ 蓝件停在世界原点**，而 `truth_blue` 是手打常量。共修 **7 个缺陷**（5 个属"检查不可能失败"族）。
  ⚠️ 标定用仿真器 geom 标签（预言机）；无 ROS / 无点云库 / 无 PCL 主张；无传感器噪声 ⇒ 不是物理相机的数字。
- **`P3-NAV-02` DONE，但判决书已重定性（`D065`/`D066`）**：权威证据 **`reports/p3-nav-requal-01/`** —— 判据原先由 **run id** 决定带载/空载，于是 `p3-nav-06`（自己的记录写着 `p3_nav_loaded.yaml` + `world_p3_nav_loaded.xml`）被判成 `bare` 并按**空载 arena** 锚定；现在身份由 run 自己的**内容哈希**经布局的变体注册表派生 ⇒ 变体 **`bare → loaded`**，**到站 97.0 / 151.0 mm 不变**。顶层聚合改为 **profile 声明的必需集合**（`NOT_RUN` ⇒ `INCOMPLETE`；`NOT_APPLICABLE` 事先声明、永远不算证据；`CONFIG_ERROR` 不给 PASS）。⛔ 载荷是**声明替身刚性挂载**，不是落在甲板辊排上的托盘。
  ★ **两个判别实验先证明了方向是错的**：去掉 Nav2 仍然死、而**不含 ROS/DDS 的纯嗅探器 1160/1160 全收** ⇒ 传输是干净的。修后 `observed_max_m` 8.1135 → **8.0**、`decode_refusals` 35 → **0**、投递 28.5% → **100%**。
  实测：空载/带载**都 `GOAL_SUCCEEDED`**，真值到站 **97.0 / 151.0 mm**（限 250），行程 8.6264 / 8.5429 m **大于** 8.150 m 直线（**绕过了三个障碍**）。
  ⚠️ 停靠靶是**场地声明的目标点**，不是 C 的接收端；带载是**声明替身挂载**；进场靠**停车位姿**。
- **`P3-FREEZE-04` DONE（`D055` + `D058`）**：`config/p3_freeze.json`（**12 产物 + 12 脚本 + 67 阈值**，全部派生），
  `--check` 在真实改动上被验证会点名变红。★ **`D058`：清单原本漏掉了 `P3-VISION-03` 自己的判据脚本**（整文件重写后 `--check` 仍绿）；修法是**把完整性从任务板派生** —— `docs/TASK_BOARD.md` 里以 `P3-` 开头的行点到的每个 `experiments/*.py` 都必须在 `CODE` 里或带理由豁免，豁免项失效即报 stale。
- ★ **`D052` 连带修好了 P1 的世界**：`assets/world_p1_cell.xml` 重新生成（sha256 c9e4b7478d1a3390…），
  `merge_world --check` 通过，P1 判据重跑为 **`p1-gate-08` = PASS 15/15**（新 run id，见 `D042`）。
  `p1-gate-07` 判的是**旧产物**，已被取代。
- ⚠️ **（已消解）`D054`/`D056` 的两处改动曾使 P1-N 的证据在旧行为下记录。**
  ⇒ **已于 2026-09-25 重跑 6 次**（`reports/p1-n-nav-09..14`），权威证据改为`reports/p1-n-arrival-01/`（重判）；此后到站误差已归因（`D062`）。

## 未完成

- **`P4-HUMAN-01` 已 DONE**（H1/H2/H3）；**`P4-ARM-02` 已 DONE**（`reports/p4-arm-02` / `p4-count-02` / `p4-interlock-01`，各自含成功与故障两条证据）；**`P4-BELT-03` 仍未开工**：
  前者要有视觉的抓放与数量核验，**不能直接等于 `P3-VISION-03` 的结论**；后者要有输送/车载保持/接收卸盘与故障恢复。
  ★ **P4 的两个前置已完成并有证据**（第二十五轮，2026-09-29）：**感知决策** `reports/p4-vision-01`（`PASS: all 6 judged rows`，**一个能返回 `UNKNOWN` 的决策**；z 轴不可用作形状闸门是**量出来的**）与**失败交接的动作审计** `reports/p4-h3-gating-01`（`FAIL: 2 of 7`：`pusher_position()` 静默返回 `{}` 的死读数、以及"抬起挡刀"**机构做不到**—— 提升关节整个行程只有 **1 mm**）。⛔ **两者都不等于 `P4-ARM-02` 完成**（还缺接到抓放技能、数量核验、共享区互锁）；⛔ **PCL 路线仍未接入**，`MASTER_PLAN` 要求"要么接入、要么正式记录范围变更"，**两件都没做**；⛔ 挡刀不成立这件事**直接动摇 `P4-BELT-03` 的"车载保持"前提**。
  ⚠️ **蹲姿不是能力**：`D_MAX = None`（`reports/p4-armik-05`），H 链全程**站姿**。
  ⚠️ **P2 的证明只覆盖纯 Python 核心**；**P3 的证明只覆盖世界/导航/视觉/冻结四项各自的边界**（见 P3 各节的 ⚠️）。
- **变异证明覆盖不到的行为，本方法一言不发** —— 86 条变异之外的代码路径没有被这件事约束过。
- **`full_H_acceptance` 只是 `DIAGNOSTIC_PASS`**（详见上）。退出路径的余量（0.07 mm 对 5 mm）来自
  **同源重复一致**的两次运行；**改变初始条件的重复试验未做** —— 稳定性还没有证据。
- **C 的 16/16 未在装配世界里重跑**；**对接公差尚未扫描**；底盘**没有真实 AMR 安装接口**；
  车**没有被驱动去对接**（停车是初始条件）。
- **A 的包络只有 8 个枚举点，不可外推成"某个半径内都行"；C 的数值不可外推到别的节距/辊径/托盘。**
- **`P1-N` 的重复性已量（`D059`/`D062`）：到站精度不是可重复的性质** —— 同目标点 n=4 里 1 个超过 0.25 m；
- **`/cmd_vel` 上有三个发布者**（collision_monitor、docking_server、following_server）。
  本轮只能声称"每条 TF 边一个写者"和"实测命令流"；严格单写者需要把 docking/following 从 bringup
  里去掉，未做。
- **`Stopper` 该加但没有测量依据**（摩擦在测试加速度下够用 ⇒ 加它没有依据）；
  **夹持力已标定**（夹爪是位置伺服，法向力由穿透量决定，实测两指压入约 7 mm）。
- MuJoCo 仅初选，尚未最终通过引擎技术门。
- **退出路径的两次失败实验保留在案**（`p1-h-seq-05`：48.7 mm；`-06`：197.0 mm，托盘被甩下桌面）。

## 证据

- `reports/p1-gate-08`：**门的权威记录**（`gate.json`，15/15，含 12 个探针与 assembly 实测块）。
- `reports/p1-a-p05`：**A 的权威记录**（14/14，含 33 条 trace）。
- `reports/p1-c-crown-02`：**C 的权威记录**（16/16；`report.json` 103 MB 是原始证据，
  `acceptance.json/md` 是可再生的判决 —— **两者分开是它能被救回来的原因**，见 `D042`（**该条现已写入 `DECISIONS.md`**：
  `--run-id` 同时是读和写，重跑判据会覆盖已有证据的判决；要只重算不动证据用 `--bypass-run-id`）。
- `reports/p1-n-nav-07`：N 的记录（32/32，**已被取代**：它由 `probe_n_sim.py` `c4d3aa19…` / `probe_n_ros.py` `4efd133e…` 产出，与盘上文件不同源）。
- `reports/p1-n-nav-09..14`：**N 的当前记录**，6 次、三个目标点（`D059`）。**它们里的 `-09` 是 FAIL**，且 7/7 都超过声明的 0.15 m 目标容差。
- `docs/HANDOFF.md`：**派生产物**（`experiments/make_handoff.py`，`--check`，`D061`）—— 手写版本腐烂过两次。
- `reports/p3-vision-02`：**P3 视觉的权威记录**（13/13，9/9 探针；`p3-vision-01` 是 6/8 的失败历史）。
- `reports/p3-world-01` / `reports/p3-nav-05` / `reports/p3-nav-06`：P3 的世界门与两臂导航记录。
- `config/p3_freeze.json`：P3 的产物/脚本/阈值冻结，含 `code_coverage`（`D058`）。
- `reports/p1-h-seq-08`：**H 的权威记录**；`-09`（007 旧托盘）与 `-10/11`（同源重复）为对照。
- `reports/_invalid/`：仪器自身失败或被取代的 run，每个 README 写明失效原因。
- `experiments/mutate_p2_core.py` 的运行输出：**P2 的权威证据**（86 变异 = 85 `CAUGHT` + 1 声明冗余，`rc=0`；
  两次逐字节相同）。**判据不是"测试绿"，而是"每条性质被改坏时有测试变红"。**
- ⚠️ **`P1-N` 的到站精度不可重复（`D059`），且机制已重新归因（`D062`）**：同目标点 (2.6, 2.4) 当前植物 n=4 = 0.225 / 0.234 / 0.246 / **0.283** m（**样本标准差 25.4 mm**，1/4 超 0.25 m 限值）；**8/8 个 run 的真值从未进入声明的 0.15 m**，`min_ever == final` 8/8，`truth − belief` ≈ AMCL 信念误差（比值 0.89–0.99）。⇒ **到站误差 = 定位误差（中位 +0.126 m）+ 停在"自己以为的目标"之外（0.089–0.184 m）**；**`D059` 的"`stateful` 锁存"解释已被推翻**，锁存不是杠杆（`D063`：预测效应 0.0 mm vs 噪声 25.4 mm）。**限值未放宽、FAIL 保留**（`-09` 真值超限、`-10` 信念超限）。**未决的选择在 `P1-N-10`。**
- `.venv/bin/python -m pytest tests/ -q`：**645 passed, 1 skipped**（自 531 起累计：`test_p3_vision.py` 13 + P4/H 链与 W5 各轮 + **本轮 `test_h3_integration.py` 新增 6 条账本守卫测试**）。

## 下一步

**P3 四项全部 DONE（`D057`/`D058` 收口）。** 下一步是 **P4**：`P4-HUMAN-01`（人形取盘/放盘/退出，成功与失败证据）、~~`P4-ARM-02`~~（**已 DONE**，见 `reports/p4-arm-02` / `p4-count-02` / `p4-interlock-01`）、`P4-BELT-03`（输送、车载保持、接收卸盘及故障恢复）。⚠️ **P4 依赖的"视觉抓放"不能直接等于 `P3-VISION-03` 的结论** —— 那里的相机与第二个零件是探针用 `MjSpec` 加的，标定用了仿真器标签，且没有传感器噪声。
1. ⚠️ **`D041` 的欠账只走完第一步**（仍是 P4 的前置，与 P3 的完成无关）：**C 在装配世界里的完整重跑仍未做**（`D045` 只证明**输送还在**，RELEASE/SETTLE/EMBARK/互锁/推杆全都还没在装配世界里跑过）；**对接公差仍未扫描**；**偏航已归因到 C 自己的机构（`D046`），肇因是侧向导轨**；**但继续偏航本身没被消除**：**逐级解冻 1+2+3（`D047`）**实测 RELEASE/SETTLE 只消掉 **0.011°**（静止偏航 **7.555°**，否证了「静止 < 2–3° 就不必管」的预测）、EMBARK 成立（甲板 **+0.2402 m**、托盘滑移 **0.0013 m**，优于 C 自己的 0.0054 m 基准）、**UNLOAD 跨上接收端辊排且未掉落**（对照：无 EMBARK 时**掉 0.118 m**）。**`D048` 结清：偏航肇因确证为导轨入口（起始 x 实测 1.9390/1.9393，预测 1.935）、整链已跑到 C 自己的 `RECEIVED`（合并世界末尾 yaw 10.904°）、`D046` 的「+0.68° 残余」经多阶段比较证明是**取样假象**（符号随阶段翻转 ⇒ 无系统性装配惩罚）、**对接公差已扫出边界**：更远 +20 mm 通过 / +40 mm 掉件，更近 −2 mm 通过 / −3 mm 掉件，偏航 +0.20° 通过 / +0.5° 掉件（接口横移 6.7 mm 过 / 16.8 mm 挂，明显不对称）。**

当前无本项目遗留运行；交接释放写入权。下一轮须核对实际进程和文件，不以本文代替现场检查。


---

## 2026-09-29 · 第二十九轮：`P4-ARM-02` **DONE**（02a / 02b / 02c 全部有成功与故障两条证据）

**由用户拍板的 `D115`（选 B）**：几何缺口用**一个敞口、落在可及范围内的装盘工装**回答，**不动冻结世界**。

- **新资产** `assets/world_p4_cell.xml`（`d44db898…`）= 冻结的 `world_p3_cell.xml`（`89ffeab8…`，**逐位未变**）
  ＋ 敞口装盘工装；建造器 `experiments/build_p4_cell_world.py`（**写入前编译**，并断言 `nq`/`nkey`/
  四个锚点 body/台面高度未被改动）。工装是**托盘自己的横截面**（甲板半幅、壁/肋厚度、隔板偏置全部建造时
  从 `c_payload` 读出）⇒ `derive_cells(p_fixture)` 得 **3 格 / 宽 [0.1533, 0.1534, 0.1533] / 沿 y**，
  **与托盘完全相同**。
- **`02a`** `reports/p4-interlock-01`（`PASS: all 7 judged rows`）· 新建 `src/workcell/interlock.py`
- **`02b`** `reports/p4-arm-02`（**`PASS: all 7 judged rows`**）· 感知误差 **0.00577 m** → 放进推出来的第 1 格；
  故障证据：`UNKNOWN` 时零件移 **0.000000000 m**、臂移 **0.000000000000 rad**（同一函数给真位姿移 0.4989 rad）
- **`02c`** `reports/p4-count-02`（**`PASS: all 7 judged rows`**）· 订单 `{'red': 2, 'blue': 1}`，读者报
  `{'red': 1}` ⇒ **报出不一致**；把零件拿回起始位姿，核验数 **1 → 0**；料位外报 **UNKNOWN**
- ★★ **相机成为声明的参数**（`p4_fixture.mount()`）：默认机位下**零件后方的远侧隔板**落在图像行 90..93、
  零件落在行 94..108 ⇒ 洪水填充**一步**并进工装壁环。五个相邻机位 × 三格全过 ⇒ 有余量
- **两个 workaround 被删掉**：不再在内存改几何（`in_memory_geometry_edits = 0`）、不再瞬移 `c_payload`
- **套件 645 passed, 1 skipped**（新增 `tests/test_p4_fixture.py` 5 条）；冻结 `--check` rc=0；交接 `--check` rc=0
- ⛔ **不声称**：工装能保持零件（肋高 12.5 mm）／世界原托盘可被这台臂服务（仍差 1.7264 m）／
  已接入 PCL（`MASTER_PLAN` §5 的 PCL 路线仍未接入也未改范围）／这是鲁棒感知
- **下一项 = `P4-BELT-03`**
