# 2026-10-04 连续实际装载 Nav2 续作

状态：P4-BELT-03 PARTIAL；V1未完成。本文不把初始化带载Nav2或纯契约测试当成完整订单。下列旧RUNNING信息为历史；最新活动以末段为准。

## 本轮目标与改动

已有真实H供盘→RGB-D/PCL三件装盘→源端输送到甲板后，继续由同一个物理世界执行Nav2接收站接近、精确停靠、卸盘和原事务核验。

- `continuous_nav_session.py`：guest只持有已有model/data/plant/owner引用，不新建物理世界、MjData、H控制器或货物；原H策略/已批准臂保持由plant控制组合保留；原唯一WorldOwner提交。
- `SkillAdapter.navigation_move` 可注入MOVE回调，旧默认不变，DOCK/UNDOCK不调用该回调。动作失败不能被零位置残差洗成成功。
- `joint_nav_worker.py` 可接受有限、合法的map-frame站点目标及声明的停车先验；无MuJoCo或真值通道。`--start`来自初始化夹具/站点声明，不来自运行姿态。
- 主探针增加显式`--nav2`选项，必须v5/transport/heading-feedback。请求的Nav2子阶段为顶层必需，缺失/失败阻塞PASS。
- 失败锁存后制动不禁用；原轮速制动有独立5s仿真/30s墙钟上限。确认停止之前不归还停车控制权，也不释放占用资源。保底SIM冻结不称机械急停。
- 独立`evaluate_continuous_nav.py`只读记录；实际双手抬盘/三件抬升、ROS action、9ACTIVE、唯一cmd_vel、实际到达、货物/车停止与滑移分别判断。NaN、缺项、长采样缺口、短停止窗口、晚异常都不能PASS。

## 证据

1. 12个新契约安装实现前全部FAIL，保存`runtime/pytest-continuous-nav-red-20261004.log`；实现首轮40项相关测试PASS。
2. `p5-continuous-loaded-nav-v5-20261004-01`实际exit1，945.766078441s。供盘PASS，三件真实装盘和视觉2红1蓝通过，前五技能SUCCEEDED，Nav2尚未启动即拒绝`DECLARED_SOURCE_LOCALIZATION_PRIOR_UNCONFIRMED`。原spawn与被动支撑夹具的source dock声明相差10mm，不能把它们当同一停车先验。原报告不补写、不覆盖。
3. 改用source dock声明，原5mm位置/0.01rad偏航前置门保持。新增ROS启动先验只覆盖声明参数；没有用实际base姿态构造AMCL先验。
4. 修正后57项相关短测PASS，9.98s；冻结33资产/124源码/110原阈值检查PASS。上轮全量1022项不是本轮全量。
5. `p5-continuous-loaded-nav-v5-20261004-02`已exit1，1013.528856211s：供盘/真实三件装载/前五技能通过，Nav2实际搬运4.164396547m至x8.585181705，距目标9.0437尚0.459134m；sim171.604最终writer冻结CONTROL_UNAVAILABLE。旧下层原因没有保存，不断言具体关节或控制根因。机械停止未证明，不卸盘、不归还轮控，货物及源/甲板资源仍占用。
6. 旧第一裁判误将tray-to-base变化19.040mm当G-2滑移。deck是柔顺机构，G-2必须tray-to-DECK；旧记录缺该量。新独立重判`p5-continuous-loaded-nav-judge-deck-frame-20261004-02`exit1，保持行缺证据FAIL（MISSING_DECK_FRAME_NOT_MEASURED），旧报告/旧裁判保留，不补造数据。ROS子报告缺失行FAIL表示输入缺证据，不等于9节点实际不活跃。
7. 新增TransportPostureGate，原界限完全保留；第一次姿态拒绝锁存运动，原轮速制动继续，不能靠姿态后来恢复自动重启。原因链/原关节值/甲板相对轨迹新增；失败时终止自有Nav2，而不是等原目标自行结束。73项相关短测PASS，8.99s；冻结33资产/125源码/110阈值PASS。
8. 初始化长程诊断`p4-loaded-nav-receiver-diagnostic-20261004-01`exit1，308.300s：sim25.158视觉RESOLVED/2红1蓝，但接触支持瞬时UNKNOWN，运动门锁存LOAD_ENVELOPE_UNKNOWN；未复现CONTROL_UNAVAILABLE。旧退出路径等待Nav2造成TimeoutExpired已修。不能由此宣称真实载荷滑移或旧冻结根因。
9. 独立声明初始化2.04m载荷中心/部署源端保持机构（lift .001m/slide .01324m），只在第一物理步前设定，零运行位姿写入。第二诊断`p4-loaded-nav-source-retention-diagnostic-20261004-02`已exit1/158.376s，bridge STATE_STREAM_STOPPED_EARLY，最后收到sim20.452；无姿态拒绝证据，旧异常路径无制动数据，不宣称机械停止。初始化诊断不能代替实际连续装载验收。
10. 完整回归1067 passed/1 skipped，174.23s，exit0，runtime/pytest-continuous-nav-20261004.log。随后新增StatePublishSchedule：RGB-D耗时使每次physics仅推进2ms、刷新再次触发，但旧state每50ms sim发布会饿死bridge。新调度按sim或0.25s墙钟发布新推进状态，绝不重复时间或制造physics/clock；bridge watchdog和0.5s载荷门不变。不因此断言旧CONTROL_UNAVAILABLE根因已找到。新峰值计时、初始化探针异常有界原制动与小检查点尾窗口接入；80项相关短测PASS（9.60s）、冻结33资产/126源码/110原阈值PASS，新增增量待全量复验。
11. 第三实际连续链`p5-continuous-loaded-nav-v5-20261004-03`当前RUNNING，PID14092/session49690/1800s上限。没有用初始化诊断替代实际H供盘/机械臂装盘；最终判决待证据。

## 命令

```bash
cd ~/projects/010_heterogeneous_robot_workcell
LP_NUM_THREADS=1 MUJOCO_GL=egl .venv/bin/python scripts/run_bounded.py \
  --run-id p5-continuous-loaded-nav-v5-20261004-02 --timeout 1800 \
  experiments/probe_candidate_multi_loading.py --world world_p5_candidate_v5.xml \
  --handover --policy-balance --heading-feedback --transport --transactions --pcl --nav2
```

该ID不能重用。只在物理worker退出之后运行全量pytest；纯fake/记录裁判短测不启动第二个物理worker。

## 剩余边界

- 真正连续链两次FAIL；源→甲板可行，Nav2到达/精确停靠/接收/独立货权裁判仍未通过，不能由加载或短程导航组件PASS替代。
- 静态几何地图先验不是SLAM。虚拟车载相机、已知颜色/形状目录、理想触觉与精确对接本体读数均为仿真假设。
- 本轮独立裁判仍不证明一般3D扫掠碰撞保护或完整订单；P4运行中故障/显式恢复、P5真实N01/N02、P6双车/有限库存/FI及P7 36实例尚未完成。
- 没有修改001～009，没有上传GitHub。

## 最新 · 原保护触发与货物静止诊断

> 最终完整回归exit0，1091 passed/1 skipped/203.95s，runtime/pytest-deck-contact-candidate-20261004.log。全部本轮物理与pytest退出。冻结35资产/131源码/110原阈值PASS；新机构授权待用户，V1未完成。

> solref .01s候选最终exit1/138.808507s，sim18.166支持拒绝，保持2.490046mm/货物停止0.086049mm/s/车停/姿态/计数PASS但未到达。该数值候选不集成订单，不继续刷同类失败长链。全部物理退出；完整pytest session1145/900s运行。新可见压持/释放机构需要用户批准，方案history/TRAY_RETAINER_PROPOSAL_20261004.md；原阈值/旧报告/资产保留，V1未完成。

> 原时刻快照最终exit1/254.804628s：sim37.352 rows=[]、无tray/deck接触，仅零件/floor接触，cmd age0.144s。保持3.238592mm、cargo停止0.254241mm/s PASS；arrival尚差2.388740m，Nav2FAIL。不再称“可能仅感知误报”。原integrator3/solver2/iterations100，已经implicit-fast/Newton；新独立init-only timeconstant .02→.01s仅c_tray_floor/c_deck_roller*，所有geometry/friction/timestep/策略/门保留，7纯testsPASS。唯一worker p4-retained-deck-solref01-20261004-01 PID690/start_ticks1088885/session84072/720s上限；最新全量1087/1skip/166.70s exit0早于此增量。当前候选最终待证据。

> 0.08m/s对照最终：exit1/208.199795s，保持4.672091mm PASS，cargo停止0.195561mm/s PASS、车辆停止/姿态/视觉支持末态PASS；但sim30.498支持UNKNOWN锁存，距接收目标2.856501m，Nav2/到达FAIL。相邻30.402/30.502/30.602均deck，不由邻近采样猜原时刻原因。新增support UNKNOWN原时刻contacts+力/距离/dim/rows快照，行为不改，物理复测待全量。所有物理已退出，全量pytest session30383/log runtime/pytest-retained-multiccd-20261004.log正在运行，无物理并发。

> 接收站距离更新：`p4-retained-multiccd-receiver-20261004-01` exit1/124.777s，保持7.963mm超原5mm，support UNKNOWN sim15.736时锁存，cargo速度0.180432mm/s、漂移0.038128mm PASS；sim12.5先过5mm，不能归因仅为视觉。只加contact/command-age记录的同条件`p4-retained-multiccd-contact-diagnostic-20261004-01` exit1/128.893s，保持11.193mm FAIL、货物1.449765mm/s PASS；近拒绝指令age0.07～0.176s有效。当前唯一worker `p4-retained-multiccd-speed08-20261004-01` PID30607/session63449/720s上限，仅限速0.15→0.08m/s的显式ROS override，原加减速度/物理/门不改；10项纯相关短测PASS。实际千秒链暂不重刷，先解释保持。

> 最新移动证据：`p4-retained-multiccd-short-20261004-01` exit0/113.345s，14/14PASS。arrival0.070697232m、vehicle speed5.32368e-8m/s、cargo speed7.98995e-5m/s、cargo drift8.03814e-6m、tray-to-deck max1.111186mm，原门不变。当前唯一worker `p4-retained-multiccd-receiver-20261004-01` PID26566/session65417/720s上限，初始化接收站距离，非实际装载/订单。下方旧活动信息过期。

> 此节旧活动信息已过期。ratio10已exit1/63.893s（蓝柱59.422mm/s更差）。MuJoCo实际版本3.3.6，MULTICCD enableflags原值0；独立候选置16。新`p4-cargo-rest-multiccd-20261004-01` exit0/70.170s，蓝柱支持接触每帧1→5，保守点速度21.573→0.119680mm/s，其他货物均<0.009mm/s。未改变几何/摩擦系数/步长/策略/判据，不集成elliptic或impratio10。仅静止诊断PASS；新唯一worker `p4-retained-multiccd-short-20261004-01` PID25668/session90733/480s上限，移动候选待验。两个主探针新增opt-in --multi-ccd，首物理步前配置，原默认不变。冻结35/130/110通过，最新19项纯短测PASS，完整回归待物理退出。

- 第三实际连续链03已exit1/989.969794853s。原姿态门首次失败为c_deck_yaw=-0.010001609556rad（界±0.01），实际运输4.209658984m、尚差0.414740324m。新的锁存运动/保留原制动生效：车辆速度5.23408e-6m/s/漂移6.45723e-7m，未冻结。货物point-speed39.877mm/s、deck-frame滑移29.816mm均FAIL；独立03裁判exit1。最大货物速度来自蓝柱，其余托盘/两红块分别≤7.489/2.917/2.398mm/s。源/甲板仍OCCUPIED，不卸盘、不归还轮控，失败brake独立有界5sim/30wall、任务不重启。不能回填旧02根因。
- 新控制候选`nav2_joint_world_v5_retained.yaml`只作为opt-in：速度.15m/s、线加速.05m/s²、角加速.01rad/s²；内部目标收敛80mm比原MOVE门120mm更严格。原所有验收阈值/默认配置保留，声明UNQUALIFIED，不把经验设置叫机械证明。
- 初始化短程01因catalog旋转前导出错误IndexError（零运动）失败；02使用不匹配90°初始摆放、视觉UNKNOWN，保留但不判控制效果。修正为声明reference yaw0，**不是回放实际供盘偏斜**；03 Nav2/72.615mm到达/车停止/原姿态/视觉/支持/控制归还与2.405mm保持PASS，但全部货物停止21.142mm/s FAIL。原图像、系数、物理记录不删。
- `probe_cargo_rest.py`只读货物真值用于评测，无ROS/导航/render；原模型/步长/系数下静止20sim（只判5sim后）。baseline `p4-cargo-rest-20261004-01`exit1/73.074s，蓝柱21.573mm/s，其他货物≤3.024mm/s。实际blue/floor接触dim6、friction[.6,.6,.005,.001,.001]，所以改condim不能修。`p4-cargo-rest-elliptic-20261004-01`exit1/74.091s，蓝柱61.694mm/s更差，候选不接入主链。
- 官方指导：https://mujoco.readthedocs.io/en/stable/modeling.html#preventing-slip 。摩擦/法向约束阻抗比影响软接触滑移，不能保证精确零滑移；T800固定2ms步长/100Hz，不能随意减少步长但保留原tick计数。当前仅在静止诊断中比较椭圆锥+impratio10（相对上一椭圆锥只改impratio，**不增摩擦、不焊接、不固定货物**）。唯一worker `p4-cargo-rest-elliptic-ratio10-20261004-01` PID23572/session49155/180s上限，最终待证据。
- 35资产/128源码/110原阈值冻结PASS；最近全量1067项早于这些新诊断/候选，最新82项相关短测PASS（6.94s），新全量待物理退出后进行。
