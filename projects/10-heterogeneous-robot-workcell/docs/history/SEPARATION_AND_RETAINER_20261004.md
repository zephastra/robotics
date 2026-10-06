# 010 · 支撑丢失测量与最小保持候选 · 2026-10-04

## 授权、边界与纠正

用户“okay go ahead”批准测量后选择最小合理保持机构。不修改001～009、不推送、
不下载或安装；旧世界、旧失败和110项原阈值全部保留。货物仍为自由刚体，
无运行时qpos写入/weld/attach/关闭碰撞。所有物理步由一个WorldOwner执行。

**纠正此前“证实短暂离地”的过强表述**：旧快照只证实solver contact缺失，
未量出离地高度/时长，不能据此宣称明显跳起或必须加大夹紧装置。

## 1. 第一版测量 · 原报告不覆盖

命令：
```bash
LP_NUM_THREADS=1 .venv/bin/python scripts/run_bounded.py \
  --run-id p4-retained-separation-20261004-01 --timeout 720 \
  experiments/probe_vehicle_nav2.py --duration 90 --parking-handoff \
  --centered-load --receiver-leg --source-load-setup --retained-motion \
  --multi-ccd --linear-speed .08
```

exit1 / 179.885938s wall，sim19.384 LOAD_ENVELOPE_UNKNOWN闭锁；
保持2.613352mm、货物停止0.132510mm/s/漂移0.016711mm通过原门，
Nav2未到达，距目标3.753321m。没有自动恢复/屏蔽UNKNOWN。

仪器每2ms记录接触区间，缓存接触与几何时间明确为data.time−dt，
不额外mj_forward/mj_step。`mj_geomDistance`出现不合理负距离
（初始化约−30mm，而目录几何下方平面间隙接近0）；因此这份报告的
**raw convex distance不作为可靠间隙数字**。保留原值，不擅自归因是MuJoCo bug。

## 2. 解析几何交叉检查 · 第二份独立测量

同上命令，run ID改为`p4-retained-separation-analytic-20261004-01`。
exit1 / 368.133334s wall。初始化之外13次无支持接触事件全部1步（2ms）；
解析下平面间隙最大5.419206µm，原始convex距离在该帧为5.443324µm。

解析计算是圆柱支撑函数＋tray-floor下平面投影；拒绝无xy投影重叠的辊。
**它是下平面分离界，不是任意形状的精确距离；负值不证明穿透，不能把最大值
冒称为整盘最大离地高度或真实碰撞裕度。**短测包含已知3mm间隙和独立状态不变检查。
测量真值只进独立裁判，不输入导航/视觉/运动许可。

这次没有相机采样命中支持UNKNOWN，但仍FAIL：

- 托盘deck-relative移动7.616813mm，超过原5mm；
- `c_deck_yaw=0.0103750176rad`，超出原±0.01rad，原姿态门闭锁；
- 车辆停止与货物停止PASS（货物2.384231mm/s、漂移4.882347mm）；
- 最后距离目标0.455358m，Nav2/独立到达FAIL。

**接触采样、托盘滑移、甲板被动偏航是三件事，不能把新机构当一键解决全部。**
ROS壁钟到达使实测轨迹有波动，两次结果不同不代表测量器修复了控制。

## 3. v6直升压力鞋 · 拒绝，不集成

源v5 SHA256：`aa3e9085bf08236193537418f5cb1d32d6254f5cf0b2b7991f4796e82a2a8ac5`。
新资产`assets/world_p5_candidate_v6_retainer.xml`，SHA256
`d11525b76ed966110c08ebb5709542bc83a7ffc74f4655884c2f264a2ea7899b`。
`p4-edge-retainer-static-20261004-01` exit0，**STATIC_ONLY**：旧执行器135→137，
旧DOF/keyframe按身份映射，旧几何/摩擦/货物自由度不变；新两鞋每鞋2N力限。

`p4-edge-retainer-close-release-20261004-01` exit1 / 37.911893s：
双侧压力接触FAIL、保持10.668059mm FAIL、三个阶段相机
OCCLUDER_ABOVE_CONTENT_ENVELOPE；释放/货物停止/未冻结PASS。
禁止掩码机构或放宽计数遮挡门来“修”这份失败。

## 4. v7向外翻转候选

独立`assets/world_p5_candidate_v7_hinged_retainer.xml`，旧v6仍保留。
v7 SHA256：`ad6a6fa98a8c83d3c4b6dc94b026398cfdfe8b379bb85f5ec603948f8a347f47`。
两处可见转轴/边缘鞋，释放向托盘外侧翻转，不在料格上方留下横梁。
角度/力矩限显式：release=π/2、preload=−0.025rad、每轴±0.06Nm。
它们是**新夹具候选声明**，不是修改原5mm保持/10mm每秒停止/±0.01rad姿态门。
释放裕度0.01rad对应29mm臂约0.29mm运动，是候选自身读数校准，需物理确认。

`probe_tray_retainer.py --hinged`只做初始化单车11s闭合/释放/三帧真实RGB-D。
理想关节/触觉假设明示，接触力实际从solver读；不授权运输，无订单/第二车能力声明。
故障`--fault-held-release`是明确“电机被保持在释放位”模型，不冒充真实断电/卡滞认证。
物理结果以该run自己的report为准，静态编译或unit PASS不能替代。

实测结果（两个worker均已退出，不是运行承诺）：

- `p4-hinged-retainer-static-20261004-01` exit0，STATIC_ONLY；
- `p4-hinged-retainer-close-release-20261004-01` exit0 / 55.228166s：
  六项单机构判据PASS：双侧真实压力/实际释放/原货物停止/原保持/三阶段RGB-D/未冻结；
  保持0.043855mm、货物速度0.113429mm/s、漂移0.010270mm，都是静止小实验，不是移动运输；
- `p4-hinged-retainer-held-release-20261004-01` exit0 / 41.597219s：
  电机保持释放故障下actual_closed_contact=false、transport_authorized=false，
  释放/视觉/原停止/保持PASS。**它只是预期故障观察PASS，不是闭合PASS或任务成功**。
  此版本旧字段actual_closed_preload=PASS表示“预期缺接触”，易误读；原报告不重写。
  后续源码改用expected_no_closed_contact，并在初始视觉UNKNOWN时拒绝闭合，保留释放。
  最新源码复验见下文；仍未做初始UNKNOWN遮挡故障的物理注入，不伪造其已测试。

本轮14项新增短测（分批运行）通过。最终全量日志
`runtime/pytest-hinged-retainer-20261004.log`：1105 passed / 1 skipped，176.75s，exit0；
pytest结束后才启动最新源码正负例复验，不与物理并发。

最新源码复验：`p4-hinged-retainer-current-positive-20261004-01` exit0/37.996540s，
六项PASS，保持/停止数值与首次v7一致；`p4-hinged-retainer-current-negative-20261004-01`
exit0/41.233532s，expected_no_closed_contact=PASS、actual_closed_contact=false、
transport_authorized=false。全部本轮worker/pytest已结束。

## 5. 后续退出门

1. 单机构真实双侧压力、释放清空、相机可读及原货物保持/停止；
   保持释放位故障必须不能得到闭合接触证明。失败先诊断，不继续刷长导航。
2. 独立甲板偏航诊断：原被动yaw stiffness40/damping8保持；找出载荷、轮转或
   转角耦合，不能放宽±0.01门。新增资产需重新审计整车/3D/导航包络。
3. 机构和甲板门真正通过才初始化短/长Nav2→真实连续H供盘/装载/导航/停靠释放卸盘，
   然后完成3D保护、运行中中断/显式恢复、P5/P6/P7。

010仍PARTIAL；此文不宣称已完成运输或V1。
