# 连续供盘后的失稳定位 · 2026-10-03

## 真实发现

`p5-loaded-control-contact-diagnostic-v5-20261003-01` exit1，935.488 s wall。
H供盘10项和装盘10项仍PASS，但后续对接拒绝、全链FAIL；原失败与参数不改。
装盘后5 s诊断中轮力矩确有输出，底盘yaw从-0.1272继续到-0.1491 rad。
跨机构接触记录发现未命名碰撞geom × n_chassis；独立零步快照解析确定其归属为 **h_LINK_FOOT_L × n_base_link**。

快照 `reports/p5-loading-contact-snapshot-judge-20261003-01/report.json`：

- H基座位置 `[4.2112,1.1167,0.35856]`，高度已低于原H1 BODY_FALL 0.65 m；不是正常退出站姿。
- n_base_link高度0.09900 m，第二台待机车基座约0.03925 m。
- 车轮-地面接触在这个加载快照中不存在；4个接触均为人形左脚-小车底盘。
- 记录的接触穿透约0.36–0.44 mm；重新forward力读数只属快照诊断，不冒充新的物理运行。

因此已有证据支持：**供盘后人形失稳并接触/抬起小车，是加载场景中轮控不能恢复偏航的关键耦合。**
“轮向符号错”“甲板相对yaw自由漂移”均没有本轮证据支持；不能继续只调对接容差或轮控增益。
还需受控短实验区分：静态关节保持自身长期失稳，还是加载过程中其它动作/接触诱发；不能凭快照就把诱因归因完成。

## 已改的安全缺口

新增 `HumanoidPosturePermit`，在同一WorldOwner每次提交前后检查本体姿态；高度<0.65 m或倾角>35°沿用原H1 BODY_FALL，不另放宽尺子。
非有限/缺失姿态为POSTURE_UNKNOWN。失败关闭并锁存运行许可，后续健康读数不能自动重启。
这是明确的仿真冻结保底，**不称机械停止或真机急停**。7项纯守卫测试通过，运行中实际触发/长期控制由新run验证。

源端对接仪器也有独立缺陷：lateral=0、初始deck_row+slide忽略实际车体位姿。
已经改为读取当前首个甲板冠面，本体测量与声明源站校准求残差；6项数学测试通过。
该变化强化读数，不改变空间判据；旧报告不能因为新读者更严就被覆盖。

## 受控顺序

1. `p4-post-supply-static-hold-20261003-01` 已exit1，371.218 s wall：原H供盘10项PASS，不装盘/不开输送的后续静态保持仍在sim118.454 s触发BODY_FALL；最后读数高度0.74916 m、倾角35.0763°。原50 s供盘结束时高度1.01125 m、倾角0.872°，至115 s y位置已漂到0.09039 m。最终写者确实冻结、stopped_confirmed=false；这是实际运行中保护，不是纯单元测试，更不是机械停止。机械臂动作不是这次失败的必要条件；仍需区分站姿漂移与近距离车体接触的诱发关系。
2. `p4-post-supply-policy-hold-20261003-01` 已exit0，446.939 s wall：同v5、同原H2供盘和90 s保持预算，退出后使用原网络零速度控制作对照。H供盘10项PASS，累计140.002 s仿真，保持90.002 s；末态高度1.014637 m、倾角0.15143°，writer_frozen=null。没有替换策略或新物理资产；仅这一条件下的持续站稳通过，不宣称完整订单。
   后续真实连续复验 `p5-continuous-policy-transactions-v5-20261003-01` 已exit0，924.748 s wall；显式 --policy-balance / --heading-feedback / --handover / --transport / --transactions / --pcl。12项顶层诊断判据PASS，H供盘10项及实际装盘10项PASS，11物流技能全部SUCCEEDED，接收支撑/计数/完整碰撞包络/停稳/缺深度UNKNOWN/两段货权七项PASS。加载后底盘及甲板yaw均为0；原来源端对接失败已在新实际连续链中通过。保持同一model/data/time、托盘自由刚体、运行cargo qpos写入0。
   接收托盘停稳速度保守界0.000614647 m/s、漂移0.0000132293 m，沿用原0.01 m/s与0.005 m门。账本两事务RELEASED/AT_DESTINATION；源端/甲板FREE，接收仍OCCUPIED归属loaded-diagnostic。仅单车理想本体控制物流诊断，Nav2、真正订单、有限库存、双车与故障恢复仍NOT_RUN，不以顶层诊断PASS声明V1完成。
3. policy必须在退出第一周期就选择，不能把跳过了神经历史的Runtime再接回网络。即便长保持通过，也还需在实际RGB-D装盘→对接→输送链重验；之后再共同世界Nav2。不得再让跌倒人形继续“装盘成功→导航”。

完整V1、双车订单、故障恢复、发布复现仍未完成。

## 当前代码验证及失败供盘负例

最新全量：runtime/pytest-policy-continuous-20261003.log，938 passed/1 skipped、161.79s、exit0。真实负例p5-policy-supply-rejected-v5-20261003-01按预期exit1；独立裁判p5-policy-supply-rejected-judge-20261003-01 exit0，13/13安全PASS：没有装盘观测/动作、没有运输、没有已完成货权；最终写者SUPPLY_UNCONFIRMED锁存SIM冻结。不是机械停止或恢复。所有本轮物理和回归进程均已退出。
