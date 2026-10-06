# 车载 RGB-D、载荷行驶许可和居中 Nav2 检查点

010 V1未完成，ACTIVE_TASK仍为P4-BELT-03。原世界v5 SHA256：
`aa3e9085bf08236193537418f5cb1d32d6254f5cf0b2b7991f4796e82a2a8ac5`。
没有改旧世界、货物碰撞参数、原110阈值、001～009或GitHub。

## 已实际运行的证据

| 报告ID | exit | 实际范围与结果 |
| --- | --- | --- |
| p4-vehicle-rgbd-optics-20261003-01 | 1 | 垂直相机轮廓少约8mm，6mm形状门拒绝；缺深度/遮挡拒绝正确 |
| p4-vehicle-rgbd-optics-20261003-02 | 1 | 1280帧超过默认离屏缓冲，异常日志保留；无完整探针报告 |
| p4-vehicle-rgbd-optics-20261003-03 | 1 | 1280对照仍差约8mm，不是采样不足的修复 |
| p4-vehicle-rgbd-optics-20261003-04/05/06 | 0 | 斜视安装，原形状门下定位/两红一蓝；初始化移位、遮挡、缺深度四例通过。最新320×240、30°、无多重采样 |
| p4-vehicle-rgbd-motion-20261003-01/02/03/04 | 1 | 图像过期保护触发SIM冻结；各旧报告保留，不能当机械停稳。02在刷新补丁部署前启动，不作当前源码通过证据 |
| p4-vehicle-rgbd-motion-20261003-05 | 1 | 图像过期撤销行驶锁存；任务FAIL，实际车辆制动PASS，未冻结。行驶0.084662m，未达到本探针运动下限 |
| p4-vehicle-rgbd-motion-20261003-06 | 0 | 单线程资源对照：实际0.118544m，托盘相对滑移0.70735mm，8执行项PASS；车辆停止2.08389e-6m/s/漂移2.95630e-8m，78.853s |
| p4-vehicle-rgbd-missing-20261003-01 | 1 | 全程缺深度，零行驶指令；任务FAIL、negative_safety_result=PASS，车辆停稳，无SIM冻结 |
| p4-initialized-load-nav2-20261003-01 | 1 | 原宽足迹：9节点ACTIVE、目标accepted但动作超时。另有数组JSON序列化异常，完整父报告丢失；保留Nav2子报告/日志，不补造真值 |
| p4-loaded-clearance-20261003-01 | 0 | 零物理步：宽足迹与机械臂底座扫描高度AABB投影重叠22.192mm，空载无重叠；只是几何诊断 |
| p4-centered-clearance-20261003-01 | 0 | 严格居中候选的初始投影无重叠；仍不等于动态防撞 |
| p4-centered-load-nav2-20261003-01 | 0 | 同世界初始化载荷Nav2，11执行项PASS；到达0.124910m、车辆停止2.19802e-6m/s/漂移3.84533e-8m，97.791s；归还停车 |

## 控制与信息边界

- 相机外参是车体固定目录标定，投影直接到base坐标，不读取运行中托盘/body世界真值。颜色、深度与目录尺寸识别托盘轮廓、计数和分格。
- 模板只在第一步之前由目录准备；初始化三个自由零件是独立试验，**不是**真实机械臂装盘或人形供盘。相机杆未建立实体硬件模型，属于声明的虚拟传感器。
- 数量测量不自行授权订单。数量比较在读者之外；甲板支持来自明确标注的理想solver接触传感器。
- `LoadMotionGate`只撤销运动权限，零轮速制动仍可由原唯一伺服执行；拒绝锁存，后续新图像不自动恢复。物理停止必须另测，不能由这个门自行宣布。
- 图像时效仍0.5s，两种时间一起检查。低开销相机关闭阴影/反射/多重采样，不改物理；运行06仍有制动期慢帧，所以**不宣称硬实时性能已保证**。
- 居中候选保留目录球半径0.335m、机构余量0.0421921m；允许中心y由±125mm收紧到±60mm，**不确定度整体也必须在域内**。这是新的受限操作许可，不是放宽原验收尺，也不是对任何装载状态缩小足迹。原宽配置原样保留。
- Nav2局部/全局足迹一致，collision_monitor唯一ROS速度发布者，载荷门接到最终原车轮伺服，不添加第二个速度写者。载荷超界/UNKNOWN产生制动请求与任务锁存。
- 静态地图仍是初始化几何先验，**不是SLAM**。二维包络不能冒充三维动态碰撞能力。
- 带载11项为执行探针判决；尚无独立货物角速度保守界/三件保持/3D扫掠裁判。这不是P4/P5/V1 DONE。

## 复测命令

```bash
cd ~/projects/010_heterogeneous_robot_workcell
.venv/bin/python experiments/make_centered_load_profile.py --check
# 每次使用新的run-id；禁止覆盖旧报告。
MUJOCO_GL=egl OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 LP_NUM_THREADS=1 \
  .venv/bin/python scripts/run_bounded.py --run-id <new-id> --timeout 420 \
  experiments/probe_vehicle_nav2.py --parking-handoff --centered-load
```

原宽导航序列化问题已加数组/scalar转换短测与原子checkpoint；旧run不改。

## 下一步（不跳过全链）

1. 在`probe_candidate_multi_loading.py`第一步前加车载相机，已有真实H供盘/三件视觉物理装盘及两段货权流程不变；仅在源→甲板实际确认后将第二个MOVE_TO_STATION接到guest Nav2会话。不重新加载世界/货物，不创建第二个writer。已保持的H原网络与机械臂批准姿态继续由plant._hold合成。
2. Nav2采用声明的站点目标与轮速odom/scan；到达后独立停止并关闭Nav2/IPC，再归还停车/精确DOCK。停靠理想本体/工装传感器假设明确；Nav2误差不替代毫米级停靠尺。车载视觉只在有货的运输段准入，未知必须锁存任务，不能重启后悄悄恢复。
3. 补独立三件货物/托盘停止、保持、3D碰撞守卫与运行中故障/显式恢复，再P5完整订单、P6双车有限库存、P7的36实例。预算与权限按AGENTS执行。

## 当前执行状态

物理正负例及完整回归全部退出，无后台续跑承诺。完整回归日志`runtime/pytest-vehicle-loaded-nav-20261003.log`：**1022 passed/1 skipped，169.20s，exit0**。冻结33资产/122源码/110原阈值`--check`通过；居中profile生成`--check`通过。
