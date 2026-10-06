# 共同世界导航接口续作 · 2026-10-03

## 已实测的范围

`p4-shared-world-ros-io-20261003-01`：exit0，43.2795 s wall、10 s sim。
人形控制器、LogisticsPlant 和 WorldOwner 共用同一个模型/状态；只允许 WorldOwner 提交向量和推进物理。
实际发送201个状态报文，ROS启动后收到184个；184次/clock、184次/scan、184次轮式/odom，decode_refusals=0。
17帧差额含ROS导入/启动期间无人接收，不能写成零丢帧。
无速度发布者，commands_sent=0；这次没有验证导航动作或机械制动。
原始证据：`reports/p4-shared-world-ros-io-20261003-01/report.json`、`ros_report.json`、`run_manifest.json`。

## 新文件及边界

- `joint_world_nav_io.py`：接入已有plant，不创建物理世界，不重置/步进；轮速和实际场景射线通过既有IPC格式发送，无body pose上网。
- `joint_world_ros_bridge.py`：从原010桥复制为独立候选，旧probe_n_ros.py不改。只接收TwistStamped的cmd_vel，保持原控制器时间戳和回调wall租约。
- `ros_command_lease.py`：原消息sim年龄和wall年龄均不超过0.5 s；重播时间戳、不合格限速、非有限值、未来/旧指令拒绝。重复发送不能给最后一条指令续命。
- `probe_joint_world_nav_io.py`：独立有界通讯短测，启动/终止本run直接拥有的ROS子进程；没有Nav2、没有订单或运行时货物位置写入。

## 本轮修正而非既成能力

1. 静态零步MuJoCo实测无命中为distance=-1、geom=-1。新接口将无命中映射到range_max，而非range_min；相关数学测试保护。
2. 明确拒绝同时安装停车航向servo和Nav2接口，避免两个控制逻辑争用yaw。
3. wall租约保护不等于机械停止。当前IPC只支持第一台车，显式拒绝n2，不声称双车已实现。
4. ROS收到帧不等于定位/导航成功。真实controller失联物理负例、Nav2到站、扫描与AMCL质量判决仍NOT_RUN。

## 下一步门（不跳过）

1. 先查清连续装盘后偏航：初始停车-.02 rad诊断能回正，但加载后的-0.127 rad以及DOCK末-0.206 rad尚未修复。保存真实控制/接触和加载快照，短诊断再决定控制修改。
2. 派生独立共同世界地图与完整车载包络，不用旧0.26 m裸车半径。甲板在车体前方1.92 m，还含滑移/偏航/推杆自由度；必须声明运输姿态并由实际本体状态监督，不能仅用ctrl范围作为物理位置证明。
3. 再启动一个ROS域、一时钟的Nav2短目标，在当前v5实际世界用扫描/轮式里程计定位，独立真值只给裁判；先单车、再真实载荷，之后才双车和订单。

这份记录没有改变V1必需项，也不把几何先验地图叫SLAM。P4-BELT-03仍PARTIAL，P5/P6/P7未完成。
