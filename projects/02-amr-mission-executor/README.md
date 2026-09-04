# 02 AMR 仓库任务执行、异常恢复与自动返航

这是一个可独立运行的 ROS 2 + Gazebo Sim + Nav2 仓库 AMR 演示工程。项目自带机器人、仓库世界、静态地图、定位与导航参数、RViz 配置和任务管理器，不读取也不启动工程 01 的任何文件。

## 项目目标

机器人从充电区出发，依次完成入库确认、货架取货、货架盘点、质检和出库任务。任务结束、人工取消或电量不足时，机器人自动返回充电区。

仓库包含两组货架区、纵向窄通道、中央横向通道、入库区、出库区、质检区、充电区以及可由 Gazebo 界面移动的测试货箱。

## 主要能力

- 独立 Gazebo 仓库场景和差速 AMR 模型。
- 独立占据栅格地图、AMCL、Nav2 和 RViz 配置。
- YAML 多目标任务定义与任务状态机。
- 单点导航超时、失败重试以及 `skip` / `abort` 策略。
- 基于机器人实际位移与目标距离进展的通道阻塞检测。
- `BLOCKED → WAITING_FOR_CLEARANCE → RECOVERING` 恢复状态机。
- 任务暂停、继续和取消。
- 低电量中止当前任务并自动返航。
- Nav2 局部代价地图处理临时障碍物。
- RViz 显示地图、路径、代价地图和任务点状态。
- JSON 任务事件与最终执行报告。
- 项目专用 ROS Domain ID 和 Gazebo Partition，避免与其他项目串线。

## 架构

```text
warehouse_world.sdf ──> Gazebo Sim ──> ros_gz_bridge
       │                                      │
       │                             /scan /odom /cmd_vel
       │                                      │
warehouse_map.yaml ──> AMCL + Nav2 <──────────┘
                              ▲
                              │ /navigate_to_pose
warehouse_demo.yaml ──> Mission Manager
                              │
                 retry / skip / pause / return-home
                              │
                    RViz markers + JSON report
```

## 环境要求

- WSL2 + WSLg
- ROS 2 Lyrical（默认安装在 `/opt/ros/lyrical`）
- Gazebo Sim
- Nav2（支持 `/opt/nav2/setup.bash` 或系统安装）
- `ros_gz_bridge`、`rviz2`、`pytest`

检查依赖：

```bash
git clone https://github.com/zephastra/robotics.git
cd robotics/projects/02-amr-mission-executor
bash scripts/check_dependencies.sh
```

## 一键运行完整演示

正常任务：

```bash
cd robotics/projects/02-amr-mission-executor
bash run_demo.sh normal
```

脚本会自动构建并依次启动 Gazebo、桥接、TF、AMCL、Nav2、RViz 和任务管理器。任务会自动开始，不需要在 RViz 中手动点击 `Nav2 Goal`。

自动阻塞 A 区货架通道、等待恢复、清除障碍后继续：

```bash
bash run_demo.sh blocked_aisle
```

该模式会等待任务管理器真正开始 `shelf_a_pickup`，然后放置障碍物；检测到
阻塞恢复计数增加后才会清除障碍，因此不受 Gazebo 启动速度和电脑负载影响。

自动在任务途中触发低电量并返航：

```bash
bash run_demo.sh low_battery
```

三个模式会分别生成任务日志与 JSON 报告，适合做正常运行、通道阻塞恢复和低电量返航的对比演示。

无界面运行：

```bash
AMR_HEADLESS=1 bash run_demo.sh
```

只启动仿真与导航、不执行仓库任务：

```bash
AMR_SKIP_MISSION=1 bash run_demo.sh
```

按 `Ctrl+C` 会停止本项目启动的全部进程。

## 手动控制通道障碍物

运行任务时，可以在另一个已加载项目环境的终端中执行：

```bash
cd robotics/projects/02-amr-mission-executor
bash scripts/block_aisle.sh
```

这会把红色测试货箱移动到 A 区货架任务点，使目标暂时不可达。清除障碍：

```bash
bash scripts/clear_aisle.sh
```

默认恢复配置为：15 秒没有明显前进则判定阻塞，等待 6 秒后重试，最多恢复 3 次。可以在 `warehouse_demo.yaml` 中调整：

```yaml
blocked_timeout_seconds: 15.0
minimum_progress_distance: 0.15
recovery_wait_seconds: 6.0
max_blocked_recoveries: 3
```

阻塞监控同时检查机器人实际位置和到目标的最佳剩余距离：机器人完全停止会快速
触发恢复；持续来回移动但没有接近目标也会被识别。导航超时与阻塞计时使用 ROS
仿真时钟，所以 Gazebo 低于实时速度运行时不会提前取消正常导航。

## 单独运行任务管理器

当本项目的导航系统已经启动时，可以运行：

```bash
bash scripts/run_mission.sh
```

指定其他任务文件：

```bash
bash scripts/run_mission.sh /absolute/path/to/mission.yaml
```

## 运行时控制

另开终端并加载本项目环境：

```bash
cd robotics/projects/02-amr-mission-executor
source scripts/amr_env.sh
```

暂停、继续和取消：

```bash
ros2 service call /mission/pause std_srvs/srv/Trigger
ros2 service call /mission/resume std_srvs/srv/Trigger
ros2 service call /mission/cancel std_srvs/srv/Trigger
```

强制低电量与重置电池：

```bash
ros2 service call /mission/battery/force_low std_srvs/srv/Trigger
ros2 service call /mission/battery/reset std_srvs/srv/Trigger
```

查看状态：

```bash
ros2 topic echo /mission/status
ros2 topic echo /mission/events
ros2 topic echo /mission/battery_percentage
```

## RViz 任务点颜色

- 蓝色：等待执行。
- 黄色：正在导航或执行。
- 绿色：任务成功。
- 红色：任务失败。
- 灰色：任务被跳过。
- 紫色：充电区 Home 点。

## 配置与测试

- 任务文件：`src/amr_mission_manager/config/warehouse_demo.yaml`
- 仓库世界：`worlds/warehouse_world.sdf`
- 仓库地图：`maps/warehouse_map.yaml`
- Nav2 参数：`config/nav2_params.yaml`
- RViz 配置：`rviz/warehouse_mission.rviz`
- 运行日志：`logs/`
- 任务报告：`reports/`

报告额外记录阻塞发生时间、剩余距离、恢复次数、等待时长以及最终处理结果。

重新生成匹配的仓库世界与地图：

```bash
python3 tools/generate_warehouse_assets.py
```

运行测试：

```bash
bash scripts/test.sh
```

当前验收基线为 `15 passed`。完整阻塞演示应完成 5 个任务、产生 1 次阻塞恢复、
普通重试为 0，并在第一次返航尝试中回到充电区；最终报告状态应为 `COMPLETED`。

## 与工程 01 的边界

工程 01 用于学习 SLAM 建图、地图保存、定位和单目标导航。本工程面向更完整的仓库任务执行。两个工程目录、地图、世界文件、启动脚本、ROS 通信域和运行进程相互独立。
