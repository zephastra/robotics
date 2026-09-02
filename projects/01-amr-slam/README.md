# AMR SLAM Demo

一个可直接运行的 ROS 2 差速 AMR 仿真项目：在 Gazebo Sim 中驾驶机器人，通过 `ros_gz_bridge` 接入激光与里程计数据，使用 `slam_toolbox` 在线构建二维栅格地图，并在 RViz 中观察结果。

这是 Zephastra Robotics 的第一个完整演示项目，重点是让初学者看清楚从仿真传感器到地图的完整数据链路。

## 功能

- Gazebo Sim 差速底盘、二维激光雷达、室内环境和障碍物。
- 键盘控制前进、后退、转弯和停止。
- `slam_toolbox` 在线异步建图。
- RViz 地图、LaserScan 和 TF 可视化。
- 一键保存 `map.pgm` 与 `map.yaml`。
- 使用保存地图启动 AMCL 与 Nav2 导航。
- 启动脚本统一管理后台进程，退出时自动清理。

## 已验证环境

| 组件 | 验证基线 |
| --- | --- |
| 操作系统 | Ubuntu 26.04（WSL 2） |
| ROS 2 | Lyrical Luth |
| Gazebo Sim | ROS vendor 基线 10.4.0；测试机同时安装 Jetty nightly 包 |
| RMW | Cyclone DDS |
| 可视化 | RViz 2 / WSLg |

其他版本可能也能运行，但尚未在本项目中完成端到端验证。

## 数据流

```text
Gazebo lidar ──> /scan ───────────────┐
Gazebo drive ──> /odom ──> odom→base ├─> slam_toolbox ──> /map + map→odom
static TF ─────────> base→laser ──────┘

/cmd_vel <── teleop_twist_keyboard
```

TF 树：

```text
map → odom → base_link → laser_link
```

更详细的设计说明见 [docs/architecture.md](docs/architecture.md)。

## 目录结构

```text
01-amr-slam/
├── config/              # bridge、SLAM 与 Nav2 参数
├── docs/                # 架构与 Nav2 使用文档
├── maps/                # 运行时保存的地图（默认不提交）
├── nodes/               # odom → TF 辅助节点
├── rviz/                # 建图与导航视图
├── scripts/             # 安装、启动、桥接和地图保存脚本
├── worlds/              # Gazebo 世界与机器人模型
├── CHANGELOG.md
└── README.md
```

## 快速开始

前置条件：WSL 2 或原生 Ubuntu 已安装 ROS 2 Lyrical，并配置好 ROS 软件源。

```bash
git clone https://github.com/zephastra/robotics.git
cd robotics/projects/01-amr-slam
bash scripts/install.sh
bash scripts/run_mapping.sh
```

`install.sh` 只需运行一次。Gazebo 和 RViz 打开后，当前终端会进入键盘控制。

| 按键 | 动作 |
| --- | --- |
| `i` | 前进 |
| `,` | 后退 |
| `j` | 左转 |
| `l` | 右转 |
| `k` | 停止 |

默认直线速度为 `0.25 m/s`，角速度为 `0.35 rad/s`。这个速度兼顾操作体验和转弯时的激光扫描重叠率。

按 `Ctrl+C` 可结束本次运行，脚本会清理它启动的 Gazebo、桥接、TF、SLAM 和 RViz 进程。

启动前脚本会检查是否已有 Gazebo、bridge 或 SLAM 会话，发现冲突时只会提示退出，不会自动终止用户进程。

### 无界面运行

```bash
AMR_HEADLESS=1 bash scripts/run_mapping.sh
```

该模式会启动 Gazebo Server，但不会启动 Gazebo GUI、RViz 或键盘控制。

## 保存地图

保持建图程序运行，另开一个终端：

```bash
cd robotics/projects/01-amr-slam
bash scripts/save_map.sh
```

默认输出：

```text
maps/map.pgm
maps/map.yaml
```

也可以指定其他输出前缀：

```bash
bash scripts/save_map.sh /tmp/my_map
```

## Nav2 导航

保存地图后运行：

```bash
bash scripts/run_nav2.sh
```

完整操作和验收标准见 [docs/nav2.md](docs/nav2.md)。建图与导航不要同时启动：`slam_toolbox` 和 AMCL 都会发布 `map→odom`。

## 建图建议

- 转弯前先按 `k` 停止前进，再使用 `j` 或 `l` 缓慢转弯。
- 不要让机器人顶着墙持续驱动；轮子空转会造成轮式里程计漂移。
- 沿环境走一圈并回到起点附近，给回环检测提供足够的重叠区域。
- 如果地图已经出现旋转重影，应停止程序并重新开始建图，不要在错误地图上继续行驶。

项目使用 `minimum_travel_heading` 和 `minimum_travel_distance` 控制关键扫描密度。不要使用属于其他 SLAM 实现的 `angular_update` 或 `linear_update` 参数名。

## 排错

| 现象 | 建议检查 |
| --- | --- |
| 没有 `/scan` | 检查 `gz topic -l` 和 `ros2 topic hz /scan` |
| 没有 `odom→base_link` | 确认 `nodes/odom_to_tf.py` 正在运行 |
| RViz 地图不更新 | Map 的 Durability 应为 Transient Local，并确认使用仿真时间 |
| LaserScan 不显示 | Reliability 应为 Best Effort；本项目 RViz 配置已预设 |
| 机器人不动 | 检查 `/cmd_vel` 和 bridge 日志，并确认 Gazebo 没有暂停 |
| 地图转弯后重影 | 降低转速、避免顶墙，确认没有修改 SLAM 关键帧参数 |
| 提示已有 AMR/ROS 会话 | 回到上一次启动终端按 `Ctrl+C`，确认退出后再运行 |
| WSLg 图形窗口异常 | `scripts/amr_env.sh` 默认使用 Qt XCB，可通过 `AMR_QT_PLATFORM` 覆盖 |

日志默认位于：

```text
/tmp/amr_slam_logs/
/tmp/amr_nav_logs/
```

## 已知限制

- 当前使用 Gazebo 差速驱动插件提供的轮式里程计，没有融合 IMU。
- 机器人碰撞、轮胎打滑或长时间顶墙时，地图仍可能漂移。
- Nav2 配置面向本演示环境，不代表真实机器人上的安全参数。
- Gazebo nightly 与 ROS vendor 版本存在差异时，应优先使用与 ROS 发行版配套的版本。

## 项目状态

`v0.1.0`：建图主流程已经完成完整启动验证；Nav2 属于第二阶段功能，验收方法见单独文档。

## License

项目代码遵循仓库根目录的 Apache License 2.0。原创文档遵循仓库文档许可说明。ROS 2、Gazebo、SLAM Toolbox、Nav2 和其他依赖继续受各自上游许可证约束。
