# 05 Mobile Robots

收录移动机器人、AMR、AGV 的运动学、里程计、状态估计、SLAM 与导航资料。

## Official and maintainer resources

| 资源 | 提供方 | 主题 | 版本提示 | 难度 | 状态 | 最后检查 |
| --- | --- | --- | --- | --- | --- | --- |
| [Nav2 Documentation](https://docs.nav2.org/) | Open Navigation / Nav2 project | 定位、规划、控制、行为树和代价地图 | 选择与 ROS 2 一致的分支 | Intermediate | Source Checked | 2026-08-31 |
| [Nav2 Quickstart](https://docs.nav2.org/rolling/getting_started/quickstart/quickstart/) | Nav2 project | 在仿真中启动 TurtleBot 并完成导航 | 页面为 Rolling | Beginner | Source Checked | 2026-08-31 |
| [Nav2 First-Time Robot Setup Guide](https://docs.nav2.org/setup_guides/index.html) | Nav2 project | 为自定义机器人配置 TF、URDF、里程计、传感器和插件 | 注意 Gazebo / Gazebo Classic 区别 | Intermediate | Source Checked | 2026-08-31 |
| [Nav2 Navigation Concepts](https://docs.nav2.org/rolling/getting_started/navigation_concepts/) | Nav2 project | 服务器、状态估计、环境表示和行为树概念 | 页面为 Rolling | Beginner–Intermediate | Source Checked | 2026-08-31 |
| [SLAM Toolbox](https://github.com/SteveMacenski/slam_toolbox) | SLAM Toolbox maintainers | ROS 2 激光 SLAM、序列化地图和定位模式 | 按 ROS 2 分支选择 | Intermediate | Source Checked | 2026-08-31 |
| [robot_localization](https://github.com/cra-ros-pkg/robot_localization) | robot_localization maintainers | EKF/UKF、多传感器状态估计和 GPS 融合 | 按 ROS 2 分支选择 | Intermediate–Advanced | Source Checked | 2026-08-31 |
| [Cartographer ROS](https://google-cartographer-ros.readthedocs.io/en/latest/) | Google Cartographer project | 2D/3D 实时 SLAM 的 ROS 集成 | 维护状态需结合仓库确认 | Advanced | Source Checked | 2026-08-31 |
| [TurtleBot 4 User Manual](https://turtlebot.github.io/turtlebot4-user-manual/) | Clearpath Robotics / Open Robotics | ROS 2 移动机器人平台、仿真、SLAM 与导航 | 按系统镜像和 ROS 版本使用 | Beginner–Intermediate | Source Checked | 2026-08-31 |

## Suggested learning order

1. 差速或全向底盘运动学、轮速与里程计。
2. TF 树、URDF/SDF、传感器消息和时间戳。
3. SLAM 与地图保存，再学习 AMCL 或 SLAM 定位模式。
4. Nav2 全局规划、局部控制、代价地图和行为树。
5. 最后再做参数调优、动态障碍、覆盖导航或多机器人。

项目排错时优先确认 TF、时间、里程计和传感器数据是否正确，再调整 Nav2 参数；参数调优无法补救错误的数据链路。

新增资源时使用 [统一模板](../RESOURCE_TEMPLATE.md)。
