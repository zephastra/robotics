# 03 ROS

收录 ROS、ROS 2 及核心生态的官方资料。ROS 2 文档与发行版强相关，使用前先确认自己的发行版。

## Core official resources

| 资源 | 提供方 | 主题 / 版本 | 语言 | 难度 | 状态 | 最后检查 |
| --- | --- | --- | --- | --- | --- | --- |
| [ROS Developer Documentation](https://docs.ros.org/) | Open Robotics / ROS community | ROS 发行版入口、教程、API 和工具 | English | All levels | Source Checked | 2026-08-31 |
| [ROS 2 Lyrical 官方教程索引](ros2-official-tutorials.md) | 本仓库索引；上游为 ROS 2 官方文档 | Lyrical Lynx；94 个官方教程链接 | 中文导航 / English source | Beginner–Advanced | Source Checked | 2026-08-31 |
| [ROS 2 Concepts](https://docs.ros.org/en/rolling/Concepts.html) | ROS 2 Documentation | 节点、接口、DDS、QoS、安全和中间件概念 | English | Beginner–Advanced | Source Checked | 2026-08-31 |
| [ROS 2 How-To Guides](https://docs.ros.org/en/rolling/How-To-Guides.html) | ROS 2 Documentation | 常见任务和配置指南 | English | Intermediate | Source Checked | 2026-08-31 |
| [REP Index](https://www.ros.org/reps/rep-0000.html) | ROS community | ROS Enhancement Proposals；设计和标准依据 | English | Advanced | Source Checked | 2026-08-31 |
| [ros2_control Documentation](https://control.ros.org/master/) | ros-controls project | 硬件接口、控制器和实时控制框架 | English | Intermediate–Advanced | Source Checked | 2026-08-31 |
| [Nav2 Documentation](https://docs.nav2.org/) | Open Navigation / Nav2 project | ROS 2 自主导航框架 | English | Intermediate | Source Checked | 2026-08-31 |
| [MoveIt 2 Tutorials](https://moveit.picknik.ai/main/doc/tutorials/tutorials.html) | MoveIt project | 机械臂规划、场景、抓取和配置 | English | Intermediate | Source Checked | 2026-08-31 |

## Version guidance

- 优先使用与你安装版本一致的文档路径，例如 `/en/lyrical/`、`/jazzy/` 或 `/humble/`，不要直接照搬 Rolling 命令。
- `master`、`main` 和 `rolling` 多数代表开发中的最新版，适合查新特性，但不保证与你的稳定发行版一致。
- Nav2、MoveIt 2 和 ros2_control 也有各自的版本选择器；ROS 2 版本一致并不自动保证所有第三方包参数完全相同。

新增资源时使用 [统一模板](../RESOURCE_TEMPLATE.md)。
