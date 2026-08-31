# 04 Simulation

收录机器人仿真器、物理引擎和数字孪生资料。仿真器版本、渲染后端和 ROS 2 发行版之间常有兼容性约束。

## Official documentation

| 资源 | 提供方 | 适用内容 | 语言 | 难度 | 状态 | 最后检查 |
| --- | --- | --- | --- | --- | --- | --- |
| [Gazebo Tutorials](https://gazebosim.org/docs/latest/tutorials/) | Gazebo project / Open Robotics | 现代 Gazebo 的入门、建模、传感器和插件教程 | English | Beginner–Advanced | Source Checked | 2026-08-31 |
| [Gazebo How-to Guides](https://gazebosim.org/docs/latest/how_to_guides/) | Gazebo project / Open Robotics | 安装、迁移、ROS 集成和常见任务 | English | Intermediate | Source Checked | 2026-08-31 |
| [Gazebo Documentation Source](https://github.com/gazebosim/docs) | Gazebo project | 文档源码、版本分支和贡献记录 | English | Intermediate | Source Checked | 2026-08-31 |
| [Getting Started with Isaac Sim](https://docs.nvidia.com/learning/physical-ai/getting-started-with-isaac-sim/latest/index.html) | NVIDIA | 机器人导入、传感器、ROS 2、SIL/HIL 和合成数据 | English | Beginner–Intermediate | Source Checked | 2026-08-31 |
| [Isaac Sim Tutorial List](https://docs.isaacsim.omniverse.nvidia.com/latest/introduction/tutorial_list.html) | NVIDIA | Isaac Sim 官方教程总入口 | English | Beginner–Advanced | Source Checked | 2026-08-31 |
| [MuJoCo Documentation](https://mujoco.readthedocs.io/en/stable/) | Google DeepMind | 模型、仿真、API、XML 和编程指南 | English | Intermediate–Advanced | Source Checked | 2026-08-31 |
| [Webots User Guide](https://cyberbotics.com/doc/guide/index) | Cyberbotics | 场景、机器人、控制器和 ROS 2 接口 | English | Beginner–Intermediate | Source Checked | 2026-08-31 |
| [CoppeliaSim User Manual](https://manual.coppeliarobotics.com/) | Coppelia Robotics | 场景、脚本、远程 API 与动力学 | English | Beginner–Advanced | Source Checked | 2026-08-31 |
| [Drake Documentation](https://drake.mit.edu/) | Robot Locomotion Group / MIT-Toyota | 多体动力学、优化、规划和仿真 | English | Advanced | Source Checked | 2026-08-31 |

## Version warnings

- **Gazebo Classic 已结束生命周期**。除非维护旧项目，否则优先使用 `gazebosim.org` 上的现代 Gazebo 文档。
- Isaac Sim、Isaac Lab、驱动和 GPU 要求变化较快；安装前必须同时检查产品版本、操作系统和显卡要求。
- `latest` 与 `stable` 会变化。可复现实验应在项目文档中记录确切版本，而不是只写“Gazebo”或“Isaac Sim”。

新增资源时使用 [统一模板](../RESOURCE_TEMPLATE.md)。
