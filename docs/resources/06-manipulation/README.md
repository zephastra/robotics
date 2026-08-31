# 06 Manipulation

收录机械臂运动学、规划、控制、抓取、标定和真实硬件接口资料。

## Courses and framework documentation

| 资源 | 提供方 | 主题 | 语言 | 难度 | 状态 | 最后检查 |
| --- | --- | --- | --- | --- | --- | --- |
| [Robotic Manipulation](https://manipulation.mit.edu/) | MIT / Russ Tedrake | 感知、规划、控制和接触丰富操作 | English | Intermediate–Advanced | Source Checked | 2026-08-31 |
| [MoveIt 2 Getting Started](https://moveit.picknik.ai/main/doc/tutorials/getting_started/getting_started.html) | MoveIt project | 环境、源码工作区和教程工程准备 | English | Beginner–Intermediate | Source Checked | 2026-08-31 |
| [MoveIt 2 Tutorials](https://moveit.picknik.ai/main/doc/tutorials/tutorials.html) | MoveIt project | RViz、C++ API、碰撞场景、抓取和规划 | English | Intermediate | Source Checked | 2026-08-31 |
| [MoveIt Task Constructor](https://moveit.github.io/moveit_task_constructor/) | MoveIt project | 多阶段操作任务、概念、教程和 API | English | Intermediate–Advanced | Source Checked | 2026-08-31 |
| [Pick and Place with MoveIt Task Constructor](https://moveit.picknik.ai/main/doc/tutorials/pick_and_place_with_moveit_task_constructor/pick_and_place_with_moveit_task_constructor.html) | MoveIt project | ROS 2 拾取放置任务的完整官方教程 | English | Intermediate–Advanced | Source Checked | 2026-08-31 |
| [ros2_control Documentation](https://control.ros.org/master/) | ros-controls project | 控制器管理器、硬件接口和控制器 | English | Intermediate–Advanced | Source Checked | 2026-08-31 |

## Real robot integrations

| 资源 | 提供方 | 适用硬件 | 状态 | 最后检查 |
| --- | --- | --- | --- | --- |
| [Universal Robots ROS 2 Driver](https://docs.universal-robots.com/Universal_Robots_ROS2_Documentation/doc/ur_robot_driver/ur_robot_driver/doc/index.html) | Universal Robots | UR CB3、e-Series 和 PolyScope X；含模拟硬件 | Source Checked | 2026-08-31 |
| [franka_ros2](https://support.franka.de/docs/franka_ros2.html) | Franka Robotics | Franka ROS 2 集成、MoveIt 和示例控制器 | Source Checked | 2026-08-31 |

## Safety and version notes

- 仿真、mock hardware 和真实机械臂不是同一风险等级。第一次连接真机前必须阅读制造商安全和操作手册。
- MoveIt 负责规划并不意味着轨迹一定适合真实硬件；还要核对关节限制、控制器接口、负载和工具坐标系。
- `main` 文档可能领先于稳定 ROS 2 发行版，构建教程前先选择匹配分支。

新增资源时使用 [统一模板](../RESOURCE_TEMPLATE.md)。
