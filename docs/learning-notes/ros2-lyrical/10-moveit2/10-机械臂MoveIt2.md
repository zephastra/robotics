# 10 MoveIt 2：机械臂运动规划概览

> 状态：Draft
> 适用环境：ROS 2 Lyrical、单独构建的 MoveIt 2 工作空间
> 当前验证范围：已发现 `/opt/moveit2/setup.bash`；尚未确认完整示例机械臂、控制器和规划流程
> 最后核对：2026-08-31
> 原文：[MoveIt 2 Tutorials](https://moveit.picknik.ai/)、[教程源码](https://github.com/moveit/moveit2_tutorials)
> 上游许可：MoveIt 2 Tutorials 为 BSD-3-Clause；文档页和依赖应按各自声明处理

MoveIt 2 为机械臂提供运动规划、碰撞检测、运动学和轨迹执行能力。本篇只是概念与验证清单，不是已经跑通的 Lyrical 教程。

## 1. 核心组件

| 概念 | 作用 |
| --- | --- |
| Planning Scene | 表示机器人状态、碰撞物体和环境 |
| Planning Group | 定义一起规划的一组关节，例如机械臂或夹爪 |
| Kinematics | 正向与逆向运动学 |
| Planning Pipeline | 连接规划请求、规划器和适配器 |
| Trajectory Execution | 把规划轨迹发送给控制器执行 |

## 2. 当前环境的正确加载顺序

当前机器存在 `/opt/moveit2/setup.bash`，说明 MoveIt 2 不是系统 `/opt/ros/lyrical` 中的普通二进制安装，而是另一个 overlay。使用前应先加载 ROS，再加载 MoveIt：

```bash
source /opt/ros/lyrical/setup.bash
source /opt/moveit2/setup.bash
```

不要仅凭目录存在就断言所有 MoveIt 包和示例已经成功构建。应进一步检查：

```bash
ros2 pkg prefix moveit_ros_move_group
ros2 pkg prefix moveit_setup_assistant
```

## 3. Setup Assistant

如果 `moveit_setup_assistant` 包存在，可启动配置向导：

```bash
ros2 launch moveit_setup_assistant setup_assistant.launch.py
```

通常需要完成：

1. 导入可正常显示、关节定义完整的 URDF/Xacro。
2. 生成自碰撞矩阵。
3. 定义 planning groups、末端执行器和虚拟关节。
4. 配置运动学、关节限制和控制器。
5. 生成独立的 `<robot>_moveit_config` 包。

## 4. 真正“跑通”的验收条件

只有满足以下条件，才能把本文状态改成 Verified：

- `move_group` 启动且无缺少插件错误。
- RViz 正确显示机器人、规划场景和交互标记。
- 能为一个合法目标生成轨迹。
- 轨迹能由 fake controller 或真实 `ros2_control` 控制器执行。
- 所有启动命令、机器人模型和配置文件都能从公开仓库复现。

原草稿中的 `<robot>`、待补报错和个人构建结论都不是可执行教程，因此本轮只保留为明确的验证路线。

[上一篇：Nav2](../09-nav2/09-Nav2导航.md) · [返回目录](../README.md) · [下一篇：视觉与 PLC 项目笔记](../11-vision-plc/11-视觉与PLC工业闭环.md)
