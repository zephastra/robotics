# 08 SLAM 建图：slam_toolbox 数据链路

> 状态：Draft
> 适用环境：ROS 2 Lyrical、2D LaserScan、`slam_toolbox`
> 当前验证范围：本机尚未安装并完成 Lyrical 端到端建图；命令必须在项目模型上再次验证
> 最后核对：2026-08-31
> 原文：[slam_toolbox](https://github.com/SteveMacenski/slam_toolbox)、[Nav2：Navigating While Mapping](https://docs.nav2.org/tutorials/docs/navigation2_with_slam.html)
> 上游许可：`slam_toolbox` 为 LGPL-2.1；Nav2 文档许可见其仓库

SLAM（Simultaneous Localization and Mapping）让机器人在未知环境中估计自身位姿并构建地图。本篇只描述完整数据链路和验证顺序，不把尚未跑通的 AMR 示例写成成品教程。

## 1. 先安装依赖

```bash
sudo apt update
sudo apt install \
  ros-lyrical-slam-toolbox \
  ros-lyrical-nav2-map-server \
  ros-lyrical-teleop-twist-keyboard
```

安装后检查：

```bash
source /opt/ros/lyrical/setup.bash
ros2 pkg prefix slam_toolbox
ros2 pkg prefix nav2_map_server
```

## 2. slam_toolbox 需要什么

| 输入或变换 | 通常由谁提供 |
| --- | --- |
| `sensor_msgs/msg/LaserScan`，通常为 `/scan` | 激光雷达驱动或 Gazebo 桥接 |
| `odom -> base_link` TF | 底盘里程计或状态估计节点 |
| `base_link -> laser` TF | `robot_state_publisher` 或静态 TF |
| `/clock`（仅仿真时间） | Gazebo 桥接 |

重要修正：`slam_toolbox` 的关键依赖是雷达消息和 TF，不是必须直接订阅一个名为 `/odom` 的话题。里程计通常产生 `/odom`，但 SLAM 使用的是对应 TF 链。

建图时典型 TF 关系为：

```text
map -> odom -> base_link -> laser
```

- `slam_toolbox` 发布 `map -> odom`。
- 底盘里程计发布 `odom -> base_link`。
- 机器人模型发布 `base_link -> laser`。

## 3. 启动前检查

```bash
ros2 topic list -t
ros2 topic hz /scan
ros2 topic echo /clock --once
ros2 run tf2_ros tf2_echo odom base_link
ros2 run tf2_ros tf2_echo base_link laser
```

`laser` 必须换成实际激光坐标系名称。若任何一项缺失，应先修复数据源或 TF，不要直接启动 SLAM 猜参数。

## 4. 启动 slam_toolbox

先复制官方参数文件到项目中，再修改 `scan_topic`、`base_frame`、`odom_frame` 和 `map_frame`，避免直接修改 `/opt/ros` 下的文件。

```bash
mkdir -p ~/ros2_ws/src/<project_package>/config
cp /opt/ros/lyrical/share/slam_toolbox/config/mapper_params_online_async.yaml \
  ~/ros2_ws/src/<project_package>/config/slam.yaml
```

启动：

```bash
ros2 launch slam_toolbox online_async_launch.py \
  use_sim_time:=true \
  slam_params_file:=/absolute/path/to/slam.yaml
```

如果使用真实机器人，应根据时间源配置 `use_sim_time`，不能固定照抄 `true`。

## 5. 移动机器人并观察地图

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

在 RViz 中设置 Fixed Frame 为 `map`，添加 `Map`、`LaserScan`、`TF` 和 `RobotModel`。

原草稿中“用 RViz 的 2D Goal Pose 在没有 Nav2 时移动机器人”的说法已删除。2D Goal Pose 只是在发布导航目标；必须有 Nav2 或其他目标处理节点才会驱动机器人。

## 6. 保存地图

```bash
mkdir -p ~/maps
ros2 run nav2_map_server map_saver_cli -f ~/maps/amr_map
```

通常会生成图像文件和对应的 YAML 元数据文件。保存后检查 YAML 中的图像相对路径是否正确。

## 7. 最小验收标准

- `/scan` 频率稳定，时间戳与仿真时钟一致。
- TF 树连续且无多个节点争抢同一变换。
- RViz 中激光与机器人模型对齐。
- 机器人移动后地图连续，不出现明显撕裂或跳变。
- 地图能保存，并能由 `nav2_map_server` 重新加载。

[上一篇：Gazebo 桥接](../07-gazebo-bridge/07-Gazebo仿真与桥接.md) · [返回教程目录](../../README.md) · [下一篇：Nav2](../09-nav2/09-Nav2导航.md)
