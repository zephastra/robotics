# 07 Gazebo 与 ROS 2 桥接

> 状态：Draft
> 适用环境：ROS 2 Lyrical、Gazebo Jetty、`ros_gz` 3.x
> 当前验证范围：已确认相关软件包和 launch 参数存在；尚未使用本篇示例完成端到端仿真
> 最后核对：2026-08-31
> 原文：[Gazebo Jetty 与 ROS 2](https://gazebosim.org/docs/jetty/ros_installation/)、[`ros_gz_bridge` 文档](https://docs.ros.org/en/ros2_packages/lyrical/api/ros_gz_bridge/)
> 上游许可：Gazebo 文档为 CC BY 4.0；包代码以 `ros_gz` 仓库声明为准

Gazebo Transport 与 ROS 2 DDS 是两套通信系统。`ros_gz_bridge` 在两种消息类型之间转换，让 ROS 2 可以接收仿真传感器数据或向仿真发送控制指令。

> Lyrical 官方推荐搭配 Gazebo Jetty，不是 Harmonic。当前审计机器安装了 nightly 版本，因此运行结果可能早于稳定版行为。

## 1. 安装和检查

```bash
sudo apt update
sudo apt install ros-lyrical-ros-gz

source /opt/ros/lyrical/setup.bash
ros2 pkg prefix ros_gz_bridge
ros2 pkg prefix ros_gz_sim
gz sim --versions
```

## 2. 分别查看两侧话题

启动 Gazebo 示例世界：

```bash
gz sim shapes.sdf
```

另开终端：

```bash
gz topic -l
ros2 topic list
```

Gazebo 话题和 ROS 2 话题不会因为名称相似而自动连接。

## 3. 单话题桥接语法

`parameter_bridge` 使用以下形式：

```text
<topic>@<ROS_TYPE>@<GZ_TYPE>  # 双向
<topic>@<ROS_TYPE>[<GZ_TYPE>  # Gazebo -> ROS 2
<topic>@<ROS_TYPE>]<GZ_TYPE>  # ROS 2 -> Gazebo
```

例如，若 Gazebo 侧的实际激光话题就是 `/scan`：

```bash
ros2 run ros_gz_bridge parameter_bridge \
  '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan'
```

不要假定传感器话题一定叫 `/scan`。实际名称应从 `gz topic -l` 读取；不同名称的映射更适合使用 YAML。

## 4. 用 YAML 管理多个桥接

新建 `bridge.yaml`，把 `<world>` 和 `<robot>` 替换为仿真中的实际名称：

```yaml
- ros_topic_name: /scan
  gz_topic_name: /world/<world>/model/<robot>/link/<link>/sensor/<sensor>/scan
  ros_type_name: sensor_msgs/msg/LaserScan
  gz_type_name: gz.msgs.LaserScan
  direction: GZ_TO_ROS

- ros_topic_name: /clock
  gz_topic_name: /clock
  ros_type_name: rosgraph_msgs/msg/Clock
  gz_type_name: gz.msgs.Clock
  direction: GZ_TO_ROS

- ros_topic_name: /cmd_vel
  gz_topic_name: /model/<robot>/cmd_vel
  ros_type_name: geometry_msgs/msg/Twist
  gz_type_name: gz.msgs.Twist
  direction: ROS_TO_GZ

- ros_topic_name: /odom
  gz_topic_name: /model/<robot>/odometry
  ros_type_name: nav_msgs/msg/Odometry
  gz_type_name: gz.msgs.Odometry
  direction: GZ_TO_ROS
```

这些 Gazebo 话题名只是常见示例。先用 `gz topic -l` 确认实际名称，再修改配置。

启动桥接：

```bash
source /opt/ros/lyrical/setup.bash
ros2 launch ros_gz_bridge ros_gz_bridge.launch.py \
  bridge_name:=ros_gz_bridge \
  config_file:=/absolute/path/to/bridge.yaml
```

原草稿中的参数 `name:=` 已更正为 Lyrical launch 文件实际支持的 `bridge_name:=`。

## 5. 同时启动 Gazebo 和桥接

```bash
ros2 launch ros_gz_sim ros_gz_sim.launch.py \
  world_sdf_file:=/absolute/path/to/world.sdf \
  bridge_name:=ros_gz_bridge \
  config_file:=/absolute/path/to/bridge.yaml
```

## 6. 验证

```bash
ros2 topic list -t
ros2 topic echo /clock
ros2 topic echo /scan
ros2 topic info /cmd_vel
```

SLAM 和 Nav2 使用 `use_sim_time:=true` 时需要有效的 `/clock`。但是 `/odom` 话题本身不等同于 TF；后续还必须确认 TF 树中存在 `odom -> base_link`。

## 7. 传感器类型提醒

不要采用“新版本只能使用 `gpu_lidar`、不支持 `lidar`”这类笼统结论。应检查所用 Gazebo 版本、SDF 规范、具体传感器类型以及控制台错误，再决定使用 `lidar`、`gpu_lidar` 或其他实现。

[上一篇：服务与客户端](../../fundamentals/06-service-client/06-服务客户端实战.md) · [返回教程目录](../../README.md) · [下一篇：SLAM](../08-slam/08-SLAM建图.md)
