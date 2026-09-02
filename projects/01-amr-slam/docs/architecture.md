# 系统架构

## ROS 数据链路

Gazebo Sim 负责物理仿真、差速驱动和激光传感器。`ros_gz_bridge` 将 Gazebo Transport 消息转换为 ROS 2 消息：

| ROS 话题 | 方向 | 用途 |
| --- | --- | --- |
| `/clock` | Gazebo → ROS | 仿真时间 |
| `/scan` | Gazebo → ROS | 二维激光扫描 |
| `/odom` | Gazebo → ROS | 轮式里程计 |
| `/cmd_vel` | ROS → Gazebo | 底盘速度命令 |

`nodes/odom_to_tf.py` 把 `/odom` 转换成 `odom→base_link` 动态 TF。启动脚本另外发布 `base_link→laser_link` 静态 TF。

## 建图模式

```text
/scan + /odom + TF
         │
         ▼
  slam_toolbox
    ├── /map
    └── map→odom
```

SLAM 参数提高了低速转弯时的关键扫描密度：

```yaml
minimum_time_interval: 0.05
minimum_travel_heading: 0.05
minimum_travel_distance: 0.05
```

默认键盘角速度限制为 `0.35 rad/s`，让相邻激光帧保持足够重叠，降低矩形环境中的错误旋转匹配概率。

## 导航模式

导航模式加载保存的地图，由 AMCL 负责定位并发布 `map→odom`，Nav2 负责规划、控制和恢复行为。导航模式不启动 `slam_toolbox`，避免重复发布同一 TF。

## 设计边界

这是教学仿真项目。轮式里程计在碰撞或打滑时会积累误差；真实机器人应增加 IMU，并通过状态估计器融合轮速、IMU 和其他定位信息。
