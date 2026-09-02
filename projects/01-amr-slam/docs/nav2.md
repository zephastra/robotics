# Nav2 自主导航

前置条件：已经完成建图，并通过 `scripts/save_map.sh` 生成 `maps/map.pgm` 和 `maps/map.yaml`。

## 一键启动

在项目根目录运行：

```bash
bash scripts/run_nav2.sh
```

无图形桌面时：

```bash
AMR_HEADLESS=1 bash scripts/run_nav2.sh
```

也可以传入其他地图：

```bash
bash scripts/run_nav2.sh /absolute/path/to/map.yaml
```

脚本会检查地图和主要 Nav2 软件包，然后启动 Gazebo、bridge、TF、AMCL、Nav2 与 RViz。

## 操作

1. 在 RViz 中选择 **2D Pose Estimate**，在机器人真实位置按住并拖出朝向。
2. 等待 AMCL 粒子云收敛。
3. 选择 **Nav2 Goal**，设置一个远离墙壁和障碍物的可达目标。
4. 观察全局路径、局部轨迹和机器人运动。

## 关键关系

- AMCL 负责在保存地图中的定位，并发布 `map→odom`。
- 全局代价地图和规划器生成全局路径。
- 局部代价地图和控制器负责避障与路径跟踪。
- 行为树负责导航编排和恢复行为。

建图时由 `slam_toolbox` 发布 `map→odom`，导航时由 AMCL 发布。两套流程不能同时运行。

## 常见问题

| 现象 | 处理 |
| --- | --- |
| 地图不存在 | 先运行建图与 `bash scripts/save_map.sh` |
| AMCL 粒子不收敛 | 重新设置更准确的初始位置和方向 |
| No valid plan | 将目标点移离墙壁或障碍物 |
| 机器人绕圈 | 检查初始朝向、里程计和激光 TF |
| 机器人撞障碍物 | 检查 `/scan` 是否进入局部代价地图 |
| 没有响应 | 查看 `/tmp/amr_nav_logs/` 中的日志 |

## 验收标准

在地图中选择两个有效位置，连续完成三次 A→B 导航，其中至少一次路径需要绕开障碍物。
