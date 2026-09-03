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

脚本会检查地图和主要 Nav2 软件包，然后依次启动 Gazebo、bridge、TF、AMCL 与 RViz。确认 AMCL 已发布 `map→odom` 后，脚本才启动 Nav2，避免规划器在定位尚未就绪时激活失败。

## 操作

1. 等待终端显示 `[ok] Localization is ready.` 和 `[6/6] Starting navigation ...`。
2. 正常情况下，仿真机器人会使用出生点 `(0, 0, 0)` 自动完成 AMCL 初始定位。
3. 如果 RViz 中的位置或朝向不准确，再使用一次 **2D Pose Estimate**。
4. 选择 **Nav2 Goal**，在远离墙壁和障碍物的可达区域按住鼠标并拖出目标朝向。
5. 观察全局路径、局部轨迹和机器人运动。

RViz 配置包含 **Navigation 2** 面板。该面板负责把 **Nav2 Goal** 工具设置的位姿发送给 `/navigate_to_pose` Action；不能只保留工具而删除面板。

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
| `Frame [map] does not exist` | 确认 Map 使用 Transient Local QoS，并检查 AMCL 是否已发布 `map→odom` |
| AMCL 粒子不收敛 | 重新设置更准确的初始位置和方向 |
| 设置 Nav2 Goal 后没有响应 | 确认 RViz 已加载 Navigation 2 面板，并等待导航节点全部激活 |
| No valid plan | 将目标点移离墙壁或障碍物 |
| 机器人绕圈 | 检查初始朝向、里程计和激光 TF |
| 机器人撞障碍物 | 检查 `/scan` 是否进入局部代价地图 |
| 没有响应 | 查看 `/tmp/amr_nav_logs/` 中的日志 |

## 验收标准

在地图中选择两个有效位置，连续完成三次 A→B 导航，其中至少一次路径需要绕开障碍物。
