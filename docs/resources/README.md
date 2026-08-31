# Zephastra Robotics Resources

机器人学习资源导航，收录官方文档、课程、视频、书籍、工具、社区和其他公开资料。

本目录不是无差别链接集合。每条资源应标明来源、主题、适用版本、语言、难度、评审状态和最后检查日期，帮助读者判断它是否适合当前任务。

当前十个分类均已加入首批公开资源，优先采用项目官方文档、维护者资料、大学公开课程和制造商手册；第三方视频与频道单独标识，不因播放量自动获得推荐状态。首批资源最后集中检查日期为 **2026-08-31**。

## Categories

| 分类 | 内容 |
| --- | --- |
| [01 Fundamentals](01-fundamentals/README.md) | 数学、控制、运动学和机器人基础 |
| [02 Programming and Tools](02-programming-and-tools/README.md) | C++、Python、Linux、Git 和开发工具 |
| [03 ROS](03-ros/README.md) | ROS、ROS 2、ros2_control、Nav2 等 |
| [04 Simulation](04-simulation/README.md) | Gazebo、Isaac Sim、MuJoCo、Webots 等 |
| [05 Mobile Robots](05-mobile-robots/README.md) | AMR、AGV、SLAM、导航和运动规划 |
| [06 Manipulation](06-manipulation/README.md) | 机械臂、MoveIt、抓取和操作规划 |
| [07 Perception](07-perception/README.md) | 视觉、激光雷达、点云和传感器融合 |
| [08 AI and Robotics](08-ai-and-robotics/README.md) | 具身智能、强化学习、VLA 和机器人基础模型 |
| [09 Hardware](09-hardware/README.md) | 电机、驱动器、传感器、嵌入式和机械设计 |
| [10 Communities and Channels](10-communities-and-channels/README.md) | YouTube、Bilibili、博客、书籍和社区 |

## Review status

| 状态 | 含义 |
| --- | --- |
| Pending | 已发现，尚未评审 |
| Source Checked | 已确认来源、作者和链接 |
| Reviewed | 已检查内容范围、版本和基本质量 |
| Recommended | 已实际使用或完整看过，并认为值得推荐 |
| Archived | 已失效、过时或仅为历史参考 |

## Adding a resource

1. 复制 [RESOURCE_TEMPLATE.md](RESOURCE_TEMPLATE.md) 中的格式。
2. 优先收录官方或第一方资料。
3. 标明具体版本；没有版本信息时写 `Not specified`。
4. 简介和评价使用自己的表述，不复制视频字幕或文章正文。
5. 同一资源只保留一个主条目，其他分类用相对链接引用。
6. 每次检查链接时更新 `Last checked`。

## Current highlights

- [ROS 2 Lyrical 官方教程索引](03-ros/ros2-official-tutorials.md)：按主题整理 94 个 ROS 2 官方教程链接。
- [Robotics Fundamentals](01-fundamentals/README.md)：MIT、Stanford、Northwestern 等公开课程与教材入口。
- [Simulation](04-simulation/README.md)：Gazebo、Isaac Sim、MuJoCo、Webots、CoppeliaSim 和 Drake 官方资料。
- [Communities and Channels](10-communities-and-channels/README.md)：官方频道、社区，以及已明确版本风险的 YouTube/Bilibili 教程。

## Maintenance cadence

- 软件发行版相关链接：至少每 3 个月检查一次。
- 课程、频道和视频：至少每 6 个月检查一次。
- 发现 404、明显过时或项目停止维护时，先标为 `Archived`，不要静默删除；这样可以保留历史项目的查找线索。
- `Recommended` 必须基于实际使用或完整审看，不可仅凭搜索摘要或播放量授予。

## Disclaimer

外部链接的内容、可用性和许可由原作者或平台负责。收录不代表 Zephastra 与相关作者、机构或平台存在隶属关系，也不自动表示推荐。
