# ROS 2 Lyrical 中文学习笔记

这是一组面向 Ubuntu 26.04、ROS 2 Lyrical Luth 的中文学习笔记。内容以官方文档为主要来源，并加入必要的中文解释、版本差异说明和项目实践备注。

> 本项目不是 ROS 2 官方中文站，也不是对官方文档的完整逐字翻译。遇到差异时，以每篇列出的官方原文和当前软件版本为准。

中文初稿使用 AI 辅助整理；公开版本经过来源、版本、命令、接口名、许可和可复现性审计。AI 辅助不等于官方审核，也不能替代在指定版本中的实际验证。

## 适用环境

| 项目 | 当前基线 |
| --- | --- |
| 操作系统 | Ubuntu 26.04 LTS（Resolute Raccoon） |
| ROS 2 | Lyrical Luth（支持至 2031-05） |
| ROS 安装路径 | `/opt/ros/lyrical` |
| Gazebo 配套发行版 | Jetty |
| ROS–Gazebo 集成 | `ros_gz` 3.x |

当前审计机器还安装了 Gazebo nightly 包；`gz sim --versions` 显示 `11.0.0~pre1`，而 ROS vendor 包声明的基线为 `gz-sim 10.4.0`。因此 Gazebo、SLAM、Nav2 和控制器相关文章在完成指定版本的端到端复现前，均保持 Draft 状态。

## 状态含义

| 状态 | 含义 |
| --- | --- |
| Environment Verified | 已在当前 WSL 中确认操作系统、ROS 发行版或软件包存在 |
| Source Reviewed | 已与指定官方文档核对，但本轮未完整执行全部步骤 |
| Draft | 含占位符、依赖缺失或尚未完成端到端复现，不构成“可以直接运行”的承诺 |
| Project Note | 个人项目资料，不属于官方教程翻译 |

## 内容索引

### ROS 2 官方教程学习笔记

- [01 环境准备](01-environment/01-环境准备.md) — Environment Verified
- [02 节点与话题](02-node-topic/02-节点与话题.md) — Source Reviewed
- [03 服务与动作](03-service-action/03-服务与动作.md) — Source Reviewed
- [04 工作空间与包](04-workspace/04-工作空间与包.md) — Source Reviewed
- [05 发布订阅实战（Python）](05-pub-sub/05-发布订阅实战.md) — Source Reviewed
- [06 服务客户端实战（Python）](06-service-client/06-服务客户端实战.md) — Source Reviewed

### AMR 项目相关草稿

- [07 Gazebo 与 ROS 2 桥接](07-gazebo-bridge/07-Gazebo仿真与桥接.md) — Draft
- [08 SLAM 建图](08-slam/08-SLAM建图.md) — Draft
- [09 Nav2 导航](09-nav2/09-Nav2导航.md) — Draft
- [12 差速底盘控制](12-diff-drive/12-底盘控制专题.md) — Draft

### 尚未归入正式项目的孵化资料

- [10 MoveIt 2](10-moveit2/10-机械臂MoveIt2.md) — Draft
- [11 视觉与 PLC 工业闭环](11-vision-plc/11-视觉与PLC工业闭环.md) — Project Note

## 使用原则

1. 先阅读每篇顶部的状态、适用版本、原文链接和验证范围。
2. Draft 文档只能作为学习或排查线索，不能直接当作已验证操作手册。
3. 命令、包名、参数和接口名保留英文原样；中文只解释含义。
4. 运行前先确认当前环境：`printenv ROS_DISTRO`、`gz sim --versions` 和相关 `ros2 pkg prefix`。
5. 发现与官方文档不一致时，记录版本和复现步骤后再修改状态。

## 来源与许可

每篇文档单独列出主要来源和上游许可。ROS 2 文档与 Gazebo 高层文档采用 CC BY 4.0；Nav2、slam_toolbox、MoveIt 2 及其他项目可能使用不同许可证，不能用一个许可证声明覆盖全部上游内容。

本目录中的原创中文说明默认采用 [CC BY 4.0](https://creativecommons.org/licenses/by/4.0/deed.zh-hans)；第三方代码、命令和改编内容仍受各自上游许可证约束。完整说明见 [THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md)。

本轮审计发现和修改摘要见 [AUDIT_REPORT.md](AUDIT_REPORT.md)。

本项目与 Open Robotics、OSRF、Gazebo、Nav2、MoveIt 或其维护者不存在官方隶属或背书关系。
