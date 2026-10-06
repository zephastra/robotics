# Zephastra Robotics

Zephastra 的机器人项目、学习笔记和可复用资料仓库。

This repository contains Zephastra robotics projects, learning notes, and reusable resources.

## Repository layout

```text
robotics/
├── projects/   # 独立、可运行的机器人项目，按 01-、02-… 编号
└── docs/       # 跨项目文档、学习笔记和参考资料
```

## Current content

- [01 ROS 2 AMR 建图与导航入门](projects/01-amr-slam/README.md)
- [02 AMR 仓库任务执行、异常恢复与自动返航](projects/02-amr-mission-executor/README.md)
- [03 移动机械臂视觉搬运仿真](projects/03-mobile-manipulator/README.md) — 独立的导航、视觉抓放、运输与返航对位演示；仅在记录的本机环境验证。
- [04 人形机器人运动控制与行走验证](projects/04-humanoid-locomotion/README.md) — 独立 MuJoCo 仿真，支持 Unitree G1 / EngineAI T800 预训练步态、手动控制、定点行走与受控停止；本机验证，跨机器复现待验收。
- [05 灵巧手动作与接触抓取实验台](projects/05-dexterous-hand/README.md) — 独立 Allegro 四指抓放与接触验证；两轴手腕夹具，不是完整人形机器人。
- [06 人形机器人抓取、负载行走与放置](projects/06-humanoid-transport/README.md) — 独立 T800 与右侧灵巧手搬运仿真；已知位置、轻载球体和固定托盘，本机有限场景验证。
- [07 主动视觉与人形机器人双手搬运](projects/07-humanoid-visual-transport/README.md) — **实验性快照**；头部 RGB-D、双手接触搬运、自动站位与诊断；当前完整物理回归尚未通过。
- [08 语言指令与受约束的人形机器人技能执行](projects/08-humanoid-language-tasks/README.md) — **实验性快照**；规则/回放规划、技能监督与 MuJoCo 后端；B/C 搬运仍有失败，真实 LLM 未实测。
- [ROS 2 Lyrical 非官方中文教程](docs/tutorials/ros2-lyrical/README.md)
- [机器人学习资源导航](docs/resources/README.md)
- [文档目录说明](docs/README.md)
- [项目目录说明](projects/README.md)

## Development checkpoints — dev branch only

- [010 异构机器人协同备料与配送](projects/10-heterogeneous-robot-workcell/README.md) — **未完成的开发备份，2026-10-07 更新**。保留源码、资产、测试、文档及精选成功/失败证据；带载保持和接收卸盘仍未通过，不是 V1 发布，不承诺克隆即完整运行。

这些检查点用于保存开发工作，不代表已验收产品，也没有合并到 main。

## Naming rules

- 可运行项目放在 `projects/NN-project-name/`。
- 正式教程放在 `docs/tutorials/`，外部资源导航放在 `docs/resources/`，项目复盘和设计记录放在 `docs/project-notes/`。
- 使用小写英文和连字符作为路径名；中文可以用于 Markdown 标题和文档文件名。
- 草稿必须明确标注 Draft，不能与已验证教程混淆。

## License

除子目录另有说明外，本仓库代码采用根目录的 [Apache License 2.0](LICENSE)。原创文档内容采用 [CC BY 4.0](docs/LICENSE.md)。第三方内容继续受其各自许可证约束。
