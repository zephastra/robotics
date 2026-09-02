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

- [01 AMR SLAM Demo](projects/01-amr-slam/README.md)
- [ROS 2 Lyrical 非官方中文教程](docs/tutorials/ros2-lyrical/README.md)
- [机器人学习资源导航](docs/resources/README.md)
- [文档目录说明](docs/README.md)
- [项目目录说明](projects/README.md)

## Naming rules

- 可运行项目放在 `projects/NN-project-name/`。
- 正式教程放在 `docs/tutorials/`，外部资源导航放在 `docs/resources/`，项目复盘和设计记录放在 `docs/project-notes/`。
- 使用小写英文和连字符作为路径名；中文可以用于 Markdown 标题和文档文件名。
- 草稿必须明确标注 Draft，不能与已验证教程混淆。

## License

除子目录另有说明外，本仓库代码采用根目录的 [Apache License 2.0](LICENSE)。原创文档内容采用 [CC BY 4.0](docs/LICENSE.md)。第三方内容继续受其各自许可证约束。
