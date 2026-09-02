# Projects

`projects/` 用于存放独立、可运行、可验证的机器人项目。

建议命名：

```text
projects/
├── 01-<project-name>/
├── 02-<project-name>/
└── 03-<project-name>/
```

每个项目应尽量包含：

- 独立 `README.md` 和明确的验收目标。
- 支持的 Ubuntu、ROS 2、Gazebo 或硬件版本。
- 可复现的安装、构建和启动命令。
- `src/`、`config/`、`launch/`、`urdf/`、`worlds/` 等实际需要的目录。
- 测试、演示结果和已知限制。
- 项目使用的许可证与第三方说明。

## Current projects

- [01-amr-slam](01-amr-slam/README.md) — Gazebo Sim、SLAM Toolbox、RViz 与 Nav2 组成的差速 AMR 演示项目。

课程期数与项目编号是两件事。一个项目可以支撑多集视频；不要为了“第二期课程”复制一份几乎相同的项目。
