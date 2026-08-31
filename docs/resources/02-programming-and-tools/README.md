# 02 Programming and Tools

收录机器人开发常用的编程语言、Linux 环境、构建系统、版本控制和容器工具。

## Official learning resources

| 资源 | 提供方 | 主题 | 语言 | 难度 | 状态 | 最后检查 |
| --- | --- | --- | --- | --- | --- | --- |
| [The Python Tutorial](https://docs.python.org/3/tutorial/) | Python Software Foundation | Python 语法、数据结构、模块和标准库基础 | English | Beginner | Source Checked | 2026-08-31 |
| [Get Started with C++](https://isocpp.org/get-started) | Standard C++ Foundation | C++ 学习入口、编译器与权威资料导航 | English | Beginner | Source Checked | 2026-08-31 |
| [CMake Tutorial](https://cmake.org/cmake/help/latest/guide/tutorial/index.html) | Kitware | 从单目标工程到安装、测试和依赖管理 | English | Beginner–Intermediate | Source Checked | 2026-08-31 |
| [Ubuntu Server Documentation](https://ubuntu.com/server/docs/) | Canonical | Ubuntu 管理、网络、存储与服务 | English | Beginner–Intermediate | Source Checked | 2026-08-31 |
| [Install WSL](https://learn.microsoft.com/windows/wsl/install) | Microsoft | WSL 安装、发行版管理和基础命令 | English | Beginner | Source Checked | 2026-08-31 |
| [Working across Windows and Linux file systems](https://learn.microsoft.com/windows/wsl/filesystems) | Microsoft | WSL 文件位置、性能和跨系统互操作 | English | Beginner–Intermediate | Source Checked | 2026-08-31 |
| [Pro Git, 2nd Edition](https://git-scm.com/book/en/v2) | Git project / Scott Chacon and Ben Straub | Git 原理、分支、远程协作和内部机制 | 多语言 | Beginner–Advanced | Source Checked | 2026-08-31 |
| [GitHub Skills](https://skills.github.com/) | GitHub | 交互式 GitHub、Markdown、PR 和 Actions 课程 | English | Beginner–Intermediate | Source Checked | 2026-08-31 |
| [Docker Get Started](https://docs.docker.com/get-started/) | Docker | 容器、镜像、Dockerfile 和 Compose | English | Beginner | Source Checked | 2026-08-31 |
| [GNU GDB Documentation](https://sourceware.org/gdb/documentation/) | GNU Project | C/C++ 调试器手册和教程入口 | English | Intermediate | Source Checked | 2026-08-31 |

## Engineering notes

- 在 WSL 中运行 ROS 2 或大型 C++ 构建时，项目通常应放在 WSL 的 Linux 文件系统中，而不是 `/mnt/c`；具体原因和例外请看 Microsoft 的文件系统指南。
- `latest` 文档会随软件更新。遇到参数或命令不一致时，优先切换到与本机软件相同的版本页面。
- C++ 标准、编译器、CMake 和 ROS 2 是不同层次；排查问题时先确认错误来自哪一层。

新增资源时使用 [统一模板](../RESOURCE_TEMPLATE.md)。
