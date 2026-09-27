# 010 环境核查 — 2026-09-19/20

## 已实查

| 项目 | 结果 |
| --- | --- |
| 系统 | Ubuntu 26.04 LTS，WSL2 kernel 6.6.87.2 |
| CPU/内存 | 32 逻辑 CPU，31 GiB 总内存；核查时约 29 GiB 可用 |
| 磁盘 | 项目分区约 846 GiB 可用（瞬时值） |
| GPU | NVIDIA RTX 4080 Laptop，12282 MiB；核查时使用约 1256 MiB |
| ROS | /opt/ros/lyrical，系统 Python 3.14.4 |
| Nav2 | /opt/nav2，nav2_bringup 1.5.0，模块导入成功 |
| SLAM | /opt/nav2，slam_toolbox 2.10.0；尚未运行建图 |
| PCL | libpcl-dev 1.15.1+dfsg-2；尚未编译或运行点云节点 |
| Gazebo | 默认 gz sim --versions 为 11.0.0~pre1；009 进程使用 ROS vendor sim10，禁止混同 |
| 010 人形环境 | 独立 .venv，Python 3.12.13，MuJoCo 3.3.6，MNN 3.6.1，numpy 2.2.6 |
| 渲染 | MUJOCO_GL=egl；头部相机 160×120 RGB 实际输出，非空图像 |

依赖由本地 uv 缓存离线安装，使用 --link-mode copy；未下载模型、未修改系统安装、未共用旧 .venv。
系统 Python 3.14 的 rclpy 扩展不能直接载入 3.12 进程；拟采用本机跨进程适配，但还未实现。
ROS setup.bash 在 set -u 下出现 AMENT_TRACE_SETUP_FILES 未定义；不修改系统脚本，source 时不用 nounset。

## MNN 兼容问题与修复

首试 reports/p1-h-stand-01/report.json：MNN 扩展请求 executable stack，系统拒绝加载。
用独立环境 patchelf 0.19.1.0 清除请求，未启用可执行栈；只处理 010 内副本。
原二进制与前后哈希保留于 runtime/mnn_original；修复脚本 experiments/fix_mnn_stack.py。
修复后站立、取物力学诊断及相机渲染完成。重新创建环境时需显式执行相同兼容处理。

## 资产来源

86 个独立复制文件核对一致，见 P1_SOURCE_MANIFEST.json。来源为 007 本地模型、策略、两份控制组件和配置。
上游 EngineAI revision 335c60e88772c26c7852d0abd6b3c7439037dd8f；Allegro revision 8161bba264d7fa7c99ca301e91e7fb44737676ad。
许可证随副本保留：EngineAI BSD-3-Clause、Allegro BSD-2-Clause、集成代码 Apache-2.0。
运行不再读取 007；audit_sources.py 是一次性来源审计工具，显式只读旧路径，不属于启动依赖。
本轮未发布资产。固定机械臂/夹爪模型尚未选择；AMR MuJoCo 模型尚未建立，不能称资产全部就绪。

## 隔离与限制

核查时 009 有活动批测，不停止、不修改。当前 010 实验单 worker、CPU 限线程、限时同步运行。
ROS 检查只导入模块，没有创建节点或加入 DDS 域；ROS 域/通信端点隔离必须在实际桥接前另行核查。
无系统服务修改。早前 systemd 用户会话警告仍未诊断；已完成实验未依赖其修复。
