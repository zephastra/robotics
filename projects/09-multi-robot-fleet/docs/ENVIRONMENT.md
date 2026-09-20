# 环境记录（ENVIRONMENT.md）

本文件只写**实测**内容。未测得的一律写 `UNKNOWN`，不用官方默认版本补空。

- 记录时间：2026-09-14（P0 只读探测）
- 记录方式：在 WSL 目标发行版执行只读命令；未安装任何依赖，未启动仿真

---

## 1. OS / 内核

| 项 | 实测值 |
| --- | --- |
| PRETTY_NAME | `Ubuntu 26.04 LTS` |
| VERSION | `26.04 LTS (Resolute Raccoon)` |
| VERSION_ID | `26.04` |
| uname | `Linux GSYY 6.6.87.2-microsoft-standard-WSL2 #1 SMP PREEMPT_DYNAMIC x86_64` |
| CPU 核数 (`nproc`) | `32` |
| 磁盘可用 | `/ 1007G 总、94G 已用、863G 可用（10%）` |

> Ubuntu 26.04 不是方案里提到的 24.04。**未授权更换系统，按实测栈推进。**

## 2. Python

| 项 | 实测值 |
| --- | --- |
| `command -v python3` | `/usr/bin/python3` |
| `python3 --version` | `Python 3.14.4` |
| `ROS_PYTHON_VERSION` | `3` |
| rclpy 所用解释器 | `3.14.4 (main, Aug 20 2026) [GCC 15.2.0]` |
| `import rclpy` | **OK** → `/opt/ros/lyrical/lib/python3.14/site-packages/rclpy/__init__.py` |

结论：ROS 与系统 Python 一致（3.14）。**P1 的独立 `.venv` 必须用 `/usr/bin/python3` 创建**，否则将来接 ROS 会分裂解释器。

## 3. ROS 2

| 项 | 实测值 |
| --- | --- |
| `/opt/ros` 下发行版 | **只有 `lyrical`** |
| `ROS_DISTRO`（source 前） | `<unset>` |
| source 后 `AMENT_PREFIX_PATH` | `/opt/ros/lyrical` |
| `ros2` | `/opt/ros/lyrical/bin/ros2`，**source 前不在 PATH** |
| `colcon` | `/usr/bin/colcon` |

> 本机**没有 Jazzy / Humble**。方案中"Ubuntu 24.04 + Jazzy + Harmonic"候选组合**本机不具备**，不安装。

`ros2 pkg prefix` 实测结果：

| 包 | 来源 |
| --- | --- |
| `nav2_msgs` | `/opt/ros/lyrical` |
| `ros_gz_bridge` | `/opt/ros/lyrical` |
| `ros_gz_sim` | `/opt/ros/lyrical` |
| `rviz2` | `/opt/ros/lyrical` |
| `robot_state_publisher` | `/opt/ros/lyrical` |
| `xacro` | `/opt/ros/lyrical` |
| `nav2_bringup` | **不在 `/opt/ros/lyrical`**（见下） |
| `nav2_amcl` | **不在 `/opt/ros/lyrical`**（见下） |
| `slam_toolbox` | `Package not found`（009 v1 用静态地图 + AMCL，不阻塞） |

### 3.1 源码构建的 overlay（关键）

`nav2_bringup` / `nav2_amcl` 需要额外 overlay：

| 路径 | 内容 | `setup.bash` |
| --- | --- | --- |
| `/opt/nav2` | Nav2 源码构建安装（含 `nav2_bringup/share/nav2_bringup`） | **有** `/opt/nav2/setup.bash` |
| `/opt/moveit2` | MoveIt 2 源码构建安装 | 有 `/opt/moveit2/setup.bash` |
| `/home/ziling/nav2_ws` | Nav2 源码工作区（`src` `build` `log`，无 `install`） | 无 |
| `/home/ziling/ros2_lyrical_ws` | 另一工作区（含 `install/setup.bash`） | 有 |

> **`ros2 pkg prefix nav2_bringup` 在未 source `/opt/nav2` 时报 "Package not found" 是正常的。**
> 009 的 `scripts/env.sh` 必须显式 source 经核实的 overlay，而不是把所有 `setup.bash` 顺手 source 一遍。
> **尚未验证**：source `/opt/nav2/setup.bash` 后 `nav2_bringup` / `nav2_amcl` 是否可用 → P2 前置，当前 `NOT_RUN`。

## 4. Gazebo

| 项 | 实测值 |
| --- | --- |
| `command -v gz`（**未 source ROS 时**） | `/usr/bin/gz` |
| `command -v gz`（**source ROS 后**） | **`/opt/ros/lyrical/opt/gz_tools_vendor/bin/gz`** ← ROS 自带的 vendored gz 抢占 PATH |
| `gz sim --versions` | **`10.4.0`**（两条路径一致） |
| gz 发行集合 | `gz-rotary 1.0.0+nightly+git20260809` |
| gz-sim 包版本 | `gz-sim-cli 10.999.999+nightly+git20260809+1r42f95b961d66dcd0248886c6953d2d41509de8fc-1~resolute` |
| 构建来源 | **nightly 快照，2026-08-09，`~resolute`** |

> ⚠️ **本机是 gz-sim 10.x（Rotary 世代），不是方案里假设的 Harmonic（gz-sim 8）。**
> 方案 §12 给的 Harmonic / Sim-8 DiffDrive 文档链接**不直接适用**，必须查 gz-sim 10 对应文档。
> 这是方案自己允许的情况（"若用其他 Gazebo 大版本，必须查对应文档"），**记录为环境差异，不擅自降级**。

> ⚠️ **source ROS 后 `gz` 会切换到 ROS vendored 版本**（`/opt/ros/lyrical/opt/gz_tools_vendor/bin/gz`）。
> 两条路径版本一致（10.4.0），但**必须记录实际被调用的是哪条**，否则"我跑的是系统 gz"会是错的。
> `scripts/doctor.sh` 会打印实际解析到的路径。

冻结用的可复现标识（写入报告 manifest 用）：

```
gz-sim-cli   10.999.999+nightly+git20260809+1r42f95b961d66dcd0248886c6953d2d41509de8fc-1~resolute
gz-rotary    1.0.0+nightly+git20260809+1r1a1864ad697360b049e025e6dea1d559e70f8e02-1~resolute
gz-transport 15.999.999+nightly+git20260809+1rf2996c6bcc55a3179ca46b06e8e7688b2655059b-1~resolute
gz-msgs      12.999.999+nightly+git20260809+1r37d791f474f4bde577207cffcceb3201e80c7fca-1~resolute
```

## 5. 图形 / GPU（本机已知脆弱，必须如实记录）

| 项 | 实测值 |
| --- | --- |
| GPU | `NVIDIA GeForce RTX 4080 Laptop GPU` |
| 驱动 | `616.92` |
| `nvidia-smi` | 可用 |
| `/usr/share/vulkan/icd.d` | `asahi_icd.json` `dzn_icd.json` `gfxstream_vk_icd.json` `intel_hasvk_icd.json` `intel_icd.json` `lvp_icd.json` `nouveau_icd.json` `radeon_icd.json` `virtio_icd.json` |
| **NVIDIA Vulkan ICD** | **不存在（无 `nvidia_icd.json`）** |
| `vulkaninfo` / `glxinfo` | 均已安装 |

结论：

- **没有 NVIDIA Vulkan ICD。** Gazebo 图形将走 `dzn`（D3D12 转发）或 `lvp`（lavapipe 软件光栅）。这是本机既有状况，不是本次引入。
- ⇒ **P2 起一律以 `--headless` 为默认路径**，GUI（RViz / gz sim GUI）视为**未验证**，不得从无头成功推断 WSLg 正常（方案 §10.3 同此要求）。
- 尚未实测：`gz sim` 能否真正启动并步进 → `NOT_RUN`。

## 6. 隔离相关默认值

| 变量 | 默认 |
| --- | --- |
| `ROS_DOMAIN_ID` | `<unset>` |
| `GZ_PARTITION` | `<unset>` |
| `RMW_IMPLEMENTATION` | `<unset>` |

> 009 必须为车队选一个**经核查未冲突**的专用 domain，写在项目配置里，**不写入全局 `.bashrc`**。
> 每次仿真生成独立 `GZ_PARTITION`，由该次运行的所有子进程继承。

## 7. 未验证清单（NOT_RUN）

| # | 项 | 阻塞谁 |
| --- | --- | --- |
| 1 | source `/opt/nav2/setup.bash` 后 `nav2_bringup` / `nav2_amcl` 可用 | P2 |
| 2 | `gz sim` 实际启动、步进、实时因子 | P2 |
| 3 | `ros_gz_bridge` 对本机 gz-sim 10.4.0 的 msg 映射 | P2 |
| 4 | Nav2 实际速度类型（Twist 还是 TwistStamped） | P2 |
| 5 | 无头 vs GUI 可用范围 | P2 |
| 6 | 本机 gz-sim 10 的 DiffDrive 插件参数与 frame 默认值 | P2 |
| 7 | `slam_toolbox` 是否存在（v1 不需要，仅记录） | 无 |

## 8. 复现本文件的命令

```bash
cat /etc/os-release; uname -a; nproc; df -h /
ls -1 /opt/ros; source /opt/ros/lyrical/setup.bash
python3 --version; python3 -c "import rclpy; print(rclpy.__file__)"
for p in nav2_msgs ros_gz_bridge ros_gz_sim rviz2 xacro; do ros2 pkg prefix "$p"; done
gz sim --versions
ls /usr/share/vulkan/icd.d; nvidia-smi
```
