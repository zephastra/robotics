# 009 多机器人任务协同 / 交通预约 / 故障接管仿真

**开发中检查点 / Development snapshot — 2026-09-20。**
本目录位于公开 `dev` 分支，仅用于备份与追踪开发，不是 v1.0 或已完成验收的发布版，未合并 `main`。
本次未运行构建、测试或仿真，不承诺当前快照开箱即用，也不声明测试全绿。

先读 [`docs/DEV_CHECKPOINT.md`](docs/DEV_CHECKPOINT.md)：上传范围、验证边界、已知文档问题与接手事项。
本工程已有多机器人 ROS/Gazebo 源码、调度与故障处理实现，但完整物理回归仍不能由本次备份宣布完成。

下方保留原开发说明；其中“本机跑过”等表述属于历史记录，不是对本快照重新验证的结论。
[`docs/IMPLEMENTATION_STATUS.md`](docs/IMPLEMENTATION_STATUS.md) 含不同日期的追加记录且顶部阶段已过时，
请结合 [`docs/LIMITATIONS.md`](docs/LIMITATIONS.md) 和决策记录阅读，不引用单个旧 PASS 作为当前版本保证。
完整验收需同时覆盖 TEST_AND_ACCEPTANCE 的 §5 物理矩阵与 §10 复现门，而非只通过发布门脚本。

## 这是什么

v1 目标：用户提交一批站点服务任务，**2～3 台四轮差速机器人在同一个 Gazebo 世界里真实物理运动**，
分工执行、经共享窄道时预约让行、遇到低电或故障时遵守货物与资源归属规则，并输出可独立核验的结果。

v1 的"取货/交付"是**站点停车 + 停留时间 + 逻辑货物账本变更**。
车辆运动、障碍与相互碰撞走物理仿真；**没有机械臂，没有实体装卸，不得称为完成实体搬运。**

**v1 的真实范围，四句话**：轮式差速**物理移动**；**逻辑货物**（不是实体装卸）；
**模拟充电**（能量模型是 demo 规则，不是电池保护模型，见 `docs/LIMITATIONS.md`）；
**集中式预约**（一个中央调度器拥有唯一账本与资源簿，没有分布式共识）。

**以下未实现，任何报告不得声称**：人形物理验证、实体货物交接、LLM / VLA 参与决策。

真正的能力是：谁接哪个任务、谁先过窄道、谁等在哪里、掉线后谁接手，
以及系统如何**证明**没有撞车、没有重复执行、没有伪造完成。

## 阅读顺序

1. [`docs/MASTER_PLAN.md`](docs/MASTER_PLAN.md) — 目标、架构、目录、环境、实现顺序
2. [`docs/CONTRACTS.md`](docs/CONTRACTS.md) — 数据结构、状态机、接口、交通与故障语义
3. [`docs/TEST_AND_ACCEPTANCE.md`](docs/TEST_AND_ACCEPTANCE.md) — 测试矩阵、阈值、结果与回归规则
4. [`AGENTS.md`](AGENTS.md) — AI 开发规则（不可违反的边界）
5. [`docs/AI_EXECUTION_PROMPTS.md`](docs/AI_EXECUTION_PROMPTS.md) — 逐阶段执行指令
6. [`docs/ENVIRONMENT.md`](docs/ENVIRONMENT.md) — 本机实测环境（不是猜测）

若文档冲突：**先停止相关实现并记录冲突，不自行选择最容易通过的解释。**
已知冲突记录在 `docs/IMPLEMENTATION_STATUS.md` 的 C1～C4。

## 环境（实测，详见 docs/ENVIRONMENT.md）

| 项 | 实测 |
| --- | --- |
| OS | Ubuntu 26.04 LTS (Resolute Raccoon)，WSL2 |
| ROS 2 | Lyrical，`/opt/ros/lyrical`（本机无 Jazzy/Humble） |
| Python | 3.14.4（与 rclpy 一致） |
| Gazebo | gz-sim **10.4.0**（Rotary 世代 nightly） |
| Nav2 | 源码构建 overlay `/opt/nav2`（可用性待 P2 验证） |
| CPU / 磁盘 | 32 核 / 863 GiB 可用 |
| GPU | RTX 4080 Laptop，驱动 616.92；**无 NVIDIA Vulkan ICD** ⇒ **默认走 `--headless`** |

## 命令

> 下列命令**已实现，并在本机跑过**。仍然 NOT_RUN 的能力清单见
> `docs/IMPLEMENTATION_STATUS.md` 与 §10 发布门报告——那不是这些命令本身不可用。

```bash
cd ~/projects/009_multi_robot_fleet

bash scripts/doctor.sh            # 只读依赖检查，0=齐 3=缺依赖
source scripts/env.sh             # 只 source 本项目认可的 ROS/overlay
bash scripts/build.sh             # 6 个静态守卫 + colcon 构建 + 渲染每车模型
bash scripts/test_core.sh         # 核心测试（不 import ROS、不起仿真）
bash scripts/run_p7_matrix.sh     # §10 发布门逐项结论，含搬迁到新路径后重跑

# 真实车队。本机无 NVIDIA Vulkan ICD，GUI 路径未验证，一律无头。
# 车队由这些脚本启动（每条自带 bring-up 与 teardown）：
bash scripts/run_p7_matrix.sh                    # §10 发布门 + 搬迁 + 从副本起真舰队
bash scripts/run_scenarios3456.sh capability     # 一条定向场景
bash scripts/batch.sh --dry-run            # §5 的 30 例冻结用例表（见下表）
# `run_demo.sh --backend gazebo_nav2` 目前仍是 stub（exit 3），见 docs/LIMITATIONS.md
bash run_demo.sh --backend fake --robots 2 --scenario opposing_requests
```

## 分层

```
CLI / YAML 任务
       |
Fleet Core：任务账本、能力匹配、分配、资源预约、事件
       |
每机器人 Adapter：命令代次、Nav2 action、状态、取消确认
       |
独立 Nav2 -> 速度链末端 -> Local Safety Gate -> Gazebo 驱动
       ^                         ^
 AMCL / odom / scan        许可、状态新鲜度、停止距离、急停

Gazebo 真值 -> 独立 Evaluator -> 报告（不回流调度或控制）
```

**五条责任边界（不可违反）**

1. 调度器决定"哪台车执行、预约哪个资源"，**不发布轮速或 cmd_vel**。
2. Nav2 决定局部路径与避障，但**不能自行穿过未授权共享区**。
3. Adapter 管理动作生命周期；**一次只允许一个有效执行代次**。
4. Safety Gate **独占最终速度输出**；任何模块都不能旁路它。
5. Evaluator 依真值裁判，**不参与规划**；控制层不得订阅真值作位置反馈。

## 明确不做（v1）

多人形共同抬运、分布式共识、任意拓扑 MAPF、动态 SLAM 地图合并、
从零训练行走策略 / VLA / 世界模型、硬件控制、安全认证、全天候无人值守承诺。

人形与异构只保留 adapter 接口；v1 **没有人形物理验证，也没有实体装卸**。

## 独立性与来源

- 本工程**不依赖** 001～008 的路径、启动脚本、虚拟环境、PYTHONPATH 或链接。
- 复制过的代码必须登记来源、版本/哈希、许可证与修改说明，见 [`THIRD_PARTY_NOTICES.md`](THIRD_PARTY_NOTICES.md)。
- 本工程**不是** 001～008 的延续验证；001～008 的稳定性未被本方案声明。

## 许可证

代码用 **Apache-2.0**，见 [`LICENSE`](LICENSE)——该文件是标准 notice 形式，完整条文以
其中的 URL 为准（不在这里近似转写：不完整的许可证文本比指向权威文本更糟）。
`docs/` 文档按 **CC BY 4.0**，署名 zephastra。第三方来源见
[`THIRD_PARTY_NOTICES.md`](THIRD_PARTY_NOTICES.md)。

## 边界

- 未经用户明确授权：**不提交、不推送 GitHub**。
- 不放宽验收阈值、不删失败 case、不用成功重跑覆盖失败、不伪造执行记录。
