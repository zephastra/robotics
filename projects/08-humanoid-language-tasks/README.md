# 008 — 语言指令与受约束的人形机器人技能执行

> **实验性开发快照；不是完整 LLM 系统或稳定多目标搬运。**
> 2026-09-18 发布范围见 [发布核对](docs/RELEASE_AUDIT.md)。
> [实施历史](docs/IMPLEMENTATION_STATUS.md) 包含过时结论；[总体计划](docs/MASTER_PLAN.md) 是目标，不是完成证明。

将有限语言指令解析为任务，由规则规划或响应回放选择技能，通过校验、前置条件与预算检查后执行。
提供 fake 离线后端，以及独立 MuJoCo / T800 / 双 Allegro 手物理后端。
真实 LLM HTTP 接口已有代码，但未核实有真实模型实测；不宣称接入 UnifoLM-WLA 或端到端 VLA。

## 完成到哪里

| 项目 | 当前边界 |
| --- | --- |
| 规则解析、契约、假技能、监督、预算、取消 | 已实现、有短测试；fake 不等于机器人搬运 |
| 8 项 MuJoCo 技能、规则/回放规划 | 有实现及历史完整搬运记录；物理执行仍不稳定 |
| B 台搬运 | 历史 30 例有 26/30，也有后续 21/30；不能用最好一次代表当前可靠性 |
| C 台识别、绑定、搬运与放置 | 已有成功；最近四例 1 成功、3 放置失败，未稳定完成 |
| 授权替代目标、有限重规划 | 有 fake 逻辑测试；不代表物理恢复验收通过 |
| 真实模型、动态占用感知、多物体泛化 | 未验收 / 未完成 |
| 干净环境安装、完整发布回归 | 未完成 |

## 安装与离线运行

Linux / WSL2、Python 3.12（建议 uv）。不需要 001～007，不共享它们的环境。

```bash
git clone https://github.com/zephastra/robotics.git
cd robotics/projects/08-humanoid-language-tasks
bash scripts/setup.sh --profile core
bash run_tests.sh --suite core
bash run_demo.sh --backend fake --planner rule --instruction "把箱子搬到 B 台"
```

fake 无物理仿真，报告 `NOT_EVALUATED` 不能解释为物理通过。

## 仿真入口（实验）

```bash
bash scripts/setup.sh --profile sim
.venv/bin/python scripts/prepare_assets.py
.venv/bin/python scripts/doctor.py --profile sim

# 只观察；需要 WSLg/图形环境
.venv/bin/python -m humanoid008.simulation.backend --split --planner rule \
  --instruction "先看看箱子。"

# 完整 B 台搬运实验，会明确报告成功或失败，不保证成功
MUJOCO_GL=egl .venv/bin/python -m humanoid008.simulation.backend \
  --split --planner rule --headless --duration 120 --instruction "把箱子搬到 B 台。"
```

**两个入口尚未统一**：`run_demo.sh --backend mujoco` 仍返回 `NOT_IMPLEMENTED`；
物理后端请使用上面的模块命令。本次没有改变原任务控制行为。

`prepare_assets.py` 从固定版本官方上游恢复资源，逐文件校验已提交清单；已有文件不覆盖，
不生成新哈希掩盖不一致。组合模型和实验配置随项目保存，不需要先安装 007。
`import_baseline.py` 是历史一次性导入工具，不是用户安装步骤。

```bash
.venv/bin/python scripts/check_independence.py
MUJOCO_GL=egl .venv/bin/python -m pytest tests/unit tests/simulation -m 'not slow'
# 长物理测试需显式选择，本次未重跑
MUJOCO_GL=egl .venv/bin/python -m pytest tests/simulation -m slow
```

## 证据与边界

使用理想 RGB-D、颜色标记、已知尺寸/工装、理想接触和预训练步态。不含真机验证。
仿真冻结不等于工业急停；物品终态 PASS 不等于全程无碰撞。
没有模型服务时 llm 应明确失败，不回退 rule 冒充模型。
`.env.example` 是环境变量样例，不保证自动加载；不得上传真实密钥。

- [发布核对与待办](docs/RELEASE_AUDIT.md)
- [已知限制](docs/LIMITATIONS.md)
- [历史汇总与精选逐次报告](docs/evidence/)
- [第三方来源与许可证](THIRD_PARTY_NOTICES.md)

原创代码 [Apache-2.0](LICENSE)；第三方模型、策略、手部保留各自 BSD 许可。
