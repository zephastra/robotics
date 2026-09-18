# IMPLEMENTATION STATUS — 008

> **历史开发日志，非当前完成清单。** 2026-09-18 核对见 [RELEASE_AUDIT.md](RELEASE_AUDIT.md)。
> 下文“未创建 AGENTS”“C 从未放置成功”等已被后续文件/报告更新；老结论保留用于追溯。

**最后更新**：2026-09-11 · 交接阶段：P0 / P1 / P2 / P3 / P4 / P5
**维护规则**：每个阶段结束时更新。只写**已验证**的事实；未验证的必须显式标记。

---

## 1. 环境事实（本次会话实测）

| 项目 | 结果 |
| --- | --- |
| 目标目录 | `~/projects/008_humanoid_language_tasks`（WSL 发行版 `Ubuntu`，owner `ziling:ziling`） |
| 目录树 | 已创建，与 `docs/MASTER_PLAN.md` §4.3 / §5 一致 |
| 工程文件写入 | ✅ 可用（UTF-8 无 BOM、LF 行尾） |
| **WSL 内命令执行** | ✅ **可用**（用户已把 `wsl.exe` 从 WorkBuddy 黑名单移除，见 §1.1） |
| 可执行位 `+x` | ✅ 已对 `run_demo.sh` / `run_tests.sh` / `scripts/*.sh` 执行 `chmod +x` |
| 虚拟环境 | ✅ `.venv` = **CPython 3.12.13**（uv 提供） |
| `AGENTS.md` | ✅ **已创建**（用户 2026-09-12 提供 `AGENTS_TEMPLATE.md` 正文，逐字落盘到工程根） |

### 1.1 执行通道的变更记录

本会话开始时 `wsl.exe` 被 WorkBuddy 的 `Security Center → Command Security → Program Blacklist` 拦截
（`PROGRAM BLOCKED BY SECURITY POLICY`，进程级、当时不可绕过），因此 P0/P1 的文件是先写、后验证。

**用户移除该黑名单后，本会话已能直接在 WSL 内执行命令。** 下面所有 §3.1 的结果都是在
WSL 的 `Ubuntu` 发行版中真实运行得到的。

### 1.2 两个必须记住的环境事实

1. **系统 `python3` 是 3.14.4**，不是 3.12。本工程**不使用**环境中的 `python3`，
   一律通过 `uv` 固定到 3.12（见 §7 的兼容基线）。uv 已在本机缓存 `cpython-3.12.13`，安装不需要联网。
2. **从 Windows 侧通过 `wsl.exe -- bash -lc '...'` 传命令时，外层会吃掉 `$?` 与 glob。**
   要拿到准确的退出码，请写成脚本文件再执行（`wsl.exe -d Ubuntu -- bash /abs/path/script.sh`），
   不要把 `$?` 塞进嵌套的 `-lc '...'` 字符串里。

---

## 2. 阶段状态

| 阶段 | 状态 | 说明 |
| --- | --- | --- |
| P0 环境与边界 | **基本完成** | 目录树 + `MASTER_PLAN.md` + `README.md` + 本文件已写；`AGENTS.md` 因缺模板未建 |
| P1 可安装核心骨架 | **已在 WSL 完成并通过验收** | 见 §3.1 |
| P2 契约 / 假技能 / 规则闭环 | **已完成并通过 WSL 验收** | 见 §3.2 |
| P3 监督与模型接口 | **已完成并通过 WSL 验收** | 见 §3.3 |
| P4 独立物理基线导入 | **基本完成**（导入 + 结构冒烟 + 30 例回归 24/30） | 见 §3.4 / §3.9 |
| P5 技能拆分等价性 | **基本完成**（8 技能拆分 + 30 例等价回归 26/30 ≈ baseline 80%） | 见 §3.5 / §3.9 |
| P6 语言驱动真实仿真 | **第一步完成**（planner 选择 + planning_pause + replay 对照 + 观察/歧义/取消闭环；真实 LLM 待端点） | 见 §3.10 |
| P7 有限多目标与反馈恢复 | **纯逻辑部分完成**（授权替代目标 + 目标更新 + replans 预算）；物理 C 台进行中（视觉区分/绑定/走到 C 台已通；**PLACE 未通过**——根因=行走 yaw 漂移 −6°~−8° 不可纠正 + 窄窗口 0.08m + 前向过冲 0.1m，非"几何矛盾"）；实例识别/占用未做 | 见 §3.11 / §3.12 |
| P8 复现、文档与研究门 | 未开始 | |

---

## 3. P1 已写入的文件

| 文件 | 作用 |
| --- | --- |
| `pyproject.toml` | 包定义、`h008` 控制台入口、core/dev/sim 依赖分组 |
| `requirements-core.lock` / `requirements-sim.lock` | 锁定依赖（core 可离线） |
| `scripts/setup.sh` | 建立本工程独立 `.venv`，**固定 Python 3.12**（uv 优先，否则 `python3.12`） |
| `scripts/doctor.py` | 只读环境诊断，不安装、不修复、不打印密钥 |
| `run_demo.sh` / `run_tests.sh` | 统一入口，必须存在 `.venv`，缺依赖明确报错 |
| `src/humanoid008/__init__.py` | 版本与 `SCHEMA_VERSION` |
| `src/humanoid008/app.py` | CLI：参数解析、配置校验、唯一报告目录、manifest/input |
| `src/humanoid008/reporting/{__init__,events,report}.py` | 报告层：`EventLog`、运行目录、manifest、三轴 report |
| `config/app.yaml` | 后端/规划器/时钟默认值与预算默认值 |
| `conftest.py` | 让 `src/` 布局在未安装时也可导入 |
| `tests/unit/test_cli_args.py`、`tests/unit/test_reporting.py` | P1 单元测试（14 项） |
| `.gitignore` / `.env.example` | 忽略规则与配置样例 |

**P1 契约要点（已实现，并已在 WSL 验证）**

- 未实现的功能**显式报错**：`--backend mujoco`、`--planner replay|llm` 均以 **exit 3** 结束并说明阻塞阶段；
  绝不静默回退到 fake/rule。
- 报告目录唯一：`reports/<UTC>-<run_id>/`，已存在则 `FileExistsError`，不覆盖历史。
- `--report-root` 被强制约束为工程内 `reports/`。
- `report.json` 保持三轴分离：`task_status` / `physical_outcome` / `test_verdict`（对应 §25.4）。

### 3.1 WSL 验收结果（Ubuntu · CPython 3.12.13 · 真实运行）

| 检查 | 命令 | 结果 |
| --- | --- | --- |
| 安装（core） | `bash scripts/setup.sh --profile core` | ✅ rc=0；装 pyyaml 6.0.2 + pytest 8.4.2 + `humanoid008` 可编辑安装 |
| 环境诊断 | `.venv/bin/python scripts/doctor.py --profile core` | ✅ 全部 `ok`，rc=0 |
| 诊断的负向用例 | `/usr/bin/python3 scripts/doctor.py --profile core`（3.14.4） | ✅ 2 项 `FAIL`，rc=1（固定版本检查确实生效） |
| 单元测试 | `bash run_tests.sh --suite core` | ✅ **14 passed**，rc=0 |
| CLI 默认路径 | `bash run_demo.sh --backend fake --planner rule --instruction "把箱子搬到 B 台"` | ✅ rc=**3** + `NOT_IMPLEMENTED`，生成唯一运行目录 |
| 模块入口 | `.venv/bin/python -m humanoid008.app --instruction x` | ✅ rc=3 |
| 控制台脚本 | `.venv/bin/h008 --instruction x` | ✅ rc=3 |
| 阶段阻塞（后端） | `--backend mujoco` | ✅ rc=3，消息点名 **P4** |
| 阶段阻塞（规划器） | `--planner llm` | ✅ rc=3，消息点名 **P3** |
| 用法错误 | `--instruction x --interactive` | ✅ rc=**2** |
| 越界输出 | `--report-root /tmp/elsewhere` | ✅ rc=**2**，拒绝写入 |
| 参数冲突 | `--headless --keep-open` | ✅ rc=**2** |
| 运行记录 | `manifest.json` | ✅ `python_version=3.12.13`，`platform=Linux-6.6.87.2-microsoft-standard-WSL2-x86_64-with-glibc2.43` |

验证产生的运行目录已清理；`reports/` 现仅含 `.gitkeep`。

### 3.2 WSL 验收中发现并已修复的两个缺陷

1. **`scripts/setup.sh` 用了环境里的 `python3`（本机是 3.14.4）** —— 违反 §7 的 Python 3.12 基线。
   已修复：`uv venv --python 3.12`，无 uv 时要求 `python3.12`，并在装完后校验解释器版本，不匹配即失败。
2. **`scripts/doctor.py` 的虚拟环境检查把提示文本无条件打印** —— 检查通过时也显示
   `[ok] virtual environment - missing; run: ...`，自相矛盾。已修复为条件化输出，
   并新增 "python pinned to 3.12" 与 "running inside a virtual environment" 两项检查。

---

### 3.2 P2 验收结果（Ubuntu · CPython 3.12.13 · 真实运行）

板块：`contracts`（枚举/模型/严格校验）、`world`（存储/绑定/占用）、`language`（归一化/规则解析）、
`planners/rule`、`skills`（生命周期/注册表/fake 运行时）、`execution`（单技能执行器 + 任务状态机），
并已接入 `app.py`。

| 用例 | 命令 | 结果 |
| --- | --- | --- |
| 完整搬运 | `--instruction "把取料台上的箱子搬到 B 台。"` | ✅ rc=0，`SUCCEEDED`，8 个技能依序执行 |
| 只观察 | `--instruction "先看看箱子。"` | ✅ rc=0，只执行 `OBSERVE_OBJECT` |
| 只观察 + 禁止搬运 | `--instruction "先看看箱子，不要搬。"` | ✅ rc=0，只执行 `OBSERVE_OBJECT` |
| 歧义指代 | `--instruction "把那个搬过去。"` | ✅ rc=4，`WAITING_CLARIFICATION`，提问而不猜 |
| 不支持指令 | `--instruction "跑步"` | ✅ rc=4，`FAILED` / `UNSUPPORTED_INTENT` |
| 目标台被占用 | `--scene two_targets --instruction "把箱子搬到 B 台。"` | ✅ rc=4，拒绝擅自换台（只跑了观察两步） |
| 注入抓取故障 | `--fail-skill GRASP_OBJECT` | ✅ rc=4，`GRASP_NOT_CONFIRMED`，报告点名失败技能 |
| 未实现后端 / 规划器 | `--backend mujoco` / `--planner llm` | ✅ rc=3，分别点名 P4 / P3 |
| 单元测试 | `run_tests.sh --suite core` | ✅ **157 passed** |

**每次运行的证据**（`reports/<UTC>-<run_id>/`）：`manifest.json`、`input.json`、`events.jsonl`、
`skills.jsonl`、`report.json`。`report.json` 三轴分离；fake 后端固定
`physical_outcome=NOT_EVALUATED`、`test_verdict=NOT_RUN`，`manifest.notes` 明确写出"fake backend"。

**语言样本**：`tests/fixtures/instructions.jsonl` 共 **60 条**（24 明确 / 12 歧义 / 8 不支持 /
8 越权注入 / 8 重复更新），每条带 ID 与期望结果，由 `tests/unit/test_language_fixtures.py` 逐条断言。

**发现并修复的缺陷**：`config/planners/rule.yaml` 的站台别名中英不对称（有 `source` 没有 `target`），
导致 `carry the part to target` 无法绑定目的地 —— 由 T07 样本暴露，已补齐。

---

### 3.3 P3 验收结果（Ubuntu · CPython 3.12.13 · 真实运行）

板块：`supervision/{validator, preconditions, runtime_guard}`（三层检查）、
`execution/{budgets, cancellation, planning_worker}`、`planners/{http_bridge, llm, replay}`，
并把监督管线、预算、取消与**离线规划**接入 `task_manager` 与 `app.py`。

**核心改动**

- 每个提案先过三层监督：结构校验（layer 1，拒绝而非修复）→ 前置条件（freshness / 声明前置 / 预算，
  layer 2）→ 运行时守护（STOP / 指令过期 / 姿态故障，layer 3）。
- 规划**离线执行**（`PlanningWorker`，工作线程 + 有界队列 size 1），控制循环绝不阻塞在模型上；
  STOP 通过轮询抢占，挂起模型的晚到结果按 generation 丢弃。
- 预算账本（`BudgetLedger`）**无 reset 方法**：重规划不能重置预算；`max_steps_per_task` 与
  `model_calls_per_task`（仅 llm）已强制，`replans_per_task`/`retries_per_skill`/`stance_realign_per_task`
  已声明并加载，其花费留到 P6/P7 反馈重规划。
- `--planner replay` 与 `--planner llm` 从"未实现"改为可用；`--planner llm` 未配置端点时
  **报 `NOT_RUN_MISSING_PROVIDER`**，绝不静默回退 rule。

| 用例 | 命令 / 断言 | 结果 |
| --- | --- | --- |
| 默认 rule 闭环 | `--planner rule --instruction "把箱子搬到 B 台。"` | ✅ rc=0，8 技能依序 |
| replay 闭环 | `--planner replay`（fixture 8 提案） | ✅ rc=0，8 技能依序，`planner.name == "replay"` |
| llm 无端点 | `--planner llm` | ✅ rc=4，`NOT_RUN_MISSING_PROVIDER` |
| STOP 抢占挂起模型 | 阻塞规划器 + 200ms 内 `request_stop` | ✅ `TaskStatus.STOPPED`，实测延迟 < 200ms |
| 非法提案 | 未注册技能 / 错误参数 / 未知实体 | ✅ `PRECONDITION_FAILED` / `UNKNOWN_ENTITY`，不执行 |
| 旧 world_revision | 包装 rule 强制 +999 | ✅ `REVISION_MISMATCH`，丢弃不执行 |
| 预算耗尽 | `max_steps_per_task=2` | ✅ `BUDGET_EXHAUSTED`，恰好 2 步 |
| 单元测试 | `run_tests.sh --suite core` | ✅ **184 passed** |

**测试文件**：`tests/unit/test_supervision.py`（三层 + 预算 + 取消的隔离单测）、
`tests/unit/test_p3_control.py`（控制回路集成测试，含 STOP 延迟实测）。

---

### 3.4 P4 验收结果（Ubuntu · CPython 3.12.13 · 真实运行）

板块：`simulation/{walking_policy,robot_runtime,camera,vision,tactile,stance,model_builder,truth_evaluator,fault_injection,backend}`、
`scripts/{import_baseline,check_independence,batch,fix_mnn_stack}`、`assets/baseline_manifest.json`、
`docs/BASELINE_IMPORT.md`、`tests/simulation/test_structure.py`。

**导入**：`import_baseline.py` 从 007 一次性复制 83 个文件（T800 模型+网格+纹理、Allegro 双手、
`combined.xml`、`walking.mnn`、许可证、config），每个文件按 007 源清单校验 SHA-256，写入
`assets/baseline_manifest.json`。上游锚点：EngineAI `335c60e8...`（BSD-3）、Menagerie `wonik_allegro`
`8161bba2...`（BSD-2）。

**仿真栈**：`setup.sh --profile sim` 用 uv 装 `mujoco==3.3.6 MNN==3.6.1 numpy==2.2.6 pillow==11.3.0
PyYAML==6.0.2 patchelf==0.19.1.0`，并对 MNN 的 `_mnncengine*.so` 执行 `patchelf --clear-execstack`
（修复 MNN 3.6.1 wheel 的可执行栈请求）。

| 检查 | 结果 |
| --- | --- |
| 结构冒烟 | ✅ 模型 74 body / 59 joint / 151 geom；payload 自由刚体；25 关节 22 动作；`eyes` 相机存在 |
| 站立测试 | ✅ 走行策略跑 0.5s 后 `base_z=1.013`、双脚受力（`feet_loaded=True`） |
| 单条完整搬运 | ✅ **COMPLETED**，独立验收 `xy_error=0.007 < 0.06`、`height_error=4.4e-5 < 0.012`、`speed<0.025`、目的地接触，墙钟 41.6s |
| 仿真测试套件 | ✅ `MUJOCO_GL=egl` 下 `tests/simulation` 2 passed |
| 独立性扫描 | ✅ `check_independence.py` 干净（无 007 导入/路径/符号链接） |
| 批量回归 harness | ✅ `scripts/batch.py --limit 2` 2/2 COMPLETED（~40s/例） |

**仍未做**：30 例全量回归（`scripts/batch.py` 全量，约 20 分钟 CPU，需显式启动）——
因此 P4 标记为"基本完成"，基线状态保持 `experimental`（§6.2）。

---

### 3.5 P5 验收结果（Ubuntu · CPython 3.12.13 · 真实运行）

板块：`simulation/mujoco_world.py`（MuJoCo → WorldState 世界桥）、
`skills/mujoco.py`（8 个真实 MuJoCo 技能 + `build_mujoco_registry`）、
`simulation/backend.py` 的 `run_skill_task()`（`--split` 入口）、`tests/simulation/test_skill_split.py`。

**拆分映射（§10）**：把 P4 的未拆分基线任务（`BaselineTask` 阶段状态机）拆成 8 个
`Skill` 子类，共享一台 `MujocoWorld`（不重建模型、不重置 qpos），跨技能保留
抓持参考/手指状态/载物上下文/头部目标/站位计划：

| 技能 | 基线阶段 |
| --- | --- |
| OBSERVE_OBJECT | STAND + SEARCH + OBSERVE（定位箱体 + 选抓取站位） |
| OBSERVE_TARGET | 探测目的地 marker |
| APPROACH_OBJECT | APPROACH（走到抓取站位） |
| GRASP_OBJECT | REACH + CLOSE（arm_ik + 双手闭合到接触） |
| LIFT_OBJECT | LIFT + HOLD（抬升 + 计算搬运路线/站位） |
| CARRY_TO_TARGET | CARRY + STOP（走路线 + 停稳锚定） |
| PLACE_OBJECT | INSPECT_DESTINATION + INSPECT_BOX + LOWER + RELEASE + RETRACT |
| VERIFY_RESULT | VERIFY |

**世界桥的两个关键决策（已踩坑并修复）**：
1. **目的地是静态 fixture**，其位置局部化后不会"过期"——`station_b.observed_at_s` 打"现在"
   （占用状态在 PLACE 时重观测）；否则 CARRY 阶段会因目的地 >5s 未观测而误报 `STALE_OBSERVATION`。
2. **箱体一旦被举起，位置由本体（双手 grasp site）追踪**，`box_01.observed_at_s` 打"现在"；
   否则 CARRY 期间头部看向目的地、箱体"过期"，PLACE 会被误拦。

| 检查 | 结果 |
| --- | --- |
| 8 技能注册表 | ✅ `build_mujoco_registry` 恰好 8 个技能 |
| 世界桥 | ✅ `MujocoWorld.view()` 产出合法 WorldState（box_01 初始不可见、station_a FREE、station_b） |
| **完整技能链** | ✅ RulePlanner 依序驱动 8 技能全部 `SUCCEEDED`（`--split` CLI：`SUCCEEDED PASS`） |
| 独立验收 | ✅ `xy_error=0.011 < 0.06`、`height_error<0.012`、`speed<0.025`、目的地接触、双手释放 → `transport_accepted=True` |
| 仿真测试套件 | ✅ `MUJOCO_GL=egl` 下 `tests/simulation` 5 passed（含 1 条 slow 全链路，默认跳过） |
| 单元测试 | ✅ `tests/unit` 184 passed（无回归） |

**等价性**：拆分后的技能链与未拆分基线产出相同的物理结果（都通过独立验收），证明拆分
没有破坏物理等价性（§20 P5 门槛的核心）。

**仍未做**：30 例全量**等价**回归（技能链 × 30 例，与未拆分基线的 30 例对照）——
因此 P5 与 P4 一样标记"基本完成"。

---

### 3.6 P0 补齐 + 15 条边界审计（2026-09-12）

**P0 收尾**：用户提供 `AGENTS_TEMPLATE.md` 正文后，已逐字落盘为工程根 `AGENTS.md`
（2648 字节，owner `ziling`）。`MASTER_PLAN.md`（789 行，§0–§25 全章齐全）与
`IMPLEMENTATION_STATUS.md`（本文件）均已确认就位。**P0 至此无缺口。**

**对照 `AGENTS.md` 15 条边界审计已有实现**（保留改动，未重做）：

| 规则 | 结论 | 证据 |
| --- | --- | --- |
| 2 运行时禁依赖 007 | ✅ 合规 | `check_independence.py` 扫描干净（无 007 导入/路径/符号链接）；`.venv` 独立 |
| 4 导入记录版本/哈希/许可 | ✅ 合规 | `assets/baseline_manifest.json` 逐文件 SHA-256 + 上游 commit + BSD 许可 |
| 8 控制/感知不读真值姿态 | ✅ 合规 | `vision.py` 颜色分割 + 深度平面拟合；`stance.py` 明示 "Payload pose never queried"（`robot` 列表排除 `payload`）；`tactile.py` 仅用 body id 算接触力；`box_center` 来自视觉 `detect()`。真值只出现在 `truth_evaluator.py`（验收）/`fault_injection.py`（注入）/`model_builder.py`（结构检查） |
| 9 禁止瞬移/焊接/固定底座 | ✅ 合规 | 箱体偏移仅在初始化（`backend.py` 注释 "Initial scene variation only"）；payload 全程自由刚体，无 weld 约束；抓取靠双手接触力 |
| 11 backend 明确区分 | ✅ 合规 | `execution_backend` / `planner_backend` 均记录，`fake` 固定 `NOT_EVALUATED`，`llm` 无端点报 `NOT_RUN_MISSING_PROVIDER` |

**发现并修复 1 处不符**：`simulation/backend.py` 的两份物理报告（`baseline_unsplit` / `skill_split`）
缺 `clock_mode` 字段（AGENTS.md "每阶段必须交付"要求三项齐备）。已补 `clock_mode: "realtime"`
（真实 MuJoCo 步进、无 planning_pause）。`app.py` 管线的 manifest 本就有该字段，无此问题。

---

### 3.7 全量回归准备：3 个影响回归正确性的修复（2026-09-12）

启动 30 例全量回归前，发现并修复 3 个问题（均用补丁，保留已有改动）：

1. **`run_skill_task` 未应用初始场景变化（真 bug）**：P5 技能链 `MujocoWorld()` 构造后没应用
   `box_offset`/`target_offset`/`box_yaw`，导致等价回归所有用例都跑默认场景。已补（复制
   `run_episode` 的初始化逻辑 + `mj_forward`）。验证：baseline 非零 offset COMPLETED；split 单例
   确认 offset 已生效（report.args 记录了非零 offset）。
2. **`batch.py` 不支持 `--split`**：扩展 `--split` 参数、`run_case`/`classify` 适配
   `SUCCEEDED` + `physical_outcome==PASS`、summary 的 `task_kind` 与文件名区分
   （`baseline-summary-*` vs `split-summary-*`）。
3. **`max_ticks_per_skill=500`（25s sim）太紧（拆分引入的预算不等价）**：非零 offset 下 PLACE
   技能触发 stance 恢复循环 >25s，而 baseline 用 120s 总时长能过（PLACE 仅 ~19.5s）。这是拆分引入的
   预算裂缝，违反 §13「技能执行时限：保留导入基线」。已把 mujoco 后端的 per-skill 预算对齐
   120s（2400 tick）。**这不是放宽验收阈值**——验收阈值（xy_error<0.06 / height_error<0.012 /
   speed<0.025 / contact）不变，只修正了执行时限这一工程预算。

回归进行中：`baseline` 30 例已启动（前 3 例 regression 组全 COMPLETED）；完成后跑 `split` 30 例等价回归。

**baseline 30 例回归结果（2026-09-12 完成，用户 WSL 终端实跑）**：`24 COMPLETED + 6 FAILED`（80%）。
summary 见 `reports/baseline-summary-20260912T034721Z.json`。6 个失败全是**未拆分基线自身**的局限
（与 split 路径的 5 个问题无关）：

| 失败用例 | 初始条件 | 失败原因 |
| --- | --- | --- |
| 02-regression | box(0.02,0.01)+target(0.05,0.03) | `PHASE_TIMEOUT_CARRY` |
| 07-box-grid | box(0.0,-0.01) | `PLACEMENT_ALIGNMENT_FAILED` |
| 10-box-grid | box(0.02,-0.01) | `PLACEMENT_ALIGNMENT_FAILED` |
| 12-box-grid | box(0.02,0.01) | `PLACEMENT_ALIGNMENT_FAILED` |
| 13-target-grid | target 偏移 | `PLACEMENT_ALIGNMENT_FAILED` |
| 24-combined-yaw | box(-0.017,-0.008)+yaw -3.76° | `PLACEMENT_ALIGNMENT_FAILED` |

**关键含义**：baseline 本身只有 80% 通过率，失败集中在非零初始条件下 PLACE 阶段的站位对齐
（`choose_stance` 连续 3 次 clearance 不 OK）。因此"拆分等价性"的正确定义是：**split 的通过率与失败
模式 ≈ baseline（约 80% / 相同失败原因），而非 split 必须 100%**。这也再次证明评审 AI 的判断对——
单次成功不足以宣称等价，需要同条件 30 例对照。

---

### 3.8 外部评审 5 个正确性问题的修复（2026-09-12）

对照外部评审逐条修复，每项都走"**失败测试先锁 bug → 改实现 → 转绿**"：

| # | 问题 | 修复 | 失败测试 |
| --- | --- | --- | --- |
| 1 | 空规划器也能 SUCCEEDED | `task_manager.py`：`proposal is None` 时校验 intent 的 terminal skill（transport→`VERIFY_RESULT`，inspect→`OBSERVE_OBJECT`）确已 SUCCEEDED，否则 `FAILED`/`GOAL_NOT_SATISFIED`（新增该 ReasonCode） | `tests/unit/test_correctness.py` |
| 2 | 旧观测伪装新鲜 | 新增 `EvidenceSource.PROPRIOCEPTION`；持物箱体标本体来源；`station_b.observed_at` 用真实观测时间（不再伪造 now） | `test_skill_split.py::held_box…` / `destination_timestamp…` |
| 3 | 拆分丢了保护 | `_guard()` 补 `BASE_DRIFT`（新增码）+ `CONTACT_LOST` 两项；`CarryToTarget.start` 记录 `carry_origin`；`MujocoWorld` 加 `contact_loss_since` | `test_skill_split.py::guard_detects_*`、`carry_records_origin` |
| 4 | hold_anchor 残留 + 阶段名 | `ApproachObject` 阶段名 `WALK`→`APPROACH`（对齐 `_locomotion`）；`PlaceObject` 重新对齐时清 `hold_anchor` | `approach_phase_matches…` |
| 5 | 验收不区分 intent | `run_skill_task` 按 `outcome.goal.intent` 区分：transport 才 `evaluate_transport`，inspect 保持 `NOT_APPLICABLE` | `test_inspect_task_is_not_rejudged_as_transport`（slow） |

**测试结果**：`tests/unit` **185 passed**（+1 正确性测试）；`tests/simulation` **10 fast passed + 2 slow**（`skill_split_completes_transport` 与 `inspect_is_not_rejudged`，默认跳过、`-m slow` 显式跑均通过）。

**说明**：问题 2 的"占用状态"部分做了如实标注——V1 单目标场景里 `station_b` 的 FREE 是 fixture 假设
（目标台初始为空、无竞争），不是真实深度占用检测（§11 明确该算法未实现）；代码注释已显式标注此假设，
留待 V2 实现真实占用检测。

---

### 3.9 split vs baseline 同条件 30 例等价回归（2026-09-12）

修复过程中 split 回归先暴露了两个自引回归（已修，见下），最终结果：

| | baseline（未拆分） | split（技能化） |
| --- | --- | --- |
| 通过 | **24/30（80%）** | **26/30（87%）** |
| 失败 | 6（5× `PLACEMENT_ALIGNMENT_FAILED` + 1× `PHASE_TIMEOUT_CARRY`） | 4（3× `PLACEMENT_OUT_OF_TOLERANCE` + 1× `BUDGET_EXHAUSTED`） |

**失败模式一致**：两者失败都集中在"非零初始条件下 PLACE 阶段的站位对齐失败"（baseline 的
`PLACEMENT_ALIGNMENT_FAILED` ≈ split 的 `PLACEMENT_OUT_OF_TOLERANCE`，同一失败点，命名不同）。

**逐用例交叉**（非完全一致）：

- 共同失败 2 例：02（box 0.02/0.01 + target 0.05/0.03）、12（box 0.02/0.01）。
- baseline 独失败 4 例：07 / 10 / 13 / 24（split 下成功）。
- split 独失败 2 例：26（`BUDGET_EXHAUSTED`，CARRY 2400 tick 耗尽）、28（`PLACEMENT_OUT_OF_TOLERANCE`）。

**等价性结论**：通过率接近（split 略高 87% vs 80%），失败模式一致（放置对齐失败），无大规模崩溃、
无失败模式改变——**拆分基本保持了物理等价性**。但存在 6 例交叉差异，说明技能化的站位重规划逻辑与
monolithic 状态机有细微差异（非 bug，是拆分的自然结果）。26 例的 CARRY 预算耗尽值得后续关注。

**回归修复记录**（split 回归先暴露的自引回归，均已修）：

1. `STALE_OBSERVATION`：问题 2 把 `station_b.observed_at` 改真实时间，但 preconditions 对静态 fixture
   位置做了 freshness 检查 → 改为 `KNOWN_FIXTURE_MAP` 来源 + preconditions 跳过静态 fixture 位置检查。
2. `CONTACT_LOST` 误触发：问题 3 用 `holding is not None` 判断持物，但 RELEASE/RETRACT 故意松手 → 改用
   阶段名 `_CONTACT_PHASES`（与 baseline 完全一致）。

summary：`reports/baseline-summary-20260912T034721Z.json` + `reports/split-summary-20260912T044140Z.json`。

---

### 3.10 P6 第一步：语言驱动真实仿真框架（2026-09-12）

P6 分两步：框架（不接模型）+ 真实 LLM（需端点）。**第一步已完成**，真实 LLM 待用户提供端点。

- **planner 选择**：`simulation/backend.py` 加 `--planner rule/replay/llm`（默认 rule）；`run_skill_task` 支持
  ReplayPlanner / LlmPlanner，llm 无端点报 `NOT_RUN_MISSING_PROVIDER`（绝不静默回退）。
- **clock_mode**：加 `--clock-mode`；默认 rule/replay→`realtime`、llm→`planning_pause`（技能边界暂停等模型，
  暂停时间不算实时性能）。
- **replay 对照**：录制响应在真实 MuJoCo 上重放 → SUCCEEDED + 独立验收 PASS，与 rule 共用同一执行器/验收层。
- **观察/歧义闭环**：inspect 只观察（`NOT_APPLICABLE`，不被搬运验收误判）；歧义（`WAITING_CLARIFICATION` 不猜）。

**测试**：`tests/simulation/test_p6_language.py`（llm 无端点、歧义澄清为 fast；replay 驱动真实搬运为 slow）。
`tests/unit` 185 passed；`tests/simulation` 12 fast + 3 slow 全通过。

**仍未做（P6 第二步 / 剩余项）**：真实 LLM 实测（需用户提供端点 + 密钥 + 授权外发图像，§12）；mujoco 后端
STOP 冻结专测（fake 后端 P3 已验证挂起模型 <200ms，mujoco 侧未单独注入验证）。

---

### 3.11 P7 纯逻辑部分：授权替代目标 + 目标更新 + 有限重规划（2026-09-12）

P7 分"纯逻辑"与"物理"两部分。**纯逻辑部分已完成**（fake 后端验证），物理部分（新增 C 台 / 实例识别 /
观测占用）需要新的 MuJoCo 场景，留作 P7 后续。

- **`ProposalKind.RETARGET`** + `ActionProposal.target_id`：planner 报告"主目标占用但有授权替代台"。
- **RulePlanner**：主目标台占用时，检查 `allowed_alternative_targets`，返回 retarget 到第一个
  "授权且非 OCCUPIED"的替代台（UNKNOWN 也接受，重执行时 OBSERVE_TARGET 会观测并再校验）；无授权 → 拒绝。
- **task_manager**：收到 retarget → 校验 target 在授权列表（§15 擅自换台仍禁止）→ 花费 `replans_per_task`
  预算（超限 `BUDGET_EXHAUSTED`）→ 目标更新（`goal_revision+1`、`target_id` 换台）→ 重置 history 重新执行。
- **测试**：`tests/unit/test_p7_retarget.py`（授权替代成功 / 无授权拒绝 / replans 预算耗尽），3 通过。

**测试结果**：`tests/unit` **188 passed**；`tests/simulation` 12 fast + 4 slow 全通过。

**仍未做（P7 物理部分）**：新增 C 台（MuJoCo 场景 + 视觉多标记区分）、实例识别（多箱子）、观测占用
（depth/ROI，§11 明确算法未实现）。

### 3.12 P7 物理 C 台：诊断 + 两处修复（2026-09-13，PLACE 仍未通过）

**已完成**：C 台加入场景（蓝色 marker，`destination_c_*` geom）、`vision.detect("destination_c")`、
`station_c` 世界桥 + `destination_for/marker_bearing/placed_target`、站位/碰撞纳入两套 fixture、
`LIFT_OBJECT` 加 `target_id`、技能按 `_target_id` 取台、`evaluate_transport(target_id=)`。
**C 台 OBSERVE→CARRY 全通；PLACE 未通过。**

**根因（离线诊断 `scripts/_diag_stance.py` + yaw 追踪，决定性）**：
1. **yaw 漂移是根因**：机器人到 C 台时身体 yaw = **−8.3°**（非 0）。漂移在 CARRY 累积。
2. **yaw 不可原地纠正**：保持位置 + yaw 目标 0 跑 20s，yaw 只从 −8.06° → −6.50°（**~0.08°/s，慢 100 倍**）。
   T800 策略几乎不能原地转体 → "到位后转正"不可行，必须从源头减漂移。
3. **reach 窗口窄且对 yaw 敏感**：同朝向同种子下 B 与 C 窗口**完全一致**（0.08m、9 feasible）——
   侧向偏移**不**改变双臂可达（推翻此前"侧向 reach 减半"的结论）。窗口对 yaw 极度敏感：0°=9、±10°=2~3、±20°=0。
4. 到达时 `LINK_HIP_YAW_L` 撞 `destination_c_table`（clearance 0.0007m）+ **前向过冲**（目标 x=1.276，
   实停 x=1.376）。

**两处修复（保留，B 台无回归 SUCCEEDED PASS）**：
- `skills/mujoco.py` LIFT_OBJECT 走廊：`side_y = min(y, final_y) - 0.65` → `min(final_y, -0.65)`。
  原式对 C(−0.5) 给 −1.15（比终点低 0.65m），强制 ~1.8m 侧向行走 → yaw 漂到 −8.3°；改后 ~0.8m。
- `stance.py` select 稳健性（评审 #4）：`score += 0.20*max(0, 4-robustness)`，优先窗口内部（复用已算网格，
  无额外 IK）。实测 selected 由边缘 x=1.209 → 内部 x=1.276（robustness=4）。

**结论**：C 台 PLACE 失败**不是**"几何矛盾不可解"（此前结论已撤回），而是**行走策略精度局限**
（yaw 漂移不可纠正 + 前向过冲 ~0.1m）叠加**窄窗口 0.08m**。**下一步候选**：① 更激进减小侧向/转弯；
② 加宽 reach 窗口（更好手臂 seed/姿态让手臂吸收基座误差）；③ 旋转 C 台朝向让机器人以 yaw≈0 正面接近。
**测试**：`tests/unit` 188 passed；`tests/simulation` 12 fast 全通过；B 单例 SUCCEEDED PASS。

### 3.13 评审清单落实（2026-09-13）

| 清单项 | 状态 |
|---|---|
| §1 结论别太绝对；区分偏航 / 携物姿态 / 相邻台 / 定位误差 | ✅ 真凶 = **偏航** |
| §3-1 离线站位诊断（同朝向 / 种子 / 手位 / 台高，B vs C）| ✅ `scripts/_diag_stance.py` |
| §3-2 分离 IK 失败 vs 碰撞失败；记录具体部件 | ✅ `BILATERAL_REACH` vs `FIXTURE_CLEARANCE` + 部件名 |
| §3-3 变量定位 | ✅ 偏航（±10° 塌 2/3），**不是** y 偏移、**不是** B 台障碍 |
| **§2 / §3-4 每工作台独立有符号标记偏移（5 处同步）** | ✅ **能力已实现**：`workcell.json.marker_offset_y` 按台配置；`mujoco_world.py` 视觉换算、`stance.py` 碰撞模型均按台取偏移；`marker_bearing` 早已按台。单测 `test_stance_marker_plate_follows_per_station_offset`。⚠️ **示例布局（C 标记装 −y 侧）实测失败**：C 标记移到 y=−1.25 后 `OBSERVE_TARGET` 检测不到（标记几乎与桌面同高、又在桌面外侧，被台面自身遮挡）→ 回退到可工作的 +0.75 |
| §3-5 静止观察 B/C 定位 | ✅ **`scripts/check_localization.py`**：B 误差 **0.003 m**、C 误差 **0.006 m**（真值仅用于独立核对，不回灌控制，§25.5）|
| §3-6 保留失败报告；B 无回归；**C 完成放置 / 松手 / 撤手** | ⚠️ B 无回归 ✅；**C 仍未完成 ❌** |
| §4 站位选择稳健性（邻域含 yaw）| ✅ `stance.py` robustness 打分（优先窗口内部，非最近的边缘）|

**#4 C 放置未成的根因与尝试**：偏航漂移 −6°~−8°（`LINK_HIP_YAW_L` 撞 `destination_c_table`）+ 窗口窄 0.08 m + 走位过冲 ~0.1 m。已试：① 收紧 PLACE 重对齐容差 0.06→0.025 → **BUDGET_EXHAUSTED**（策略能精确**持位** 0.004 m，但**走不到**亚厘米目标，会过冲）；② 走廊改"终点直走"→ 同样 hang。**结论：不是调参能解，需在"加宽 reach 窗口 / 换接近朝向"层面解**（P7 物理待办）。

**测试**：`tests/unit` 188 passed；`tests/simulation` 13 fast 全通过；B 单例 SUCCEEDED PASS。

### 3.14 C 台放置：加宽窗口（①）与换接近朝向（②）尝试（2026-09-13，均未成功）

**① 加宽 reach 窗口** —— 先量清窗口（新增 `scripts/_diag_reach.py`，固定朝向沿接近轴扫 d）：

- 窗口实际是 **[0.22, 0.34] m ≈ 0.14 m 宽**（不是此前估的 0.08 m；0.08 m 是旧网格只采了 0.24/0.28/0.32 三点造成的假象）。
- 下界 = `FIXTURE_CLEARANCE`（d≤0.20 髋部撞台——是机器人**自己**的髋，难改）；上界 = `BILATERAL_REACH`（d≥0.36 手臂够不到，err=0.016 刚超阈 0.012）。
- **结论：窗口被"髋部间隙 + 手臂极限"双向夹死，加宽余地很小。** 加密候选网格（步长 0.04→0.02）实测**反而把 C 从干净失败变成 CARRY BUDGET_EXHAUSTED 挂死**，已撤回。

**② 换接近朝向** —— 把 C 台从 y=−0.5 移到 y=−0.65（与走廊对齐，CARRY 末段变纯直行）：

- 实测**整个 CARRY 超时挂死（>400 s）**，且视觉定位误差从 0.006 m 涨到 0.040 m。
- 已撤回，C 台恢复 y=−0.5。

**根本障碍（本轮实锤）**：机器人 CARRY 后**前向过冲 ≈0.11 m**（目标 x=1.276，实停 x=1.387）+ **偏航漂移 −6°~−8°**，**两者都超过窗口半宽 0.07 m**，即"走位精度 < 窗口半宽"不成立。**这不是调参能解**——需要控制层（减小过冲/漂移）或工作台重设计层面的改动。

**A · 控制层也试过并撞墙**：在 `hold_command` 末端（error<0.20 m）把限速 0.30→0.12 m/s 以缩短制动距离（过冲 ∝ v²）→ **CARRY 直接 BUDGET_EXHAUSTED 挂死**（太慢，走不到航点）→ 已撤回 `robot_runtime.py`。至此**三条路（①加宽窗口 / ②换接近朝向 / A 控制减速）全部因"行走精度极限"撞墙**：任何让接近更难/更慢的改动，都让机器人在预算内到不了目标。

**测试**：`tests/unit` 188 passed；`tests/simulation` 13 fast 全通过；B 单例 SUCCEEDED PASS。

---

## 4. 未实现 / 未创建

- **未创建**：`THIRD_PARTY_NOTICES.md`、
  `docs/` 其余文件（`ARCHITECTURE.md`/`CONTRACTS.md`/`VALIDATION.md`/`LIMITATIONS.md`/
  `REPRODUCTION.md`/`UNIFOLM_WLA_FEASIBILITY.md`）。
- **已在 P2/P3/P4/P5 实现**：`contracts`、`world`、`language`、`planners/{rule,replay,llm,http_bridge}`、
  `skills`（fake + **mujoco**）、`execution/{executor,task_manager,budgets,cancellation,planning_worker}`、
  `supervision/{validator,preconditions,runtime_guard}`、`simulation/*`（MuJoCo 后端 + 未拆分基线 +
  **技能化后端**），全部接入 `app.py` / `simulation.backend`。
- **未实现**：真实仿真语言驱动（P6，即用真实 LLM 提出下一技能 + 规划暂停 + 观察/歧义/取消）、GUI（P6）、
  多目标与反馈重规划（P7）、复现与文档门（P8）。
- **未接入真实模型服务**：`llm` 规划器与 HTTP 桥已就绪，但无端点，实测记 `NOT_RUN_MISSING_PROVIDER`。
  因此当前**仍不能叫"LLM 完整版"**（§23）。

---

## 5. 下一步

**等待用户：**

1. **提供 `AGENTS_TEMPLATE.md` 正文** —— P0 要求把它保存为工程根 `AGENTS.md`；这仍是 P0 的唯一缺口。
2. **P4/P5 收尾（可选）**：是否现在启动 **30 例全量回归**（未拆分基线 + 技能链两条对照，
   约 40 分钟 CPU），把 `experimental` 固化为有完整证据的等价回归记录。
3. **确认是否进入 P6**（语言驱动真实仿真）：接入真实 LLM 规划器（或先用 `replay` 对照），
   在技能边界做 planning_pause，实现观察/搬运/歧义/取消/有限更新的真实仿真闭环。

P1 / P2 / P3 / P4 / P5 均已通过 WSL 验收（P4/P5 除全量回归外）。

### 3.15 C 台放置：控制链路诊断（2026-09-13，read-only，根因已定位）

**综述**：按"先诊断不要再调参"的评审方案，新增只读诊断 `scripts/diagnose_c_placement.py`
（trace / yaw / response 三部分，入口与出口各校验一次资产 SHA-256，未写入 src/ 或 config/，
未改默认参数）。**根因不是策略精度极限，是两层缺陷叠加。**

**结论表**

| 层 | 缺陷 | 证据 |
|---|---|---|
| 规划层 | `mujoco.py:374` 把选定站位硬推 −0.08 m | 窗口仅 0.08 m 宽，偏移量正好 0.08 m → 落点每次都到近边界外，B/C 双双 `BILATERAL_REACH`（B:0.950→0.870、C:1.320→1.240） |
| 执行层 | 策略 ω 跟踪严重不足（约 16× 欠额） | 命令 ω=0.112 持续 **34 s（13167/30825 tick）**，yaw 冻结在 −6.41°；`req_w == eff_w`，限幅也放行了 → 命令到了策略，策略不响应 |
| 控制层 | `hold_command` slew 约 0.0016/tick | 爬到 0.1745 需 ~0.44 s 仿真（非致命） |
| 站位搜索 | 搜索空间不含朝向 | yaw ±10° → 可行数从 9 塌到 2~3；±15° → 0。B 与 C 基本一致，B 高 yaw 下更差 |

**关键更正面板**（推翻此前文档结论）

| 旧表述 | 本轮实测 |
|---|---|
| 策略几乎不能原地转体（~0.08°/s） | **能转**：ω=0.3 → 2.0–2.2°/s。0.08°/s 是错误控制输入下的观测值 |
| reach 窗口 0.14 m | 端点跨度 **0.12 m**；0.14 含采样步长 |
| 持物行走是变量，需单独立项 | 三态（空手/搬运姿态/持物）ω 响应**一致**（±0.3 → ~4.0–4.7°/2s），**持物不是变量** |
| 三条路全部因行走精度极限撞墙 | 修 ①−0.08 偏移、②ω 逆映射补偿 **尚未尝试**，且均有明确机制支撑 |

**修复优先级（一次只改一个）**
1. 删掉 / 参数化 `mujoco.py:374` 的 −0.08 偏移（确定性 bug，回归 B 台）。
2. 按实测曲线对 ω 做逆映射补偿（要 0.1745 rad/s 就下发更大 ω）。
3. 站位搜索加入有界朝向扫描（±10°）。
4. 以上都试过仍不过，才写"能力边界报告"，措辞为"当前控制器+预算下未达成"，**不是**物理做不到。

**交付物**
- `scripts/diagnose_c_placement.py`（只读）
- `reports/diag-trace-*/diag_locomotion.json`（逐 tick 命令→运动链）
- `reports/diag-yaw-*/diagnosis.json`（固定 vs 扫描朝向窗口）
- `reports/diag-response-*/diagnosis.json`（三态直发速度响应）
- `docs/C_STATION_DIAGNOSIS.md`（完整报告）
