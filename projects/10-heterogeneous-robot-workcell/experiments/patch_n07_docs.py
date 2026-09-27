"""P1-N-07 documentation patches: TASK_BOARD, IMPLEMENTATION_STATUS, P1_FEASIBILITY, DECISIONS.

Exact-anchor patches with a count assertion, so an anchor that stops matching fails loudly instead
of silently skipping (the P1-N-06 round had a documentation patch that reported success while
changing nothing, because its "already applied" test was the new text's first line -- which was
also the old text's last line).
"""
import argparse
import hashlib
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]

PATCHES = {
    'docs/TASK_BOARD.md': [
        (
            'N-07 row: READY -> PARTIAL with the measurement',
            '| P1-N-07 | READY | N-06 | 单车 Nav2 实际到站（同一世界、同一桥）：建图/静态地图、'
            'AMCL 或已知初始位姿、控制器实际把车开到目标点，并记录到站误差 |\n',
            '| P1-N-07 | PARTIAL | N-06 | **控制器确实把车开到目标点并停稳，到站误差已记录；'
            '但真值到站误差 0.431 m 超过 0.15 m 目标容差。** 世界/静态地图/AMCL/规划器/控制器/'
            '命令门整链已通（13 个 lifecycle 全 ACTIVE、一条 `/clock` 发布者、每条 TF 边一个写者、'
            '地图与首帧 scan 99.4% 一致、命令链四跳都有流量），绕障与停车都有真值证据'
            '（绕 `pillar_a` 最近 0.514 m、末段速度 0.0000 m/s）。未过的是 AMCL 位姿精度。'
            '证据 reports/p1-n-nav-06（30 项判据：28 PASS / 2 FAIL，两个 FAIL 同一根因）；'
            '`navigate_to_pose` 返回 SUCCEEDED 而真值离目标 0.431 m —— 所以到站一律用真值量 |\n'
            '| P1-N-08 | READY | N-07 | **AMCL 位姿精度**：`amcl_pose` 沿路径**超前**真值 '
            '0.24–0.31 m（中位 0.247 m）而航向正确（中位 0.013 rad），同段路上轮式里程计只有 '
            '0.066 m ⇒ 扫描匹配比里程计差 4.7 倍，这是 0.431 m 到站误差的唯一根因。'
            '要在同一世界做受控对比（位姿来源、`update_min_d/a`、`transform_tolerance`、波束数、'
            '`laser_likelihood_max_dist`），判据复用 `evaluate_n_nav.py`，**不改阈值** |\n',
        ),
        (
            'header note',
            '状态更新时间：2026-09-20。P1 已获授权并开始组件实验，尚未通过共同世界门。'
            '证据见 P1_FEASIBILITY.md。\n',
            '状态更新时间：2026-09-20。P1 已获授权并开始组件实验，尚未通过共同世界门。'
            '证据见 P1_FEASIBILITY.md。\n'
            '**N 的整链已打通（`P1-N-07` PARTIAL）：控制器把车开到目标点并停稳，但真值到站误差 '
            '0.431 m，根因是 AMCL 位姿超前真值 0.247 m（中位）——`P1-N-08`。**\n',
        ),
    ],
    'docs/IMPLEMENTATION_STATUS.md': [
        (
            'date and phase',
            '更新：2026-09-20（P1-H-04）。阶段 P1，组件验证继续，未进入 P2。\n',
            '更新：2026-09-20（P1-N-07）。阶段 P1，组件验证继续，未进入 P2。\n',
        ),
        (
            'done: N-07 chain',
            '- **P1-H-04：退出阶段改为两段式，六阶段在 V1 托盘上首次全部通过。**\n',
            '- **P1-N-07（PARTIAL）：单车 Nav2 到站链路已建立并可复现。** 用 `experiments/make_map_n.py` '
            '从世界几何派生静态地图（181×181 @0.05 m，起点落在像元中心，free space = 从起点可达的空间），'
            '用 `experiments/make_nav2_params.py` 从 nav2 自带默认参数派生 `config/nav2_n_probe.yaml`'
            '（每处偏离 stock 都具名并写明理由），`experiments/probe_n_nav.py` 编排 sim + 桥 + 观察者 + '
            'Nav2 并发出一个 `NavigateToPose` 目标，`experiments/evaluate_n_nav.py` 用真值判 30 项。\n'
            '  实测：13 个 lifecycle 全 `active`；`/clock` 恰好一个发布者；动态 TF 恰好 '
            '`map→odom`+`odom→base_link` 两条边且每条边一个写者；`/map` 与生成文件逐项一致；'
            '**地图与首帧真实 scan 99.4% 在 0.2 m 内一致**；命令链四跳 '
            '`cmd_vel_nav 267 / cmd_vel_smoothed 277 / cmd_vel 277` 全部有流量、0 限幅；命令门接受 335、'
            '拒绝 0、结束时回退到零；目标 `status 4` 成功、13.6 s 完成；真值路径 3.184 m 且离 '
            '`pillar_a` 最近 0.514 m（需 ≥0.430 m）；末段 1.0 s 速度 0.0000 m/s；轮式里程计误差 '
            '0.0664 m / 0.0025 rad。\n'
            '  **未过：真值到站误差 0.431 m（容差 0.25 m）。** 根因是 AMCL 位姿沿路径超前真值 '
            '0.247 m（中位，p95 0.303 m）而航向中位只差 0.013 rad —— 目标检查器据此在真值 0.431 m 处'
            '判成功。`P1-N-08` 负责这一点。证据 `reports/p1-n-nav-06`。\n'
            '- **P1-H-04：退出阶段改为两段式，六阶段在 V1 托盘上首次全部通过。**\n',
        ),
        (
            'not done: replace the N line',
            '- A 未实施；C 缺车载支撑/对接。**N 的 IPC、时钟、传感器与 TF 已通过（`P1-N-06`）；'
            '缺的是单车 Nav2 实际到站（`P1-N-07`）。**\n',
            '- A 未实施；C 缺车载支撑/对接。**N 的 IPC、时钟、传感器与 TF 已通过（`P1-N-06`），'
            '整链到站已跑通（`P1-N-07` PARTIAL）；缺的是 AMCL 位姿精度（`P1-N-08`）与重复性。**\n'
            '- **`P1-N-07` 只跑了一个目标点，没有重复性**；地图是世界几何先验**不是 SLAM**，'
            '所以不能声称建图；底盘仍是 010 自制的差速代身，不是 AMR 选型，'
            '选型后 N 的验收必须在真模型上重跑。\n'
            '- **`/cmd_vel` 上有三个发布者**（collision_monitor、docking_server、following_server）。'
            '本轮只能声称"每条 TF 边一个写者"和"实测命令流"；严格单写者需要把 docking/following '
            '从 bringup 里去掉，未做。\n',
        ),
        (
            'evidence: add the N-07 run and the test count',
            '- `reports/_invalid/`：仪器自身失败的 run（判据块 KeyError、两次 6 秒空转、退出路径诊断的 tracer bug）。\n',
            '- `reports/p1-n-nav-06`：**N-07 的权威记录**（30 项判据，28 PASS / 2 FAIL）。'
            '`acceptance.md` 是给人看的逐项结果，`acceptance.json` 是机器可读的。\n'
            '- `reports/_invalid/p1-n-nav-01…05`：五个被归档的运行，每个 README 写明失效原因 —— '
            '分别是 `filters: []` 打死节点、行为树握手超时、观察者自己也用错类型、'
            '以及第一次真正把车开动的那次（保留因为它证明了整链）。\n'
            '- `reports/_invalid/`：仪器自身失败的 run（判据块 KeyError、两次 6 秒空转、退出路径诊断的 tracer bug）。\n',
        ),
        (
            'evidence: test count 82 -> 107',
            '- `.venv/bin/python -m pytest tests/ -q`：**82 passed**（含 P1-N 的 25 条：IPC 契约、命令门、'
            '以及判据必须能对合成输入判 FAIL）。\n',
            '- `.venv/bin/python -m pytest tests/ -q`：**107 passed**。新增 25 条 P1-N-07：'
            '地图栅格化（起点落在像元中心、手工几何对得上、墙外那圈不是 free、rotated geom 必须拒绝、'
            '`--check` 幂等）、nav2 参数生成器（能解析、无空序列、每节 `use_sim_time`、锚点失配必须拒绝）、'
            '以及**判据必须能 FAIL** —— 包括"nav2 说成功但停在 0.431 m 外"、'
            '"直线穿过柱子"、"AMCL 偏 0.32 m"、"命令类型是 Twist"、"某一跳计数为 0"、'
            '"输入缺失必须报 FAIL 而不是沉默"。\n',
        ),
        (
            'next steps',
            '1. **`P1-N-07`**：单车 Nav2 实际到站，复用本轮的世界、桥与命令门（IPC 部分已在 `P1-N-06` 通过）。'
            '开工前先 `source scripts/env.sh`——它会拒绝 009 的域 42。\n',
            '1. **`P1-N-08`（AMCL 位姿精度）**：这是 0.431 m 到站误差的唯一根因，也是 009 那条'
            '"自报位姿说在容差内、真值说不在"缺陷的同类。判据已就绪，先做受控对比再动参数。\n',
        ),
        (
            'next steps: repeat and multi-goal',
            '2. `P1-A-04` 先选型（下载体积需单独申请授权）；`P1-C-05` 的车载支撑与对接互锁。\n',
            '2. `P1-N-07` 的重复性与多目标：不同目标点、不同起点、不同 seed —— 现在只有一个样本。\n'
            '3. `P1-A-04` 先选型（下载体积需单独申请授权）；`P1-C-05` 的车载支撑与对接互锁。\n',
        ),
    ],
    'docs/P1_FEASIBILITY.md': [
        (
            'N row in the evidence table',
            '| 结构测试 | **82 passed**（含 25 条 P1-N：IPC 契约、命令门、判据自检） '
            '| 不等同于物理整体验收 |\n',
            '| **单车 Nav2 整链（静态地图→map_server→AMCL→planner→controller→velocity_smoother'
            '→collision_monitor→命令门）** | **链路通、到站未达标**：13 个 lifecycle 全 ACTIVE；'
            '动态 TF 恰好两条边、每条边一个写者；地图与首帧真实 scan **99.4%** 一致；命令链四跳 '
            '267/277/277 条、0 限幅；门接受 335、拒绝 0；目标 `status 4`；真值路径 3.184 m、'
            '离 `pillar_a` 最近 0.514 m、末段停下；**真值到站误差 0.431 m（容差 0.25 m）** | '
            '`reports/p1-n-nav-06`。**地图是世界几何先验、不是 SLAM**；单目标点、无重复性；'
            '底盘不是 AMR 选型 |\n'
            '| 结构测试 | **107 passed**（含 25 条 P1-N-07：地图、参数生成器、判据必可 FAIL） '
            '| 不等同于物理整体验收 |\n',
        ),
        (
            'N section: the "still missing" paragraph',
            '**仍未做**：单车 Nav2 实际到站（`P1-N-07`）、建图与定位、位姿扰动、到站误差。\n',
            '**`P1-N-07`（PARTIAL）**：整链已通（见上表），控制器把车开到目标点并停稳，'
            '但真值到站误差 0.431 m。根因是 **AMCL 位姿沿路径超前真值 0.247 m（中位）、p95 0.303 m**，'
            '而航向中位只差 0.013 rad；同一段路上**轮式里程计只有 0.066 m**，'
            '即扫描匹配比里程计差 4.7 倍。目标检查器据此在真值 0.431 m 处判成功 —— '
            '所以 N-07 的判据一律用真值量，不采信 nav2 自己的结论。\n\n'
            '**仍未做**：AMCL 位姿精度（`P1-N-08`）、重复性（单目标点）、建图（地图是几何先验）、'
            '位姿扰动、严格意义的单命令写者（`/cmd_vel` 上仍有三个发布者）。\n',
        ),
    ],
    'docs/DECISIONS.md': [
        # The text lives in the D024 constant below and is resolved in main(), because the
        # literal is far longer than anything worth inlining in this table.
        ('append D024', None, 'D024'),
    ],
}

D024 = """

## D024 — P1-N-07：到站用真值判、命令类型按实测订阅、静态地图声明为先验

日期：2026-09-20。范围：`P1-N-07`（单车 Nav2 实际到站）。

**1. 到站误差只用真值，不用 nav2 自己的结论。** 权威记录里 `navigate_to_pose` 返回 SUCCEEDED，
而真值离目标 **0.431 m**（容差 0.15 m）——因为 AMCL 自己算出的是 0.132 m。
这与 009 的受保护区失败是同一类缺陷（自报位姿带 `localization_valid=True` 却是错的），
所以 `evaluate_n_nav.py` 里"到站"这条判据只读 MuJoCo 的真值轨迹。

**2. 命令话题的**类型**按实测订阅，并把实测结果写进报告。** 本轮 nav2 在
`controller_server`、`velocity_smoother`、`collision_monitor` 上发布的都是
`geometry_msgs/msg/TwistStamped`，而桥接最初订阅 `Twist`。**DDS 会为 QoS 不匹配报警，
不会为类型不匹配报警** —— 没有日志、没有异常、计数为 0，症状与"控制器没发命令"完全一样，
而 nav2 内部一致所以它自己也不报错。⇒ 订阅类型成为显式选项（默认实测值），
报告记录 `publisher_types_seen` / `type_mismatch`，让下一次变化响亮失败。
（旁证：我第一次把 0 条命令归因到 `/cmd_vel` 的 "incompatible QoS" 警告上，是**错的**；
同一次跑观察者用 RELIABLE 订阅也是 0 条，说明话题本身没有流量。）

**3. 静态地图是"世界几何先验"，不是 SLAM，并在报告里如此标注。** 地图由
`experiments/make_map_n.py` 从 010 自己写的世界文件派生 —— 相当于真实部署里加载的勘测图纸，
是 operator prior，不是运行时测量，也不是 AGENTS.md 禁止回流定位的"物体真值"。
代价必须写明：**N-07 因此不验证建图**，报告里 `map.source = "world_geometry_prior"`、
`slam_exercised = false`。为了不让地图默默错掉，生成器 `--check` 幂等，
并且判据用**同一份地图投射射线**与**真实 scan** 对比（实测 99.4% 在 0.2 m 内一致）——
origin、分辨率、图像翻转中任何一项错都会失败。

**4. 地图 origin 刻意不是 resolution 的整数倍。** 让起点落在**像元中心**
（origin = 起点 − (n+0.5)·res）才能让"透过地图投射射线"与真实 scan 是同一件事；
代价是 `StaticLayer` 会打一条 "origin coordinates are not perfectly aligned with the
resolution" 的 WARN（≤2.5 cm 的取整）。选择留在这里，因为一条无害的 WARN 比一个
半格偏差的验证工具便宜。

**5. 运行结束用 SIGUSR1 让仿真**优雅**退出，而不是杀进程。** 真值轨迹写在仿真退出时写的报告里，
杀进程等于把整轮要收集的证据丢掉。该行为是 opt-in（`probe_n_sim.py --allow-stop-signal`），
不带标志时与 `P1-N-06` 完全一致。

**6. 参数文件从 nav2 自带默认值派生，而不是手写。** stock 文件 604 行、覆盖 20 个节点；
漏一节不会报错（节点跑默认值）。派生让每一处偏离都是脚本里一条具名条目，
并且**拒绝输出任何 `key: []`** —— 空 YAML 序列会让 rclcpp 在构造节点时抛
"rcl parameter structure contains no value"，`controller_server`/`planner_server` 直接 SIGABRT，
而症状只是 lifecycle 停在 `unconfigured`。

**7. 行为树握手超时按**时钟**定，不按默认值。** stock `default_server_timeout: 20`
配上本探针 20 Hz 的 `/clock`（一 tick 50 ms）短于一个 tick：实测 BT 206 ms 就放弃
`follow_path` 握手，而 `controller_server` 52 ms 后才收到该目标（error 107）。改成 1000。
这是耐心值，**不是验收阈值**。
"""


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--check', action='store_true')
    parser.add_argument('--root', default=str(ROOT))
    args = parser.parse_args()
    root = Path(args.root)
    failures = []

    for relative, patches in PATCHES.items():
        path = root / relative
        text = path.read_text(encoding='utf-8')
        before = hashlib.sha256(text.encode()).hexdigest()
        changed = 0
        for label, old, new in patches:
            if isinstance(new, str) and new == 'D024':
                new = D024
            if old is None:                       # an append-only patch
                if new.strip() in text:
                    continue
                if args.check:
                    failures.append(f'{relative}: not applied: {label}')
                    continue
                text = text.rstrip('\n') + '\n' + new
                changed += 1
                continue
            if new in text:
                continue
            count = text.count(old)
            if count != 1:
                failures.append(f'{relative}: anchor for "{label}" appears {count} time(s), '
                                f'expected 1')
                continue
            if args.check:
                failures.append(f'{relative}: not applied: {label}')
                continue
            text = text.replace(old, new)
            changed += 1
        if changed and not args.check:
            path.write_text(text, encoding='utf-8')
            after = hashlib.sha256(text.encode()).hexdigest()
            print(f'{relative}: {changed} patch(es) applied, sha256 {before[:16]} -> {after[:16]}')
        elif changed:
            print(f'{relative}: {changed} patch(es) pending')

    if failures:
        for failure in failures:
            print(f'[FAIL] {failure}', flush=True)
        return 1
    print('[OK] documentation patched', flush=True)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
