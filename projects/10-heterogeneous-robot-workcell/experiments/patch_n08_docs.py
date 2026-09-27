"""P1-N-08 documentation patches.

Uses prefix-based line replacement for the task-board rows and the status bullets, because those
lines legitimately change every round and an exact-anchor patch would have to be rewritten each
time. Everything else is an exact anchor with a count assertion.
"""
import argparse
import hashlib
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]

N07_ROW = (
    '| P1-N-07 | DONE | N-06 | **单车 Nav2 实际到站已通过：真值到站误差 0.230 m（限 0.25 m）、'
    '航向差 0.112 rad，32 项判据 32 PASS / 0 FAIL。** 整链（静态地图→map_server→AMCL→planner→'
    'controller→velocity_smoother→collision_monitor→命令门→车轮）通：13 个 lifecycle 全 ACTIVE、'
    '一条 `/clock` 发布者、每条 TF 边一个写者、地图与首帧真实 scan 99.4% 一致、命令链四跳 '
    '273/283/283 且 0 限幅、门接受 339/拒绝 0/末态回零、真值路径 3.373 m 且绕 `pillar_a` 最近 '
    '0.541 m、末段速度 0.0000 m/s。**它是修正 AMCL 运动噪声之后才通过的**，修正前的 FAIL 记录完整保留在 '
    '`reports/p1-n-nav-06`（0.431 m）——两份报告构成 A/B。证据 `reports/p1-n-nav-07` |'
)

N08_ROW = (
    '| P1-N-08 | DONE | N-07 | **AMCL 位姿精度：误差在 AMCL 内部产生，已归因并用受控 A/B 修好。** '
    '同一世界的只测定位实验里，AMCL 位置中位 **0.256 m** 而它消费的轮式里程计只有 **0.005 m**'
    '（差 47.6 倍）⇒ 不是继承来的误差。机制：stock `alpha1..5 = 0.2` 假设里程计有 ~10% 行程噪声，'
    '而本底盘实测 0.005 m / 3.7 m，于是 AMCL 每 0.25 m 更新注入 σ≈0.05 m 的运动噪声，15 次更新约 '
    '0.19 m 的随机游走，而 4 m 外的墙 + `z_rand 0.5` 的似然面太平、`recovery_alpha_* = 0.0` '
    '无恢复 ⇒ 拉不回来。**A/B（只改 alpha 0.2 → 0.02）：AMCL 0.256 → 0.070 m**，'
    'N-07 到站 0.431 → 0.230 m，判决 FAIL → PASS。证据 `reports/p1-n-08-loc-03`（前置）、'
    '`-loc-04`（A/B）、`p1-n-nav-07`（验收）；⚠️ 未测五个 alpha 里哪一个起作用，'
    '也未试"把似然面变尖"这条替代路径；**0.02 是按这台代身底盘推的，AMR 选型后必须重标** |'
)

HEADER_NOTE = (
    '**N 已跑通并达到验收：`P1-N-07` DONE，真值到站 0.230 m（限 0.25 m），32 项判据全 PASS；'
    '`P1-N-08` 把根因（AMCL 运动噪声模型与本底盘不匹配）定位并修好，证据 `reports/p1-n-nav-07`。'
    '仍缺：重复性（只有一个目标点）、建图（地图是几何先验）。**'
)

STATUS_BULLET = """- **P1-N-08（DONE）：AMCL 位姿精度 —— 误差在 AMCL 内部产生，已归因并用受控 A/B 修好。**
  **先分开"谁的错"**：新增 `experiments/analyse_n_amcl.py` 把误差分解成沿轨迹/垂直轨迹两个分量；
  N-07 的误差是**沿轨迹超前 0.234 m**、垂直只差 0.061 m、航向只差 0.013 rad。
  **再量输入**：给观察者加了 `odom_samples`（整条轨迹，不是首末点），并做了一个**只测定位**的实验
  （同世界同桥、Nav2 起来但不导航、桥接发 P1-N-06 profile）——**同一次运行内** AMCL 位置中位
  **0.256 m** vs 轮式里程计 **0.005 m**（差 47.6 倍）⇒ 误差在 AMCL 内部产生。
  **机制**：stock `alpha1..5 = 0.2` 假设里程计有 ~10% 行程噪声，本底盘实测 0.005 m / 3.7 m；
  `update_min_d 0.25` ⇒ 每次更新注入 σ≈0.05 m，15 次 ≈ 0.19 m 随机游走；而墙在 4 m 外、
  `sigma_hit 0.2`、`z_rand 0.5` 的似然面太平，`recovery_alpha_* = 0.0` 又无恢复 ⇒ 拉不回来。
  **A/B（一次只改一个变量：alpha 0.2 → 0.02）**：AMCL 位置中位 0.256 → **0.070 m**。
  **回到验收**：`reports/p1-n-nav-07` 真值到站 **0.230 m**（限 0.25 m，阈值一行未改）⇒ 判决
  **PASS，32/32**；修正前的 FAIL 完整保留在 `reports/p1-n-nav-06`（0.431 m），两份构成 A/B。
  ⚠️ **未证明**：五个 alpha 里哪个起作用；未试"把似然面变尖"（`z_rand`/`max_beams`）这条替代路径；
  **0.02 是按这台自制差速代身推的**，AMR 选型后必须重标。
"""

DECISION = """

## D025 — P1-N-08：先量输入再怀疑估计器；运动噪声必须按本底盘标定

日期：2026-09-20。范围：`P1-N-08`（AMCL 位姿精度）。

**1. 要判"谁产生的误差"，必须把估计器和它消费的输入放在同一次运行里比。** 本轮差点去查车轮：
AMCL 沿轨迹超前真值 0.234 m，一个很自然的推断是"它忠实地传播了一份超前的里程计"。
真正做的是两件事：观察者开始记录**整条里程计轨迹**（首末点不够），以及把规划器与控制器
从环里拿掉、只留 AMCL + 已知运动。结论是同一次运行内 AMCL 0.256 m vs 里程计 0.005 m，
差 47.6 倍 ⇒ **误差在 AMCL 内部产生**。⇒ 判据与观察者都按这条改：`odom_samples` 常驻，
`analyse_n_amcl.py` 同时给两个来源的分解。

**2. `alpha1..5` 从 0.2 改成 0.02，理由是本底盘的实测里程计精度。** nav2 的 stock 值描述的是一台
里程计有 ~10% 行程噪声的真实机器人；这台底盘是刚性代身、速度源理想、无打滑模型，实测
**0.005 m / 3.7 m**。于是每次 0.25 m 的更新注入 σ≈0.05 m 的模型噪声，15 次更新约 0.19 m 随机游走，
而本场地的似然面（墙 4 m 外、`sigma_hit 0.2`、`z_rand 0.5`，四者之和还是 1.10）太平，拉不回来，
`recovery_alpha_* = 0.0` 也没有恢复。A/B 实测 0.256 → 0.070 m，到站 0.431 → 0.230 m。
⚠️ **这个值是按代身底盘推的，不是通用值**：AMR 选型后必须重标（`P1-ENV-01`）。
⚠️ 五个 alpha 里哪一个起作用**没有测**；"把似然面变尖"这条替代路径**没有试** ——
机制上"漂移源"与"纠不回来"两个因素都存在，本轮只动了前者。

**3. 修正后重跑验收，**且判据与阈值一行未改**（0.25 m 的限值是 `evaluate_n_nav.py` 的声明默认值，
`p1-n-nav-06` 就是拿它判 FAIL 的）。** 两份报告构成 A/B：`-06` FAIL（0.431 m）/ `-07` PASS（0.230 m）。
不重跑就等于拿旧配置的判决配新配置的代码。

**4. 只测定位的实验需要"等观察者就绪再动"。** `--bridge-command-source profile` 原本从 sim 0 秒
就开始跑，而 Nav2 要 ~20 s 才起来；第一次这样跑时车已开出 1 m，而 `set_initial_pose` 声明的
(0,0,0) 已经过期，且这台 AMCL `recovery_alpha_fast/slow = 0.0` **完全没有恢复机制**，
残留误差 2.31 m / 2.83 rad。⇒ 新增 `probe_n_ros.py --profile-start-sim-s`（默认 0，
不改 P1-N-06 行为）与编排器 `--observe-sim-s`。**声明的初始位姿只在车还没动时成立。**

**5. 补丁脚本的幂等判定以指纹为准，不以锚点为准。** 后期补丁会重写前期补丁引入的行，
于是"锚点数量不等于 1"既可能是"还没打"也可能是"已被取代"；本轮因此让一条新补丁**静默没落地**
（桥接收到不认识的参数秒退，而编排器照样开仿真、白跑 30 s 且看起来像场景失败）。
现在 `EXPECTED_SHA256` 是唯一仲裁者，且编排器发现桥接没进图时**中止并记录桥接退出码**。
"""


def replace_line_with_prefix(path, prefix, new_line, failures):
    lines = path.read_text(encoding='utf-8').splitlines(keepends=True)
    if any(line.startswith(new_line[:40]) for line in lines):
        return False
    hits = [index for index, line in enumerate(lines) if line.startswith(prefix)]
    if len(hits) != 1:
        failures.append(f'{path.name}: {len(hits)} lines start with {prefix!r}, expected 1')
        return False
    lines[hits[0]] = new_line if new_line.endswith('\n') else new_line + '\n'
    path.write_text(''.join(lines), encoding='utf-8')
    return True


def insert_before_prefix(path, prefix, block, failures):
    lines = path.read_text(encoding='utf-8').splitlines(keepends=True)
    marker = block.splitlines()[0]
    if any(line.startswith(marker[:40]) for line in lines):
        return False
    hits = [index for index, line in enumerate(lines) if line.startswith(prefix)]
    if len(hits) != 1:
        failures.append(f'{path.name}: {len(hits)} lines start with {prefix!r}, expected 1')
        return False
    text = ''.join(lines[:hits[0]]) + block.rstrip('\n') + '\n' + ''.join(lines[hits[0]:])
    path.write_text(text, encoding='utf-8')
    return True


def append(path, block):
    text = path.read_text(encoding='utf-8')
    if block.strip()[:60] in text:
        return False
    path.write_text(text.rstrip('\n') + '\n' + block, encoding='utf-8')
    return True


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--check', action='store_true')
    parser.add_argument('--root', default=str(ROOT))
    args = parser.parse_args()
    root = Path(args.root)
    failures, changes = [], []

    def record(result, label):
        if result:
            changes.append(label)

    if args.check:
        # check mode only asserts that the patched text is present
        for relative, marker in (('TASK_BOARD.md', N08_ROW[:40]),
                                 ('IMPLEMENTATION_STATUS.md', STATUS_BULLET[:40]),
                                 ('DECISIONS.md', DECISION.strip()[:40])):
            text = (root / 'docs' / relative).read_text(encoding='utf-8')
            if marker not in text:
                failures.append(f'{relative}: patched text missing')
        if failures:
            for failure in failures:
                print(f'[FAIL] {failure}', flush=True)
            return 1
        print('[OK] documentation patches present', flush=True)
        return 0

    board = root / 'docs' / 'TASK_BOARD.md'
    record(replace_line_with_prefix(board, '| P1-N-07 |', N07_ROW, failures),
           'TASK_BOARD: P1-N-07 row')
    record(replace_line_with_prefix(board, '| P1-N-08 |', N08_ROW, failures),
           'TASK_BOARD: P1-N-08 row')
    record(replace_line_with_prefix(board, '**N 的整链', HEADER_NOTE, failures),
           'TASK_BOARD: header note')

    status = root / 'docs' / 'IMPLEMENTATION_STATUS.md'
    record(replace_line_with_prefix(
        status, '更新：2026-09-20（P1-N-0',
        '更新：2026-09-20（P1-N-08）。阶段 P1，N 的同世界轮式导航已达到验收，未进入 P2。\n',
        failures), 'IMPLEMENTATION_STATUS: date')
    record(insert_before_prefix(status, '- **P1-H-04：退出阶段改为两段式', STATUS_BULLET,
                                failures), 'IMPLEMENTATION_STATUS: P1-N-08 bullet')
    record(replace_line_with_prefix(
        status, '1. **`P1-N-08`（AMCL 位姿精度）**',
        '1. **`P1-N-07` 的重复性**：只跑了一个目标点。换目标点、换起点、换 seed，'
        '才有"扰动下稳定"的资格；`probe_n_nav.py` 与 `evaluate_n_nav.py` 已经支持换目标。\n',
        failures), 'IMPLEMENTATION_STATUS: next step')

    feasibility = root / 'docs' / 'P1_FEASIBILITY.md'
    record(replace_line_with_prefix(
        feasibility, '| **单车 Nav2 整链（',
        '| **单车 Nav2 整链 + 到站验收** | **通过**：真值到站 **0.230 m**（限 0.25 m）、'
        '航向 0.112 rad、32 项判据 **32 PASS / 0 FAIL**；AMCL 位置中位 **0.064 m**、'
        '轮式里程计 0.0719 m；绕 `pillar_a` 最近 0.541 m；末段速度 0.0000 m/s | '
        '`reports/p1-n-nav-07`。修正前的 FAIL 记录保留在 `p1-n-nav-06`（0.431 m）。'
        '**地图是世界几何先验、不是 SLAM**；单目标点、无重复性；底盘不是 AMR 选型 |\n',
        failures), 'P1_FEASIBILITY: N table row')
    record(replace_line_with_prefix(
        feasibility, '**仍未做**：AMCL 位姿精度（',
        '**仍未做**：重复性（只有一个目标点）、建图（地图是几何先验）、位姿扰动、'
        '严格意义的单命令写者（`/cmd_vel` 上仍有三个发布者）。'
        '**AMCL 的运动噪声 `alpha1..5` 已按本代身底盘的实测里程计精度标为 0.02**'
        '（`reports/p1-n-08-loc-03/04`），**AMR 选型后必须重标**。\n',
        failures), 'P1_FEASIBILITY: still-missing paragraph')
    record(append(root / 'docs' / 'DECISIONS.md', DECISION), 'DECISIONS: D025')

    for item in changes:
        print(f'  patched: {item}')
    if not changes:
        print('  nothing to do (already patched)')
    if failures:
        for failure in failures:
            print(f'[FAIL] {failure}', flush=True)
        return 1
    for name in ('TASK_BOARD.md', 'IMPLEMENTATION_STATUS.md', 'P1_FEASIBILITY.md',
                 'DECISIONS.md'):
        digest = hashlib.sha256((root / 'docs' / name).read_bytes()).hexdigest()
        print(f'{name}: sha256 {digest[:16]}')
    print('[OK] documentation patched', flush=True)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
