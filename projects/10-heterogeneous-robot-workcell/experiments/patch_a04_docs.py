"""Register the P1-A-04 model survey in the 010 project documents.

Each patch is anchored on literal text that must appear exactly once. If an anchor
does not match, the script refuses to write anything and exits non-zero, so a
silently-skipped patch cannot happen. --check reports pending work without writing.
"""
import argparse
import hashlib
import pathlib

ROOT = pathlib.Path(__file__).resolve().parents[1]

TASK_BOARD_ROW_OLD = '| P1-A-04 | READY | SIM-02 | 尚未实施，先选模型和夹爪 |'

TASK_BOARD_ROW_NEW = (
    '| P1-A-04 | PARTIAL | SIM-02 | **模型侦察已完成（只读，未下载任何东西）**：'
    '**夹爪已具备**（`assets/allegro/`，Allegro Hand V3，BSD-2-Clause，逐文件 sha256 在 `assets/manifest.json`）；'
    '**本机不存在任何独立工业臂模型** —— 按名称检索 `~/projects` 得 0 个，`~/.mujoco` 不存在，'
    'Menagerie 缓存（005/006/007）只含 `wonik_allegro`。T800 的臂只有 **5 DoF 且无腕**（`J13–J17`），'
    '当“固定臂”需用户先确认范围。候选 `franka_emika_panda`（9 DoF，Apache-2.0）/ '
    '`universal_robots_ur5e`（6 DoF，BSD-3-Clause），**下载需授权**。'
    '证据 `docs/history/P1_A_MODEL_SURVEY.md` |'
)

TASK_BOARD_HEAD_OLD = (
    '**H 的六阶段序列现在在 V1 托盘上全部通过（`full_H_acceptance = DIAGNOSTIC_PASS`，reports/p1-h-seq-08）。** '
    '范围仍是 DIAGNOSTIC_ONLY（无视觉、无位姿扰动），不等于 V1 验收。'
)

TASK_BOARD_HEAD_NEW = TASK_BOARD_HEAD_OLD + (
    '\n**A 的模型侦察已完成（`P1-A-04` PARTIAL）：夹爪已具备，独立臂本机不存在，'
    '卡在模型选型这一个用户决策上（`DECISIONS.md` D026）；下载须授权。** '
    '见 `docs/history/P1_A_MODEL_SURVEY.md`。'
)

HANDOFF_HEAD_OLD = (
    '# P1 交接 — 2026-09-20（P1-N-07 / P1-N-08 之后）'
)

HANDOFF_HEAD_NEW = (
    '# P1 交接 — 2026-09-20（P1-A-04 模型侦察之后）'
)

HANDOFF_NEXT_OLD = (
    '当前作者：本任务 AI，交付后释放写入权。ACTIVE_TASK：NONE。\n'
    '下一任务：**`P1-N-07` 的重复性（换目标点/起点/seed）**，或用户指定的 A / C。'
)

HANDOFF_NEXT_NEW = (
    '当前作者：本任务 AI，交付后释放写入权。ACTIVE_TASK：NONE（`P1-A-04` 侦察已收尾）。\n'
    '下一任务：**`P1-A-04` 卡在一个用户决策上** —— 见 `docs/DECISIONS.md` D026：\n'
    '  ① 授权下载 `franka_emika_panda`（推荐，Apache-2.0，9 DoF，臂+夹爪一体）；或\n'
    '  ② 授权下载 `universal_robots_ur5e`（BSD-3-Clause，6 DoF）+ 复用已有 Allegro；或\n'
    '  ③ 确认“V1 的固定臂 = T800 臂”（5 DoF、无腕，且与人形同平台）。\n'
    '**不依赖该决策、也不需要任何授权的下一轮是 `P1-C-05`（车载接驳）**：'
    '车体支撑、保持机构、间隙与偏航、对接互锁全部可用现有滚筒与底盘资产做，无需新模型。\n'
    '**`P1-N-07` 的重复性优先级最低** —— 它不挡 `P1-GATE-07`，且那个数字不可迁移（D025）。'
)

FEAS_A_OLD = (
    '先核查可用模型/夹爪来源与许可，再建立独立副本。不得用移动方块替代关节机械臂。\n'
    '在相同引擎验证一个分离零件的夹持、离台、放置和松手稳定；规划 attach 不代表真实夹持。\n'
    '若缺模型，先报告下载体积/许可再申请大型资源下载，不暗装新平台。'
)

FEAS_A_NEW = FEAS_A_OLD + (
    '\n\n**侦察已完成（只读，未下载）。** 夹爪侧已具备：`assets/allegro/`（Allegro Hand V3，'
    'BSD-2-Clause，左右手各 16 关节，逐文件 sha256 在 `assets/manifest.json`）。'
    '臂侧**本机不存在**：按名称检索 `~/projects` 得 0 个独立臂模型，`~/.mujoco` 不存在，'
    'Menagerie 缓存（005/006/007，各 5.6 MB）只含 `wonik_allegro`；ROS 侧也没有臂的 description 包。\n'
    '**T800 的臂不作为默认选项**：实测每条臂只有 5 个自由度（`J13–J17` / `J18–J22`）且**无腕关节** '
    '—— Allegro 是替换腕位占位球直接装在肘后的；加上它与人形同平台，会削弱 '
    '“四实验各自独立”的门意义。\n'
    '**提议**：稀疏检出只取 `franka_emika_panda`（9 DoF，Apache-2.0，上游要求 MuJoCo ≥ 2.3.3；'
    '本机 3.3.6 满足）或 `universal_robots_ur5e`（6 DoF，BSD-3-Clause）+ 已有 Allegro。'
    '**下载需用户授权**（`D026`）。证据 `docs/history/P1_A_MODEL_SURVEY.md`。'
)

D026 = (
    '\n\n## D026 — P1-A-04：A 的臂/夹爪模型选型（**待用户确认**）\n'
    '\n'
    '日期：2026-09-20。范围：`P1-A-04`（固定机械臂）。状态：**提议，未决**。\n'
    '\n'
    '**1. 侦察结论。** 夹爪侧已具备：`assets/allegro/`（Allegro Hand V3，BSD-2-Clause，16 关节，'
    '逐文件 sha256 已在 `assets/manifest.json`）。臂侧**本机不存在**：按名称检索 `~/projects` 得 '
    '0 个独立臂模型，`~/.mujoco` 不存在，Menagerie 缓存（005/006/007）只含 `wonik_allegro`。'
    '证据 `docs/history/P1_A_MODEL_SURVEY.md`。\n'
    '\n'
    '**2. T800 的臂不作为默认选项。** 实测每条臂只有 5 个自由度（`J13–J17` / `J18–J22`：肩 3 + 肘 2）'
    '且**没有腕关节** —— `assets/manifest.json` 记着 Allegro 是**替换腕位占位球**直接装在肘后。'
    '它与人形同平台，而 `MASTER_PLAN.md` §1 的 V1 是“1 人形 + 1 固定臂”两个设备；'
    '混用会让 `P1-GATE-07`“四实验各自独立”的意义变弱。**可用，但需用户明确范围解释。**\n'
    '\n'
    '**3. 提议选 `franka_emika_panda`**（9 DoF，**Apache-2.0**，上游要求 MuJoCo ≥ 2.3.3；本机 3.3.6 满足）。'
    '理由：臂与平行夹爪一体，正对“把简单零件放进托盘料位”这一任务；许可与本工程现有 '
    'EngineAI BSD-3 / Allegro BSD-2 / 集成代码 Apache-2.0 全部兼容；是 MuJoCo 里参考最多的抓放臂。'
    '备选 `universal_robots_ur5e`（6 DoF，BSD-3-Clause）+ **已有 Allegro**，代价是自建法兰转接件。\n'
    '\n'
    '**4. 下载需要用户授权**（`AGENTS.md`：大模型/大型资源下载须明确授权）。计划用稀疏检出只取单个'
    '模型目录（`git clone --depth 1 --filter=blob:none --sparse` 后 `git sparse-checkout set <model>`），'
    '体积为数 MB 量级（二手来源，**实测确认后再申请**）。不整克隆、不装新平台。\n'
    '\n'
    '**5. 若既不授权下载、也不接受 T800 臂**，则 `P1-A-04` 保持 PARTIAL，先做 `P1-C-05` —— '
    '车载接驳只用现有滚筒与底盘资产，不需要任何新模型。\n'
)

PATCHES = [
    ('docs/TASK_BOARD.md', TASK_BOARD_HEAD_OLD, TASK_BOARD_HEAD_NEW, 'A 状态写到表头'),
    ('docs/TASK_BOARD.md', TASK_BOARD_ROW_OLD, TASK_BOARD_ROW_NEW, 'P1-A-04 行'),
    ('docs/HANDOFF.md', HANDOFF_HEAD_OLD, HANDOFF_HEAD_NEW, '交接标题'),
    ('docs/HANDOFF.md', HANDOFF_NEXT_OLD, HANDOFF_NEXT_NEW, '下一任务与授权点'),
    ('docs/P1_FEASIBILITY.md', FEAS_A_OLD, FEAS_A_NEW, 'A 小节侦察结论'),
]


def sha(text):
    return hashlib.sha256(text.encode('utf-8')).hexdigest()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--check', action='store_true')
    parser.add_argument('--root', default=str(ROOT))
    args = parser.parse_args()
    root = pathlib.Path(args.root)

    failures = []
    touched = {}

    for relative, old, new, label in PATCHES:
        path = root / relative
        if not path.is_file():
            failures.append(f'{relative}: file missing')
            continue
        text = touched.get(relative) or path.read_text(encoding='utf-8')
        before = sha(text)
        if new in text:
            print(f'{relative}: [{label}] already applied')
            touched[relative] = text
            continue
        count = text.count(old)
        if count != 1:
            failures.append(f'{relative}: [{label}] anchor appears {count} time(s), expected 1')
            touched[relative] = text
            continue
        if args.check:
            failures.append(f'{relative}: [{label}] pending')
            touched[relative] = text
            continue
        text = text.replace(old, new)
        touched[relative] = text
        print(f'{relative}: [{label}] applied  sha256 {before[:12]} -> {sha(text)[:12]}')

    decisions = root / 'docs' / 'DECISIONS.md'
    if decisions.is_file():
        text = decisions.read_text(encoding='utf-8')
        if '## D026' in text:
            print('docs/DECISIONS.md: [D026] already present')
        elif args.check:
            failures.append('docs/DECISIONS.md: [D026] pending')
        else:
            text = text.rstrip('\n') + D026
            touched['docs/DECISIONS.md'] = text
            print(f'docs/DECISIONS.md: [D026] appended')
    else:
        failures.append('docs/DECISIONS.md: file missing')

    if failures:
        for item in failures:
            print(f'[FAIL] {item}', flush=True)
        return 1

    if not args.check:
        for relative, text in touched.items():
            (root / relative).write_text(text, encoding='utf-8')

    print('[OK] P1-A-04 registration complete', flush=True)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
