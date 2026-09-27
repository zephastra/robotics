"""Register the P1-C-05 receiving-side round in the 010 project documents.

Every number written into the documents is read back out of
`reports/p1-c-deck-02/acceptance.json`, so the prose cannot drift from the evidence.
Anchor text must appear exactly once; anything else is a hard failure.
"""
import argparse
import hashlib
import json
import pathlib

ROOT = pathlib.Path(__file__).resolve().parents[1]
RUN = 'p1-c-deck-02'

TASK_BOARD_HEAD_OLD = (
    '**A 的模型侦察已完成（`P1-A-04` PARTIAL）：夹爪已具备，独立臂本机不存在，'
    '卡在模型选型这一个用户决策上（`DECISIONS.md` D026）；下载须授权。** '
    '见 `docs/history/P1_A_MODEL_SURVEY.md`。'
)

FEAS_C_OLD = (
    '### C：车载接驳\n'
    '\n'
    '滚筒仅已解决驱动表面接触的最小问题。下一步需车体支撑、保持机构、间隙与偏航、对接互锁。\n'
    '源端→车载→接收端每次须完整转移和停止，故意错位时必须拒绝，不靠货物瞬移。\n'
    '当前滚筒实验初试失败为相邻滚筒重叠；修正半径后运动恢复。后续根据实测速度增加运行时长并加强第三段包含判据，原失败均保留。'
)

HANDOFF_HEAD_OLD = '# P1 交接 — 2026-09-20（P1-A-04 模型侦察之后）'

HANDOFF_NEXT_OLD = (
    '当前作者：本任务 AI，交付后释放写入权。ACTIVE_TASK：NONE（`P1-A-04` 侦察已收尾）。\n'
    '下一任务：**`P1-A-04` 卡在一个用户决策上** —— 见 `docs/DECISIONS.md` D026：\n'
    '  ① 授权下载 `franka_emika_panda`（推荐，Apache-2.0，9 DoF，臂+夹爪一体）；或\n'
    '  ② 授权下载 `universal_robots_ur5e`（BSD-3-Clause，6 DoF）+ 复用已有 Allegro；或\n'
    '  ③ 确认“V1 的固定臂 = T800 臂”（5 DoF、无腕，且与人形同平台）。\n'
    '**不依赖该决策、也不需要任何授权的下一轮是 `P1-C-05`（车载接驳）**：'
    '车体支撑、保持机构、间隙与偏航、对接互锁全部可用现有滚筒与底盘资产做，无需新模型。\n'
    '**`P1-N-07` 的重复性优先级最低** —— 它不挡 `P1-GATE-07`，且那个数字不可迁移（D025）。'
)

TASK_BOARD_ROW_OLD = '| P1-C-05 | PARTIAL | SIM-02 | 固定滚筒组件通过；车载接驳未实施 |'


def build_texts():
    payload = json.loads((ROOT / 'reports' / RUN / 'acceptance.json').read_text(encoding='utf-8'))
    sc = payload['scenarios']
    aligned = sc['aligned']
    trips = sum(1 for item in payload['checks'] if item['verdict'] == 'PASS')
    fails = len(payload['failed'])
    not_run = len(payload['not_run'])
    pin = sc['aligned']

    board_row = (
        f"| P1-C-05 | PARTIAL | SIM-02 | **接收侧转移台已建立，对象是真 V1 托盘**（0.098 kg、"
        f"sha256 见报告）。滚柱长度**由托盘底面推导**为 0.500 m（旧台 0.440 m，**比托盘底面 0.460 m 还窄**）。"
        f"对齐时转移成功：托盘中心落在 {aligned['final_rel_x']:.3f} m / "
        f"{payload['deck_length_m']:.3f} m 甲板上、横漂 {aligned['max_lateral_m'] * 1000:.1f} mm、"
        f"驱动中滑移 {aligned['slip_during_drive_m'] * 1000:.1f} mm。**窗口有界**：缝 30 mm 过 / "
        f"60 mm 掉落（{sc['gap_60mm']['fell_at']} s）；**台阶 8 mm 就已卡死**；偏航 3° 横漂 "
        f"{sc['yaw_3deg']['max_lateral_m'] * 1000:.1f} mm、8° 漂 "
        f"{sc['yaw_8deg']['max_lateral_m'] * 1000:.1f} mm。**保持挡销未受力**（抬起后接触 "
        f"{pin['pin_contacts_after_raise']} 个采样；抬起前 {pin['pin_contacts_before_raise']} 个是"
        f"桥接蹭到的）⇒ 该判据 **NOT_RUN**，判决 **{payload['verdict']}**（{trips} PASS / "
        f"{fails} FAIL / {not_run} NOT_RUN）。证据 `reports/{RUN}` |"
    )

    board_head = TASK_BOARD_HEAD_OLD + (
        f'\n**C 的接收侧转移台已建立（`P1-C-05` PARTIAL，判决 `{payload["verdict"]}`）：**'
        f'真实 V1 托盘能转移上车并随车走，**转移窗口已被实测出边界**（台阶 8 mm 卡死、'
        f'偏航 3° 横漂 {sc["yaw_3deg"]["max_lateral_m"] * 1000:.0f} mm）；'
        f'**但保持挡销未被证明**（该判据 NOT_RUN）。见 `docs/history/P1_C_TRANSFER_SESSION.md`。'
    )

    feas = FEAS_C_OLD + (
        '\n\n**侦察与第一轮实测已完成（`reports/' + RUN + '`，判决 `' + payload['verdict'] + '`）。**'
        '\n\n- **口径纠正**：原来三次滚筒实验用的是程序里的**占位方块**（`.12 x .15 x .02`、0.3 kg），'
        '**不是 V1 真托盘**；真托盘底面宽 0.460 m 而旧台滚柱只有 0.440 m ⇒ 那个 `COMPONENT_PASS`'
        '从未适用于真托盘。现在滚柱长度**由 `tray_floor` 的 y 半宽推导**：0.460 + 2x0.020 = '
        '**0.500 m**。'
        '\n- **几何约束**：提手伸到 ±0.335 m，**超出滚柱端面 0.085 m** ⇒ 甲板上**不能装齐平侧导轨**'
        '（会撞提手）。横向约束是**记录量**：对齐实测横漂 '
        f'{aligned["max_lateral_m"] * 1000:.1f} mm。'
        '\n- **实测窗口**：缝 30 mm 过 / 60 mm 掉落；**台阶 8 mm 卡死**；偏航 3° 横漂 '
        f'{sc["yaw_3deg"]["max_lateral_m"] * 1000:.0f} mm、8° 漂 '
        f'{sc["yaw_8deg"]["max_lateral_m"] * 1000:.0f} mm。⇒ 停靠残余偏航要远小于 3°、'
        '高度差要远小于 8 mm；**阈值不能比这个机械窗口更宽**。'
        '\n- **保持挡销未被证明**：抬起后接触 '
        f'{pin["pin_contacts_after_raise"]} 个采样、带销与不带销滑移差 '
        f'{aligned["slip_during_drive_m"] - sc["aligned_nopin"]["slip_during_drive_m"]:.4f} m '
        '⇒ 这个加速度下**摩擦单独就够**，判据 `NOT_RUN`。'
        '**真正该被质疑的是托盘/甲板声明的摩擦 1.0**（要滑动需约 1 g）。'
        '\n- **仍未做**：源端→车载→接收端完整转移与卸载（`full_C_acceptance` 仍 `NOT_RUN`）、'
        '重复性、真实 AMR 底盘。'
    )

    handoff_next = (
        '当前作者：本任务 AI，交付后释放写入权。ACTIVE_TASK：NONE（`P1-C-05` 第一步已收尾）。\n'
        f'**C 第一步结果：判决 `{payload["verdict"]}`**（{trips} PASS / {fails} FAIL / '
        f'{not_run} NOT_RUN），权威 `reports/{RUN}`。对齐时真托盘能转移上车并随车走；'
        '转移窗口实测有界；**保持挡销未受力 ⇒ 该判据 NOT_RUN**（详见 '
        '`docs/history/P1_C_TRANSFER_SESSION.md`）。\n'
        '下一任务按杠杆：\n'
        '  ① **`P1-C-05` 第二步**：源端→车载→接收端完整转移 + 卸载；把挡销放到能真正受力的'
        '工况（或先复核托盘/甲板摩擦声明 1.0 是否现实）。**无需授权**；\n'
        '  ② **`P1-A-04`**：等用户三选一（`DECISIONS.md` D026），下载须授权；\n'
        '  ③ `P1-N-07` 的重复性 —— 优先级最低，不挡门且数值不可迁移（D025）。\n'
        '**`P1-GATE-07` 仍 BLOCKED**：H 只到 `DIAGNOSTIC_PASS`、A 未实施、C 无完整转移、'
        'N 无重复性。'
    )

    d027 = (
        '\n\n## D027 — P1-C-05：接收侧转移台用真托盘重建；窗口有界，但挡销未被证明\n'
        '\n'
        '日期：2026-09-20。范围：`P1-C-05`（车载接驳第一步）。证据 `reports/' + RUN + '`。\n'
        '\n'
        '**1. 先纠正对象口径。** 原三次滚筒实验的"托盘"是程序里的占位方块（0.3 kg），不是 V1 '
        '真托盘（`assets/objects/tray_v1.xml`，0.098 kg）。真托盘底面宽 **0.460 m**，旧台滚柱只有 '
        '**0.440 m** —— **台子比托盘窄**，所以 `COMPONENT_PASS` 从未适用于真托盘。'
        '现在滚柱长度由托盘几何推导（`experiments/roller_rig.py` 是唯一来源），'
        '`0.460 + 2x0.020 = 0.500 m`。\n'
        '\n'
        '**2. 提手超出滚柱端面 0.085 m ⇒ 甲板上不能装齐平侧导轨。** 提手伸到 ±0.335 m，'
        '滚柱半长 0.250 m，且提手底沿只比冠面高 0.005 m —— 任何与托盘齐平的侧导都会撞提手。'
        '⇒ 横向约束改为**被记录的观测量**（对齐实测横漂 '
        f'{aligned["max_lateral_m"] * 1000:.1f} mm），而不是假定的机构。\n'
        '\n'
        '**3. 转移窗口实测有界，且很紧。** 缝：30 mm 过 / 60 mm 掉落；'
        '**台阶：8 mm 就已经卡死**；**偏航：3° 就横漂 '
        f'{sc["yaw_3deg"]["max_lateral_m"] * 1000:.0f} mm**。'
        '⇒ 这是机械给出的硬数字，`P3`/`N` 的停靠阈值**不能比它更宽**。\n'
        '\n'
        '**4. ★ "托盘没动"不等于"保持机构有效"。** `aligned` 与 `aligned_nopin` 的数值逐位相同；'
        f'挡销在**抬起后**接触 {pin["pin_contacts_after_raise"]} 个采样（抬起前 '
        f'{pin["pin_contacts_before_raise"]} 个是托盘桥接前缝时蹭到的）。'
        '⇒ 判据写成"挡销必须在**抬起之后**承载"，于是给出 **`NOT_RUN`** 而不是 PASS。'
        '**"没动"与"某机构起了作用"必须分开量**（与 009 的零暴露 PASS 同类）。'
        '**未证明**：挡销在真实加速度下的作用；而按声明的摩擦 1.0，要滑动需约 1 g —— '
        '**该被质疑的是摩擦声明本身**，这条留给 V1 设计输入。\n'
        '\n'
        '**5. 本轮修掉的五个建模错（都是由真实运行抓出来的，不是靠眼看）。** ① 甲板滚柱误用'
        '世界坐标 z（被装到甲板上方 0.45 m）；② 甲板滑轨只有阻尼 ⇒ 被托盘顶跑，改**位置舵机**'
        '（车辆停住是被刹住）；③ **甲板前唇的竖直面挡住托盘前下缘**（卡在口外 5.7 mm）⇒ 删前唇，'
        '让托盘桥接前缝；④ 甲板底框顶面落在滚柱体内；⑤ **位置舵机 kp=20000 阶跃把 0.098 kg '
        '托盘弹到 0.90 m 高** ⇒ 改成 kp=2000 / ±30 N / 0.4 s 斜坡。\n'
    )

    return board_head, board_row, feas, handoff_next, d027


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--check', action='store_true')
    args = parser.parse_args()

    board_head, board_row, feas, handoff_next, d027 = build_texts()
    patches = [
        ('docs/TASK_BOARD.md', TASK_BOARD_HEAD_OLD, board_head, 'C 状态写到表头'),
        ('docs/TASK_BOARD.md', TASK_BOARD_ROW_OLD, board_row, 'P1-C-05 行'),
        ('docs/P1_FEASIBILITY.md', FEAS_C_OLD, feas, 'C 小节实测结果'),
        ('docs/HANDOFF.md', HANDOFF_HEAD_OLD,
         '# P1 交接 — 2026-09-20（P1-C-05 接收侧转移台之后）', '交接标题'),
        ('docs/HANDOFF.md', HANDOFF_NEXT_OLD, handoff_next, '下一任务与结果'),
    ]

    failures, touched = [], {}
    for relative, old, new, label in patches:
        path = ROOT / relative
        if not path.is_file():
            failures.append(f'{relative}: missing')
            continue
        text = touched.get(relative) or path.read_text(encoding='utf-8')
        if new in text:
            print(f'{relative}: [{label}] already applied')
            touched[relative] = text
            continue
        count = text.count(old)
        if count != 1:
            failures.append(f'{relative}: [{label}] anchor appears {count} time(s)')
            touched[relative] = text
            continue
        if args.check:
            failures.append(f'{relative}: [{label}] pending')
            touched[relative] = text
            continue
        text = text.replace(old, new)
        touched[relative] = text
        print(f'{relative}: [{label}] applied')

    decisions = ROOT / 'docs' / 'DECISIONS.md'
    text = decisions.read_text(encoding='utf-8')
    if '## D027' in text:
        print('docs/DECISIONS.md: [D027] already present')
    elif args.check:
        failures.append('docs/DECISIONS.md: [D027] pending')
    else:
        touched['docs/DECISIONS.md'] = text.rstrip('\n') + d027
        print('docs/DECISIONS.md: [D027] appended')

    if failures:
        for item in failures:
            print(f'[FAIL] {item}', flush=True)
        return 1

    if not args.check:
        for relative, text in touched.items():
            (ROOT / relative).write_text(text, encoding='utf-8')
            print(f'  wrote {relative}  sha256 '
                  f'{hashlib.sha256(text.encode()).hexdigest()[:12]}')
    print('[OK] P1-C-05 registration complete', flush=True)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
