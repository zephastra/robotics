"""Append note (19) to the P4-HUMAN-01 board row and move its status to DONE.

A script, not a text edit: the row is a single 13,530-character table line containing Chinese,
circled numerals and escaped pipes. It ASSERTS the structure it expects before writing, because a
structural edit to a markdown table has silently corrupted this board twice before (the project's
own lesson: assert the SHAPE -- escaping, cell count -- not just that the string changed).
"""
import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent
DOC = ROOT / 'docs' / 'TASK_BOARD.md'
TARGET = '| P4-HUMAN-01 |'

NOTE = (
    ' ⑲ **第二十一轮：H3 完成 —— H 链接进了 W5 物流链（同一个世界、同一条时间线、同一个托盘）。**'
    '权威证据 **`reports/p4-h3-w5-01/`**（正例）与 **`reports/p4-h3-w5-neg-01/`**（集成负例）。'
    '★ **世界**：`experiments/w5_h085_plan.py`（audit-then-apply、`--check` 幂等、**先编译再写入**）'
    '→ `assets/world_w5_h085_loop.xml`（sha `f964477f…`）；相对候选 B **只差 1 行**'
    '（`h_LINK_BASE` 的 `pos`）＋ 1 行与候选 B **共同拥有**（`<key name="home">`），'
    '`nq/nbody/ngeom = 151/149/309` **逐项与候选 B 相同**，**候选 B 逐位未变（`702e22a6…`）**。'
    '★★ **发现候选 B 从来没有把人形放进去**（`h_LINK_BASE` 仍是 **0.660132**，而工装在 4.31）——'
    'H2 是把站位**写在探针里**（一次初始化 `qpos[0]=4.13`）。对单探针合法，对 H3 是错的形状：'
    '人形运行时与 `LogisticsPlant` **共用一个 `MjData`**。★ 站位**仍然是 4.13**：'
    '两个"要挪站位"的读数为**假** —— ① `mj_geomDistance` 报"踝在 `n_chassis` 内 15.6 mm"'
    '其实是 `contype=0` 的**视觉网格**（同一机器人在 home 位姿有 **272 对自穿透**，含'
    '`h_LINK_BASE` 与自身 **−0.16740 m**，与 `D078` 同族）；② 自由放松说"每个站位都摔"'
    '——站立律**保姿势不保平衡**。**决定性测量是让机器人自己的控制器跑起来的接触测试**：'
    '四个站位下交叉实体接触**都只有 `world/cell_ground`**。**换尺不换阈值，而这次尺错的方向是凭空造碰撞。**'
    '★ **探针**：`experiments/probe_h3_w5.py`；**复用** H1 的仪器与 H2 的阶段驱动、'
    '`tray_task` **同一套冻结阈值**。★★ **本轮抓出我自己的四个缺陷**：'
    '① **palnt 把几何从"不是 home 的状态"推导** —— 建 plant 时它的 `MjData` 还是零 qpos，'
    '于是 `dock_residuals(station_c)` 报 **−4.608 m** 而车其实**离目标 2.9 mm 且 `travelled_m=0`**'
    '（已改为**先置 home 一次**，并加守卫：home 位姿残差 >0.6 m 直接抛错）；'
    '② **人形作业期间车没被刹住** —— 轮子是 `biastype=0` 的**裸电机**，`ctrl=0` 是**零力矩不是刹车**，'
    '车**自己滑了 4.04 m**（4.41372 → 8.45782）⇒ 下游 `NAV_FAILED` 全线塌；已加 plant 的 '
    '`park_control()`（唯一一处"停驻"定义），实测漂移 **0.0000 m**；'
    '③ **`mj_geomDistance` 的 `distmax` 是量程不是上限** —— 把整轮最小距离报成 **0.00000 m**，'
    '而那两个几何**相距 2.0752 m**；量程改 0.5 m 并把"达到量程"作为独立字段上报；'
    '④ **`TransferLedger` 建了却从未驱动** ⇒ **物权转移根本没被记录**（"必录字段从未写入"'
    '换了件衣服）—— 现已按 `probe_w5_loop.py` 的方式真正走 ledger，判据从**账本自己的记录**读。'
    '★ **集成契约四条实测**：一个世界一个 data（`same_model`/`same_data`，`owns_world=False` ⇒ '
    '**不会重置被交来的世界**、全程**不重载**）；同一个托盘实体（`c_payload` 首尾都是 **body 124 / 0.098 kg**）；'
    '**下游等人形松手**（手指 t≈12.4 s 合上、t≈34.4 s 彻底松开、链的第一次运动 t≈51.3 s，'
    '判据是**时间顺序**且要求"松开在合上之后"）；**两条禁令**（托盘等式 **0**；'
    'qpos 写入只有 `calibrate: 3`、**`run` 阶段不存在** —— 且是**结构性的**：guest 的 plant **拒绝 reset**）。'
    '★ **结果**：**六个 H 阶段全过**、**11 条链命令全 `SUCCEEDED`**、托盘 `source_band → deck → receiver_band`'
    '（x 4.448 → 12.595，**链自己搬了 8.15 m**）、两段转运**按契约阶段顺序走到 `RELEASED`、'
    'custody `AT_DESTINATION`**、**异常碰撞 0 对**、退出后最近人形几何距辊道 **0.1247 m**（限 0.050）。'
    '★★ **集成负例**（`--negative-grasp`）**要两处替换，不是一处**：只把手指张开时，'
    '张开的**手会把托盘从工装上"推"到辊道上**（support 变成 `source_band`、托盘 x 4.4869），'
    '链照样把它送到接收端 —— **一个把托盘送到了的"失败"不是失败**；'
    '所以负例同时把搬运目标钉在托盘起始 x（**抓不住东西的手也搬不动东西**）。'
    '负例结果：人形自己的裁判判 **FAIL**（`lift`/`hold` 两阶段）、链的 `START_TRANSFER`/'
    '`VERIFY_TRANSFER`/`VERIFY_DELIVERY` **全部拒绝**、托盘**始终留在源端**。'
    '★ **行必须分臂**：第一版负例用**正例的期望**去判 `chain_reached_receiver`，'
    '把一个"正确拒绝交付"的运行打成 FAIL —— 现在**未被该臂检验的行明确标 `NOT_RUN`**，'
    '**PASS 计数只统计真被检验的行**。'
    '⛔ **不能说**：**"人形把托盘交给了车"**（交付的是**放到源端辊道上**，物权转移是链的 TRANSFER）；'
    '**"人形能走路"**；**"这是生产级工件线"**（`scope=DIAGNOSTIC_ONLY`、无视觉、站位是声明的固定工位）；'
    '**"候选世界已是 W5 基线"**（不能引用旧 W5/W3/W4 的 PASS）；**"H3 覆盖订单 N01"**；'
    '**蹲姿已具备**（`D_MAX = None`，H 链只在**站姿**下成立）。'
)


def main():
    text = DOC.read_text(encoding='utf-8')
    lines = text.split('\n')
    hits = [k for k, line in enumerate(lines) if line.startswith(TARGET)]
    if len(hits) != 1:
        sys.exit('[FAIL] found %d rows starting with %r, expected 1' % (len(hits), TARGET))
    k = hits[0]
    line = lines[k]

    # -- structure, asserted BEFORE anything is written -----------------------------------------
    if '**PARTIAL**' not in line:
        sys.exit('[FAIL] the row does not carry **PARTIAL**; refusing to guess its state')
    if '\u2472' in line:
        sys.exit('[FAIL] note (19) is already present; refusing to append twice')
    if '\u2471' not in line:
        sys.exit('[FAIL] note (18) is missing; the append would be out of order')
    cells = line.split(' | ')
    if len(cells) < 4 or cells[0] != '| P4-HUMAN-01':
        sys.exit('[FAIL] unexpected cell structure: %r' % cells[:2])
    n_cells_before = len(cells)
    n_pipes_before = line.count('|')

    # -- the two edits --------------------------------------------------------------------------
    new_line = line.replace('**PARTIAL**', '**DONE**', 1).rstrip()
    new_line = new_line + NOTE
    if new_line.count('|') != n_pipes_before:
        sys.exit('[FAIL] the note changed the pipe count %d -> %d; a table cell would break'
                 % (n_pipes_before, new_line.count('|')))
    if len(new_line.split(' | ')) != n_cells_before:
        sys.exit('[FAIL] the note changed the cell count %d -> %d'
                 % (n_cells_before, len(new_line.split(' | '))))
    if not new_line.startswith('| P4-HUMAN-01 | **DONE** |'):
        sys.exit('[FAIL] the status cell did not change as intended: %r' % new_line[:60])

    lines[k] = new_line
    DOC.write_text('\n'.join(lines), encoding='utf-8')
    print('row %d: **PARTIAL** -> **DONE**, note (19) appended (%d chars)'
          % (k + 1, len(NOTE)))
    print('pipes %d, cells %d, both unchanged' % (n_pipes_before, n_cells_before))


if __name__ == '__main__':
    main()
