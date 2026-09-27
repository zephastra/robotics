# -*- coding: utf-8 -*-
"""Record round 11: the contact feed-forward reviewed against its own proposal.

Everything is written from `reports/p4-ff2-01/report.json` and the console transcript of the same
run, and the two structural edits (task board row, claims record) assert the SHAPE of what they
produce -- last round a clause landed after a row's closing pipe and made the row 5 cells wide while
my guard only checked that an anchor existed.
"""
import json
import re
from pathlib import Path

ROOT = Path('/home/ziling/projects/010_heterogeneous_robot_workcell')

# --- D094 / D095 / D096 ------------------------------------------------------------------------
D = ROOT / 'docs' / 'DECISIONS.md'
decisions = '''

## D094

**闭链/接触前馈：公式与符号三条独立验证过，但<读实测接触力>的那一版是<当前输出本身>，
不是前馈 —— 实测自证；改用<期望支撑力>后仍然更差，两种接法都更差 ⇒ 不采纳。**
判据 `experiments/probe_p4_ff2.py`；证据 **`reports/p4-ff2-01/`**。**`D_MAX` 不变，仍是 0.20 m。**

- **约定与符号（三条独立验证，其中一条不碰 `qfrc_constraint`）**：
  ① 四种（`frame` × 符号）组合对 `qfrc_constraint`，**`f_world = frame.T @ f_local`、机器人一侧取 `+f`** 胜出；
  ② ★ **独立验证**：`Σ Jᵀ f` 的**六个浮动基座行**必须等于**总接触力旋量** ——
     实测竖直分量 **844.658 N** 对**人形体重 845.706 N（99.88 %）**，**这是牛顿，不是拟合**；
  ③ 缺口是真的且很大：0.20 m 蹲姿下**膝**的自由浮动 `qfrc_bias` 只有 **+12.387 N·m**，
     而静平衡需要 **−92.0 N·m** ⇒ **缺 ~104 N·m，而且已有的那一项连符号都是反的。**
- ⛔ **但评审方案的第一步不能成立，且可测**：在**实测状态**上算 `bias − qfrc_constraint`
  **就是当前已经施加的力矩**。人形自身 25 个受驱动 dof 上，两者**均值差 0.55 N·m、最坏 2.74 N·m**
  （踝 0.001、膝 0.07）。这是**静平衡下的牛顿定律**，不是前馈。
  ⇒ **把实测接触力投影后注入控制环，等于把当前输出加到它自己身上。**
  ★ **负控制臂实测**：`ff=measured` 那条臂前馈峰值打到 **415.0 N·m —— 正好是髋/膝执行器的量程上限**，
  踝打满 160 N·m，脚底 **41.0°**，**10 行里 9 行失守**（0.40 m 甚至失 `ik_converged`）。
- **不循环的那一版（期望支撑力）也没赢**：把"地面要托住体重"当作已知量——
  每只脚 `m·g/2`，沿脚底接触点分布，**每控制步重算**，并按**每个关节自己的申报量程**钳位、计钳位次数。
  | 深度 | ff | 平衡项 | 到达误差 | 脚底 | 踝（L/R） | 失守行 |
  |---|---|---|---|---|---|---|
  | 0.20 | off | 有 | **+14.9 mm** | **1.81°** | 5.11 / 5.41 | **无（10/10）** |
  | 0.20 | desired | 有（叠加） | −3.0 mm | **9.08°** | −29.99 / **+15.96** | `sole_flat`、`no_self_contact` |
  | 0.20 | desired | **换成它** | **+531.1 mm** | **114.48°** | −1.56 / −10.14 | 8 行 |
  | 0.20 | measured | 有 | −46.5 mm | 41.02° | 121.20 / **160.0** | 9 行 |
  | 0.40 | off | 有 | +89.0 mm | 34.51° | −160 / −160 | 全部 |
  | 0.40 | desired | 换成它 | +33.4 mm | 20.11° | −21.54 / −27.63 | 5 行（仍全不过） |
- **可测的机理**：投影给出的踝力矩是**强不对称**的（0.20 m 下 **L −29.99 对 R +15.96 N·m**）
  ⇒ 注进去一个滚转力矩 ⇒ **脚底滚到 9.08°**，把本来 1.81° 的行判失守。
  该不对称来自两只脚接触点几何不同，不是数值噪声。
- ⚠️ **首次运行时 `desired` 的"叠加"和"换成它"两臂<逐位相同>**（−3.0 mm / 9.08° / −29.99、+15.96 全同）——
  因为 `balance_on` 这个开关**被赋值但从未被读取**（死开关）。修好后两臂才分开（换成它 → +531.1 mm）。
  ★ **两条只差一个布尔、数字却逐位相同 —— 这就是开关没接上的签名。**
  现已加**派生检查**：只差 `balance_on` 的两臂若数字相同，报告直接写 `dead_switch_suspects`。
- ⇒ **不采纳**。前馈按<期望支撑力>做对了方向，**但它和踝质心平衡项是同一个物理机制**
  （都在踝上加力矩），叠加是重复计数、替换则失去平衡项，**两种接法都比不做差**。

## D095

**手臂那一侧的真正缺陷是"目标值等于实测值"，不是"缺一个搬盘姿态"。**
- `probe_p4_balance.Rig.solve_posture` 只解**腿**，手臂那些关节的目标值**就是种子＝实测角**
  ⇒ 手臂 PD 的**设定值等于它自己的测量值**，**永远无法纠正任何偏差**（一个不可能失败的律）。
- **搬盘姿态是派生的**：由 `probe_p4_reach` 的 IK 解到托盘的**两个 handle geom**上，
  最坏残差 **0.0005 mm**（10 个臂关节）。
- ⚠️ **效果是混合的，没有被确证为 0.20 m 通过的原因**：
  | 臂 | 0.20 m 到达 | 脚底 | 0.20 m 自交 | 0.40 m 自交 |
  |---|---|---|---|---|
  | 锁搬盘姿 | +14.9 mm | 1.81° | 通过 | — |
  | 不锁（改前行为） | **+15.4 mm** | — | 通过（瞬态 −0.180 mm，**已消失**） | 失守（静置 −2.141 mm） |
  改前行为在 0.20 m 的到达误差 **+15.4 mm 与第九轮逐位相同** ⇒ 一致性成立，
  但**锁臂并没有把 0.20 m 从不过变成过**。
- ★ **真正改变 0.20 m 判决的是 `no_self_contact` 行对齐 `D078`**（判**静置态** + 要求瞬态**消失**），
  而不是锁臂。**上一轮我按"全运行最坏值"判，比项目自己的规则更严。**
- ⛔ **我原来设的"可失败臂"（`arms='home'`）不能失败**：把手臂主动保持在申报的 home 目标角
  **是第二个修法**（给 PD 一个设定值），不是改前行为——0.20 m 下它报自交 **0.000 mm**，什么都没否证。
  真正的改前行为是 `carry_pose = None`，现已成为控制臂。

## D096

**证据文件自己会"说不清"：`json.dumps(default=str)` 把 numpy 布尔变成字符串 `"False"`，
而字符串 `"False"` 是**真值** —— 于是"不过"能被读成"过"。已改为显式 `bool(...)`。**
- 症状：`report.json` 里 `rows` 写的是 `"True"` / `"False"`。任何 `if row['arrives']:` 的消费者
  都会把 FAIL 读成 PASS。**这是"不可能失败的检查"那一族，长在证据产物自己身上。**
- 同时修掉冻结清单 CODE 的两处真实缺陷：
  ① **`experiments/probe_p4_balance.py` 被列了两次** ⇒ 同一脚本被哈希两遍，
     对外报的"32 脚本"**多算了一个**（**条目数是打字常量，覆盖率才是判据**）；
  ② **`experiments/probe_p4_holddepth.py` 缺失**，而它产出的 `reports/p4-holddepth-01/`
     **是可引用证据**（`D090` 的 `D_MAX` 就引它）—— **冻结清单察觉不到自己漏了什么**（`D058`/`D060` 同族）。
- **现在清单自查两条属性**（此前都不可派生、也都没被查）：**无重复条目**、**每条都在磁盘上**。
  ★ **可失败性实测**：注入一条重复 → `rc=1` 且打印
  `coverage: CODE lists experiments/probe_p4_holddepth.py more than once`；
  注入一条不存在的路径 → `rc=1`，**但是以 `FileNotFoundError` 崩溃的，不是那条新守卫报的**
  ⇒ **我为"缺失条目"加的那条守卫在实践中不可达**（指纹步骤先崩）。**如实记录，不当作已覆盖。**
- 现为 **14 产物 + 33 脚本 + 78 阈值**，覆盖 14/14。
'''

t = D.read_text(encoding='utf-8')
tail = '=== 见 DECISIONS 尾部 ==='
if '## D094' not in t:
    D.write_text(t.rstrip('\n') + '\n' + decisions, encoding='utf-8')
    print('DECISIONS: appended D094/D095/D096')
else:
    print('DECISIONS: already present, left alone')

# --- the task board row: the clause goes BEFORE the closing pipe -------------------------------
B = ROOT / 'docs' / 'TASK_BOARD.md'
lines = B.read_text(encoding='utf-8').split('\n')
pat = re.compile(r'(?<!\\)\|')
clause = ('⑧ **第十轮：接触前馈（闭链项）按评审原话做了三条独立验证** —— 符号/框架由"浮动基座行 = 总体重"'
          '确证（**99.88 %**），缺口在膝上约 **104 N·m**；'
          '⛔ **但读<实测>接触力的那一版就是<当前输出>**（25 个 dof 上均值差 **0.55 N·m**），'
          '注入后前馈峰值打到 **415.0 N·m**（＝执行器上限）、10 行失 9 行；'
          '换成<期望支撑力>后 0.20 m 脚底 **1.81° → 9.08°**、0.40 m 仍全不过 ⇒ **不采纳**（`D094`）。'
          '`D_MAX` **仍为 0.20 m**。★ 真正改变 0.20 m 判决的是把 `no_self_contact` 对齐 `D078`（`D095`）。')
changed = 0
for i, ln in enumerate(lines):
    if not ln.startswith('| P4-HUMAN-01 '):
        continue
    cells = pat.split(ln)
    assert len(cells) == 6, f'row has {len(cells)} split parts; expected 6'
    assert ln.rstrip().endswith('|'), 'row does not end with a pipe'
    body = ln.rstrip()
    body = body[:body.rfind('|')].rstrip() + ' ' + clause + ' |'
    lines[i] = body
    changed += 1
    newparts = pat.split(lines[i])
    assert len(newparts) == 6, f'after edit: {len(newparts)} parts'
    assert len([p for p in newparts if p.strip()]) == 4, 'row is no longer 4 cells'
assert changed == 1, f'expected exactly one P4-HUMAN-01 row, edited {changed}'
B.write_text('\n'.join(lines), encoding='utf-8')
print('TASK_BOARD: row P4-HUMAN-01 updated, shape re-asserted (4 cells)')

# --- the claims record -------------------------------------------------------------------------
C = ROOT / 'docs' / 'CLAIMS.md'
claims = '''
## 2026-09-27 — 接触前馈的评审复核（第十轮）

- 判据 `experiments/probe_p4_ff2.py`；证据 **`reports/p4-ff2-01/`**（`D094`）。
- **`D_MAX` 仍为 0.20 m**（`ff=off` + 臂锁 + 平衡项，**0.20 m 10/10 行通过**）。
- ★ **可引用的新事实**：`Σ Jᵀ f` 的浮动基座竖直分量 = 体重（**844.658 / 845.706 N，99.88 %**）——
  接触力投影的**框架与符号**由此确证；接触项在 0.20 m 蹲姿的膝上约 **104 N·m**。
- ⛔ **不要引用** `ff=desired` 与 `ff=measured` 两臂作为"已改进"：两者都比不做差。
  `ff=measured` 只作为**负控制臂**引用（它证明实测力投影＝当前输出：峰值 415.0 N·m＝量程上限）。
- ⛔ **首次运行的 `desired` 两臂作废**（`balance_on` 死开关，两臂逐位相同）；
  已在 `report.json` 的 `dead_switch_suspects` 下留有派生检查。
'''
if '第十轮' not in C.read_text(encoding='utf-8'):
    C.write_text(C.read_text(encoding='utf-8').rstrip('\n') + '\n' + claims, encoding='utf-8')
    print('CLAIMS: appended the round-10 section')
else:
    print('CLAIMS: already present')
print('done')
