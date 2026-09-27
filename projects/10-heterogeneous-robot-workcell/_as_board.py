import pathlib

P = pathlib.Path('/home/ziling/projects/010_heterogeneous_robot_workcell/docs/TASK_BOARD.md')
src = P.read_text(encoding='utf-8')

# 1) header line: the gate is now 15 checks including the assembly
old_hdr = "证据见 P1_FEASIBILITY.md。"
new_hdr = ("证据见 P1_FEASIBILITY.md。**2026-09-23 更新：V1 装配已实施，门禁升为 15 条判据；"
           "权威证据改为 `reports/p1-gate-06`（`PASS`，15/15），全套 pytest 230 passed。见 `docs/DECISIONS.md` D041。**")
assert src.count(old_hdr) == 1
src = src.replace(old_hdr, new_hdr, 1)

# 2) the GATE-07 row: append the assembly outcome
old_gate = "真正被丢掉的是 **N 自己的场地**。见 D039。 |"
new_gate = ("真正被丢掉的是 **N 自己的场地**。见 D039。"
            " **2026-09-23 追加（D041）：V1 装配已实施** —— C 的甲板子树（11 body）现在是 **N 的 `base_link` 的子树**，"
            "固定辊排（24）与接收排（14）留世界、`payload` 留世界。**权威证据更新为 `reports/p1-gate-06` = `PASS`（15 checks: 15 PASS / 0 FAIL / 0 NOT_RUN）**，"
            "新增 4 条判据：**甲板确实装在底盘上**（`parent = n_base_link`，且 `c_deck` 只有一个）、"
            "**移底盘甲板跟着走而夹具不动**（+0.5 m → 甲板 +0.5、夹具 0.0）、"
            "**装配后的装置逐字复现 C 自己的数字**（`dock_offset` 实测 **+3.943e-06 m**，全部冠在 `z = CROWN_Z`）、"
            "以及**判据自身可失败**（5 个装配探针各自给出声明过的状态）。"
            "★ 关键设计：**停车是初始条件，不是一次行驶** —— 命令机器人开过去对接会让被测对象自己生产初始条件，C 的\"夹具固定、甲板开出来迎它\"会变成循环论证。"
            "★ 两条结构判据现在**知道这个例外**（`deck_relocated()` 是唯一谓词），并且**要求例外被真正用到**：例外为空但声明非空即 FAIL，所以装配被撤销时它们不会安静地变绿。"
            "★ 本轮 8 个缺陷里有 4 个同族（量参照错坐标系 / 比错了量 / 打字常量），其中一个是**硬教训 8 本人**："
            "测试用字面量字符串 `'deck mounted on N'` 去钉一条**否定陈述**，装配成立后那条否定就该删，测试随即为\"它曾经是对的\"而失败 —— 已改为断言**性质**。"
            "⚠️ **边界**：这**不**等于 C 的转移在装配世界里通过（装配改变了、但**没有重跑** C 的转移实验；C 的证据来自 `roller_rig` 自己的单角色世界）；"
            "**不**等于对接公差可接受（底盘停在旧常数假设的精确间距上，公差**没有被扫过**）；**不**等于跨角色物理交接。见 D041。 |")
assert src.count(old_gate) == 1, src.count(old_gate)
src = src.replace(old_gate, new_gate, 1)

# 3) P2-CORE-01: dependency satisfied (already marked) -- make the next action explicit
old_p2 = "| P2-CORE-01 | TODO | GATE-07（已满足） | schema + 账本状态机 + 单元测试 —— **依赖已解除**，等授权开工 |"
new_p2 = ("| P2-CORE-01 | TODO | GATE-07（已满足） | schema + 账本状态机 + 单元测试 —— "
          "**依赖已解除，是下一步**（`P1-GATE-07` 已 DONE 且 V1 装配已实施）。本轮装配把 P2 的前置全部清干净："
          "共同世界能加载、甲板在车上、判据可失败。等授权开工 |")
assert src.count(old_p2) == 1
src = src.replace(old_p2, new_p2, 1)

P.write_text(src, encoding='utf-8')
print('TASK_BOARD updated;', len(src), 'chars')
