"""Bring docs/IMPLEMENTATION_STATUS.md up to date after H3.

Every replacement is asserted: a docs edit that silently misses its target leaves a stale claim
alive, which is the failure mode this project has already been bitten by (`D061` -- the hand-written
status pointer has rotted twice).
"""
from pathlib import Path

ROOT = Path(__file__).resolve().parent
DOC = ROOT / "docs" / "IMPLEMENTATION_STATUS.md"

EDITS = [
    # ---- the header and its date --------------------------------------------------------------
    ("## 当前状态（2026-09-27）", "## 当前状态（2026-09-29）"),

    # ---- the status table ---------------------------------------------------------------------
    ("| P1（含 H 链） | **完成**（H 链 = `DIAGNOSTIC_PASS`，见下） | `reports/p1-gate-08`、`p1-h-seq-10` |",
     "| P1（含 H 链） | **完成**（H 链在候选世界里 **H1/H2/H3 全部 PASS**） | "
     "`reports/p1-gate-08`、`p1-h-seq-10`、`p4-h1-w5-01`、`p4-h2-w5-03`、`p4-h3-w5-01` |"),
    ("| **人形真实供盘** | ⛔ **未完成——这是当前唯一的主线缺口** | H1/H2/H3 均未开始 |",
     "| **人形真实供盘 → 链把同一个托盘送进接收端** | ✅ **完成（H3 PASS，正例 17/17 行、负例 12/12 行）** | "
     "**`reports/p4-h3-w5-01`**（正例，含 `acceptance.md`/`diagnostics`）＋ "
     "**`reports/p4-h3-w5-neg-01`**（集成负例） |"),
    ("| P4 蹲姿 | **已被降级为对照项**（不再当主线） | `reports/p4-armik-05`：诚实门下 `D_MAX = None` |",
     "| P4 蹲姿 | **已被降级为对照项**（不再当主线；H 链**只在站姿下成立**） | "
     "`reports/p4-armik-05`：诚实门下 `D_MAX = None` |"),

    # ---- the two conclusions become three -----------------------------------------------------
    ("### ★ 两条必须知道的新结论", "### ★ 三条必须知道的新结论"),

    # ---- the next-round section becomes a completion record ------------------------------------
    (
        "### 下一轮只有两件（+1）\n"
        "1. 把 H 链**移植到候选世界几何下**运行；\n"
        "2. 补齐 H1 缺的**四项记录**：脚底接触与姿态、异常碰撞、控制来源、**运行 qpos 写入次数**；\n"
        "3. 两项成立后推进 **H2**（放到停止的源端机构、释放、退出）。",
        "### 人形供盘链：H1 → H2 → H3 全部完成\n"
        "1. **H1**（`reports/p4-h1-w5-01`）：双手抓持、升 **+0.1259 m**、持 **13.9 s**、放回、释放、退出只动 0.04 mm。\n"
        "2. **H2**（`reports/p4-h2-w5-03`）：把托盘放到**停止的源端辊道**上（`c_fixed_roller_0_0`/`_0_1`）、释放、退出。\n"
        "   阻断曾经是**手宽 109 mm 对辊端余量 50 mm**，由 **CHANGE 4**（源端辊半长 0.250 → 0.200，**扫出来的**）解除。\n"
        "3. **H3**（`reports/p4-h3-w5-01` ＋ `reports/p4-h3-w5-neg-01`）：**把 H 链接进 W5 物流链** ——\n"
        "   人形放到源端辊道并释放之后，**同一个 `c_payload`** 被链经 "
        "MOVE/DOCK/TRANSFER/VERIFY/UNDOCK 送到**接收端**，两段转账走完契约阶段、**custody 在 COMMITTED 转移**。\n"
        "   正例 **`PASS: all 17 judged rows`**（11 条链命令全 `SUCCEEDED`、托盘 x 4.448 → 12.596、异常碰撞 0 对）；\n"
        "   集成负例 **`PASS: all 12 judged rows`**（人形判 FAIL、链的 transfer/verify **全部拒绝**、托盘留在源端）。\n"
        "   世界 `assets/world_w5_h085_loop.xml`（候选 B ＋ 把人形站位写进关键帧）；**候选 B 逐位未变**。\n"
        "4. ⚠️ **H3 不是生产级工件线**：`scope = DIAGNOSTIC_ONLY`、**无视觉**、站位是声明的固定工位、"
        "**只在站姿下成立**（`D_MAX = None`）。"
    ),

    # ---- the unfinished list -------------------------------------------------------------------
    ("- **P4 未开工**（`P4-HUMAN-01` / `P4-ARM-02` / `P4-BELT-03`）。**P2 的证明只覆盖纯 Python 核心**；"
     "**P3 的证明只覆盖世界/导航/视觉/冻结四项各自的边界**（见 P3 各节的 ⚠️）。",
     "- **`P4-HUMAN-01` 已 DONE**（H1/H2/H3）；**`P4-ARM-02` / `P4-BELT-03` 仍未开工/BLOCKED**：\n"
     "  前者要有视觉的抓放与数量核验，**不能直接等于 `P3-VISION-03` 的结论**；后者要有输送/车载保持/接收卸盘与故障恢复。\n"
     "  ⚠️ **蹲姿不是能力**：`D_MAX = None`（`reports/p4-armik-05`），H 链全程**站姿**。\n"
     "  ⚠️ **P2 的证明只覆盖纯 Python 核心**；**P3 的证明只覆盖世界/导航/视觉/冻结四项各自的边界**（见 P3 各节的 ⚠️）。"),
]

text = DOC.read_text(encoding="utf-8")
for old, new in EDITS:
    if old not in text:
        raise SystemExit("[FAIL] anchor not found, refusing to patch blind:\n%s" % old[:120])
    if text.count(old) != 1:
        raise SystemExit("[FAIL] anchor appears %d times:\n%s" % (text.count(old), old[:120]))
    text = text.replace(old, new, 1)

DOC.write_text(text, encoding="utf-8")
final = DOC.read_text(encoding="utf-8")
print("patched %d anchors" % len(EDITS))
for probe in ("2026-09-29", "p4-h3-w5-01", "三条必须知道", "H1 → H2 → H3 全部完成",
              "`P4-HUMAN-01` 已 DONE"):
    print("  %-34s present: %s" % (probe, probe in final))
print("  stale 'H1/H2/H3 均未开始' gone:", "H1/H2/H3 均未开始" not in final)
print("  stale '两条必须知道' gone:", "### ★ 两条必须知道" not in final)
