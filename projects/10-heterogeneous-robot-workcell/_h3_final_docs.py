"""Add the third 'must-know' conclusion to IMPLEMENTATION_STATUS.md, and write the diagnostics
README for the H3 report.

Two small, separate edits; each asserts its anchor first.
"""
from pathlib import Path

ROOT = Path(__file__).resolve().parent

# ---------------------------------------------------------------- 1. the third conclusion
status = ROOT / "docs" / "IMPLEMENTATION_STATUS.md"
text = status.read_text(encoding="utf-8")

anchor = (
    "   ⛔ 但它是 `DIAGNOSTIC_ONLY`：**声明的固定工装位姿、无视觉**，且跑在**未合并的 007 世界**。\n"
)
addition = (
    anchor
    + "3. **H3 现在把「供盘」和「物流链」接起来了，而接法是可核对的**（`reports/p4-h3-w5-01`）：\n"
      "   人形把托盘放到源端辊道并释放，**同一个 `c_payload`（body 124 / 0.098 kg，首尾同 id）** 被链取走，\n"
      "   经 11 条命令送到接收端（x **4.448 → 12.596**），两段转账走完契约的 8 个阶段、\n"
      "   **custody 只在 `COMMITTED` 从源端转到接收端**。判据是**四条实测的契约**：\n"
      "   一个世界一个 `MjData`（plant 作 guest，**拒绝 reset** ⇒「零运行 qpos 写入」是结构性的）、\n"
      "   同一个托盘实体、**下游比松手晚 16.888 s 才动**（时间顺序，不是「两件事都发生了」）、托盘上零等式约束。\n"
      "   ⛔ 但**「人形把托盘交给了车」不能说** —— 交付的是「放到了源端辊道上」，物权转移是**链的** TRANSFER；\n"
      "   而且**站位是声明的固定工位、无视觉、只在站姿下成立**。\n"
)
if anchor not in text:
    raise SystemExit("[FAIL] the conclusions anchor was not found")
if "\n3. **H3 " in text:
    raise SystemExit("[FAIL] the third conclusion is already present")
status.write_text(text.replace(anchor, addition, 1), encoding="utf-8")
print("third conclusion added:", "\n3. **H3 " in status.read_text(encoding="utf-8"))

# ---------------------------------------------------------------- 2. the diagnostics README
readme = ROOT / "reports" / "p4-h3-w5-01" / "diagnostics" / "README.md"
readme.write_text(
    "# H3 诊断件：每一条都能从这里重新推出来\n"
    "\n"
    "接受记录里引用的每条数字，其**原始输出**与**产生它的脚本**都在这个目录里。\n"
    "（规则：**指向仓库外文件的报告不是可复现的报告。**）\n"
    "\n"
    "| 文件 | 证明什么 | 决策 |\n"
    "|---|---|---|\n"
    "| `out_h3_build.txt` | 建造器自己的报告：站位、候选 B 的 sha256、差异分类 | D108 |\n"
    "| `out_h3_audit.txt` | **运行时驱动**的站姿接触测试（四个站位都只碰地面）—— 为什么站位仍是 4.13 | D108 |\n"
    "| `out_h3_home_derivation.txt` | 三个世界的关键帧 vs `merged_home`；**差 3.94 µm** 的 float32 迭代残差 | D110 |\n"
    "| `out_h3_geomdist.txt` | `mj_geomDistance` 对**相距 2.0752 m** 的两个几何返回 **0.00000**（量程不是上限） | D109 |\n"
    "| `out_h3_coast.txt` | 人形作业期间没人命令 plant ⇒ 车滑到 **x 8.45782**（从 4.41372 滑了 **4.04 m**） | D111 |\n"
    "| `out_h3_neg_open_hand.txt` | 负例只张开手指时，**张开的手把托盘推上辊道**（x 4.4869，`source_band`）→ 链照样送达 | D112 |\n"
    "| `out_h3_globals.txt` | 作用域检查（`XFER` / `tx` 那一族，`D114`） | D114 |\n"
    "| `out_h3_P_console.txt` | 正例的原始控制台输出（含每条行的 detail） | — |\n"
    "| `out_h3_N_console.txt` | 集成负例的原始控制台输出 | — |\n"
    "| `probe_h3_w5.py` | 判据本身（报告里的 `source_sha256` 是它的哈希） | — |\n"
    "| `w5_h085_plan.py` | 世界建造器（audit-then-apply、`--check` 幂等、先编译再写入） | D108 |\n"
    "| `check_globals.py` | 上面那条作用域检查的实现（`symtable`） | D114 |\n"
    "| `_h3_*.sh` | 产生上面各条输出的脚本 | — |\n"
    "\n"
    "## 复现\n"
    "\n"
    "```bash\n"
    "cd /home/ziling/projects/010_heterogeneous_robot_workcell\n"
    "./.venv/bin/python experiments/w5_h085_plan.py --check            # 世界幂等\n"
    "./.venv/bin/python experiments/w5_h085_plan.py --audit            # 站姿接触测试\n"
    "./.venv/bin/python experiments/probe_h3_w5.py --run-id <id>       # 正例\n"
    "./.venv/bin/python experiments/probe_h3_w5.py --run-id <id> --negative-grasp   # 负例\n"
    "bash _h3_collect.sh <id>                                           # 重新收集本目录\n"
    "```\n"
    "\n"
    "⚠️ 两次运行各约 6–7 分钟（正例 `world_s = 112.05` / wall 402 s；负例 `world_s = 114.79` / wall 391 s）。\n"
    "⚠️ **不要**`pkill -f probe_h3_w5` —— 它会杀掉发出它的那个 shell（命令行里含同样的字符串）。用 `[p]robe_h3_w5`。\n",
    encoding="utf-8")
print("diagnostics README written:", readme.is_file(), readme.stat().st_size, "bytes")
