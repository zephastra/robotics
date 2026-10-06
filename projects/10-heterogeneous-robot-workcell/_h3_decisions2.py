"""Append D114 to docs/DECISIONS.md.

D114 records the two judge defects the H3 round hit and the instrument/test that now catches them,
plus the three declared-but-never-read thresholds. Appended by script with assertions, and it
refuses to run twice.
"""
import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent
DOC = ROOT / 'docs' / 'DECISIONS.md'

NEW = '''

## D114

**只在最后才跑的那段代码是缺陷存活的地方 —— 判据本身要有能在 1 秒内跑的测试。**

- ★★ **同一个形状在本轮出现两次，各烧掉一整轮 6 分钟的运行**（而每一次，**所有物理量都已经量完了**）：
  ① **跨作用域**：`judge_h3` 读 `XFER.STAGES`，而 `XFER` 是在 `run()` **里面**导入的
     ⇒ `NameError: name 'XFER' is not defined`，报告变成 `POST_RUN_ERROR`、`h3` 一条行都没有。
  ② **同函数内的顺序**：`tx` 定义在它的**第一个**使用者旁边，而后加的一条行长在它**前面**读它
     ⇒ `UnboundLocalError: cannot access local variable 'tx'`。
  **导入检查抓不到 ②，作用域检查也抓不到 ②** —— 它们都是"代码能不能跑到那里"的问题，
  而"那里"在一条 2 分钟的仿真之后。
- ✅ **修法不是"下次小心"，是把判据变成可测的**：`tests/_h3_fixture.py` 构造一份**自洽的**报告
  （正例与负例各一套），`tests/test_h3_integration.py` **直接调用 `judge_h3`**，
  断言每条声明行都出现、负例的 `NOT_RUN` 真的被标出来。⇒ 这两个缺陷现在都是 **1 秒**的失败，
  而不是 6 分钟的。
- ✅ **并新增 `experiments/check_globals.py`**（用 `symtable` 做作用域分析）专门抓 ①，
  并把它**也纳入测试**：`test_the_globals_checker_can_fail` 拿一个**故意写坏**的模块喂给它，
  证明它**真的会失败** —— 这是本项目的老规矩，因为"根本不可能失败的检查"本身就是两个缺陷族之一。
  ⚠️ **它的第一版是错的**：手写 AST 遍历把 **lambda 形参**（`side`/`s`）和**闭包变量**当成未绑定全局，
  一次报 **46 条**误报。**会喊狼来了的检查会被无视，比没有检查更糟**；改用 `symtable` 后 6 个文件 **0 误报**。
- ★★ **第三个缺陷：三条阈值"声明了但没有任何行读它们"。** `H3_THRESHOLDS` 里的
  `handoff_drift_m`、`custody_on_source_rollers`、`chain_reaches_receiver` 从未被引用，
  而冻结契约会把它们**照常发布**，读起来像"有东西在按它们判"。已全部接进行里：
  ① 新增 **`tray_still_between_release_and_the_chain`**（从松手那一刻到人形阶段结束，托盘移动 ≤ 0.010 m
     —— 这是"人形把它放稳了"唯一的窗口，因为链之后就会动它）；
  ② `tray_on_source_band_at_handoff` 现在读 **`custody_on_source_rollers`**，要求两个指定辊子各自的接触采样数；
  ③ `chain_reached_receiver` 现在读 **`chain_reaches_receiver`**，要求每条腿都在它**声明的**目的行上被接触证实。
  并新增测试 **`test_every_declared_h3_threshold_is_read_by_a_row`** —— 就是它发现这三条的。
- ⚠️ **操作层（第 N 次）**：`pkill -f "probe_h3_w5"` **会杀掉发出它的那个 shell**，
  因为该 shell 自己的命令行里就含这个字符串（`sha256sum experiments/probe_h3_w5.py` 那一段）。
  症状是 `Exit Code: 15` 而没有任何输出。**要用自排除模式 `[p]robe_h3_w5`，且别在同一条命令里再提这个文件名。**
'''

text = DOC.read_text(encoding='utf-8')
ids = re.findall(r'^##\s+(D\d+)\b', text, re.M)
if not ids:
    sys.exit('[FAIL] no decision headings found')
if ids[-1] != 'D113':
    sys.exit('[FAIL] the last decision is %s, expected D113' % ids[-1])
if '## D114' in text:
    sys.exit('[FAIL] D114 is already present')
DOC.write_text(text.rstrip('\n') + '\n' + NEW, encoding='utf-8')
after = re.findall(r'^##\s+(D\d+)\b', DOC.read_text(encoding='utf-8'), re.M)
nums = [int(i[1:]) for i in after]
gaps = [b for a, b in zip(nums, nums[1:]) if b != a + 1]
print('appended D114; ids now D%d..D%d = %d entries, gaps %s'
      % (nums[0], nums[-1], len(nums), gaps or 'none'))
