"""Append C-005 (H3) to docs/CAPABILITIES.md and correct C-004's stale H3 sentence.

Asserts the anchors before writing: a docs edit that silently misses its target is worse than one
that fails, because the stale claim survives.
"""
from pathlib import Path

ROOT = Path(__file__).resolve().parent
DOC = ROOT / "docs" / "CAPABILITIES.md"

C004_OLD = (
    "- ⛔ **仍未验证**：**H3 未开始** —— 没有接入 MOVE/DOCK/TRANSFER/VERIFY/UNDOCK、没有接收端交付、\n"
    "  没有物权转移；候选世界不能引用旧 W5/W3/W4 的 PASS。"
)
C004_NEW = (
    "- ⛔ **仍未验证（就 H2 本身而言）**：H2 只证明**人形**能把托盘放到源端辊道上并释放；\n"
    "  **它没有证明链会接过去** —— 那件事由 H3 完成（见 C-005）。候选世界不能引用旧 W5/W3/W4 的 PASS。\n"
    "- ⚠️ 本能力的世界是 `assets/world_w5_h085.xml`；H3 用的是它的后继 `assets/world_w5_h085_loop.xml`\n"
    "  （同一世界 ＋ 人形站位写进关键帧）。**H2 的数字只属于 H2 那个世界**。"
)

C005 = """

## C-005 · **H3：H 链接进 W5 物流链（同一世界 / 同一时间线 / 同一托盘）** —— **PASS**

- **能力名称与支持范围**：人形把 V1 托盘放到**停止的源端辊道**上并释放之后，**同一个托盘实体**
  被 W5 的单车物流链接走，经 `MOVE_TO_STATION / DOCK / START_TRANSFER / VERIFY_TRANSFER / UNDOCK`
  送到接收端并完成**物权转移**。范围 = 单一 V1 托盘（0.098 kg）、**声明的固定工位站位**、
  **无视觉**、`scope = DIAGNOSTIC_ONLY`、**只在站姿下成立**（`D_MAX = None`）。
- **代码/模型版本与报告路径**：判据 `experiments/probe_h3_w5.py`（**导入** `probe_h_w5` 的仪器与
  `probe_h2_w5` 的阶段驱动，**不复制**）；世界建造器 `experiments/w5_h085_plan.py`
  （audit-then-apply，`--check` 幂等，**先编译再写入**）；世界 `assets/world_w5_h085_loop.xml`
  sha256 `f964477fb8c2ffda…`。权威证据 **`reports/p4-h3-w5-01/`**（正例，含 `acceptance.md` 与
  `diagnostics/`）与 **`reports/p4-h3-w5-neg-01/`**（集成负例）。测试 `tests/test_h3_integration.py`（17 项）。
- **输入、前置条件、在线观测来源**：输入 = 世界关键帧里的人形基准 **x 4.13**（＝ H2 的站位，
  `D108` 说明为什么它不改）、托盘起点 x 4.343532 z 0.85、放置点 x 4.45372。
  **在线观测：无。** 锚点、站位、放置点都是**声明的固定工位量**，初始化时读一次。
  **前置**：世界被放到 home 位姿**一次**（唯一一次 qpos 写入），然后同一个 `MjModel`/`MjData`
  交给**人形运行时**与 `LogisticsPlant`（guest 模式，**拒绝 reset**）。
- **实际物理过程**：手指 **t=12.398 s** 合上 → 抬 +0.1264 m → 悬持 15.5 s → 水平搬运 → 落到
  `c_fixed_roller_0_0`/`_0_1` → **t=34.398 s** 松手 → 退出（离辊道 **0.1247 m**）→
  **链的第一次运动 t=51.286 s**（**比松手晚 16.888 s**）→ 11 条命令全 `SUCCEEDED` →
  托盘 x **4.448 → 12.596**（**链自己搬了 8.16 m**）→ 落在 `receiver_band`。
- **成功判据与独立验收**：**H 六阶段契约 = `humanoid007.tray_task` 的冻结阈值，一行未改**
  （`PASS: all six stages`）；另加 **18 条 H3 声明行**（`H3_THRESHOLDS`，报告内标为
  **DECLARED AND NOT FROZEN**）。正例 `PASS: all 17 judged rows, 1 NOT_RUN`；
  负例 `PASS: all 12 judged rows, 6 NOT_RUN`。**`NOT_RUN` 不计入 PASS 数。**
- **失败/超时和安全处理**：`WALL_DEADLINE_S = 3000`；任何涉及托盘的等式出现即**抛错终止**；
  **`ROW_ARMS` 强制分臂** —— 在本臂里没被评判的行若给出 PASS/FAIL 即 `RuntimeError`（`D114`）；
  任何行 detail 里留有未格式化的 `%s` 即 `RuntimeError`；未驱动的阶段/行记 `NOT_RUN`。
- ★ **集成契约（四条，全部实测）**：① 一个世界一个 `MjData`（`same_model` / `same_data`，
  `plant.owns_world = False`），全程**不重载**；② 同一个托盘实体（`c_payload` 首尾都是
  **body 124 / 0.098 kg**）；③ **下游等人形松手**（判据是**时间顺序**，且要求"松开在合上之后"）；
  ④ 托盘上的等式约束 **0**、`run` 阶段 qpos 写入 **0**（**结构性**：guest 的 plant 拒绝 reset）。
- ★ **集成负例**（`--negative-grasp`）：人形自己的裁判判 **FAIL**（`lift`/`hold`），
  链的 `START_TRANSFER`/`VERIFY_TRANSFER`/`VERIFY_DELIVERY` **全部拒绝**
  （`TRANSFER_TIMEOUT`、`PAYLOAD_LOST`），托盘**始终留在源端**。
  机制需要**两处替换**（`OPEN` 姿态 ＋ 搬运目标钉在托盘起始 x）—— 只张开手指时，
  **张开的手会把托盘推上辊道**，链照样送达（`D112`）。
- ⛔ **不能外推**：**"人形把托盘交给了车"**（交付的是"放到源端辊道上"，物权转移是链的 TRANSFER）；
  **"人形能走路"**；**"这是生产级工件线"**（无视觉、站位是声明的固定工位）；
  **"候选世界已是 W5 基线"**（不能引用旧 W5/W3/W4 的 PASS）；
  **"H3 覆盖订单 N01"**（无拣选/BOM/库存）；**蹲姿已具备**（`D_MAX = None`）。
  ⚠️ `merged_home` 与世界关键帧差 **3.94 µm**（float32 迭代残差），是横向容差 0.002 m 的 0.197%。
- **如何最小复现**：
  `.venv/bin/python experiments/w5_h085_plan.py --check && `
  `.venv/bin/python experiments/probe_h3_w5.py --run-id p4-h3-w5-01 && `
  `.venv/bin/python experiments/probe_h3_w5.py --run-id p4-h3-w5-neg-01 --negative-grasp`
"""

text = DOC.read_text(encoding="utf-8")
if "## C-005" in text:
    raise SystemExit("[FAIL] C-005 already present")
if C004_OLD not in text:
    raise SystemExit("[FAIL] the C-004 anchor was not found; refusing to patch blind")
text = text.replace(C004_OLD, C004_NEW, 1)
text = text.rstrip("\n") + "\n" + C005
DOC.write_text(text, encoding="utf-8")

final = DOC.read_text(encoding="utf-8")
print("C-004 corrected:", C004_NEW.split(chr(10))[0] in final)
print("C-005 present:", "## C-005" in final)
print("stale sentence gone:", C004_OLD.split(chr(10))[0] not in final)
print("capabilities now:", len([l for l in final.split(chr(10)) if l.startswith("## C-")]))
