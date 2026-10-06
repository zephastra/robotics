"""Append the H3 decision entries to docs/DECISIONS.md.

A script rather than a text edit because the file is Chinese, 3700+ lines, and lives on the WSL
filesystem where a tool-level edit has failed before. It ASSERTS what it expects to find before
writing anything, so a wrong file or an out-of-order append is an error rather than a silent
corruption.
"""
import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent
DOC = ROOT / 'docs' / 'DECISIONS.md'

NEW = '''

## D108

**候选世界 B 从来没有把人形放进去 —— H3 要做的第一件事是把"站位"写进世界，而不是写进探针。**

- ★★ **实测**（`_diag/out_h3_clear.txt`、`experiments/w5_h085_plan.py --audit`）：候选 B 的关键帧里
  `h_LINK_BASE` 仍在 **x 0.660132**，而它自己的工装 `w5_h085_station` 在 **4.31**、托盘在 **4.343532**、
  AMT 在 **4.41372**。**机器人和工装相距约 3.8 m。** 候选 B 抬高了物流接口、加了工装，
  **唯独没有把人形搬过去**。
- ★ **为什么 H2 还能过**：**H2 把站位写在探针里** —— 一次初始化 `qpos[0] = 4.13`
  （`probe_h2_w5.py`，该写入在 `reports/p4-h2-w5-03` 里记为 `init_qpos_writes`）。
  对**单探针**运行这是合法的**声明站位**；对 **H3 是错的形状** —— H3 的人形运行时与 W5 `LogisticsPlant`
  **共用同一个 `MjData`**，一个只存在于探针里的站位就是**世界没有的站位**，
  下一个消费者会继承一个"机器人离工件线 4 米"的世界。
- ★ **两个"看起来像碰撞"的读数，都不是碰撞**（两条都实测过）：
  ① `mj_geomDistance` 在站位 4.13 报"踝关节在 `n_chassis` 内 15.6 mm、髋碰到托盘" —— **假的**。
     涉事几何是 `contype=0 conaffinity=0` 的**视觉网格**；同一台机器人在 home 位姿下
     `mj_geomDistance` 报 **272 对自穿透**，其中包括 `h_LINK_BASE` 与自身 **−0.16740 m**
     —— 与 `D078` 推翻的"58.5 mm 自穿透"同一族。**两个几何之间的距离不是接触。**
  ② 自由落体式放松（free settle）说"每个站位都会摔倒"（4.13 漂 +0.050 m、4.07 漂 +0.164 m、手落地）
     —— 也**不构成证据**：站立律**保姿势、不保平衡**，这是 P4 的既有事实；放松测试量的是
     **缺少平衡控制器**，不是站位好坏。
- ★★ **决定性的测量是"让机器人自己的控制器跑起来"的接触测试**（就是 H3 驱动它的方式），400 tick：
  站位 0.660132 / 4.13 / 4.09 / 4.20 四处的交叉实体接触**都只有 `world/cell_ground`（左右脚各 8 次）**，
  x 漂移 +4.1 mm、z 沉降 −18.5 mm。**⇒ 站位就是 H2 的 4.13，不改。**
  4.09 会把臂重新量在一个 H2 从未验证过的站位上，为的是一个**不存在的碰撞**。
  **换尺不换阈值 —— 而这次那把尺错的方向是"凭空造出一个碰撞"。**
- ✅ **产物**：`experiments/w5_h085_plan.py`（audit-then-apply 建造器）→
  `assets/world_w5_h085_loop.xml`（sha `f964477f…`）。相对候选 B 的**差异只有 1 行**
  （第 555 行 `h_LINK_BASE` 的 `pos`）＋ 1 行**与候选 B 共同拥有**（第 1501 行 `<key name="home">`，
  两代建造器都必须改它）；`nq/nbody/ngeom = 151/149/309`，**逐项与候选 B 相同**。
  建造器**先编译再写入**，并把每一行差异**分类**为"候选 B 已改过"或"本步引入"；
  出现第三类就**不写盘**。`--check` 幂等。**源文件候选 B 逐位未变（`702e22a6…`）。**

## D109

**`mj_geomDistance` 的 `distmax` 是"量程"，不是"上限" —— 把量程当成距离会造出一个碰撞。**

- ★★ **实测**：H3 的第一次"碰撞启用几何"读数把整轮最小距离报成 **0.00000 m**，
  配对是 `h_LINK_SHOULDER_ROLL_R` 与 `c_fixed_roller_2_5`。**直接量这两个几何**：
  x 分别 **4.1228** 与 **6.0937** —— **相距 2.0752 m**；该次调用返回 0，且**只在这一对、这一个采样上**
  （其余样本 0 对、最小值 0.094 m）。原因就是第一版传的 **`distmax = 2.0`**，
  而这一对的表面距离**恰好贴在 2.0 上**：`mj_geomDistance` 对这一量程**返回的不是距离**。
- ★ **改法**：量程设为 **0.5 m**（是各行真正要判的 0.05 m 阈值的 **10 倍**），
  并把"达到量程"作为**独立字段 `beyond_budget` 上报**，**不再当成一个数值引用**。
  一个越界的配对现在报"没有配对进入量程"，而不是把量程印成 0。
- ★ **同一把尺的第二个缺陷**：第一版**只报人形一侧**，于是那一行读作
  "closest 0.0000 m (h_LINK_SHOULDER_ROLL_R)" —— **半个配对**，读者无法复核。
  现在**两侧名字 + 两侧世界坐标 + 时间**一起报。**最小距离是关于一对几何的陈述，
  只报一半就是让假象活过评审的方式。**
- ⚠️ **对既有证据的影响**：H2 的 `clearances()`（`probe_h2_w5.py`，**未改动**，为的是 H2 的数字继续
  出自它原来那段代码）用的是同一把 `distmax=2.0` 的尺，`reports/p4-h2-w5-03` 里
  `robot_clear_of_band` 的 **run-min 上下文**（"0.0000 m (h_LINK_SHOULDER_ROLL_R)"）**同受影响**：
  那一行**判的是终值 0.1248 m（PASS，不受影响）**，但它的 **run-min 不能当作物理事实引用**。
  H3 同时上报两个仪器（H2 的用于连续性、碰撞启用的用于物理），并明确
  **"是否碰到"以求解器自己的接触表为准**（`no_abnormal_collision`）。

## D110

**世界必须在任何消费者读它之前被放到 home 位姿一次，否则 `_derive()` 会从"不是 home 的状态"推导几何。**

- ★★ **实测缺陷**：第一版 H3 **先建人形运行时**（它自建 `MjData`，qpos 为 0）、**再把它交给 plant**。
  `LogisticsPlant._derive()` 是从 `data.geom_xpos` 读甲板辊与底盘位置来**推导**靠泊几何的，
  于是它从一个**不是 home 的状态**推导。后果：`dock_residuals('station_c')` 报
  **`longitudinal_m = −4.608 m`**，而底盘**离目标只有 2.9 mm、`travelled_m = 0.0`** ——
  第一条 `DOCK` 就报 `FAILED / DOCK_OUT_OF_TOLERANCE`，**为一辆已经停好的车**。
- ✅ **修法**：`Mujoco.MjModel` + `MjData` 由探针建一次，**先** `qpos[:] = model.key_qpos[0]`
  （**唯一的一次初始化写入**，逐条记入 `qpos_write_sites`），**再**把同一个 model/data
  交给人形运行时和 plant（两者都新增了**可选** `model=`/`data=` 参数，**默认路径逐字不变**，
  因此 H1/H2 与 W2–W5 的证据不受影响）。
- ★ **加了一条能失败的守卫**：home 位姿下 `station_c` 的纵向残差若 **> 0.6 m**，
  建造/探针**直接抛错**并说明"这是状态错误，不是停车误差"。没有这条守卫，这个缺陷会
  以"下游 `NAV_FAILED`"的形式出现在离病因四步远的地方。

## D111

**人形作业期间车必须被"主动刹住" —— 轮子是裸电机，`ctrl=0` 是零力矩不是刹车。**

- ★★ **实测**（本轮最重的一个）：人形供盘的 50 s 里**没有任何东西命令这个 plant**，
  于是它的轮子停在**零力矩**状态，**车自己滑了 4.04 m（4.41372 → 8.45782）**。
  下游 `MOVE_TO_STATION` 于是有 4 m 要补，报 `NAV_FAILED`，
  整条链塌成 `DOCK_OUT_OF_TOLERANCE` / `TRANSFER_TIMEOUT` / `PAYLOAD_LOST`。
- ★ **根因是可查的、不是猜的**：`n_wheel_left_motor` / `n_wheel_right_motor` 的
  **`biastype=0`（mjBIAS_NONE）、`biasprm` 全 0** —— **裸电机**。
  `w4_plant._wheel_rate_torque` 自己的文档串早就写了这件事（2026-09-26，`_dbg_creep.py`）：
  *"Cutting torque is not braking; it is coasting."* 而 `home_hold_ctrl()` 会**跳过**非仿射偏置的执行器，
  所以连 hold 律也不刹轮子。
- ✅ **修法**：plant 新增 **`park_control()`** —— 一个"停驻控制周期"的**控制向量**（hold 律 +
  `_wheel_rate_torque(side, 0.0)`），**不步进**；`_tick` 重构为 `_control()` + 一步，
  **"停驻"只有一个定义**。H3 在人形阶段的**每个 tick** 先写 `park_control()`，
  再让运行时写上自己的执行器，最后由 `Runtime.step` 内的**唯一一次** `mj_step` 结束周期。
  实测：底盘在人形阶段的漂移 **0.0000 m**（原 4.04 m）。
- ★ **这不是 H3 的权宜之计，它是场景本身**：**一辆已靠泊的 AMR 会等**。
  并且它把"车在等人形"从一句断言变成**一条行**（`vehicle_parked_while_humanoid_works`）
  ＋ **一条采样轨迹**（`chassis_trace_during_humanoid`）。
- ⚠️ **顺带发现并要求如实记录**：世界里还有**第二台底盘** `n2_base_link`（停在 y 1.15，走廊之外）。
  它的 `n2_wheel_*` **不在** `wheel_actuators` 里（plant 要求恰好两个），所以
  `park_control()` **不刹它**。它的轨迹已记入 `second_vehicle_trace_during_humanoid`，
  **本文件不再声称"世界静止不动"**。

## D112

**集成负例需要两处替换，不是一处 —— 只把手指张开，手会把托盘"推"上辊道。**

- ★★ **实测**（`_diag/out_h3_neg1.txt`）：负例第一版只把 `GRASP` 换成 `OPEN`。
  结果 `place` 阶段的**水平搬运位移**仍然执行，**张开的手把托盘从工装上推到了源端辊道上** ——
  `support_at_handoff = ['source_band']`、托盘 x **4.4869**，然后**整条链把它送到了接收端**。
  **一个"把托盘送到了"的失败不是失败**，而那条负例行会**在一个什么都没证明的场景上判 PASS**。
- ✅ **修法**：负例同时 (a) 用 `OPEN` 替换抓握姿态、(b) 把搬运目标钉在**托盘自己的起始 x**。
  理由直接：**抓不住东西的手也搬不动东西**。为此给 `probe_h2_w5.stage_driver` 增加了**可选**
  `place_x` 参数（默认 `PLACE_X`，**H2 的行为逐字不变**）。
- ★ **判据同时要求"人形自己的裁判也判它失败"**：负例行读 `full_H_acceptance`，
  必须是**非 PASS**。否则负例只是"我的场景没送达"，而不是"人形的契约判它失败"。

## D113

**H3 完成：H 链接进了 W5 物流链 —— 同一个世界、同一条时间线、同一个托盘。**

- ✅ **权威证据 `reports/p4-h3-w5-01/`**（正例）与 **`reports/p4-h3-w5-neg-01/`**（集成负例）。
- ★ **集成契约四条的实测**：
  ① **一个世界一个 data**：人形运行时与 plant 的 `MjModel`/`MjData` 是**同一个对象**
     （报告里 `same_model` / `same_data`，且 `plant.owns_world = False` ⇒ **它不会重置被交来的世界**）；
     世界 `assets/world_w5_h085_loop.xml` 全程**不重载**。
  ② **同一个托盘实体**：`c_payload` 首尾都是 **body 124、0.098 kg**，没有第二个托盘。
  ③ **下游等人形松手**：手指在 **t≈12.4 s** 合上、在 **t≈34.4 s** 彻底松开，
     链的第一次运动在 **t≈51.3 s**；判据是**时间顺序**（且要求"松开在合上之后"），
     不是"两件事都发生了"。
  ④ **两条物理禁令**：托盘上的等式约束 **0**；plant 的 qpos 写入只有 **`calibrate: 3`**、
     **`run` 阶段不存在**（`runtime_qpos_writes = 0`）—— 而且是**结构性的**：
     guest 模式的 plant **拒绝 reset**，循环里**没有写入点可以用**。
- ★ **六个 H 阶段全过**、**11 条链命令全 `SUCCEEDED`**、托盘 `source_band → deck → receiver_band`
  （x 4.448 → 12.595，链自己搬了 **8.15 m**）、**异常碰撞 0 对**、
  退出后最近人形几何距辊道 **0.1247 m**（限 0.050）。
- ⛔ **不能说**：**"人形把托盘交给了车"** —— 交付的是**放到了源端辊道上**，物权转移发生在
  **链的 TRANSFER**（`D113` 只覆盖到"链把托盘送到了接收端"）；
  **"人形能走路"**（站立律只保持姿势，人形全程靠自己的站姿律停在站位上，
  且在链阶段由 `park_control()` 之外的**它自己的 ctrl** 维持 —— 见下）；
  **"这是一个生产级的工件线"**（`scope = DIAGNOSTIC_ONLY`、无视觉、
  站位是**声明的固定工位目标**，运行时无感知）；
  **"候选世界已是 W5 基线"**（候选世界**不能引用**旧 W5/W3/W4 的 PASS）；
  **"H3 覆盖了订单 N01"**（无拣选 / 无 BOM / 无库存）。
- ⚠️ **本轮未动**：`probe_h_w5.py`、`p1-h-seq-*`、`assets/world_w2_logistic.xml` 等四个冻结世界
  **逐位未变**；`001`–`009` 未动。
'''

text = DOC.read_text(encoding='utf-8')
ids = re.findall(r'^## (D\d+)\s*$', text, re.M)
if not ids:
    sys.exit('[FAIL] no decision headings found')
last = ids[-1]
if last != 'D107':
    sys.exit('[FAIL] the last decision is %s, expected D107; refusing to append' % last)
if '## D108' in text:
    sys.exit('[FAIL] D108 is already present; refusing to append twice')
n_before = len(ids)
new_text = text.rstrip('\n') + '\n' + NEW
new_ids = re.findall(r'^## (D\d+)\s*$', new_text, re.M)
expected = ['D%d' % n for n in range(int(last[1:]), int(last[1:]) + 1 + 6)]
if new_ids[-7:] != expected:
    sys.exit('[FAIL] the appended ids are %s, expected %s' % (new_ids[-7:], expected))
# and no gap anywhere in the sequence
nums = [int(i[1:]) for i in new_ids]
dupes = sorted({n for n in nums if nums.count(n) > 1})
gaps = [b for a, b in zip(nums, nums[1:]) if b != a + 1]
if dupes or gaps:
    sys.exit('[FAIL] duplicates %s / gaps %s' % (dupes, gaps))
DOC.write_text(new_text, encoding='utf-8')
print('appended %d entries: %s -> %s' % (len(new_ids) - n_before, last, new_ids[-1]))
print('decision ids now: D%s..D%s, %d entries, no gaps, no duplicates'
      % (new_ids[0][1:], new_ids[-1][1:], len(new_ids)))
