import ast, io, pathlib

ROOT = pathlib.Path('/home/ziling/projects/010_heterogeneous_robot_workcell')

# ---------------------------------------------------------------- DECISIONS D041
D = ROOT / 'docs' / 'DECISIONS.md'
src = D.read_text(encoding='utf-8')

entry = '''
---

## D041 — V1 装配：把 C 的甲板真的装到 N 的底盘上（2026-09-23，**已实施**）

**决策**：采纳 `D039` 第七节那份"配方实测通过但未实施"的装配。C 的甲板子树（`deck` + 8 个
`deck_roller_*` + `pusher_carriage` + `pusher_blade`，共 **11 个 body**）成为 **N 的 `base_link` 的子树**；
C 的固定辊排（24 个）与接收排（14 个）**留在世界坐标系**；`payload` 也留世界（它是货物，不是车的一部分）。
**权威证据 `reports/p1-gate-06` = `PASS`（15 checks: 15 PASS / 0 FAIL / 0 NOT_RUN）**，全套 pytest **230 passed**。

### 一、为什么这一步是"改主张"而不是"改代码"

`D039` 说得很准：`RECV_FIRST_CROWN_X = DECK_LAST_CROWN_X + 2r + CROWN_CLEARANCE` 断言的是甲板与接收台
之间的**固定距离**（甲板世界焊接时它是设计常数）。甲板一旦装在底盘上，这个距离由 **AMR 停在哪儿**决定
—— **常数变成对接公差**。

所以装配的正确目标不是"把甲板挂上去"，而是 **"把底盘停在让装配后的装置逐字复现 C 自己那套数字的位置上"**，
并**用判据量它**（`check_rig_reproduced`，`dock_offset` 实测 **+3.943e-06 m**）。

★ **停车是初始条件，不是一次行驶。** 如果命令机器人"开过去对接"，被测对象就自己生产了自己的初始条件，
C 的"夹具固定、甲板开出来迎它"这条主张会变成循环论证。

### 二、实施要点（三条硬约束，全部实测）

1. **`attach` 保留子模型的绝对世界位姿，只改写父子关系** ⇒ 甲板不能直接往世界上一挂了事，
   必须先**拆分**再 `attach`；且 `MjsBody.parent` **只读**（本项目已经付过一次学费）。
2. **挂载点必须携带底盘根的负偏移**。N 的 `base_link` 在它自己的源里位于 `z = 0.04`，
   站点写在 `(0,0,0)` 会让甲板**高 40 mm**。修法：站点写成 `-n_data.xpos[base_link]`
   —— **从编译模型读出来，不是打字打上去的**。
3. **`attach` 会带来子模型的 `<keyframe>` 与 `meshdir`**，两者都要在 attach 前清掉；
   **顶层 `<default name=...>` 是 schema 违规**，改名要走 `spec.default.name`。

### 三、装配后必须重新划线的三条判据

装配**按设计**打破了"四角色互不相干"这条不变量，所以两条结构判据必须**知道这个例外**，
而且必须**要求例外被真正用到**（否则装配被撤销时它们会安静地变绿）：

- `check_names_follow_ownership`：`c_deck` / `c_deck_mount` 是**声明过的例外**，
  走 `deck_relocated()` 这个**唯一**的谓词；仍在 if/else 里"点名豁免"就必须报 FAIL。
- `check_dof_independence`：扰动 N 时甲板**应当**跟着走（这就是"装着"的全部含义），
  这一条**单独报告**；但若 `deck_relocated()` 非空而 **`rode` 为空**，直接 FAIL —— 例外的**使用**也是被检查的。

★ **`verify_kinematics` 不再比较甲板的局部 `pos`**：它的局部位姿是**故意被改写**的。
换成一个能失效的说法：**"甲板的世界位姿要等于 C 自己把它摆在哪儿"**，这件事由 `check_rig_reproduced`
量（残差 `dock_offset`）。被跳过的 body **逐一列出**，不做静默跳过。

### 四、本轮修掉的缺陷（8 个，其中 4 个是同族）

| # | 症状 | 真因 |
|---|---|---|
| 1 | 构建报 `unrecognized attribute: 'name'` on `default` | 顶层 `<default name=…>` 是 schema 违规；改名走 `spec.default.name` |
| 2 | 甲板**高 40 mm** | 挂载站点没带底盘根的负偏移 |
| 3 | `dock_offset = 4.41372`（停车等于没停） | **两个**原因：`want` 与 `have` 用了同一个表达式（恒等式）；且停车块写在**角色覆盖循环之前**，被 `n_slide_x = 0` 覆盖 |
| 4 | 门禁 `NameError: check_deck_mounted not defined` | 第一次插入用的锚点没匹配上，`sub_once` 却报了 OK |
| 5 | 门禁 4 条 FAIL 里有 2 条**不是**世界的问题 | `check_rig_reproduced` 拿**辊心** `z` 和 `CROWN_Z` 比（差正好 `ROLLER_RADIUS = 0.035`）；期望容差 `1e-9` 在 **float32 存储下限之下** |
| 6 | 探针 1/3 在**未改动**的世界上报 PASS | `b.parent = spec.worldbody` —— **赋值给只读属性**，是被丢弃的表达式语句；且 `run_variant(...)[0]` 返回**三元组**却和单个字符串比 |
| 7 | `NameError: DECK_ROLLERS` 让构建跑不起来 | 我把 `relocated_c_bodies` 的集合**打字打出来**，凭空造了个常量名 |
| 8 | pytest 一条 FAIL：`'deck mounted on N' in not_established` | 测试**用字面量字符串**去钉一条**否定陈述**；装配一旦成立，这条否定就该被删，测试随即为"它曾经是对的"而失败 |

★★ **第 8 条是本项目的硬教训 8 本人**：*一个靠"断言某句字面量存在"来保证性质的测试，
把那条性质也变成了打字常量。* 已改成断言**性质**（`not_established` 必须仍否证"装配世界里的 C 转移"、
必须提到 docking tolerance；且 `assembly.mount_parent` 要与**派生**出来的名字相符）。

### 五、⚠️ 仍然不能说

- **不能说"C 的转移在装配世界里通过了"** —— 装配**改变了**、但**没有重跑** C 的转移实验。
  C 的转移证据（`reports/p1-c-crown-02`）来自它**自己的单角色世界**（`roller_rig.build_model()`），
  与合并世界是两回事。C 的五个源文件装配前后**逐字节相同**（已验证哈希），所以那份证据本身没有失效，
  但它**不覆盖**装配后的构型。
- **不能说对接公差可以接受** —— 底盘被停在**旧常数假设的那个精确间距**上，公差**没有被扫过**。
- **不能说四项任务在一个世界里跑通** —— 四角色仍分工位、从不交互。
- **不能说人形能走路**、**不能说跨角色物理交接通过**、**不能说 C 的甲板与真 AMR 的安装接口成立**
  （甲板装的是**这个代身底盘**）。
- **不能说"只剩小修小补"**。
'''

assert 'D041' not in src, 'D041 already present'
D.write_text(src.rstrip() + '\n' + entry, encoding='utf-8')
print('DECISIONS: +D041,', len(src), '->', len(D.read_text(encoding="utf-8")), 'chars')
