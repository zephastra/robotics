# P1-A-04 侦察：固定机械臂与夹爪的模型来源

日期：2026-09-20。范围：`P1-A-04` 的**第一步**。
上游依据：`P1_FEASIBILITY.md` §A"先核查可用模型/夹爪来源与许可，再建立独立副本"；
`MASTER_PLAN.md` §4 复用协议；`RISKS_AND_SCOPE.md`"若缺模型，先报告下载体积/许可再申请大型资源下载"。

**本轮只读：未下载、未安装、未创建资产目录、未改动 `assets/` 与 `src/`。**

## 1. 结论

| 部件 | 状态 | 依据 |
| --- | --- | --- |
| **夹爪** | ✅ **已具备且已审计** | `assets/allegro/`（2.5 MB）Allegro Hand V3，**BSD-2-Clause**；左右手各 16 关节（`ffj0–3 / mfj0–3 / rfj0–3 / thj0–3`）、每份 XML 25 个执行器标签；逐文件 sha256 已在 `assets/manifest.json`；上游 revision `8161bba264d7fa7c99ca301e91e7fb44737676ad` |
| **独立机械臂** | ❌ **本机不存在** | 在 `~/projects` 按名称检索（`franka/panda/ur5/ur10/xarm/kuka/iiwa/kinova/gen3/gripper/allegro/shadow_hand/arm*.xml/arm*.mjcf`，prune `.venv/.git/site-packages/build/install`，深度 5）⇒ **0 个独立臂模型**；`~/.mujoco` **不存在**；Menagerie 缓存（005/006/007 各 5.6 MB）**只含 `wonik_allegro`** |
| **可替代的臂** | ⚠️ 有，但不合适 | T800 自身的臂，见 §3 |

唯一一处臂类命中：`004_humanoid_locomotion/.cache/unitree_rl_gym/resources/robots/g1_description/g1_dual_arm.xml`
（Unitree G1 双臂 MJCF）。它是**人形臂**且位于**另一工程的下载缓存**内 —— 要用必须走
`MASTER_PLAN.md` §4 的复用审计（记录来源、哈希、许可、限制、复制日期），不能直接引用旧路径。

ROS 侧同样没有臂：`/opt/ros/lyrical/share` 内与操纵相关的只有 `ros2_control`、
`gz_ros2_control`、`parallel_gripper_controller`、`nav2_minimal_tb4_description`，
**没有任何臂的 description 包**。

## 2. 010 现有资产（均已审计）

| 路径 | 体积 | 许可 | 内容 |
| --- | --- | --- | --- |
| `assets/t800/` | 60 MB | EngineAI **BSD-3-Clause** | 25 关节串行模型 + 22 动作行走策略 `policies/t800/walking.mnn`；上游 `engineai_robotics_native_sdk @ 335c60e88772c26c7852d0abd6b3c7439037dd8f` |
| `assets/allegro/` | 2.5 MB | Allegro **BSD-2-Clause** | 左右手 XML + 14 个 STL |
| `assets/objects/` | 12 KB | 本工程 | `tray_v1.xml`、`payload_007.xml` |
| `assets/worlds/` | 12 KB | 本工程 | `world_n_probe.xml` |
| `assets/combined.xml` / `world_tray_v1.xml` | 各 60 KB | 本工程 | 由 `experiments/make_world.py` 生成 |
| `assets/maps/` | 44 KB | 本工程 | `arena_n_probe.pgm/yaml` |

清单文件是 `assets/manifest.json`（T800 + Allegro，逐文件 sha256）与 `assets/manifest-t800.json`；
工程级来源清单在 `docs/P1_SOURCE_MANIFEST.json`（已确认存在）。
`assets/manifest.json` 记 `model = "T800 + bilateral Allegro + head RGB-D camera"`，并记录三处改动：
双边刚性手部转接件、**替换两处腕位占位球**、头部相机下俯 35°。

## 3. 为什么 T800 的臂不能直接当 V1 的"固定臂"

从 `assets/t800/robot/t800/xml/serial_*` 实测的 25 个关节：

| 区段 | 关节 | 含义 |
| --- | --- | --- |
| `J00–J11` | HIP×3 / KNEE / ANKLE×2（左），右同 | 腿 |
| `J12` | TORSO_YAW | 腰 |
| **`J13–J17`** | **SHOULDER_PITCH / ROLL / YAW、ELBOW_PITCH / YAW** | **左臂 = 5 自由度** |
| `J18–J22` | 同构 | 右臂 = 5 自由度 |
| `J23/J24` | HEAD_PITCH / HEAD_YAW | 头 |

**每条臂只有 5 个自由度，且没有腕关节。** 这与 `assets/manifest.json` 的改动记录一致：
007 时期 Allegro 是**直接替换腕位占位球**装在肘后的，所以这条臂链上不存在独立腕。

因此用它当 V1 的固定臂有三个问题：

1. 5 DoF、无腕 ⇒ 姿态可达性受限，"把零件放进托盘料位"需要的手腕定向能力不足；
2. 它与人形是**同一个平台**，而 `MASTER_PLAN.md` §1 的 V1 配置写的是"1 人形、**1 固定臂**"
   （两个独立设备）；混用会让 `P1-GATE-07`"四实验各自独立"的意义变弱；
3. 需要新建底座装配，属未记录的新改动。

**可以用，但要你先明确"V1 的固定臂 = T800 臂"这一范围解释。**

## 4. 候选臂（均需下载 ⇒ 均需你的授权）

来源：MuJoCo Menagerie（`github.com/google-deepmind/mujoco_menagerie`）。
**逐模型许可不同**，下表许可取自各模型**自己的 README（上游原文，非二手）**：

| 模型 | DoF | 许可 | 上游声明的引擎要求 | 适配度 |
| --- | --- | --- | --- | --- |
| **`franka_emika_panda`** | **9**（7 臂 + 2 夹爪） | **Apache-2.0** | MuJoCo ≥ 2.3.3 | ★★★★★ |
| `universal_robots_ur5e` | 6 | **BSD-3-Clause** | MuJoCo ≥ 2.3.3 | ★★★★ |
| `kuka_iiwa_14` | 7 | BSD-3-Clause | — | ★★★ |
| `kinova_gen3` | 7 | BSD-3-Clause | — | ★★★ |
| `ufactory_xarm7` | 13 | BSD-3-Clause | — | ★★ 过重 |

本机 MuJoCo **3.3.6** ⇒ 满足两者的 ≥ 2.3.3。许可方面 BSD-3-Clause 与 Apache-2.0
与本工程既有的 EngineAI BSD-3 / Allegro BSD-2 / 集成代码 Apache-2.0 **全部兼容**。

**体积**：Menagerie 全量约几 GB，但每个模型是独立顶层目录，可只取一个：

```bash
git clone --depth 1 --filter=blob:none --sparse \
    https://github.com/google-deepmind/mujoco_menagerie.git
cd mujoco_menagerie && git sparse-checkout set franka_emika_panda
```

单模型目录为**数 MB 量级**（二手来源，**申请授权前会先实测确认**）。
2026-09 起该仓库另提供 `mujoco-menagerie` Python 包（按名下载到内容寻址缓存，含摘要 `verify`）——
装包属"安装依赖"，同样需要授权。

## 5. 建议

**推荐 `franka_emika_panda`。** 理由：

1. **臂 + 平行夹爪一体**（`panda.xml` / `hand.xml` / `panda_nohand.xml` / `scene.xml`），
   而 `P1-A-04` 的任务是"夹持、离台、放置、松手稳定"、V1 的固定臂任务是
   "逐件抓放到托盘独立料位" —— 两指平行夹爪正是把方块/圆柱放进料位的工具；
2. **Apache-2.0**，与本工程现有许可全部兼容；
3. 是 MuJoCo 里最常用的抓放臂，出问题时参考资料最多；
4. 自带 `mjx_single_cube.xml`（单方块场景），可作为最小抓放场景的起点参考。

**备选 `ur5e` + 已有 Allegro**：若你希望 `A` 复用 005/007 的手部经验、而不是引入新夹爪。
代价是要自建法兰→手基座转接件并重新标定（007 已有同类 "rigid hand adapter" 先例可循）。

## 6. 本轮未做

未下载、未安装、未创建新目录、未改动 `assets/` 与 `src/`。
`P1-A-04` 仍**不是** DONE —— 卡在"模型未选定"，需要一个用户决策（见 `docs/DECISIONS.md` D026）。
