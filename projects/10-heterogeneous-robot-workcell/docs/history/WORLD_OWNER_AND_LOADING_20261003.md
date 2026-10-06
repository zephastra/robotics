# 010 共享最终写者与真实运输托盘加载 · 2026-10-03

## 续作检查点：接收端实物验收与无真值视觉

旧报告全部保留，不改旧世界与原 110 项阈值。只开发 010，不推送。

1. `p4-global-transfer-writer-20261003-01`：exit 0，24/24 PASS。四故障中先调用实际 `plant.transfer()`，再调用 H/A/C/N/N2；各入口均拒绝，qpos/qvel/ctrl/time 不推进，恢复健康信号也不自动解锁。只是 SIM_FROZEN，不是物理停止。
2. `p4-loaded-receiver-20261003-01`：exit 1。实际两红一蓝→11 物流技能成功，但加强必需项后整体 FAIL。三件接触真实托盘；接收端停稳速度界 0.1808422804 m/s、角速度峰 0.501723 rad/s，超过原 0.01m/s 门槛。相机报 UNKNOWN，不把订单数量作为实测。
3. 独立 v4：在 v3 接收段主辊之间加 38 个分段被动小辊，原主辊/执行器/旧 v3 未改。`p5-receiver-support-v4-build-20261003-01` 首次审计误把 idler 算作主辊，FAIL 保留；主辊改按 `c_recv_roller_数字` 身份统计，重建一致检查 `...-02` 静态 12/12 PASS。`p5-receiver-support-v4-settle-20261003-01` 原稳定性正/负例 2/2 PASS。
4. `p4-loaded-receiver-v4-20261003-01`：exit 1，不是成功订单；接收停稳已 PASS（速度界 0.001044281 m/s、漂移 0.000004085 m），三个实物接触均 PASS，仅视觉模板失败。
5. 相机原缺陷是**世界轴 AABB 不等于转动托盘轮廓**，不是一张新图能覆盖旧图。已录制真实 RGB-D：`receiver_rgbd.npz`。从实际点云拟合朝向约 -0.25049 rad。底板边缘被壁遮住，不能拿底板可见宽度去比整盘目录尺寸；目录上沿高于底板 55 mm，其可见外框 0.129973×0.459990m，保持原 6mm 尺寸门即可解出。
6. 零件也可独立于托盘转动。第一次离线上沿复核 `p4-receiver-recorded-rim-20261003-01` 解出盘但红件世界 AABB 尺寸误差 7.547mm，FAIL 保留。第二版按每个零件自己点云的最小面积矩形检验，仍用原 6mm 判据；`...-02` 5/5 PASS。独立读出两红一蓝，三格齐全，尺寸误差最大约 1.552mm；缺深度、遮去一部分托盘均 UNKNOWN。**这是录制传感器复核，不替代新源码在线执行。**
7. `PhysicalTransferSession` 新层 8 项纯契约测试通过：复用原 P2 资源和货权账本，运动前两端占用，缺确认保留 TRANSFERRING/NEEDS_ATTENTION，占用不因租约到期被释放。尚未物理接入。后续必须解决两腿之间已有资源占用的有证据交接，不能为启动下一腿虚构清空/改 FREE。
8. 现有 PCL 1.15.1/pkg-config/g++ 已核实。新增真实 PCL 体素滤波＋RANSAC 接收平面探针源码；尚未编译/实测，不能声称 PCL 已接进订单。

新命令（每条均使用唯一 run ID、有界 runner；只能单物理 worker）：

```bash
.venv/bin/python scripts/run_bounded.py --run-id p5-receiver-support-v4-build-20261003-02 --timeout 180 experiments/build_support_candidate.py --integrated --split --receiver
.venv/bin/python scripts/run_bounded.py --run-id p5-receiver-support-v4-settle-20261003-01 --timeout 180 experiments/probe_p5_candidate_settle.py --world world_p5_candidate_v4.xml
.venv/bin/python scripts/run_bounded.py --run-id p4-receiver-recorded-rim-20261003-02 --timeout 120 experiments/probe_recorded_receiver.py --recording p4-loaded-receiver-v4-20261003-01
.venv/bin/python scripts/run_bounded.py --run-id p4-loaded-receiver-v4-20261003-02 --timeout 900 experiments/probe_candidate_multi_loading.py --transport --world world_p5_candidate_v4.xml
```

最后一条此检查点仍运行；最终退出码须从 runner manifest/acceptance 读取。旧 771 项回归不是新源码的全量结果。P4 仍 PARTIAL；连续人形供盘、Nav2、PCL 在线感知、货权事务恢复、双车/库存、36 实例/P7 发布未完成。

## 权限与结论

用户要求持续完成 010。沿用已授权独立集成候选及被动支撑；不改旧冻结世界、停止验收阈值或 001～009，不推送 GitHub。

本轮完成的是新的组件门，不是完整订单：

1. 共享 `WorldOwner` 对取消/占区/过期/停止链异常锁闭提交与 mj_step，24/24 PASS。
2. 同样四类故障从真实 T800 `Runtime.step` 入口注入，24/24 PASS。人形控制已拆出不写 ctrl、不 step 的 `control()`；旧默认步进仍兼容；H3 可选 `--global-writer`，其完整长链 NOT_RUN。
3. v3 真正自由运输托盘（非 p_fixture），Panda 用 RGB-D 决策抓起红件，实际接触夹持、搬运、释放到第 1 格；独立 RGB-D 计数 1 红、0 蓝，缺深度 UNKNOWN，13/13 PASS。

## 物理与信息边界

- 使用 `assets/world_p5_candidate_v3.xml`，SHA256 `568a31060d6fd82b1bf345e6dd1ec1a9c78766b371ca18c6276229b47e5d9f30`；只在独立 probe 的编译模型增加固定相机、声明红件颜色。相机不渲染获批 group 5 非物理光学标记；碰撞、质量不变。
- 托盘在首次 step 前初始化到源端装盘站。目标料位来自此时声明的静止工位几何，不从运行中真实托盘位姿反算；真实零件位姿和最终托盘位姿仅作裁判。没有运行时 qpos 写入、weld、关碰撞、重新加载世界来完成动作。
- RGB-D 读者按饱和红/蓝色先掩膜，再对深度世界点聚类；不读取订单或物体标签。原最少像素、尺寸误差和定位门槛沿用。新读者的颜色排斥、地板可见像素及内容高度是受限读者声明，不是改动旧停止/物理验收阈值。
- 每格地板需可见；过高遮挡、无深度、彩色小碎片、未解析组件返回 UNKNOWN。不是任意遮挡/任意材质的完整可观测性证明；非彩色低矮遮挡与真实传感器还需专项验证。不是 PCL、不是真机安全认证。
- `WorldOwner` 的拒绝是显式仿真冻结，`stopped_confirmed=False`；不是机械停止。原 H3 默认路径兼容且仍未被强制迁移；禁止宣称全部旧控制已完成全局监督。
- 其它角色的四故障 proof 验证提交入口，不等于它们全部实际技能同时执行；外部许可观测为声明理想证据，不是独立安全传感器。

## 实测与保留失败

| run ID | 判决 | 范围 |
|---|---|---|
| p4-global-writer-20261003-01 | 24/24 PASS | 共享提交门四故障，仿真冻结 |
| p4-real-humanoid-writer-20261003-01 | 24/24 PASS | 真实 Runtime.step 入口，四故障 |
| p4-real-tray-arm-20261003-01 | 9/9 PASS | 真实托盘第 1 格红件抓放，尚无计数 |
| p4-real-tray-count-20261003-01 | 13/13 PASS（较弱读者） | 未加入过高遮挡拒绝，不能代表最新代码 |
| p4-real-tray-count-20261003-02 | FAIL | 强化遮挡拒绝后，初始臂挡盘，正确 UNKNOWN；保留 |
| p4-real-tray-count-20261003-03 | 13/13 PASS | 实际退臂后核验空盘、抓放、退臂后独立计数 |

首轮单元遮挡测试错误只遮到第 0 格前 75 行，仍有可见地板，修正为按 cell 区间构造遮挡，并额外覆盖超过内容高度拒绝。没有通过改阈值抹掉失败。

## 命令与退出码

均在项目根目录，用 `scripts/run_bounded.py` 自带 PID、wall deadline、日志、唯一报告 ID。已执行：

```bash
.venv/bin/python scripts/run_bounded.py --run-id p4-global-writer-20261003-01 --timeout 180 experiments/probe_world_owner.py
.venv/bin/python scripts/run_bounded.py --run-id p4-real-humanoid-writer-20261003-01 --timeout 180 experiments/probe_world_owner.py --real-humanoid
MUJOCO_GL=egl .venv/bin/python scripts/run_bounded.py --run-id p4-real-tray-count-20261003-03 --timeout 180 experiments/probe_candidate_arm_loading.py
.venv/bin/python -m pytest -q tests/test_h3_integration.py tests/test_humanoid_world_owner.py tests/test_tray_rgbd_reader.py tests/test_world_owner.py
```

上述四条 exit 0；短测试 42 passed / 6.41 s。报告自行记录世界与源码哈希。

## 未完成与下一门

蓝色圆柱/多件连续装盘、其余料位物理验证、源端停驻定位误差与噪声包络、同一托盘连续 H→装盘→车辆→接收、甲板/接收段机械停止、保留 custody/资源闭锁的中断事务恢复、Nav2 真正进入共同物流链、双车/有限库存、F/I 矩阵、P7 36 实例复现均未完成。

P4-BELT-03 仍 PARTIAL；P5/P6/P7 不标 DONE。最终全量测试、冻结/交接 check 结果与其它料位实测将独立补充，不用上轮 752 项替代新代码。

## 本轮续作检查点（连续物流运行中）

- 第 0/2 格单件各 13/13 PASS，三格独立验证齐全；不等于连续三件。
- `p4-multi-loading-bias-comp-20261003-01`：连续三件 9/9 PASS，实际零件全抬起、落在三格并接触托盘；独立 RGB-D 读出 2 红/1 蓝。新加库存识别与计数短测试 13 passed / 0.66s。
- 新增两个自由库存体在首次 step 前初始化，包含蓝圆柱；相机变为 640×480，原尺寸/像素/定位门槛未放宽。理想机器人模型 `qfrc_bias / position_gain` 补偿需传动比 1 并遵守原 ctrl 限制；这不是未知真机参数的能力。2s 到位 reserve 是新控制器预算，不是改旧验收窗口。
- 失败均留在各独立目录：`p4-multi-loading-20261003-01/-02/-03`、`p4-multi-loading-diagnostic-20261003-01`、`p4-multi-loading-clear-view-20261003-01`、`p4-multi-loading-rear-observe-20261003-01`、`p4-multi-loading-ik-seed-20261003-01`、`p4-multi-loading-tracking-20261003-01`、`p4-multi-loading-hold-fix-20261003-01`。诊断图片和 segmentation 仅供裁判，绝不传入控制/计数函数。
- 原多件日志“completed physical item”只是动作结束，不是完成证据；IK-seed 版本实际上未夹起，两次空手动作不能叫两件完成。后续有双指同一库存体理想接触闭锁；支撑/抬起/计数裁判继续独立。理想触觉借用碰撞体接触身份，不是真机触觉识别能力。
- 单件/共享写者版本全量：771 passed / 1 skipped / 192.99s / exit 0（`runtime/pytest-world-owner-20261003.log`）。后续新 multi/transfer 补丁的最终全量 NOT_RUN。
- `WorldOwner.commit` 支持零时间控制提交；真实 `transfer()` 的两处驱动步进与终态 hold 也必须走该门，先前只测 _tick 不足以证明真实输送守门。新增真实 transfer 假对象契约 21 项 PASS（含旧守卫/终态），未宣称实际装载转移中断已经验收。
- 当前唯一物理 run：`p4-loaded-continuous-20261003-01`，1200s wall deadline，PID 586/启动 tick 以 `runtime/bounded_runs/p4-loaded-continuous-20261003-01/manifest.json` 为准。前两物流技能 MOVE_TO_STATION/DOCK 已报告成功，后续接盘/运输/卸盘仍运行。没有在接入物流时重置托盘/新建模型。
- 该 run 即使物流通过，也只能是“实际装盘后的连续物理物流诊断”：未走货权事务、未接 Nav2（旧理想本体状态控制）、接收端独立视觉未做、人形供盘尚未接入，绝不是 P5 完整订单。

### 连续物流最终结果

`p4-loaded-continuous-20261003-01` 已退出，exit 0，310.17194s wall、96.1579999999573s sim。原 11 技能全部 SUCCEEDED，三件终点分别位于三个料位；没有重置货物/新建世界。实际末端位置：红件 [12.485133,-0.144341,0.843871]、第二红件 [12.549935,-0.018501,0.843216]、蓝圆柱 [12.549296,0.143925,0.842301]m。

此 PASS 只到装盘和额外的运输空间子项：最后三件真实接触/速度窗口、接收端独立 RGB-D、货权事务、Nav2、人形连续供盘仍 NOT_RUN。早版本的顶层 required 只声明装盘，运输为分列子项；**禁止依据顶层 PASS/exit 0 把未声明项读为通过**。下一版 transport profile 必须把运输列入 required 聚合，并加强终态验收。

启动清单 `run_manifest.json` 包含全部 src/experiments/scripts/config 实际启动时内容哈希，不依赖冻结清单内旧 hash；包括本轮 `w4_plant.py` 为 `d35fb74cd34233952f83ea6649f120a25e4ae3237d41e39fe8e0146b02837c8c`。冻结清单在试验期间更新不改变这份实际启动指纹。
