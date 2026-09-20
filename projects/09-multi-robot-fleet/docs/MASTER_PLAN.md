# 009 多机器人协同工程：总实施方案

版本：设计草案 v1，2026-09-14。全部能力均为待实现；实际进度只能写入 `IMPLEMENTATION_STATUS.md`。

## 1. 最终要交付什么

一句话：用户提交一批站点服务任务，多台机器人在同一真实物理仿真世界中分工，经过共享窄道时预约让行，遇到低电或故障时遵守货物与资源归属规则，并输出可独立核验的结果。

第一版可演示故事：

1. r01 在仓库左区、r02 在右区，各自接到对侧任务。
2. 两车都会到达窄道入口，但不会迎面开进通道。
3. r01 获得通道与出口停车位的组合预约；r02 在不挡路的等待位停车。
4. r01 完全离开共享区域，r02 才获准通行。
5. 新任务进来，调度器根据能力、电量、距离、已有任务决定分配。
6. r01 在取货前故障，确认停止和旧任务失效后，该任务可分给 r03。
7. 如果 r01 已领取逻辑货物，则任务不能凭空转移给 r03；报告需处理，其他任务继续。
8. 低电机器人按条件返回空闲充电位；充电位有人时排队，不叠在一起。

这个故事中的“取货/交付”在 v1 是**站点停车 + 停留时间 + 逻辑货物账本变更**。车辆运动、障碍物和相互碰撞使用物理仿真；没有机械臂、没有真实接触装卸，不得称为完成实体搬运。

## 2. 必做、后做与不做

### v1 必做

- 一个 Gazebo 世界，独立四轮差速模型，静态地图与地图版本检查。
- 2 台机器人全链路，3 台机器人最终回归。
- 每台独立定位/Nav2/适配器/本地安全门；共享地图、时钟、车队调度。
- 结构化任务、能力匹配、电量约束、幂等提交、排队与取消。
- 单窄道互斥、入口等待位、出口占用检查、授权失效后的保守处理。
- 低电、阻塞、通信中断、导航失败、服务端重启等故障测试。
- 真实状态日志、独立仿真验收、简单只读网页、CLI。
- 独立复现说明、固定测试集、来源与许可证、已知限制。

### 后续扩展，不占用 v1 关键路径

- 多交叉口、多资源路线预约、死锁检测和解锁规划。
- 异构能力适配器：AMR、移动机械臂、人形机器人。
- 与 Open-RMF 对接，比较自研小型协调器和成熟框架。
- 人形把货放在交接台、AMR 再执行长距离运输。
- LLM 将自然语言转换为同一任务接口，但无权绕过约束。

### v1 明确不做

- 多人形机器人共同抬同一件物体、跨仿真器接触耦合。
- 分布式无中心共识、任意拓扑最优 MAPF、任意车数扩展保证。
- 动态 SLAM 地图合并、未知环境探索。
- 从零训练行走策略、VLA、世界模型。
- 硬件控制、商业调度器替代、安全认证、全天候无人值守承诺。

## 3. 如何利用 001～008，而不依赖它们

| 经验来源 | 009 利用什么 | 不带进来什么 |
| --- | --- | --- |
| 001 | TF、地图、Nav2 启动与定位经验 | 启动旧工程、读取旧地图路径 |
| 002 | 任务状态、低电返航、阻塞诊断、隔离环境 | 直接把两套任务脚本并排启动 |
| 003 | 导航与作业能力拆分 | 抓取链路作为第一阶段前置条件 |
| 004～006 | 本体适配、动作完成的实际验收 | 人形控制器、权重和资源包 |
| 007 | 失败子原因、几何可达性、独立真值验收 | 未稳定的站位或搬运代码 |
| 008 | 结构化任务、白名单、监督、重规划预算 | LLM 服务或失败的人形放置链 |

可以复制有用的小段代码，但必须记录原始路径、版本/哈希、许可证和修改说明，复制后的模块属于 009 自己。禁止软链接、硬链接、指向其他工程的环境变量或共享虚拟环境。优先重写小型接口，避免复制整套复杂应用。

## 4. 环境策略：先核实，不拍脑袋换版本

### P0 实测清单

在 WSL 目标发行版记录：

```bash
pwd
cat /etc/os-release
uname -a
ls /opt/ros
command -v python3
python3 --version
command -v ros2
command -v gz
printenv ROS_DISTRO
printenv ROS_DOMAIN_ID
printenv AMENT_PREFIX_PATH
printenv GZ_PARTITION
```

这些是只读调查命令，不代表 shell 已正确 source。若 ROS 不在 PATH，先列出安装再显式 source **已核实的一个**版本。不得顺手把所有 setup.bash 都 source 一遍。

进一步记录 `ros2 pkg prefix` 对 `nav2_bringup`、`nav2_msgs`、`ros_gz_bridge`、`ros_gz_sim`、`rviz2` 的结果；读取实际版本的 Nav2 launch、Twist/TwistStamped 配置与 Gazebo 插件参数。`gz sim --versions` 若该版本不支持，记录帮助输出并选择受支持的查询方式。

本机旧项目 README 写 ROS 2 Lyrical，不能强制按 Jazzy 模板安装。**首选沿用已安装、可验证匹配的栈，并冻结实际版本**。如果没有可用组合，提出环境安装方案并等待用户批准。Ubuntu 24.04 + ROS 2 Jazzy + Gazebo Harmonic 可作为另外的候选组合，不授权降级、升级或替换现有系统。

环境选择结论写 `docs/ENVIRONMENT.md`：OS、Python ABI、ROS、Nav2、Gazebo、ros_gz、来源、包版本、RMW、图形/无头支持。版本未知就标未知，不用当前官方默认版本补空。

### Python 与依赖

- ROS 模块使用所选 ROS 对应的系统 Python，不能用不兼容的新 Python 加载 rclpy。
- P1 无 ROS 核心允许项目独立 `.venv`，由对应系统 Python 创建。
- 如需在 venv 中加载 ROS，明确记录 `--system-site-packages`，验证 `import rclpy`；不能依赖别的工程 `.venv`。
- colcon 的解释器与 shebang 必须检查，避免“pytest 用一个 Python、ROS 节点用另一个”。
- 不执行 `pip install rclpy` 充当 ROS 安装；不无条件 `sudo apt upgrade`。
- 最小依赖：pytest、PyYAML、标准库 SQLite；ROS/仿真包按实际发行版安装。网页先标准库只读 HTTP，不引入大前端工具链。
- shell 脚本 LF 换行、正确执行权限，项目文件不写死 `/mnt/c/Users/...`。

### 隔离

- 一个车队内所有机器人使用相同 ROS_DOMAIN_ID；不能每台不同 domain 后期待直接互通。
- 为 009 选经核查未冲突的专用 domain，写配置而非写入全局 `.bashrc`。
- 每次仿真生成独立 GZ_PARTITION，所有该次子进程继承。
- domain/namespace 是防串线手段，不是认证或安全隔离。
- 不启动旧项目，不终止用户其他 Gazebo/ROS 进程。

## 5. 架构与所有权

```text
CLI / YAML 任务
       |
Fleet Core：任务账本、能力匹配、分配、资源预约、事件
       |
每机器人 Adapter：命令代次、Nav2 action、状态、取消确认
       |
独立 Nav2 -> 速度链末端 -> Local Safety Gate -> Gazebo 驱动
       ^                         ^
 AMCL / odom / scan        许可、状态新鲜度、停止距离、急停

Gazebo 真值 -> 独立 Evaluator -> 报告（不回流调度或控制）
实际事件与估计状态 -> 只读网页
```

责任边界：

1. 调度器决定“哪台车执行、预约哪个资源”。不发布轮速或 cmd_vel。
2. Nav2 决定局部路径和避障，但不能自行穿过未授权共享区。
3. Adapter 管理动作生命周期；一次只允许一个有效执行代次。
4. Safety Gate 独占最终速度输出；来自 Nav2、行为恢复、速度平滑器的所有速度必须汇入它，不能有旁路。
5. Evaluator 根据真值和执行记录裁判，不参与规划。

第一版集中式车队服务单写者 + SQLite 事务；不做跨机一致性协议。进程内使用队列/锁保证状态迁移串行，ROS 回调不直接修改多处账本。

## 6. 独立目录结构

以下是目标结构，不要求 P1 把所有空模块伪实现出来。

```text
009_multi_robot_fleet/
  AGENTS.md
  README.md
  LICENSE
  THIRD_PARTY_NOTICES.md
  .gitignore
  .gitattributes
  pyproject.toml                  # 测试/格式配置，核心不依赖 ROS import
  requirements-dev.txt
  run_demo.sh
  stop_demo.sh
  config/
    environment.example.env
    fleet.yaml
    robots.yaml
    stations.yaml
    resources.yaml
    lane_graph.yaml
    safety.yaml
    nav2.template.yaml
    scenarios/                   # 固定测试/演示清单
  assets/
    models/amr_4wd/               # 原创简单模型，四轮差速
    worlds/warehouse.sdf
    maps/warehouse.pgm
    maps/warehouse.yaml
    rviz/fleet.rviz
  src/
    fleet_interfaces/            # ament_cmake；msg/srv/action
      msg/RobotState.msg
      msg/ResourcePermit.msg
      srv/SubmitTask.srv
      srv/CancelTask.srv
      srv/AcquirePassage.srv
      srv/ConfirmClear.srv
      srv/RenewPermit.srv
      action/ExecuteLeg.action
    fleet_core/                  # ament_python；内部核心文件可无 ROS 测试
      fleet_core/
        domain.py
        validation.py
        task_machine.py
        allocator.py
        resources.py
        ledger.py
        recovery.py
        clock.py
        events.py
        ros_node.py
    fleet_adapter/               # ament_python
      fleet_adapter/
        adapter_base.py
        fake_adapter.py
        nav2_adapter.py
        safety_gate.py
        battery_model.py
        state_publisher.py
    fleet_bringup/               # ament_python；launch/config 渲染
      launch/world.launch.py
      launch/robot.launch.py
      launch/fleet.launch.py
    fleet_evaluation/            # 与控制隔离的真值采集/验收
      fleet_evaluation/
        collector.py
        evaluator.py
        geometry.py
        report.py
    fleet_tools/                 # CLI、故障注入、只读网页
      fleet_tools/
        cli.py
        fault_injector.py
        dashboard.py
      static/
  scripts/
    env.sh
    doctor.sh
    bootstrap.sh
    build.sh
    test_core.sh
    test_integration.sh
    batch.py
    validate_assets.py
    render_config.py
  tests/
    unit/
    contracts/
    integration/
    simulation/
  docs/
    MASTER_PLAN.md
    CONTRACTS.md
    TEST_AND_ACCEPTANCE.md
    AI_EXECUTION_PROMPTS.md
    IMPLEMENTATION_STATUS.md
    ENVIRONMENT.md
    ARCHITECTURE.md
    TROUBLESHOOTING.md
    LIMITATIONS.md
    DECISIONS.md
  reports/                       # 每次独立目录，不进 Git
  runtime/                       # SQLite、PID、临时渲染配置，不进 Git
  build/ install/ log/ .venv/     # 不进 Git
```

新增 ROS 包须具备正确 package.xml、setup.py/setup.cfg 或 CMakeLists、resource index、依赖声明。不要仅创建 Python 文件就声称 `ros2 run` 能运行。

`env.sh` 只 source 本项目认可的 ROS/overlay；所有资源路径由项目根路径/ament share 解析，不能依赖调用者 cwd。启动失败打印具体缺包、配置和日志位置。

## 7. 世界与机器人：先简单但能测协作

### 7.1 初始几何设计（须通过资源校验才能冻结）

- 地图范围建议 x∈[-7,7] m、y∈[-5,5] m，分辨率 0.05 m。
- 两个工作区由中间实体隔断分开；唯一贯通通道 x∈[-1.5,1.5]、y∈[-0.6,0.6]。
- 通道上下实体障碍延伸到外墙，不能从背后绕过。
- 初始车体外接长度约 0.60 m、宽度约 0.45 m；轮子等突出部分必须计入 footprint。
- 建议初始限速 0.35 m/s、限转速 0.6 rad/s；实测停止距离后冻结。
- 左右候车位分别建议 (-2.5,1.4)、(2.5,1.4)；出口缓冲位分别 (-2.5,-1.4)、(2.5,-1.4)。入口对齐点约 (±2.3,0)。
- 取放站分布在 x≈±5、y≈±2.5；两处充电位分布在 x≈±5、y≈-3.8；实际碰撞余量经校验确定。
- 出口缓冲位之外再设 release 节点，例如 (±3.5,-1.4)；车辆到这里并确认整车离开保护区后，才能释放整组预约，不能刚到仍在保护区内的缓冲位就释放。
- 初始出生位不能占候车位、出口位、通道或充电对接区域。

这些坐标只是首个资产设计，不是已经验证的最优布局。若转弯外轮扫掠会撞墙，P2 调整并重新冻结资产，不能关闭碰撞。服务站是地面停车区，不要求靠实体台面进行厘米级对接。

### 7.2 避免隐含的第二个死锁

入口、通道、出口缓冲及其连接弯道作为一个“受保护穿越区域”。预约通道时原子预约出口缓冲位；该区域内一次仅允许一个穿越者。

等待机器人留在独立候车位，不能堵住刚出通道的车。候车位满则留在其他合法停车位，不在行驶线上随意停车。站点和充电区各有独立容量。

第一版只允许持有一组穿越资源，不允许占着通道继续等待第二条通道；多路口以后另行设计。资产校验要验证候车路径与受保护区域的关系，不能只画一张好看的地图。

### 7.3 物理模型与传感器

- 四轮差速/滑移转向，不称为阿克曼或全向底盘。
- SDF 模型含真实质量、惯性、碰撞几何、四个轮关节、激光传感器、DiffDrive、关节状态。
- 驱动使用所选版本 Gazebo 对应插件；left/right joint 列表、frame_id、topic 显式设置，不能依赖同名默认值。
- 不从真值 Pose 给控制层“完美里程计”。若底层插件实际提供理想化运动学里程计，必须记录来源与限制，不能声称验证真实轮滑。
- 一张静态地图；每车 AMCL，初始化位姿来自场景初始化配置，运行中不得用真值不断重置 AMCL。
- 激光可见其他车与障碍；不能关闭车车碰撞来解决拥堵。

## 8. TF 与话题：统一采用一套方案

v1 选用 **共享 `/tf`、`/tf_static`，唯一 frame 名，共享 map**：

```text
map
 ├── r01/odom ── r01/base_link ── r01/laser_link
 ├── r02/odom ── r02/base_link ── r02/laser_link
 └── r03/odom ── r03/base_link ── r03/laser_link
```

- `map→rXX/odom` 只由该车 AMCL 发布。
- `rXX/odom→rXX/base_link` 只由选定里程计/桥接发布。
- 车体静态/关节 TF 只由该车 robot_state_publisher 等指定发布者负责。
- namespace 不会自动给消息内部的 frame_id 加前缀，必须从模型到激光、里程计、Nav2 参数逐项检查。
- 若 Nav2 launch 默认把 `/tf` 重映射成相对 `tf`，本项目包装 launch 必须显式一致地接回共享 TF；不能盲目复制发行版 launch。
- URDF link 本身加前缀，或 robot_state_publisher frame_prefix 二选一，不能双加成 `r01/r01/base_link`。
- 全局共享 `/map`，durability/QoS 与 map_server、AMCL 匹配。每车 costmap 的 map_topic 显式 `/map`。

每车接口：`/r01/scan`、`/r01/odom`、`/r01/joint_states`、`/r01/initialpose`、`/r01/navigate_to_pose`、`/r01/cmd_vel_nav`、`/r01/cmd_vel`。全局：`/clock`、`/map`、`/tf`、`/tf_static`、`/fleet/*`。

桥接：一个 Gazebo `/clock`→ROS `/clock`；各车 scan/odom 分开。TF 桥接消息型及 Gazebo TF topic 根据实际版本核查，不编造可运行配置。

速度类型以 P0 所选 Nav2 版本实际配置为准；全链 Twist 或 TwistStamped 必须匹配，有转换器则显式测试。Gate 必须位于平滑器等模块之后，最终 `/rXX/cmd_vel` 只有它一个 ROS 发布者。

## 9. 分阶段实现与交付门

### P0：环境与独立脚手架

读完整交接包；核对目标目录与旧工程只读状态；记录环境；复制完整 AGENTS；创建状态文档、来源说明与 ignore；写 doctor 和只读依赖检查。无权限安装时继续纯核心，不冒充仿真环境就绪。

交接包在本机 Windows 的路径为 `C:\Users\ZiLing\Documents\ChatGPT\GitHub\output\009_handoff`。如果 WSL 按默认方式挂载 C 盘，可在确认路径存在后采用以下初始化步骤。**这些命令由接手 AI 在用户批准开发时执行，本次未执行；已存在目标必须走检查/续作，不运行此新建分支。**

```bash
set -euo pipefail
project_root=/home/ziling/projects/009_multi_robot_fleet
handoff_root=/mnt/c/Users/ZiLing/Documents/ChatGPT/GitHub/output/009_handoff
test -f "$handoff_root/AGENTS_TEMPLATE.md"
if test -e "$project_root"; then
  printf '%s\n' '目标已存在：停止初始化，先检查现有内容。'
  exit 1
fi
mkdir -p "$project_root/docs"
cp "$handoff_root/AGENTS_TEMPLATE.md" "$project_root/AGENTS.md"
for name in MASTER_PLAN.md CONTRACTS.md TEST_AND_ACCEPTANCE.md AI_EXECUTION_PROMPTS.md; do
  cp "$handoff_root/$name" "$project_root/docs/$name"
done
```

这里仅复制已提供的交接文档；后续新源码与状态文档使用开发工具的补丁编辑。若 C 盘没有这样挂载，查实际位置，不猜另一台机器路径。项目不应在运行时读取上述 handoff_root，复制完成后即独立。

交付：目录存在、文档齐全、doctor 有明确退出码、现有项目未改。项目 README 写“开发中”。

### P1：纯 Python 协调核心

顺序：domain/validator → SQLite ledger → task machine → allocator → resources → clocks → fake adapter → fault tests。

以明确输入事件推进状态，避免 `sleep` 驱动测试；假时钟同时支持 sim 与 monotonic。先测试去重、迟到回调、通道超时不释放、货物不瞬移，再写 happy path。不要为了 pytest import 启动 rclpy/Gazebo。

交付：核心/契约测试全通过；fake demo 会输出正确标识；没有真实导航声明。P1 不需要图形界面。

### P2：一台真实仿真机器人

原创模型、世界、地图与资产校验；单车 ROS bridge/TF/AMCL/Nav2；速度 Gate 和 watchdog；启动 readiness；单车停靠、往返、障碍停车。

先把四轮模型开稳，确认停止距离，再进入多车。不把小车精确对接难题扩大到 007 的人形控制难题。

交付：单车矩阵通过、记录真值与估计误差、冻结 footprint/限速/停靠阈值；没有 fake fallback。

### P3：两台车隔离与任务闭环

参数化 spawning、namespace、frame；一张地图两套定位/Nav2；接真实 ExecuteLeg；从 CLI 到账本再到停靠的完整链路。

本阶段先在不争抢通道的分区路线测试。单车 goal 不能让另一台动；取消 r01 不能取消 r02；只 r01 低电不能改 r02 电量。

交付：双车互不串线、全链任务可核验，未完成 P4 前不宣称协同交通通过。

### P4：窄道协同（核心亮点）

先接资源请求/许可/续期与本地速度门，再接实际 staged navigation，最后做对向、排队、出口占用与通道失联测试。导航不得绕开 gate。

不能靠对两台车写不同起步 sleep 冒充动态预约。测试要交换请求顺序、同时请求和改变任务到达时间。

交付：无同时占用、无未授权进入、无正常场景饥饿；明确故障封锁而非永久静默等待。

### P5：三车、充电与接管

增加 r03 只改配置；测试能力过滤、低电准入、充电容量、取货前安全接管、持货不可接管、adapter/调度器重启。正常吞吐与故障阻塞分别统计。

不要求在唯一通道永久被占时仍完成所有跨区任务；要求可诊断终止/等待政策、仍可执行不受影响的同区任务。

### P6：只读展示与诊断

RViz 展示各车路径、候车区、预约区域和站点。网页展示任务归属、状态、电量、资源所有者/队列、故障原因、事件时间线。数据来自运行事件，不是动画脚本。

网页默认只绑定 localhost，不做控制按钮；CLI 承担受验证的提交/取消/故障注入。物理急停不称为真实硬件安全功能。

### P7：冻结、回归和独立复现

冻结资产/配置/测试清单；执行 TEST_AND_ACCEPTANCE 中的完整矩阵；报告失败不隐藏。将项目复制到独立临时路径测试不依赖旧工程或原始绝对路径，保留报告。

整理 README 运行步骤、演示视频位置、局限、来源与许可证。没有用户单独授权不提交/推送 GitHub。

## 10. 不允许无限调试循环

每次实验记录：复现用例 → 假设 → 证据 → 唯一修改 → 定向对照 → 回归结果。

同一失败两次针对性实验无改善，停止扩大批测，先补诊断：

- WAIT_RESOURCE：谁占资源？出口是否空？授权代次是否正确？
- NAV_STALLED：服务器没响应、Gate 挡住、真实障碍、定位错误，还是速度为零？
- BUDGET_EXHAUSTED：等待占多少、导航占多少、仿真实时因子多少？
- 机器人没动：逐级检查 Nav2 action→速度→Gate拒绝原因→bridge→wheel joint，别先改容差。
- RESOURCE_UNKNOWN：丢的是状态、许可、清空证明还是中央服务？

不能从一次超时推导“达到物理极限”，也不能从一次成功宣称无回归。需要增加运行预算时，保留旧结果，解释预算来源，更新场景版本，不能混到旧统计中。

## 11. 后续多人形/异构路线

v1 的 adapter 接口保持本体中立：位置、footprint、运动限制、支持技能、状态、执行与取消。

后续先加一个 **fake humanoid adapter 契约测试**，证明任务按能力分配；不称为人形实测。真实人形接入时替换运动适配器，重新验证步行停止距离、身体摆动 footprint、许可失效停止与交接验收，不能直接套小车安全参数。

异构运输建议用交接台：人形完成放置并经独立验收，站点账本确认物体可领取，再由 AMR 的装卸设备/人工作业明确确认；只有设备齐全才做实体交接。最初逻辑交接必须如实标记。

同时创建多个 MuJoCo 实例不等于共同世界；没有共同碰撞与空间占用的多窗口演示，不算完成多人形物理协作。

## 12. 技术参考（核查于 2026-09-14）

这些是实现查阅入口，不是让 AI 把 main 分支代码无条件复制到本机版本。

- [Gazebo Harmonic ROS 2 集成](https://gazebosim.org/docs/harmonic/ros2_integration/)：确认桥接与仿真/ROS 之间的数据交换方式。
- [ros_gz 兼容表](https://github.com/gazebosim/ros_gz/blob/ros2/README.md)：查安装组合；Jazzy/Harmonic 仅为可选基线，不替换本机已核实栈。
- [Gazebo Sim 8 DiffDrive](https://gazebosim.org/api/sim/8/classgz_1_1sim_1_1systems_1_1DiffDrive.html)：多左右轮关节、显式 topic/frame 参数。若用其他 Gazebo 大版本，必须查对应文档。
- [Nav2 Jazzy navigation launch 源码](https://github.com/ros-navigation/navigation2/blob/jazzy/nav2_bringup/launch/navigation_launch.py)：观察 namespace、TF remap 与速度链；实际实现查所选发行版。
- [Nav2 导航插件配置说明](https://docs.nav2.org/jazzy/configuration_and_development/first_time_robot_setup_guide/navigation_plugins/setup_navigation_plugins/)：规划/控制插件与 footprint 选择；不能把二维路径当作任何外形机器人都可安全通过的证明。
- [Open-RMF Python fleet adapter](https://github.com/open-rmf/rmf_ros2/blob/main/rmf_fleet_adapter_python/README.md)：后续适配成熟协调框架的参考。v1 不安装或声称兼容 RMF，除非另有适配与验证。

本方案中的集中式预约、保护区划分、测试预算与状态机是项目设计选择，不声称来自这些文档的现成实现，也不声称达到工业安全等级。
