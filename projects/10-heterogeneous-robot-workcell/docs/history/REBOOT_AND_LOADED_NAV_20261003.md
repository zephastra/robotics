# 重启恢复、载荷包络与车轮独占交接

## 已核对

重启后没有残留010物理探针；原29资产/112源码/110阈值冻结检查绿，独立空载profile字节检查绿。
重启前全量`runtime/pytest-shared-nav-20261003.log`已落盘：972 passed/1 skipped，179.94s，exit0。
旧连续供盘成功run的共同18个资产/config指纹未改变。
旧manifest中有12个阈值来源文件可逐字节对照，全部未变；`assets/worlds/world_p3_nav.layout.json`
未录入旧manifest（不能把“缺旧哈希”当成“被修改”）。当前layout仍在冻结清单通过检查。

## 新载荷配置（不是验收）

`loaded_nav_envelope.py`/`make_loaded_nav_profile.py`及独立loaded配置：
声明托盘中心在初始化甲板冠面范围内，托盘球半径0.335m，对任意朝向有效。
同目录红盒/蓝圆柱以保守球半径并入，另加甲板平移和偏航允许量，和完整机构包络取并集。
声明支撑范围仅是候选工作条件，不能证明货物自然保持其中。
原空载配置不覆盖。新profile明确`runtime_gate=NOT_WIRED`、retention/loaded_navigation/full_order=NOT_RUN。
14项测试通过（6.66s）：全方向包含、观测缺失/过期/未来时间/数量未知/区域越界均拒绝。
观测门只能吃RGB-D测量与理想甲板触觉，不从订单数或货物body位置猜。

## 实际独占交接

`wheel_authority.py`显式授权，保存原始停车和原始轮速伺服的callable身份，唯一token对应本次授权。
Nav2使用原始伺服，没有停车航向补偿叠加；旧默认安装路径不改行为。
异常不自动归还：必须导航进程退出、IPC关闭，且实际速度<=0.01m/s、漂移<=0.005m才恢复停车。
伪token/双重授权/活跃生产者/缺停止/NaN/超速/漂移/控制器冲突均拒绝。

实际命令：
```
.venv/bin/python scripts/run_bounded.py --run-id p4-shared-nav-parking-handoff-20261003-01 --timeout 420 experiments/probe_joint_world_nav2.py --duration 60 --parking-handoff
```
exit0，98.446s wall；8执行判据PASS，到达0.135565759m，停止速度1.2574026e-8m/s、漂移1.0065776e-8m。
归还停车后同owner继续1s无安全冻结；原始停稳测量属于归还前窗口，不宣称归还后长期稳态。
独立空载导航裁判`p4-shared-nav-parking-judge-20261003-01` exit0，12/12PASS。
独立裁判只验证导航原始证据，不把执行器的交接PASS冒充独立物理交接判据。
控制交接/原停车/拓扑21项测试PASS（0.32s）。

## 修复验收错误

先新增`error_after_complete`复现：报告已写COMPLETED，但后续失败时裁判仍PASS。
修复为报告存在异常即不满足execution_completed；异常路径也写ERROR。16裁判用例通过（0.07s）。
不能用上游“到达”遮掉下游“交接失败”。旧报告不回写。

## 下一步（仍未完成）

最新增量全量：`runtime/pytest-loaded-nav-authority-20261003.log`，997 passed/1 skipped，269.65s，exit0。
31资产/115源码/110原阈值冻结检查通过；物理探针和pytest全部退出，当前无后台仿真。

1. 初始化前车载RGB-D候选探针：不改旧世界、先验证能独立看清托盘，不直接开整段订单。
2. 车载RGB-D载荷位置观测接入，确认遮挡/过期不会放行；补3D机构碰撞边界。
   目前相机是工位固定机位，不能直接声称运输全程有新载荷观测；需要初始化前增加独立车载光学候选。
3. 同一连续H供盘/真实装载/甲板接收后的Nav2运输和精确停靠交接，不reload世界。

P4仍PARTIAL，P5/P6/P7未完成；不修改001～009、不推送GitHub。
