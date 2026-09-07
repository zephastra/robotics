# T800 本机仿真实测记录

日期：2026-09-06（北京时间；原始运行编号使用 UTC）。环境：WSL2/WSLg、Ubuntu 26.04、Python 3.12.13、MuJoCo 3.3.6、MNN 3.6.1。

## 自动路线

不同初始条件共 20 次，完成 20 次。仿真用时范围 66.11～66.33 秒；完成案例最终位置误差范围 7.21～7.63 cm。

条件：初始 xy 分别在 ±0.04 m、yaw 在 ±0.08 rad 内变化，其余模型、平地环境和控制参数固定。此统计不代表真实机器人可靠性。

[批量概要](evidence-t800/route-summary.json)；逐次完整报告在 `evidence-t800/route/`，包含配置、源代码哈希和策略哈希。

## 图形界面完整任务

结果 `COMPLETED`，仿真时间 66.16 秒，墙钟时间 79.87 秒，最终位置误差 7.57 cm。运行时还存在并行批量测试，耗时不是性能基准。

[完整报告](evidence-t800/gui-route.json)

![实际轨迹](images/t800-route-trajectory.png)

## 原始策略与动态位置保持

原始零速度模式只评价是否运行至指定时长，`COMPLETED` 不代表没有漂移。位置保持依赖任务层的理想定位反馈；G1 零指令下会踏步，T800 本机零指令漂移较小。

闭环保持测试完成 30.01 秒连续保持，最终位置误差 2.40 cm。

| 基线 | 最终平面位移 m | 累计转向 deg | 结果 |
| --- | ---: | ---: | --- |
| zero | 0.044 | -2.3 | COMPLETED |
| stand | 0.024 | -1.5 | COMPLETED |
| forward | 2.927 | 12.7 | COMPLETED |
| backward | 2.210 | 9.3 | COMPLETED |
| left | 0.113 | 316.0 | COMPLETED |
| right | 0.300 | -277.5 | COMPLETED |
| side_left | 2.880 | -2.3 | COMPLETED |
| side_right | 2.134 | 21.0 | COMPLETED |

[基线详细证据](evidence-t800/baseline-summary.json)

## 取消停车

仿真第 0、8、30、45 秒请求取消：4/4 通过。先减速 3 秒，再进行 10 秒闭环位置保持验收。

[取消详细证据](evidence-t800/cancel-summary.json)

## 有限推力实验

相同初始状态，仿真第 8 秒向骨盆施加世界坐标 +Y 方向外力，持续 0.2 秒。下表每种强度仅测试一次，不能据此声称精确的抗推阈值。

| 外力 | 结果 | 原因 |
| --- | --- | --- |
| push-10N | COMPLETED | 完成路线 |
| push-30N | COMPLETED | 完成路线 |
| push-60N | COMPLETED | 完成路线 |
| push-100N | COMPLETED | 完成路线 |
| push-150N | COMPLETED | 完成路线 |
| push-250N | COMPLETED | 完成路线 |
| push-400N | COMPLETED | 完成路线 |
| push-800N | FAILED | FALL_DETECTED |

[扰动详细证据（包含失败）](evidence-t800/push-summary.json)

## 已知限制

- G1 为固定/合并上身的 12 关节腿部模型；T800 为 25 关节模型、22 维策略，不提供抓取操作。
- 仿真真值参与定位与保持，不含实物估计误差。
- 不含复杂地形、避障、跌倒起身、实物部署或安全认证。
- WSL 默认软件渲染曾出现严重低速；项目启动器局部选择已有 D3D12 驱动。
- 批量测试为无界面；另有一次带界面的全路线测试。尚不将人工键盘操作称为经过人工验收。
