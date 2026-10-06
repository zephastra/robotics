# 010 精选证据备份 · 2026-10-07

本目录保存开发机 **22 份原始 JSON 文件和 5 份派生精简摘要**，不是新的实验或完整报告仓库。

文件按原字节复制。原始路径、字节数与 SHA-256 见项目根 `DEV_SNAPSHOT_MANIFEST.json`。对应源码可能继续变化：报告内 source/world 哈希描述当时运行版本，不能据此认证当前代码。报告中的 RUN、COMPLETED、PASS 也必须按 scope 和逐项判决解释。

## 重点阅读

- `p4-g7a-smoke-01`：源端到甲板实际交接冒烟，不是完整订单。
- `p4-nav2-v7-03`、`p4-nav2-v7-04-retained`、`p4-nav2-v7-06-turn`：短程导航到达与保持/货物停止分开判决，不能只读 nav2_succeeded。
- `p4-nav2-v7-05-receiver`：接收方向长程的姿态/保持失效记录。
- `p4-nav2-v7-07-srcl5`、`p4-nav2-v7-07-srcl6`：实际取盘后接收诊断失败；srcl6 卸盘超时，worker/bridge 没有启动，不是完整 Nav2 配送。
- `p4-g7c-transit-dial-03-slow`：慢速逼近仍掉盘。
- `p4-g7d-03`：原始 20 项为 18 PASS、2 FAIL；账本确认拒绝不等于物理安全通过。
- `p4-edge-retainer-close-release-20261004-01`：被拒绝的 v6 保持候选，失败保留。
- `p4-hinged-retainer-current-positive-20261004-01`：v7 静止机构正例，不能推出运动资格。

完整图像、接触逐步 JSONL、ROS 日志与其他未精选报告不在本目录。单份超过 2 MiB 的 JSON 不自动复制，而以 `REPORT_SUMMARY.json` 保存选定原字段与完整原报告哈希；它明确标注 `DERIVED_SUMMARY_ONLY`，不是完整原始证据。以清单记录的实际文件为准。

备份没有重新运行实验，也没有修复或更改原始判决。上线说明中的更正不覆盖原报告。
