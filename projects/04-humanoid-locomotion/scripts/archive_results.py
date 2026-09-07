"""Archive explicit GUI evidence and latest complete suites into project docs."""
import argparse
import json
from pathlib import Path
import shutil

ROOT=Path(__file__).resolve().parents[1]
parser=argparse.ArgumentParser()
parser.add_argument('--gui-report',required=True)
parser.add_argument('--robot',choices=['g1','t800'],default='g1')
args=parser.parse_args()
evidence=ROOT/('docs/evidence' if args.robot=='g1' else 'docs/evidence-t800')
evidence.mkdir(parents=True,exist_ok=True)
summaries={}
for suite in ['route','baseline','cancel','push']:
    candidates=list((ROOT/'reports').glob('batch-'+args.robot+'-'+suite+'-*/summary.json'))
    if args.robot=='g1':
        candidates+=list((ROOT/'reports').glob('batch-'+suite+'-*/summary.json'))
    candidates=sorted(candidates,key=lambda p:p.stat().st_mtime)
    if not candidates: raise SystemExit('Missing suite '+suite)
    summary=json.loads(candidates[-1].read_text())
    if summary.get('robot','g1')!=args.robot: raise SystemExit('Wrong robot in suite '+suite)
    if summary['finished']!=summary['planned']: raise SystemExit('Incomplete suite '+suite)
    target=evidence/suite
    target.mkdir(exist_ok=True)
    for case in summary['cases']:
        if not case.get('report'): raise SystemExit('Missing report '+case['name'])
        source=ROOT/case['report']
        case['original_report']=case['report']
        case['report']=suite+'/'+case['name']+'.json'
        shutil.copy2(source,evidence/case['report'])
    (evidence/(suite+'-summary.json')).write_text(json.dumps(summary,indent=2)+'\n')
    summaries[suite]=summary
gui_path=Path(args.gui_report).resolve()
if not gui_path.is_relative_to(ROOT/'reports'): raise SystemExit('GUI report must be inside this project reports')
gui=json.loads(gui_path.read_text())
if gui['status']!='COMPLETED' or gui['arguments']['headless']: raise SystemExit('Need a completed GUI run')
if gui.get('robot','g1')!=args.robot: raise SystemExit('Wrong robot in GUI report')
shutil.copy2(gui_path,evidence/'gui-route.json')
images=ROOT/'docs/images'
images.mkdir(exist_ok=True)
for source,name in [('final-camera.png','route-final.png'),('summary.png','route-trajectory.png')]:
    if args.robot=='t800': name='t800-'+name
    shutil.copy2(gui_path.parent/source,images/name)
route=summaries['route']
times=[r['sim_seconds'] for r in route['cases']]
errors=[r['events'][-1]['position_error'] for r in route['cases'] if r['status']=='COMPLETED']
stand=json.loads((evidence/'baseline/stand.json').read_text())
lines=['# 本机仿真实测记录','',
       '日期：2026-09-06（北京时间；原始运行编号使用 UTC）。环境：WSL2/WSLg、Ubuntu 26.04、Python 3.12.13、MuJoCo 3.3.6、PyTorch 2.7.1+cpu。',
       '', '## 自动路线', '',
       f'不同初始条件共 {route["finished"]} 次，完成 {route["completed"]} 次。仿真用时范围 {min(times):.2f}～{max(times):.2f} 秒；完成案例最终位置误差范围 {min(errors)*100:.2f}～{max(errors)*100:.2f} cm。',
       '', '条件：初始 xy 分别在 ±0.04 m、yaw 在 ±0.08 rad 内变化，其余模型、平地环境和控制参数固定。此统计不代表真实机器人可靠性。',
       '', '[批量概要](evidence/route-summary.json)；逐次完整报告在 `evidence/route/`，包含配置、源代码哈希和策略哈希。',
       '', '## 图形界面完整任务', '',
       f'结果 `{gui["status"]}`，仿真时间 {gui["sim_seconds"]:.2f} 秒，墙钟时间 {gui["wall_seconds"]:.2f} 秒，最终位置误差 {gui["events"][-1]["position_error"]*100:.2f} cm。运行时还存在并行批量测试，耗时不是性能基准。',
       '', '[完整报告](evidence/gui-route.json)', '', '![实际轨迹](images/route-trajectory.png)',
       '', '## 原始策略与动态位置保持', '',
       '原始零速度模式只评价是否运行至指定时长，`COMPLETED` 不代表没有漂移。位置保持依赖任务层的理想定位反馈；G1 零指令下会踏步，T800 本机零指令漂移较小。',
       '', f'闭环保持测试完成 {stand["events"][-1]["hold_seconds"]:.2f} 秒连续保持，最终位置误差 {stand["events"][-1]["position_error"]*100:.2f} cm。',
       '', '| 基线 | 最终平面位移 m | 累计转向 deg | 结果 |', '| --- | ---: | ---: | --- |']
for case in summaries['baseline']['cases']:
    lines.append(f'| {case["name"]} | {case["net_displacement_m"]:.3f} | {case["accumulated_yaw_degrees"]:.1f} | {case["status"]} |')
lines += ['', '[基线详细证据](evidence/baseline-summary.json)', '', '## 取消停车', '',
          f'仿真第 0、8、30、45 秒请求取消：{summaries["cancel"]["controlled_stops"]}/4 通过。先减速 3 秒，再进行 10 秒闭环位置保持验收。',
          '', '[取消详细证据](evidence/cancel-summary.json)', '', '## 有限推力实验', '',
          '相同初始状态，仿真第 8 秒向骨盆施加世界坐标 +Y 方向外力，持续 0.2 秒。下表每种强度仅测试一次，不能据此声称精确的抗推阈值。',
          '', '| 外力 | 结果 | 原因 |', '| --- | --- | --- |']
for case in summaries['push']['cases']:
    lines.append(f'| {case["name"]} | {case["status"]} | {case["reason"] or "完成路线"} |')
lines += ['', '[扰动详细证据（包含失败）](evidence/push-summary.json)', '', '## 已知限制', '',
          '- G1 为固定/合并上身的 12 关节腿部模型；T800 为 25 关节模型、22 维策略，不提供抓取操作。',
          '- 仿真真值参与定位与保持，不含实物估计误差。',
          '- 不含复杂地形、避障、跌倒起身、实物部署或安全认证。',
          '- WSL 默认软件渲染曾出现严重低速；项目启动器局部选择已有 D3D12 驱动。',
          '- 批量测试为无界面；另有一次带界面的全路线测试。尚不将人工键盘操作称为经过人工验收。', '']
document='\n'.join(lines)
if args.robot=='t800':
    document=document.replace('# 本机仿真实测记录','# T800 本机仿真实测记录').replace('PyTorch 2.7.1+cpu','MNN 3.6.1')
    document=document.replace('(evidence/','(evidence-t800/').replace('`evidence/','`evidence-t800/')
    document=document.replace('(images/route-','(images/t800-route-')
target_doc=ROOT/('docs/validation.md' if args.robot=='g1' else 'docs/validation-t800.md')
target_doc.write_text(document,encoding='utf-8')
print('Archived', args.robot, 'evidence into', evidence, 'and', target_doc)
