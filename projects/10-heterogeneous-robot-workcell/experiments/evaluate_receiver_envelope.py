"""Independent final longitudinal receiver check; NOT complete P4/order acceptance."""
import argparse
import hashlib
import json
import math
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def judge(report, window, world_sha256):
    deliveries = [row for row in report.get('chain_rows', [])
                  if row.get('skill') == 'VERIFY_DELIVERY']
    state = deliveries[-1].get('final_state', {}) if deliveries else {}
    low, high = window
    tail, head = state.get('x_trailing_m'), state.get('x_leading_m')
    finite = (all(isinstance(v, (int, float)) and math.isfinite(v)
                  for v in (low, high, tail, head)) and low < high)
    checks = {
        'source_world_matches': report.get('world_sha256') == world_sha256,
        'final_delivery_recorded': bool(deliveries),
        'finite_physical_bounds': finite,
        'trailing_edge_inside_receiver': finite and tail >= low,
        'leading_edge_inside_receiver': finite and head <= high,
        'receiver_exclusive_support': state.get('supported_by') == ['receiver_band'],
        'on_crowns': state.get('on_crowns') is True,
        'no_runtime_teleport': report.get('runtime_qpos_writes') == 0,
    }
    return {'scope': 'FINAL_LONGITUDINAL_RECEIVER_ENVELOPE_ONLY',
            'checks': {k: 'PASS' if v else 'FAIL' for k, v in checks.items()},
            'diagnostic_result': 'PASS' if all(checks.values()) else 'FAIL',
            'receiver_crown_window_m': [low, high], 'final_state': state,
            'lateral_containment': 'NOT_RUN', 'loaded_delivery': 'NOT_RUN',
            'full_order': 'NOT_RUN', 'v1_complete': False}


def main():
    import mujoco
    parser = argparse.ArgumentParser()
    parser.add_argument('--run-id', required=True)
    parser.add_argument('--source', required=True)
    args = parser.parse_args()
    source = ROOT / 'reports' / args.source / 'report.json'
    report = json.loads(source.read_text())
    world = (ROOT / report['world']).resolve()
    if not world.is_relative_to(ROOT / 'assets'):
        raise ValueError('source world must be an asset in this project')
    model = mujoco.MjModel.from_xml_path(str(world))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    crowns = [float(data.geom_xpos[g][0]) for g in range(model.ngeom)
              if (mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, g)
                  or '').startswith('c_recv_roller')]
    result = judge(report, (min(crowns), max(crowns)),
                   hashlib.sha256(world.read_bytes()).hexdigest())
    result['source_report'] = str(source.relative_to(ROOT))
    result['source_report_sha256'] = hashlib.sha256(source.read_bytes()).hexdigest()
    directory = ROOT / 'reports' / args.run_id
    directory.mkdir(exist_ok=False)
    (directory / 'acceptance.json').write_text(json.dumps(result, indent=2) + '\n')
    print(json.dumps(result, indent=2), flush=True)
    return 0 if result['diagnostic_result'] == 'PASS' else 1


if __name__ == '__main__':
    raise SystemExit(main())
