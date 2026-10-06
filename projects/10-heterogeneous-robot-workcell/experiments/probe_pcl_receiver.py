"""Actual native PCL receiver floor probe on a captured cloud; not live integration."""
import argparse
import hashlib
import json
import shlex
import subprocess
from pathlib import Path
import numpy as np

ROOT=Path(__file__).resolve().parents[1]


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run-id',required=True)
    parser.add_argument('--recording',required=True)
    parser.add_argument('--build',action='store_true')
    args=parser.parse_args()
    recording=(ROOT/'reports'/args.recording).resolve()
    if recording.parent!=(ROOT/'reports').resolve():raise ValueError('invalid recording')
    source=ROOT/'experiments/pcl_receiver_plane.cpp'
    binary=ROOT/'runtime/pcl_receiver_plane'
    version=subprocess.check_output(['pkg-config','--modversion','pcl_segmentation'],text=True).strip()
    if args.build:
        flags=shlex.split(subprocess.check_output(['pkg-config','--cflags','--libs',
            'pcl_common','pcl_filters','pcl_segmentation'],text=True))
        subprocess.run(['g++','-O2','-std=c++17',str(source),'-o',str(binary),*flags],
                       check=True,timeout=120)
    frame=np.load(recording/'receiver_rgbd.npz',allow_pickle=False)
    w,c,v=frame['world'],frame['rgb'].astype(float),frame['valid']
    roi=frame['roi'];prior=float(frame['floor_z'])
    mask=v&np.all(np.isfinite(w),axis=-1)&(c[...,1]>.45*c[...,2])&(c[...,1]<.9*c[...,2])&(c[...,0]<.5*c[...,2])
    mask&=(w[...,0]>=roi[0])&(w[...,0]<=roi[1])&(w[...,1]>=roi[2])&(w[...,1]<=roi[3])
    mask&=abs(w[...,2]-prior)<=.006
    def execute(points):
        rows=''.join('%.12g %.12g %.12g\n'%tuple(p) for p in points)
        answer=subprocess.run([str(binary)],input=rows,text=True,capture_output=True,
                              timeout=30,check=True)
        return json.loads(answer.stdout)
    normal=execute(w[mask]);missing=execute(np.empty((0,3)))
    checks=dict(actual_pcl_floor=normal['status']=='RESOLVED' and abs(
        normal.get('floor_z_m',float('inf'))-prior)<=.006,
        missing_cloud_unknown=missing['status']=='UNKNOWN')
    out=ROOT/'reports'/args.run_id;out.mkdir(exist_ok=False)
    report=dict(scope='PCL_RECORDED_FLOOR_PROBE_NOT_ORDER',pcl_version=version,
        cpp_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),
        binary_sha256=hashlib.sha256(binary.read_bytes()).hexdigest(),
        recording_sha256=hashlib.sha256((recording/'receiver_rgbd.npz').read_bytes()).hexdigest(),
        declarations=dict(voxel_m=.001,ransac_distance_m=.001,minimum_points=60,
                          minimum_inlier_ratio=.9,normal_angle_rad=.05),
        normal=normal,missing=missing,live_integration='NOT_RUN')
    acceptance=dict(scope=report['scope'],checks={k:'PASS' if v else 'FAIL' for k,v in checks.items()},
        diagnostic_result='PASS' if all(checks.values()) else 'FAIL',full_order='NOT_RUN',v1_complete=False)
    (out/'report.json').write_text(json.dumps(report,indent=2)+'\n')
    (out/'acceptance.json').write_text(json.dumps(acceptance,indent=2)+'\n')
    print(json.dumps(acceptance,indent=2))
    return 0 if all(checks.values()) else 1


if __name__=='__main__':raise SystemExit(main())
