"""Read-only dependency, model, policy, and license checks."""
import json
import argparse
from importlib.metadata import version
from pathlib import Path
import platform
import sys
import mujoco
import numpy as np
import torch
from humanoid004.app import check_assets
from humanoid004.robots import get_robot, joint_addresses

ROOT=Path(__file__).resolve().parents[1]
parser=argparse.ArgumentParser()
parser.add_argument('--robot',choices=['g1','t800'],default='g1')
args=parser.parse_args()
robot=get_robot(args.robot,ROOT)
manifest=check_assets(args.robot)
model=mujoco.MjModel.from_xml_path(str(ROOT/robot.scene))
joint_addresses(model,robot)
policy=robot.policy(ROOT)
policy.infer(policy.default,np.zeros(model.nu),np.array([1,0,0,0]),np.zeros(3),np.zeros(3),0)
print(json.dumps({'robot':robot.name,'python':sys.version.split()[0],'platform':platform.platform(),'mujoco':mujoco.__version__,
                  'torch':torch.__version__,'policy_runtime_version':version('MNN' if args.robot=='t800' else 'torch'),
                  'actuated_joints':model.nu,'policy_device':'cpu','assets_verified':len(manifest['sha256']),
                  'upstream_commit':manifest['commit'],'status':'PASS'},indent=2))
