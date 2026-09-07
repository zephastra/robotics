"""Robot-specific models, policies and address mappings; no cross-robot weights."""
from dataclasses import dataclass
import mujoco
import numpy as np
import yaml

G1_JOINTS = [s+'_'+j+'_joint' for s in ['left', 'right']
             for j in ['hip_pitch', 'hip_roll', 'hip_yaw', 'knee', 'ankle_pitch', 'ankle_roll']]


def manual_commands(name):
    if name not in ('g1', 't800'):
        raise ValueError('Unknown robot: '+name)
    backward, lateral, turn = (-.12, .12, .4) if name == 'g1' else (-.3, .2, .6)
    return {'W': [.4, 0, 0], 'S': [backward, 0, 0],
            'A': [0, 0, turn], 'D': [0, 0, -turn],
            'Q': [0, lateral, 0], 'E': [0, -lateral, 0],
            'R': [.25, 0, .3 if name=='g1' else .45],
            'T': [.25, 0, -.3 if name=='g1' else -.45],
            'X': [0, 0, 0], ' ': [0, 0, 0]}


@dataclass(frozen=True)
class Robot:
    name: str
    title: str
    scene: str
    manifest: str
    weight: str
    base: str
    feet: tuple
    joints: tuple
    decimation: int
    fall_height: float

    def policy(self, root):
        if self.name == 'g1':
            from .policy import G1Policy
            return G1Policy(root)
        from .t800_policy import T800Policy
        return T800Policy(root)


def get_robot(name, root):
    if name == 'g1':
        return Robot('g1', 'Unitree G1', 'assets/g1/lab.xml', 'assets/manifest.json',
                     'policies/motion.pt', 'pelvis', ('left_ankle_roll_link', 'right_ankle_roll_link'),
                     tuple(G1_JOINTS), 10, .45)
    if name != 't800':
        raise ValueError('Unknown robot: '+name)
    path = root/'config/t800/model.yaml'
    if not path.exists():
        raise FileNotFoundError('T800 is not installed. Run: bash scripts/setup_t800.sh')
    cfg = yaml.safe_load(path.read_text())
    joints = tuple(j for limb in cfg['limbs'] for j in limb['joints'])
    return Robot('t800', 'EngineAI T800', 'assets/t800/lab.xml', 'assets/manifest-t800.json',
                 'policies/t800/walking.mnn', 'LINK_BASE', ('LINK_FOOT_L', 'LINK_FOOT_R'), joints, 5, .65)


def joint_addresses(model, robot):
    n = len(robot.joints)
    if (model.nq, model.nv, model.nu) != (n+7, n+6, n) or model.jnt_type[0] != mujoco.mjtJoint.mjJNT_FREE:
        raise ValueError('Unexpected floating-base model dimensions for '+robot.name)
    ids = model.actuator_trnid[:, 0]
    names = tuple(mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, int(j)) for j in ids)
    if names != robot.joints:
        raise ValueError('Actuator joint order does not match '+robot.name)
    if not np.all(model.jnt_actfrclimited[ids]):
        raise ValueError('Missing joint torque limits')
    for name in (robot.base, *robot.feet):
        if mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name) < 0:
            raise ValueError('Missing body: '+name)
    return model.jnt_qposadr[ids], model.jnt_dofadr[ids]
