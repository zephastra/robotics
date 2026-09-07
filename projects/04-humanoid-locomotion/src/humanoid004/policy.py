"""G1 observation/action conventions adapted from Unitree RL Gym (BSD-3-Clause).

Copyright (c) 2016-2023 HangZhou YuShu TECHNOLOGY CO.,LTD.
See THIRD_PARTY_NOTICES.md and policies/LICENSE-Unitree.txt.
"""
from pathlib import Path
import numpy as np
import torch
import yaml


class G1Policy:
    def __init__(self, root: Path):
        cfg = yaml.safe_load((root / 'config/upstream_g1.yaml').read_text())
        if cfg['num_actions'] != 12 or cfg['num_obs'] != 47:
            raise ValueError('This runner only supports the pinned 12-leg-joint / 47-observation G1 policy')
        self.cfg = cfg
        self.default = np.asarray(cfg['default_angles'], dtype=float)
        self.kp, self.kd = np.asarray(cfg['kps']), np.asarray(cfg['kds'])
        self.last_action = np.zeros(12)
        self.target = self.default.copy()
        torch.set_num_threads(1)
        self.network = torch.jit.load(str(root / 'policies/motion.pt'), map_location='cpu').eval()

    def infer(self, q, dq, quat, angular_velocity, command, now):
        w, x, y, z = quat
        gravity = np.array([2*(-z*x+w*y), -2*(z*y+w*x), 1-2*(w*w+z*z)])
        phase = 2 * np.pi * ((now % 0.8) / 0.8)
        obs = np.concatenate([angular_velocity * self.cfg['ang_vel_scale'], gravity,
                              command * np.asarray(self.cfg['cmd_scale']),
                              (q-self.default)*self.cfg['dof_pos_scale'], dq*self.cfg['dof_vel_scale'],
                              self.last_action, [np.sin(phase), np.cos(phase)]]).astype(np.float32)
        if not np.all(np.isfinite(obs)):
            raise FloatingPointError('Nonfinite policy observation')
        with torch.inference_mode():
            action = self.network(torch.from_numpy(obs).unsqueeze(0)).numpy().reshape(-1)
        if action.shape != (12,) or not np.all(np.isfinite(action)):
            raise FloatingPointError('Invalid policy action')
        self.last_action = action.copy()
        self.target = self.default + self.cfg['action_scale'] * action

    def torques(self, q, dq):
        return (self.target-q)*self.kp - dq*self.kd
