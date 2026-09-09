"""Simulation-only Python adapter for EngineAI's BSD-3-Clause walking example.

Copyright (c) 2025- Shenzhen Zhongqing Robot Technology Co., Ltd. ("EngineAI").
See licenses/EngineAI-BSD-3-Clause.txt. No hardware SDK or network transport.
"""
import numpy as np
import yaml


class T800Policy:
    def __init__(self, root):
        import MNN
        self.mnn = MNN
        self.cfg = yaml.safe_load((root/'config/t800/walking.yaml').read_text())
        self.cfg.update(simulation_dt=.002, control_decimation=5)
        if self.cfg['control_dt'] != .01:
            raise ValueError('T800 requires the pinned 100 Hz walking policy')
        model_cfg = yaml.safe_load((root/'config/t800/model.yaml').read_text())
        self.joints = [j for limb in model_cfg['limbs'] for j in limb['joints']]
        self.active = np.array([self.joints.index(j) for j in self.cfg['active_joint_names']])
        self.default = np.concatenate(self.cfg['default_joint_q'])
        self.kp = np.concatenate(self.cfg['joint_kp'])
        self.kd = np.concatenate(self.cfg['joint_kd'])
        self.scale = np.concatenate(self.cfg['action_scale'])
        if len(self.joints) != 25 or len(self.active) != 22 or self.cfg['num_observations'] != 72:
            raise ValueError('T800 model/policy dimensions mismatch')
        self.last_action = np.zeros(22)
        self.history = None
        self.target = self.default.copy()
        self.interpreter = MNN.Interpreter(str(root/'policies/t800/walking.mnn'))
        self.session = self.interpreter.createSession({'backend': 'CPU', 'numThread': 1, 'precision': 'high'})
        self.input = self.interpreter.getSessionInput(self.session)
        self.output = self.interpreter.getSessionOutput(self.session)
        if tuple(self.input.getShape()) != (1, 1083) or tuple(self.output.getShape()) != (1, 22):
            raise ValueError(f'T800 network dimensions mismatch: {self.input.getShape()} -> {self.output.getShape()}')

    def observation(self, q, dq, quat, angular_velocity, command):
        w, x, y, z = quat
        gravity = np.array([2*(-z*x+w*y), -2*(z*y+w*x), 1-2*(w*w+z*z)])
        c = self.cfg
        obs = np.concatenate([(q-self.default)[self.active]*c['observation_scale_dof_pos'],
                              dq[self.active]*c['observation_scale_dof_vel'], self.last_action,
                              angular_velocity*c['observation_scale_angular_vel'],
                              gravity*c['observation_scale_quat']])
        if not np.all(np.isfinite(obs)) or not np.all(np.isfinite(command)):
            raise FloatingPointError('Nonfinite T800 policy observation')
        obs = np.clip(obs, -c['observation_clip'], c['observation_clip'])
        if self.history is None:
            self.history = np.tile(obs, (c['num_include_obs_steps'], 1))
        else:
            self.history[:-1] = self.history[1:].copy()
            self.history[-1] = obs
        # Eigen MatrixXd's columns contain oldest -> newest frames. Transpose().data()
        # in upstream does not reorder the underlying column-major buffer.
        return np.concatenate([self.history.ravel(), command*np.array([
            c['observation_scale_linear_vel'], c['observation_scale_linear_vel'],
            c['observation_scale_angular_vel']])]).astype(np.float32)

    def infer(self, q, dq, quat, angular_velocity, command, now):
        obs = self.observation(q, dq, quat, angular_velocity, command)
        host = self.mnn.Tensor((1, 1083), self.mnn.Halide_Type_Float, obs,
                               self.mnn.Tensor_DimensionType_Caffe)
        self.input.copyFrom(host)
        self.interpreter.runSession(self.session)
        action = np.asarray(self.output.getData(), dtype=float).reshape(-1)
        if action.shape != (22,) or not np.all(np.isfinite(action)):
            raise FloatingPointError('Invalid T800 policy action')
        self.last_action = np.clip(action, -self.cfg['action_clip'], self.cfg['action_clip'])
        self.target = self.default.copy()
        self.target[self.active] += self.scale*self.last_action

    def torques(self, q, dq):
        return (self.target-q)*self.kp - dq*self.kd
