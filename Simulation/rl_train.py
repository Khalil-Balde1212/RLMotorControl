import numpy as np
import torch
import torch.nn as nn
import torch.optim as optim
from rl_model import MotorEffortRLModel
import bdc_motor_model as motor

class MotorEnv:
    def __init__(self, setpoint):
        self.setpoint = setpoint
        self.cumulative_error = 0.0
        self.prev_error = None
        self.reset()

    def reset(self):
        motor.reset_motor_state()
        self.cumulative_error = 0.0
        self.prev_error = None
        state = self._get_state()
        return state

    def step(self, control_effort):
        pos, vel, acc, jrk = motor.motorStep(control_effort)
        error = abs(self.setpoint - pos)
        self.cumulative_error += error
        error_delta = 0.0
        if self.prev_error is not None:
            error_delta = self.prev_error - error  # positive if error decreased
        self.prev_error = error
        next_state = self._get_state()
        reward = self._compute_reward(pos, control_effort, error_delta)
        done = False  # Customize episode termination
        return next_state, reward, done, {}

    def _get_state(self):
        s = motor.get_motor_state()
        return np.array([s['position'], s['velocity'], s['acceleration'], s['jerk'], s['control_effort']])

    def _compute_reward(self, pos, control_effort, error_delta):
        error = abs(self.setpoint - pos)
        w_error = -10
        w_cum_error = -1
        w_error_delta = 10  # reward for reducing error

        reward = w_error * error * error
        reward += w_cum_error * self.cumulative_error
        reward += w_error_delta * error_delta
        if error < 0.1:
            reward += 20.0  # bonus for being very close
            reward += 5.0 * (1.0 - abs(motor.get_motor_state()['velocity']))
        return reward