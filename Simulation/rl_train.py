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

    def step(self, control_effort, step=None, max_steps=None):
        pos, vel, acc, jrk = motor.motorStep(control_effort)
        error = abs(self.setpoint - pos)
        self.cumulative_error += error
        error_delta = 0.0
        if self.prev_error is not None:
            error_delta = self.prev_error - error  # positive if error decreased
        self.prev_error = error
        next_state = self._get_state()
        reward = self._compute_reward(pos, vel, acc, jrk, control_effort, error_delta, step, max_steps)
        done = False  # Customize episode termination
        return next_state, reward, done, {}

    def _get_state(self):
        s = motor.get_motor_state()
        return np.array([s['position'], s['velocity'], s['acceleration'], s['jerk'], s['control_effort']])

    def _compute_reward(self, pos, vel, acc, jrk, control_effort, error_delta, step=None, max_steps=None):
        error = abs(self.setpoint - pos)
        # Tuned weights for smoother, accurate control
        w_error = -20.0      # Strong penalty for position error
        w_cum_error = -0.5   # Moderate penalty for cumulative error
        w_error_delta = 15.0 # Reward for reducing error
        w_vel = -2.0         # Moderate penalty for velocity
        w_acc = -0.2         # Small penalty for acceleration
        jerk_threshold = 10.0
        w_jerk_high = -50.0  # Heavy penalty for extreme jerk

        reward = w_error * error * error
        reward += w_cum_error * self.cumulative_error
        reward += w_error_delta * error_delta
        reward += w_vel * abs(vel)
        reward += w_acc * abs(acc)
        # Only penalize jerk if above threshold
        if abs(jrk) > jerk_threshold:
            reward += w_jerk_high * (abs(jrk) - jerk_threshold)
        # Time-scaled bonus for being close
        if error < 0.05 and step is not None and max_steps is not None:
            time_factor = (max_steps - step) / max_steps
            reward += 30.0 * time_factor
        return reward