import numpy as np
STATE_SIZE =  5 # [angularPosError, angularVel, angularAcc, angularJrk, lastControlEffort]

# Model Action = s^T * Policy1 * s + Policy0 * s + Constant
Policy2 = np.random.rand(STATE_SIZE, STATE_SIZE)
Policy1 = np.random.rand(STATE_SIZE)
Policy0 = np.random.rand(STATE_SIZE)

def model_action(inputState):
    action = inputState.T @ Policy2 @ inputState
    action += Policy1 @ inputState
    action += Policy0
    return action # the action should be a number between 0 and 1

def calculate_reward(state):
    reward = 0
    reward -= abs(state[1]) * 1.0  # velocity penalty
    reward -= abs(state[2]) * 0.1  # acceleration penalty
    reward -= abs(state[3]) * 0.01 # jerk penalty
    return reward



