import numpy as np
import matplotlib.pyplot as plt
from typing import Tuple
from bdc_motor_model import motorStep, reset_motor_state, get_motor_state, MaxVel, MaxAcc, MaxJrk
from sim_system_identification import run_sim, MAX_POSITION

# Port of RLPolicy to Python (numpy). This mirrors the MCU C++ implementation for training.
class RLPolicyPy:
    HORIZON_SIZE = 20
    STATE_SIZE = 4
    HIDDEN_SIZE = 16

    # LTI matrices (same as C++ constants)
    stateTransitionMatrix = np.array(
        [[1.0, 0.01, 0.0, 0.0],
         [0.0, 1.0, 0.01, 0.0],
         [0.0, 0.0, 1.0, 0.01],
         [0.0, 0.0, 0.0, 1.0]], dtype=float)

    B = np.array([[0.0], [0.01], [0.001], [0.0001]], dtype=float)

    def __init__(self, seed: int = 0, A: np.ndarray = None, B: np.ndarray = None, use_physical_reward: bool = False):
        rng = np.random.RandomState(seed)
        self.W1 = rng.randn(self.HIDDEN_SIZE, self.STATE_SIZE * 2 + 2) * 0.1
        self.b1 = np.zeros((self.HIDDEN_SIZE, 1))
        self.W2 = rng.randn(self.HORIZON_SIZE, self.HIDDEN_SIZE) * 0.1
        self.b2 = np.zeros((self.HORIZON_SIZE, 1))
        self.policy_lr = 1e-3
        # If an LTI A,B are provided (from identification), use them for prediction
        if A is not None:
            self.stateTransitionMatrix = A.copy()
        else:
            self.stateTransitionMatrix = RLPolicyPy.stateTransitionMatrix.copy()
        if B is not None:
            self.B = B.copy()
        else:
            self.B = RLPolicyPy.B.copy()
        # Use reward in physical units (radians) rather than normalized if True
        self.use_physical_reward = use_physical_reward

        # caches
        self.cachedStates = []
        self.cachedHiddenLayer = []
        self.cachedActions = []
        self.cachedSetpoint = 0.0
        self.cachedPos = []
        self.cachedCmds = []

    def _relu(self, x):
        return np.maximum(0.0, x)

    def computeMotorSequence(self, currentStateNormalized: np.ndarray, u_prev_normalized: float, setpoint_normalized: float):
        # build x vector
        x = np.zeros((self.STATE_SIZE * 2 + 2, 1))
        for i in range(self.STATE_SIZE):
            x[i, 0] = float(currentStateNormalized[i, 0])
            x[i + self.STATE_SIZE, 0] = float(currentStateNormalized[i, 0] - setpoint_normalized)
        x[self.STATE_SIZE * 2, 0] = float(u_prev_normalized)
        x[self.STATE_SIZE * 2 + 1, 0] = float(setpoint_normalized)

        # forward pass
        y = self.W1 @ x + self.b1
        y = self._relu(y)

        output = self.W2 @ y + self.b2
        # Numerical safety: if output contains NaNs/Infs due to large weights, clip them to zero
        if not np.isfinite(output).all():
            output = np.nan_to_num(output, nan=0.0, posinf=0.0, neginf=0.0)
        # Limit magnitude of output to avoid exploding control residuals
        output = np.clip(output, -5.0, 5.0)

        cmds = []
        # caches
        self.cachedStates = []
        self.cachedHiddenLayer = []
        self.cachedActions = []
        self.cachedCmds = []
        self.cachedPos = []
        self.cachedSetpoint = setpoint_normalized

        stateF = currentStateNormalized.copy()
        for i in range(self.HORIZON_SIZE):
            u_out_norm = u_prev_normalized + output[i, 0]
            u_out_norm = float(np.clip(u_out_norm, -1.0, 1.0))
            motorCmd = int(round(u_out_norm * 255.0))
            motorCmd = max(min(motorCmd, 255), -255)
            cmds.append(motorCmd)

            # cache
            self.cachedStates.append(x.copy())
            self.cachedHiddenLayer.append(y.copy())
            self.cachedActions.append(u_out_norm)
            self.cachedCmds.append(motorCmd)

            # compute next normalized state using LTI A,B
            # Use instance A/B for predictive model (can be learned from sim id)
            nextState = self.stateTransitionMatrix @ stateF + self.B * u_out_norm
            self.cachedPos.append(float(nextState[0, 0]))
            stateF = nextState

        return cmds

    def calculateReward(self, currentStateNormalized: np.ndarray) -> float:
        if len(self.cachedPos) == 0:
            return 0.0
        w_pos = -1.0
        w_jerk = -0.5

        prevU = self.cachedActions[0]
        reward = 0.0
        for i, pos in enumerate(self.cachedPos):
            # pos here is normalized position; convert to physical units for reward if requested

            pos_abs = pos * MAX_POSITION
            setpoint_abs = self.cachedSetpoint * MAX_POSITION

            err = setpoint_abs - pos_abs
            reward += w_pos * err * err  # punish squared error in physical units

            # Estimate jerk from LTI model: jerk = nextState[3, 0]
            jerk = (self.stateTransitionMatrix @ np.array([[pos],
                                                              [0.0],
                                                              [0.0],
                                                              [0.0]]) + self.B * self.cachedActions[i])[3, 0]
            reward += w_jerk * jerk * jerk  # punish squared jerk

            

        return reward
    


    
    def updatePolicyWeights(self):
        if len(self.cachedPos) == 0:
            return
        policyLR = self.policy_lr
        l2Reg = 1e-4
        w_pos = 1.0
        w_u = 0.01
        w_du = 0.05

        dW2 = np.zeros_like(self.W2)
        db2 = np.zeros_like(self.b2)
        dW1 = np.zeros_like(self.W1)
        db1 = np.zeros_like(self.b1)

        prevU = self.cachedActions[0]
        for t in range(len(self.cachedPos)):
            pos = self.cachedPos[t]
            setpoint = self.cachedSetpoint
            dL_dpos = 2.0 * w_pos * (pos - setpoint)
            dpos_dout = self.stateTransitionMatrix[0, 1] * self.B[1, 0]
            # If reward measured in physical units, dL_dpos should scale by MAX_POSITION so we scale the
            # chain derivative appropriately (physical_pos = normalized_pos * MAX_POSITION)
            if self.use_physical_reward:
                dpos_dout *= MAX_POSITION
            dL_dout = dL_dpos * dpos_dout + 2.0 * w_u * self.cachedActions[t] + 2.0 * w_du * (self.cachedActions[t] - prevU)

            # update gradients
            # dW2 row t = dL_dout * y^T
            for j in range(self.HIDDEN_SIZE):
                dW2[t, j] += dL_dout * float(self.cachedHiddenLayer[t][j, 0])
            db2[t, 0] += dL_dout

            w2row = self.W2[t, :].reshape(self.HIDDEN_SIZE, 1)
            for j in range(self.HIDDEN_SIZE):
                reluDeriv = 1.0 if self.cachedHiddenLayer[t][j, 0] > 0.0 else 0.0
                coeff = dL_dout * float(w2row[j, 0]) * reluDeriv
                for k in range(self.STATE_SIZE * 2 + 2):
                    dW1[j, k] += coeff * float(self.cachedStates[t][k, 0])
                db1[j, 0] += coeff

            prevU = self.cachedActions[t]

        self.W2 -= policyLR * (dW2 + l2Reg * self.W2)
        self.b2 -= policyLR * (db2 + l2Reg * self.b2)
        self.W1 -= policyLR * (dW1 + l2Reg * self.W1)
        self.b1 -= policyLR * (db1 + l2Reg * self.b1)
        # Clip weights to avoid runaway values
        np.clip(self.W1, -50.0, 50.0, out=self.W1)
        np.clip(self.W2, -50.0, 50.0, out=self.W2)


# Training harness

def evaluate_step_response(policy: RLPolicyPy, sim_time: float = 3.0, setpoint: float = 1.0, setpoint_abs: bool = False) -> Tuple[float, list]:
    # Runs a step response test using policy (no training). Returns settling time and full position trace.
    reset_motor_state()
    dt = 1.0 / 100.0  # 100 Hz control
    steps = int(sim_time / dt)
    prev_u = 0.0
    trace = []
    times = []
    settled = False
    settle_time = None

    pos_trace = []
    vel_trace = []
    acc_trace = []
    jrk_trace = []

    # If setpoint_abs==True, interpret setpoint as radians and normalize for policy input
    if setpoint_abs:
        norm_setpoint = setpoint / 100.0
        absolute_setpoint = setpoint
    else:
        norm_setpoint = setpoint
        absolute_setpoint = setpoint * 100.0

    for i in range(steps):
        # sample motor state
        ms = get_motor_state()
        pos, vel, acc, jrk = ms['position'], ms['velocity'], ms['acceleration'], ms['jerk']
        # normalized
        norm_state = np.array([pos / 100.0, vel / MaxVel, acc / MaxAcc, jrk / MaxJrk]).reshape(4, 1)
        cmds = policy.computeMotorSequence(norm_state, prev_u, norm_setpoint)
        # apply first command
        u_normalized = cmds[0] / 255.0
        motorStep(u_normalized)
        prev_u = u_normalized
        trace.append(pos)
        pos_trace.append(pos)
        vel_trace.append(vel)
        acc_trace.append(acc)
        jrk_trace.append(jrk)
        times.append(i * dt)
        # Check settle: within 2% of setpoint for 0.2 sec
        if not settled and abs(pos - absolute_setpoint) <= 0.02 * abs(absolute_setpoint):
            # require hold for some steps
            hold_steps = int(0.2 / dt)
            ok = True
            # do inner loop to confirm hold
            for _ in range(hold_steps):
                motorStep(prev_u)
                ms2 = get_motor_state()
                if abs(ms2['position'] - absolute_setpoint) > 0.02 * abs(absolute_setpoint):
                    ok = False
                    break
            if ok:
                settle_time = i * dt
                settled = True
                break
    return settle_time if settle_time is not None else float('inf'), pos_trace, vel_trace, acc_trace, jrk_trace


def train_policy(num_steps=2000, setpoint=1.0, seed=0, policy_lr: float = 1e-3, updates_per_step: int = 2):
    # Run system identification to get an LTI model used for RL predictions
    print("Running system identification in-sim...")
    id_duration = 20.0  # seconds of simulated ID
    times_id, error_norms_id, update_norms_id, A_id, B_id, A_hist, B_hist = run_sim(duration_sec=id_duration, learning_rate=0.02, seed=seed)
    print("Estimated A (sys-id):\n", A_id)
    print("Estimated B (sys-id):\n", B_id)

    policy = RLPolicyPy(seed=seed, A=A_id, B=B_id)
    policy.policy_lr = policy_lr
    # Training loop: call computeMotorSequence and updatePolicyWeights repeatedly while applying
    # the first command to the real motor in simulation to collect true state.
    reset_motor_state()

    dt = 1.0 / 100.0
    prev_u = 0.0
    settle_history = []
    for step in range(num_steps):
        ms = get_motor_state()
        pos = ms['position']
        vel = ms['velocity']
        acc = ms['acceleration']
        jrk = ms['jerk']
        norm_state = np.array([pos / 100.0, vel / MaxVel, acc / MaxAcc, jrk / MaxJrk]).reshape(4, 1)

    # policy expects normalized setpoint (pos / MAX_POSITION)
        norm_setpoint = setpoint / 100.0
        cmds = policy.computeMotorSequence(norm_state, prev_u, norm_setpoint)
        # apply first command for some inner substeps
        prev_u = cmds[0] / 255.0
        for _ in range(int(1.0 / dt / 50.0)):  # apply for ~0.02s
            motorStep(prev_u)

    # update policy
        reward = policy.calculateReward()
        # Try several small policy gradient updates for each step to speed training
        reward = policy.calculateReward(norm_state)
        policy.updatePolicyWeights()

        # Record reward history for health check
        if step % 1 == 0:
            # keep small list to compute rolling average if desired
            settle_history.append((step, None))
        # Maintain reward history for health monitoring
        if 'reward_history' not in locals():
            reward_history = []
        reward_history.append(float(reward))

        # Evaluate occasional step response to measure progress
        if step % 50 == 0:
            settle_time, _, _, _, _ = evaluate_step_response(policy, sim_time=3.0, setpoint=setpoint, setpoint_abs=True)
            settle_history.append((step, settle_time))
            # Compute reward change per 50 steps
            nwin = 50
            recent = np.array(reward_history[-nwin:]) if len(reward_history) >= nwin else np.array(reward_history)
            prev = np.array(reward_history[-2 * nwin:-nwin]) if len(reward_history) >= 2 * nwin else None
            recent_avg = float(np.mean(recent)) if recent.size > 0 else 0.0
            prev_avg = float(np.mean(prev)) if prev is not None and prev.size > 0 else None
            delta = recent_avg - prev_avg if prev_avg is not None else None
            print(f"step={step}, settle_time={settle_time}, reward_avg_last50={recent_avg:.4f}", end='')
            if delta is not None:
                print(f", delta_from_prev50={delta:.6f}")
            else:
                print()
            # break if settle_time is good (less than 0.5s)
            if settle_time < 0.5:
                print(f"Found good policy at step {step}, settling {settle_time}s")
                break
    return policy, settle_history


if __name__ == '__main__':
    import argparse

    parser = argparse.ArgumentParser(description='Train RL policy in simulation and evaluate step response')
    parser.add_argument('--steps', type=int, default=2000, help='Number of training steps')
    parser.add_argument('--seed', type=int, default=3, help='Random seed')
    parser.add_argument('--policy-lr', type=float, default=1e-3, help='Policy learning rate')
    parser.add_argument('--updates', type=int, default=2, help='Policy updates per step')
    parser.add_argument('--eval-setpoint', type=float, default=2.0 * np.pi, help='Absolute step setpoint (radians)')
    args = parser.parse_args()

    policy, history = train_policy(num_steps=3000, setpoint=6.28, seed=args.seed, policy_lr=args.policy_lr, updates_per_step=args.updates)
    # Evaluate final policy at a step to 2*pi real radians
    setpoint_abs = args.eval_setpoint
    _, pos_trace, vel_trace, acc_trace, jrk_trace = evaluate_step_response(policy, sim_time=4.0, setpoint=setpoint_abs, setpoint_abs=True)

    # Print a short numeric summary of the step response
    if len(pos_trace) > 0:
        final_pos = pos_trace[-1]
        max_pos = max(pos_trace)
        overshoot = (max_pos - setpoint_abs) / setpoint_abs * 100.0
        print(f"Step to {setpoint_abs:.3f} rad: final={final_pos:.3f}, max={max_pos:.3f}, overshoot={overshoot:.1f}%")

    # Plot position, velocity, acceleration, jerk traces
    fig, axs = plt.subplots(4, 1, figsize=(8, 10), sharex=True)
    t = np.arange(len(pos_trace)) / 100.0
    axs[0].plot(t, pos_trace)
    axs[0].set_ylabel('pos (rad)')
    axs[0].set_title('Step to 2π')

    axs[1].plot(t, vel_trace)
    axs[1].set_ylabel('vel (rad/s)')

    axs[2].plot(t, acc_trace)
    axs[2].set_ylabel('acc (rad/s^2)')

    axs[3].plot(t, jrk_trace)
    axs[3].set_ylabel('jerk (rad/s^3)')
    axs[3].set_xlabel('Time (s)')

    plt.tight_layout()
    plt.show()
    print('history samples:', history)
