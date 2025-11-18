import numpy as np
import matplotlib.pyplot as plt
from rl_train import MotorEnv
from rl_policy_gradient import PolicyGradientAgent
import torch
import torch.optim as optim


def run_policy_gradient_simulation(setpoint, num_episodes=200, steps_per_episode=200):
    env = MotorEnv(setpoint)
    agent = PolicyGradientAgent(state_dim=5)
    optimizer = optim.Adam(agent.parameters(), lr=0.0008)
    all_pos = []
    all_vel = []
    all_acc = []
    all_jerk = []
    for ep in range(num_episodes):
        print(f"Episode {ep+1}/{num_episodes} starting...")
        state = env.reset()
        prev_control = 0.0
        setpoint = env.setpoint
        pos_list, vel_list, acc_list, jerk_list = [], [], [], []
        log_probs = []
        rewards = []
        for t in range(steps_per_episode):
            next_state = state  # For demo, use current state as next_state
            action, log_prob = agent.select_action(state, next_state, setpoint, prev_control)
            # Pass step and max_steps to env.step
            next_state, reward, done, _ = env.step(action, t, steps_per_episode)
            log_probs.append(log_prob)
            rewards.append(reward)
            pos_list.append(next_state[0])
            vel_list.append(next_state[1])
            acc_list.append(next_state[2])
            jerk_list.append(next_state[3])
            state = next_state
            prev_control = action
            if done:
                break
        # Compute returns
        returns = []
        G = 0
        gamma = 0.99
        for r in reversed(rewards):
            G = r + gamma * G
            returns.insert(0, G)
        returns = torch.FloatTensor(returns)
        log_probs = torch.cat(log_probs)
        # Policy gradient update
        loss = -torch.sum(log_probs * returns)
        optimizer.zero_grad()
        loss.backward()
        optimizer.step()
        print(f"Episode {ep+1} finished. Total steps: {len(pos_list)} | Total Reward: {sum(rewards):.2f}")
        all_pos.append(pos_list)
        all_vel.append(vel_list)
        all_acc.append(acc_list)
        all_jerk.append(jerk_list)
    return all_pos, all_vel, all_acc, all_jerk

def plot_step_response(pos, vel, acc, jerk, setpoint):
    steps = len(pos[0])
    t = np.arange(steps)
    plt.figure(figsize=(12, 8))
    plt.subplot(4, 1, 1)
    plt.plot(t, pos[0], label='Position')
    plt.axhline(setpoint, color='r', linestyle='--', label='Setpoint')
    plt.legend()
    plt.ylabel('Position (rad)')
    plt.subplot(4, 1, 2)
    plt.plot(t, vel[0], label='Velocity')
    plt.legend()
    plt.ylabel('Velocity (rad/s)')
    plt.subplot(4, 1, 3)
    plt.plot(t, acc[0], label='Acceleration')
    plt.legend()
    plt.ylabel('Acceleration (rad/s^2)')
    plt.subplot(4, 1, 4)
    plt.plot(t, jerk[0], label='Jerk')
    plt.legend()
    plt.ylabel('Jerk (rad/s^3)')
    plt.xlabel('Time step')
    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    setpoint = 2 * np.pi
    pos, vel, acc, jerk = run_policy_gradient_simulation(setpoint, num_episodes=500, steps_per_episode=100)
    plot_step_response(pos, vel, acc, jerk, setpoint)
