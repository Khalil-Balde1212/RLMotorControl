import numpy as np
import torch
import torch.nn as nn
import torch.optim as optim
from rl_train import MotorEnv
import matplotlib.pyplot as plt

class PolicyGradientAgent(nn.Module):
    def __init__(self, state_dim, setpoint_dim=1, control_dim=1, hidden_dim=128):
        super().__init__()
        input_dim = state_dim * 2 + setpoint_dim + control_dim
        self.net = nn.Sequential(
            nn.Linear(input_dim, hidden_dim),
            nn.ReLU(),
            nn.Linear(hidden_dim, hidden_dim),
            nn.ReLU(),
            nn.Linear(hidden_dim, 1),  # Output single control effort
            nn.Tanh()  # Output in [-1, 1]
        )

    def forward(self, state, next_state, setpoint, prev_control):
        x = torch.cat([state, next_state, setpoint, prev_control], dim=-1)
        return self.net(x)

    def select_action(self, state, next_state, setpoint, prev_control):
        state = torch.FloatTensor(state).unsqueeze(0)
        next_state = torch.FloatTensor(next_state).unsqueeze(0)
        setpoint = torch.FloatTensor([setpoint]).unsqueeze(0)
        prev_control = torch.FloatTensor([prev_control]).unsqueeze(0)
        mean = self.forward(state, next_state, setpoint, prev_control)
        std = torch.ones_like(mean) * 0.1  # Fixed std for exploration
        dist = torch.distributions.Normal(mean, std)
        action = dist.sample()
        action_clipped = torch.clamp(action, -1, 1)
        log_prob = dist.log_prob(action)
        return action_clipped.item(), log_prob

def reinforce_train(env, agent, optimizer, num_episodes=50, steps_per_episode=200, gamma=0.99):
    all_rewards = []
    for ep in range(num_episodes):
        state = env.reset()
        prev_control = 0.0
        setpoint = env.setpoint
        episode_rewards = []
        log_probs = []
        for t in range(steps_per_episode):
            next_state = state  # For demo, use current state as next_state
            action, log_prob = agent.select_action(state, next_state, setpoint, prev_control)
            next_state, reward, done, _ = env.step(action)
            log_probs.append(log_prob)
            episode_rewards.append(reward)
            state = next_state
            prev_control = action
            if done:
                break
        # Compute returns
        returns = []
        G = 0
        for r in reversed(episode_rewards):
            G = r + gamma * G
            returns.insert(0, G)
        returns = torch.FloatTensor(returns)
        log_probs = torch.cat(log_probs)
        # Policy gradient update
        loss = -torch.sum(log_probs * returns)
        optimizer.zero_grad()
        loss.backward()
        optimizer.step()
        all_rewards.append(sum(episode_rewards))
        print(f"Episode {ep+1}/{num_episodes} | Total Reward: {sum(episode_rewards):.2f}")
    return all_rewards

def plot_rewards(rewards):
    plt.plot(rewards)
    plt.xlabel('Episode')
    plt.ylabel('Total Reward')
    plt.title('Policy Gradient Training Reward')
    plt.show()

def run_policy_gradient_sim():
    setpoint = 2 * np.pi
    env = MotorEnv(setpoint)
    agent = PolicyGradientAgent(state_dim=5)
    optimizer = optim.Adam(agent.parameters(), lr=1e-3)
    rewards = reinforce_train(env, agent, optimizer, num_episodes=50, steps_per_episode=200)
    plot_rewards(rewards)

if __name__ == "__main__":
    run_policy_gradient_sim()
