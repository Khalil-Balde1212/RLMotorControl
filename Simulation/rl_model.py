import torch
import torch.nn as nn

class MotorEffortRLModel(nn.Module):
    def __init__(self, state_dim, setpoint_dim=1, control_dim=1, hidden_dim=128, output_steps=5):
        super(MotorEffortRLModel, self).__init__()
        input_dim = state_dim * 2 + setpoint_dim + control_dim
        self.net = nn.Sequential(
            nn.Linear(input_dim, hidden_dim),
            nn.ReLU(),
            nn.Linear(hidden_dim, hidden_dim),
            nn.ReLU(),
            nn.Linear(hidden_dim, output_steps)  # output_steps=5 for 5 future speeds
        )

    def forward(self, state, next_state, setpoint, prev_control):
        # Flatten and concatenate all inputs
        x = torch.cat([state, next_state, setpoint, prev_control], dim=-1)
        return self.net(x)

# Example usage:
# state_dim = 5 (for example)
if __name__ == "__main__":

    # Model and training setup
    model = MotorEffortRLModel(state_dim=5, output_steps=5)
    criterion = nn.MSELoss()
    optimizer = torch.optim.Adam(model.parameters(), lr=0.001)

    # Dummy training data (replace with your real data)
    batch_size = 16
    num_epochs = 10
    for epoch in range(num_epochs):
        # Generate random batch
        state = torch.randn(batch_size, 5)
        next_state = torch.randn(batch_size, 5)
        setpoint = torch.randn(batch_size, 1)
        prev_control = torch.randn(batch_size, 1)
    target_speeds = torch.randn(batch_size, 5)  # Replace with true future speeds

        # Forward pass
    output = model(state, next_state, setpoint, prev_control)
    loss = criterion(output, target_speeds)

    # Backward and optimize
    optimizer.zero_grad()
    loss.backward()
    optimizer.step()

    print(f"Epoch {epoch+1}/{num_epochs}, Loss: {loss.item():.4f}")
