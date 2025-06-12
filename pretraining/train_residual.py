import torch
import torch.nn as nn
import torch.optim as optim
from torch.utils.data import Dataset, DataLoader
import numpy as np
from collections import deque

# Hyperparameters
CONTEXT_LENGTH = 5  # Can increase
RESIDUAL_SCALE = 0.2  # Max residual adjustment
HISTORY_SIZE = 5     # Number of previous observations to track

class ContextEncoder(nn.Module):
    def __init__(self, obs_dim=58, hidden_dim=256):
        super().__init__()
        self.obs_dim = obs_dim
        self.hidden_dim = hidden_dim
        
        self.encoder = nn.Sequential(
            nn.Linear(obs_dim * HISTORY_SIZE, hidden_dim),
            nn.LayerNorm(hidden_dim),
            nn.GELU(),
            nn.Linear(hidden_dim, hidden_dim),
            nn.LayerNorm(hidden_dim)
        )
        
    def forward(self, history):
        batch_size = history.size(0)
        return self.encoder(history.view(batch_size, -1))

class ResidualAgent(nn.Module):
    def __init__(self, obs_dim=58, act_dim=12, context_dim=256):
        super().__init__()
        self.obs_dim = obs_dim
        self.act_dim = act_dim
        self.context_dim = context_dim
        
        self.net = nn.Sequential(
            nn.Linear(obs_dim + context_dim, 512),
            nn.LayerNorm(512),
            nn.GELU(),
            nn.Linear(512, 256),
            nn.LayerNorm(256),
            nn.GELU(),
            nn.Linear(256, act_dim),
            nn.Tanh()
        )
        
    def forward(self, obs, context):
        x = torch.cat([obs, context], dim=-1)
        return RESIDUAL_SCALE * self.net(x)

class QuadrupedPolicy(nn.Module):
    def __init__(self, base_policy, value_net, q_net):
        super().__init__()
        # Load pretrained components
        self.base_policy = base_policy
        self.value_net = value_net
        self.q_net = q_net
        
        # Initialize new components
        self.context_encoder = ContextEncoder()
        self.residual_agent = ResidualAgent()
        
        # Freeze base networks
        for param in self.base_policy.parameters():
            param.requires_grad = False
        for param in self.value_net.parameters():
            param.requires_grad = False
            
    def get_action(self, obs, history):
        with torch.no_grad():
            base_action = self.base_policy(torch.FloatTensor(obs))
            
        context = self.context_encoder(history)
        residual = self.residual_agent(torch.FloatTensor(obs), context)
        return base_action + residual, context
    
    def get_value(self, obs, history):
        context = self.context_encoder(history)
        return self.value_net(torch.cat([obs, context], dim=-1))

# Data structures
class QuadrupedDataset(Dataset):
    def __init__(self, buffer_size=1e6):
        self.buffer = deque(maxlen=int(buffer_size))
        self.history = deque(maxlen=HISTORY_SIZE)
        
    def add_experience(self, obs, action, reward, next_obs, done):
        self.history.append(obs)
        if len(self.history) == HISTORY_SIZE:
            self.buffer.append((
                np.array(self.history),
                action,
                reward,
                next_obs,
                done
            ))
            
    def __len__(self):
        return len(self.buffer)
    
    def __getitem__(self, idx):
        history, action, reward, next_obs, done = self.buffer[idx]
        return (
            torch.FloatTensor(history),
            torch.FloatTensor(action),
            torch.FloatTensor([reward]),
            torch.FloatTensor(next_obs),
            torch.FloatTensor([done])
        )

# Training Loop
def train_residual_agent(env, policy, dataset, epochs=1000, batch_size=512):
    optimizer = optim.AdamW([
        {'params': policy.context_encoder.parameters()},
        {'params': policy.residual_agent.parameters()}
    ], lr=3e-4)
    
    mse_loss = nn.MSELoss()
    value_loss = nn.SmoothL1Loss()
    
    for epoch in range(epochs):
        # Collect trajectories
        obs = env.reset()
        history = deque(maxlen=HISTORY_SIZE)
        episode_rewards = []
        
        while True:
            if len(history) < HISTORY_SIZE:
                action = policy.base_policy(torch.FloatTensor(obs)).detach().numpy()
            else:
                action, _ = policy.get_action(obs, torch.FloatTensor(np.array(history)))
                action = action.detach().numpy()
                
            next_obs, reward, done, _ = env.step(action)
            dataset.add_experience(obs, action, reward, next_obs, done)
            episode_rewards.append(reward)
            obs = next_obs
            
            if done:
                break
                
        # Update networks
        if len(dataset) > batch_size:
            loader = DataLoader(dataset, batch_size=batch_size, shuffle=True)
            
            for batch in loader:
                history_batch, action_batch, reward_batch, next_obs_batch, done_batch = batch
                
                # Get base actions
                with torch.no_grad():
                    base_actions = policy.base_policy(history_batch[:, -1, :])
                
                # Context encoding
                contexts = policy.context_encoder(history_batch)
                
                # Residual actions
                residuals = policy.residual_agent(history_batch[:, -1, :], contexts)
                
                # Q-value regularization
                with torch.no_grad():
                    target_q = policy.q_net(torch.cat([
                        history_batch[:, -1, :],
                        contexts,
                        base_actions + residuals
                    ], dim=-1))
                    
                # Value prediction
                values = policy.get_value(history_batch[:, -1, :], history_batch)
                
                # Loss components
                q_loss = mse_loss(values, target_q)
                policy_loss = -values.mean()  # Maximize expected returns
                entropy_loss = 0.01 * (residuals**2).mean()  # Regularization
                
                total_loss = q_loss + policy_loss + entropy_loss
                
                optimizer.zero_grad()
                total_loss.backward()
                torch.nn.utils.clip_grad_norm_(policy.parameters(), 1.0)
                optimizer.step()
                
        print(f"Epoch {epoch+1}/{epochs} | Avg Reward: {np.mean(episode_rewards):.2f}")

# Usage
if __name__ == "__main__":
    # Load pretrained components
    base_policy = torch.load('pi_0.pt')
    value_net = torch.load('value_0808.pt')
    q_net = torch.load('Q_bc.pt')
    
    # Initialize policy
    policy = QuadrupedPolicy(base_policy, value_net, q_net)
    
    # Create environment and dataset
    env = YourQuadrupedEnvironment()  # Replace with actual environment
    dataset = QuadrupedDataset(buffer_size=1e6)
    
    # Train residual agent
    train_residual_agent(env, policy, dataset)
    
    # Save final components
    torch.save(policy.context_encoder.state_dict(), 'context_encoder.pt')
    torch.save(policy.residual_agent.state_dict(), 'residual_agent.pt')