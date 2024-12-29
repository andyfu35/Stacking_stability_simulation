import torch
import torch.nn as nn
import torch.optim as optim

class PPOActorCritic(nn.Module):
    def __init__(self, state_dim, action_dim, hidden_dim=128):
        """
        state_dim: 狀態空間維度
        action_dim: 動作空間維度
        hidden_dim: 隱藏層大小
        """
        super(PPOActorCritic, self).__init__()
        # Actor 網絡
        self.actor = nn.Sequential(
            nn.Linear(state_dim, hidden_dim),
            nn.ReLU(),
            nn.Linear(hidden_dim, action_dim),
            nn.Softmax(dim=-1)  # 輸出動作概率
        )
        # Critic 網絡
        self.critic = nn.Sequential(
            nn.Linear(state_dim, hidden_dim),
            nn.ReLU(),
            nn.Linear(hidden_dim, 1)  # 輸出狀態價值
        )

    def forward(self, state):
        action_probs = self.actor(state)
        state_value = self.critic(state)
        return action_probs, state_value


class PPO:
    def __init__(self, state_dim, action_dim, lr=3e-4, gamma=0.99, epsilon=0.2, hidden_dim=128):
        """
        state_dim: 狀態空間維度
        action_dim: 動作空間維度
        lr: 學習率
        gamma: 折扣因子
        epsilon: 策略裁剪參數
        hidden_dim: 隱藏層大小
        """
        self.gamma = gamma
        self.epsilon = epsilon
        self.model = PPOActorCritic(state_dim, action_dim, hidden_dim)
        self.optimizer = optim.Adam(self.model.parameters(), lr=lr)

    def compute_returns(self, rewards, dones, next_value):
        """
        計算返回值（Returns）
        """
        returns = []
        R = next_value
        for r, done in zip(reversed(rewards), reversed(dones)):
            R = r + self.gamma * R * (1 - done)
            returns.insert(0, R)
        return torch.tensor(returns, dtype=torch.float32)

    def update(self, states, actions, log_probs_old, returns, advantages):
        """
        更新 Actor 和 Critic 網絡
        """
        action_probs, state_values = self.model(states)
        new_probs = action_probs.gather(1, actions.unsqueeze(-1)).squeeze(-1)

        # 策略損失
        ratio = new_probs / log_probs_old
        clipped_ratio = torch.clamp(ratio, 1 - self.epsilon, 1 + self.epsilon)
        policy_loss = -torch.min(ratio * advantages, clipped_ratio * advantages).mean()

        # 值損失
        value_loss = (returns - state_values.squeeze(-1)).pow(2).mean()

        # 總損失
        loss = policy_loss + 0.5 * value_loss

        self.optimizer.zero_grad()
        loss.backward()
        self.optimizer.step()
