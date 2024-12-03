import torch
import torch.optim as optim
from torch.distributions import MultivariateNormal
from .policy_network import PolicyNetwork
from .value_network import ValueNetwork

class PPOAgent:
    def __init__(self, state_dim, action_dim, lr=3e-4, gamma=0.99, eps_clip=0.2, k_epochs=4):
        self.policy = PolicyNetwork(state_dim, action_dim).cuda()
        self.value = ValueNetwork(state_dim).cuda()
        self.policy_optimizer = optim.Adam(self.policy.parameters(), lr=lr)
        self.value_optimizer = optim.Adam(self.value.parameters(), lr=lr)
        self.gamma = gamma
        self.eps_clip = eps_clip
        self.k_epochs = k_epochs

    def select_action(self, state):
        state = torch.FloatTensor(state).unsqueeze(0).cuda()
        mean = self.policy(state)
        std = torch.ones_like(mean) * 0.5
        dist = MultivariateNormal(mean, torch.diag_embed(std))
        action = dist.sample()
        log_prob = dist.log_prob(action)
        return action.cpu().detach().numpy(), log_prob
