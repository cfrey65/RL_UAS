import torch
import numpy as np

class ActorCritic(torch.nn.Module):
    def __init__(self, actor_lr, critic_lr, input, output, layerSize, discount, 
                 isCamera, maxAng=torch.tensor([1,1])):
        super().__init__()
        self.input = input
        self.maxAng = maxAng
        self.output = output
        self.discount = discount
        self.actor_lr = actor_lr
        self.critic_lr = critic_lr
        self.isCamera = isCamera
        self.actor = torch.nn.Sequential(
            torch.nn.Linear(input, layerSize),
            torch.nn.Tanh(),
            torch.nn.Linear(layerSize, layerSize // 2),
            torch.nn.Tanh(),
            torch.nn.Linear(layerSize // 2, layerSize // 4),
            torch.nn.Tanh(),
            torch.nn.Linear(layerSize // 4, output),
        )
        self.actor_logstds_param = torch.nn.Parameter(torch.full((output,), 0.1))

        self.critic = torch.nn.Sequential(
            torch.nn.Linear(input, layerSize),
            torch.nn.ReLU(),
            torch.nn.Linear(layerSize, layerSize // 2),
            torch.nn.ReLU(),
            torch.nn.Linear(layerSize // 2, layerSize // 4),
            torch.nn.ReLU(),
            torch.nn.Linear(layerSize // 4, 1)
        )

        self.distributions = None
        self.actor_optimizer = torch.optim.Adam(self.parameters(), lr=actor_lr)
        self.critic_optimizer = torch.optim.Adam(self.parameters(), lr=critic_lr)

    def forwardActor(self, x):
        mus = self.actor(x)
        if (self.isCamera):
            out = torch.nn.Tanh()
            mus = out(mus)* self.maxAng
        stds = torch.clamp(self.actor_logstds_param.exp(), 1e-3, 50)
        return torch.distributions.Normal(mus, stds)
    
    def getActorLoss(self, adv):
        log_prob = self.distributions[0].log_prob(self.distributions[1])
        return (-log_prob * adv.detach()).mean()

    def forwardCritic(self, x):
        return self.critic(x)
    
    def getCriticLoss(self, x, y):
        return torch.nn.functional.mse_loss(x, y)
    
    def getAction(self, obs):
        dis = self.forwardActor(torch.tensor(obs, dtype=torch.float32))
        a = dis.sample()
        self.distributions = (dis, a)
        return a
    
    def learn(self, obs, fu_obs, reward):
        val = self.forwardCritic(torch.tensor(obs, dtype=torch.float32))
        targ = reward + self.discount*self.forwardCritic(torch.tensor(fu_obs, dtype=torch.float32))
        adv = targ - val
        critLoss = self.getCriticLoss(val, adv)
        self.critic_optimizer.zero_grad()
        critLoss.backward()
        self.critic_optimizer.step()
        actor_loss = self.getActorLoss(adv)
        self.actor_optimizer.zero_grad()
        actor_loss.backward()
        self.actor_optimizer.step()
