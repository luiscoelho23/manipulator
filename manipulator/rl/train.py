import os
import gym
import torch
import torch.nn as nn
import torch.nn.functional as F
import torch.optim as optim
from torch.distributions import Normal
import numpy as np
import random
import env_dmp_obstacle as Env
from collections import namedtuple, deque
import matplotlib.pyplot as plt
from nn import PolicyNetwork, QNetwork


class ReplayBuffer():
    def __init__(self, buffer_limit, DEVICE):
        self.buffer = deque(maxlen=buffer_limit)
        self.dev = DEVICE

    def put(self, transition):
        self.buffer.append(transition)

    def sample(self, n):
        mini_batch = random.sample(self.buffer, n)
        s_lst, a_lst, r_lst, s_prime_lst, done_mask_lst = [], [], [], [], []

        for transition in mini_batch:
            s, a, r, s_prime, done = transition
            s_lst.append(s)
            a_lst.append(a)
            r_lst.append([r])
            s_prime_lst.append(s_prime)
            done_mask = 0.0 if done else 1.0
            done_mask_lst.append([done_mask])

        s_batch = torch.tensor(np.array(s_lst), dtype=torch.float).to(self.dev)
        a_batch = torch.tensor(np.array(a_lst), dtype=torch.float).to(self.dev)
        r_batch = torch.tensor(np.array(r_lst), dtype=torch.float).to(self.dev)
        s_prime_batch = torch.tensor(np.array(s_prime_lst), dtype=torch.float).to(self.dev)
        done_batch = torch.tensor(np.array(done_mask_lst), dtype=torch.float).to(self.dev)

        # r_batch = (r_batch - r_batch.mean()) / (r_batch.std() + 1e-7)

        return s_batch, a_batch, r_batch, s_prime_batch, done_batch

    def size(self):
        return len(self.buffer)


class SAC_Agent:
    def __init__(self,state_dim = 6,action_dim = 3):

        self.state_dim      = state_dim  
        self.action_dim     = action_dim 
        self.lr_pi          = 0.00001
        self.lr_q           = 0.00001
        self.gamma          = 0
        self.batch_size     = 2000
        self.buffer_limit   = 1000000
        self.tau            = 0.005   # for soft-update of Q using Q-target
        self.init_alpha     = 0.01
        self.target_entropy = - self.action_dim 
        self.lr_alpha       = 0.05
        self.DEVICE         = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        self.memory         = ReplayBuffer(self.buffer_limit, self.DEVICE)
        print(self.DEVICE)

        self.log_alpha = torch.tensor(np.log(self.init_alpha)).to(self.DEVICE)
        self.log_alpha.requires_grad = True
        self.log_alpha_optimizer = optim.Adam([self.log_alpha], lr=self.lr_alpha)

        self.PI  = PolicyNetwork(self.state_dim, self.action_dim, self.lr_pi).to(self.DEVICE)
        self.Q1        = QNetwork(self.state_dim, self.action_dim, self.lr_q).to(self.DEVICE)
        self.Q1_target = QNetwork(self.state_dim, self.action_dim, self.lr_q).to(self.DEVICE)
        self.Q2        = QNetwork(self.state_dim, self.action_dim, self.lr_q).to(self.DEVICE)
        self.Q2_target = QNetwork(self.state_dim, self.action_dim, self.lr_q).to(self.DEVICE)

        self.Q1_target.load_state_dict(self.Q1.state_dict())
        self.Q2_target.load_state_dict(self.Q2.state_dict())

    def choose_action(self, s):
        with torch.no_grad():
            action, log_prob = self.PI.sample(s.to(self.DEVICE))
        return action, log_prob

    def calc_target(self, mini_batch):
        s, a, r, s_prime, done = mini_batch
        with torch.no_grad():
            a_prime, log_prob_prime = self.PI.sample(s_prime)
            entropy = - self.log_alpha.exp() * log_prob_prime
            q1_target, q2_target = self.Q1_target(s_prime, a_prime), self.Q2_target(s_prime, a_prime)
            q_target = torch.min(q1_target, q2_target)
            target = r + self.gamma * done * (q_target + entropy)
        return target

    def train_agent(self):
        mini_batch = self.memory.sample(self.batch_size)
        s_batch, a_batch, r_batch, s_prime_batch, done_batch = mini_batch

        td_target = self.calc_target(mini_batch)

        #### Q1 train ####
        q1_loss = F.smooth_l1_loss(self.Q1(s_batch, a_batch), td_target)
        self.Q1.optimizer.zero_grad()
        q1_loss.mean().backward()
        self.Q1.optimizer.step()
        #### Q1 train ####

        #### Q2 train ####
        q2_loss = F.smooth_l1_loss(self.Q2(s_batch, a_batch), td_target)
        self.Q2.optimizer.zero_grad()
        q2_loss.mean().backward()
        self.Q2.optimizer.step()
        #### Q2 train ####

        #### pi train ####
        a, log_prob = self.PI.sample(s_batch)
        entropy = -self.log_alpha.exp() * log_prob

        q1, q2 = self.Q1(s_batch, a), self.Q2(s_batch, a)
        q = torch.min(q1, q2)

        pi_loss = -(q + entropy)  # for gradient ascent
        self.PI.optimizer.zero_grad()
        pi_loss.mean().backward()
        self.PI.optimizer.step()
        #### pi train ####

     

        #### alpha train ####
        self.log_alpha_optimizer.zero_grad()
        alpha_loss = -(self.log_alpha.exp() * (log_prob + self.target_entropy).detach()).mean()
        alpha_loss.backward()
        self.log_alpha_optimizer.step()
        #### alpha train ####

        #### Q1, Q2 soft-update ####
        for param_target, param in zip(self.Q1_target.parameters(), self.Q1.parameters()):
            param_target.data.copy_(param_target.data * (1.0 - self.tau) + param.data * self.tau)
        for param_target, param in zip(self.Q2_target.parameters(), self.Q2.parameters()):
            param_target.data.copy_(param_target.data * (1.0 - self.tau) + param.data * self.tau)
        #### Q1, Q2 soft-update ####


if __name__ == '__main__':

    model_save_dir = 'saved_models/'

    state_dim = 7
    action_dim = 5

    env = Env.DmpObstacleEnv()
    agent = SAC_Agent(state_dim=state_dim, action_dim=action_dim)

    EPISODE = 10000
    print_once = True
    score_list = []
    train_ep = 0

    for EP in range(EPISODE):
        state, _ = env.reset()
        score, done = 0.0, False

        while not done:
            action, log_prob = agent.choose_action(torch.FloatTensor(state))
            action = action.detach().cpu().numpy() 

            state_prime, reward, done, _, _ = env.step(action)

            agent.memory.put((state, action, reward, state_prime, done))

            score += reward

            state = state_prime

            if agent.memory.size() > 10000:
                if not train_ep:
                    print("Training")
                    train_ep = EP

                agent.train_agent()

        print("EP:{}, Avg_Score:{:.1f}".format(EP, score))
        score_list.append(score)
        
        torch.save(agent.PI.state_dict(), model_save_dir + "/sac_actor_EP"+str(EP)+".pt")

        if EP > 10000:
            if abs((score - score_list[EP - 1])/score_list[EP - 1]) < 0.01:
                print("Done Training")
                break

    #plt.plot(score_list[train_ep:])
    #plt.show()