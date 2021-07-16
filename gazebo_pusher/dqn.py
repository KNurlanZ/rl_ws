import numpy as np
import gym
import gazebo_pusher
import torch
import torch
from torch import nn
from torch import optim
import torch.nn.functional as F
from torch.autograd import Variable
import torchvision
class dqn(nn.Module):
    def __init__(self, n_input, n_output, epsilon, alpha, gamma, size):
        super(Net,self).__init__()
        self.fc1 = nn.Linear(n_input, 16)
        self.fc2 = nn.Linear(16, 64)
        self.fc3 = nn.Linear(64, 256)
        self.fc4 = nn.Linear(256, 1024)
        self.fc5 = nn.Linear(1024,256)
        self.fc6 = nn.Linear(256,128)
        self.fc7 = nn.Linear(128,64)
        self.fc8 = nn.Linear(64,32)
        self.fc9 = nn.Linear(32,n_output)
        self.epsilon = epsilon
        self.alpha = alpha
        self.gamma = gamma
        self.size = size
        self.buffer = deque(maxlen=size)
    def forward(self,x):
        x = F.relu(self.fc1(x))
        x = F.relu(self.fc2(x))
        x = F.relu(self.fc3(x))
        x = F.relu(self.fc4(x))
        x = F.relu(self.fc5(x))
        x = F.relu(self.fc6(x))
        x = F.relu(self.fc7(x))
        x = F.relu(self.fc8(x))
        x = self.fc9(x)
        return x

    def add(self, state, action, reward, done, nextState):
        exp = (state, action, reward, done, nextState)
        self.buffer.append(exp)

    def sample(self, sample_size):
        state_batch = []
        action_batch = []
        reward_batch = []
        done_batch = []
        nextState_batch = []

        batch = random.sample(self.buffer, sample_size)
        for exp in batch:
            s, a, r, d, n = exp
            state_batch.append(s)
            action_batch.append(a)
            reward_batch.append(r)
            done_batch.append(d)
            nextState_batch.append(n)
        return batch

    def length(self):
        return len(self.buffer)


    def learn(self):
        [state, action, reward, done, nextState] = self.sample(16)
