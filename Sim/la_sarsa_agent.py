#!/usr/bin/python
# -*- coding: utf-8 -*-
"""Sarsa agent using a NN.
"""


import random
import parameters
import numpy as np
import math

from random import Random
import numpy
import copy

import neurolab as nl

""" Value observations from running a random test.
x:0.000000:2.000000
dx:-2.975710:2.229757
angle:-4.712300:1.570793
dangle:-11.844069:11.590067

"""
# For 7 actions, 3 will be lefts(0, 1, 2), 1 will be neutral(3), 3 will be rights(4, 5, 6).
ACTIONS = [-2, -1, -0.5, 0, 0.5, 1, 2]

class Agent:
    def __init__(self):
        print("Initializing SARSA agent!")
        self.alpha = parameters.ALPHA
        self.epsilon = parameters.EPSILON
        self.lmbda = parameters.LAMBDA  # Because lambda is a keyword.
        
        self.xs = []
        self.angles = []
        self.dxs = []
        self.dangles = []
        
        self.gamma = 1.0
        self.net = None
        self.softmax = False
        self.num_hidden = 50
        
        
        self.action = None
        self.state = None
        
        
    def init_trial(self, *meta):
        exp_num, _, _, _, _ = meta
        print("Sarsa agent initializing for experiment %d" % (exp_num + 1))
        
        self.num_actions = 7

        # Set up the function approximation
        # Meh. Close enough.
        minmax = [[0, 2], [-3, 2.3], [-5, 2], [-12, 12]]
        self.net = nl.net.newff(minmax, [self.num_hidden, self.num_actions], [nl.net.trans.TanSig(), nl.net.trans.PureLin()])
        self.traces = copy.deepcopy(map(lambda x: x.np, self.net.layers))
        self.clearTraces()
        
        self.episode_rewards = []
        self.cum_episode_reward = 0
        
    def init_episode(self, *meta):
        _, x, angle, dx, dangle = meta
        self.state = [x, angle, dx, dangle]

        reward = -(math.pi - angle)**2
        action_index = self.agent_step(reward, [x, angle, dx, dangle])
        self.action = action_index
        
        self.clearTraces()
        self.cum_episode_reward = 0

        return ACTIONS[action_index]
        
    def get_action(self, x, angle, dx, dangle):
        reward = -(math.pi - angle)**2
        self.cum_episode_reward += reward
        #action = random.gauss(0, 1)
        action_index = self.agent_step(reward, [x, angle, dx, dangle])
        
        return ACTIONS[action_index]
    
    def end_episode(self, *meta):
        _, x, angle, dx, dangle = meta
        reward = -(math.pi - angle)**2
        last_state = [x, angle, dx, dangle]
        last_action = self.action

        # Update eligibility traces
        self.decayTraces()
        self.update(last_state, last_action, None, 0, reward)
        
        
        self.episode_rewards.append(self.cum_episode_reward)
        print(self.cum_episode_reward)
        
    def end_trial(self, *meta):
        print(self.episode_rewards)
        
    def sample_softmax(self, state):
        Q = self.net.sim([state]).flatten()
        Q = numpy.exp(numpy.clip(Q / self.epsilon, -500, 500))
        Q /= Q.sum()

        # Would like to use numpy, but haven't upgraded enough (need 1.7)
        # numpy.random.choice(self.numActions, 1, p=Q)
        Q = Q.cumsum()
        return numpy.where(Q >= numpy.random.random())[0][0]

    def egreedy(self, state):
        if random.random() < self.epsilon:
            return random.randint(0, self.num_actions - 1)

        return self.net.sim([state]).argmax()

    def agent_step(self, reward, new_state):
        new_action = self.egreedy(new_state)

        # Update eligibility traces
        self.decayTraces()
        self.update(self.state, self.action, new_state, new_action, reward)

        self.action = new_action
        self.state = new_state
        return new_action

    def clearTraces(self):
        for layer in range(len(self.traces)):
            self.traces[layer]['b'][:] = 0.0
            self.traces[layer]['w'][:] = 0.0

    def decayTraces(self):
        for layer in range(len(self.traces)):
            self.traces[layer]['b'][:] *= self.lmbda * self.gamma
            self.traces[layer]['w'][:] *= self.lmbda * self.gamma

    def updateTraces(self, gradient, delta):
        for layer in range(len(self.traces)):
            self.traces[layer]['b'] += gradient[layer]['b'] / delta
            self.traces[layer]['w'] += gradient[layer]['w'] / delta

    def update(self, x_t, a_t, x_tp, a_tp, reward):
        # Compute Delta (TD-error)
        Q_t = self.net.sim([x_t]).flatten()
        Q_tp = self.net.sim([x_tp])[0, a_tp] if x_tp is not None else 0.0
        deltaQ = Q_t.copy()
        delta = self.gamma * Q_tp + reward - Q_t[a_t]
        # XXX I don't even know...
        try:
            delta = sum(delta[0]) / len(delta[0])
        except:
            pass
        deltaQ[a_t] = self.gamma * Q_tp + reward
#        print delta
        grad = nl.tool.ff_grad_step(self.net, Q_t, deltaQ)
        self.updateTraces(grad, delta)

        # Update the weights
        for layer in range(len(self.traces)):
            self.net.layers[layer].np['b'] -= self.alpha * delta * self.traces[layer]['b']
            self.net.layers[layer].np['w'] -= self.alpha * delta * self.traces[layer]['w']

        # newQ = self.net.sim([x_t]).flatten()
        # print Q_t[a_t], deltaQ[a_t], newQ[a_t]
