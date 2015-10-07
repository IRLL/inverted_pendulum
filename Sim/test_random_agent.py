#!/usr/bin/python
# -*- coding: utf-8 -*-
"""Test agent with random behavior.
"""


import random
import math


class Agent:
    def __init__(self):
        print "initializing agent!"

    def get_action(self, x, angle, dx, dangle):
        reward = -(math.pi - angle)**2
        self.cum_episode_reward += reward
        
        action = random.gauss(0, 1)
        return action

    def init_trial(self, *meta):
        exp_num, _, _, _, _ = meta
        print("Sarsa agent initializing for experiment %d" % (exp_num + 1))
        self.episode_rewards = []
        self.cum_episode_reward = 0
        
    def init_episode(self, *meta):
        self.cum_episode_reward = 0
        
    def end_episode(self, *meta):
        self.episode_rewards.append(self.cum_episode_reward)
        print(self.cum_episode_reward)
        
    def end_trial(self, *meta):
        print(self.episode_rewards)
        