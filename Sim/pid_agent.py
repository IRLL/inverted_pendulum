#!/usr/bin/python
# -*- coding: utf-8 -*-
"""Test agent with random behavior.
"""


import random
import math


class Agent:
    def __init__(self):
        self.kp = 500
        self.ki = 1
        self.kd = 100

        self.error_integral = 0
        self.prev_error = 0
        print "initializing agent!"

    def get_action(self, x, angle, dx, dangle, edge):
        error = copysign(1, angle) * (pi-abs(angle)) + x
        derivative = self.prev_error - error
        self.prev_error = error
        self.error_integral += error

        action = self.kp * error + 
                 self.ki * self.error_integral +
                 self.kd * derivative

        return action

    def init_trial(self, *meta):
        self.error_integral = 0
        self.prev_error = 0
        exp_num, _, _, _, _, _ = meta
        print("PID agent initializing for experiment %d" % (exp_num + 1))
        self.episode_rewards = []
        self.cum_episode_reward = 0
        
    def init_episode(self, *meta):
        self.cum_episode_reward = 0
        
    def end_episode(self, *meta):
        self.episode_rewards.append(self.cum_episode_reward)
        print(self.cum_episode_reward)
        
    def end_trial(self, *meta):
        print(self.episode_rewards)
        
