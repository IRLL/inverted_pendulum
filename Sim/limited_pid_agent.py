#!/usr/bin/python
# -*- coding: utf-8 -*-
"""Pid controller agent limited to discrete actions.

The idea here is that the PID controller models the perfect policy to maintain
an upright pole on the cart. If the policy is forced to approximate perfect
control with a limited/discrete set of action, this could represent the
feasibility of an intelligent agent ability to solve the task with the same set
of limited/discrete actions. If the PID controller can't, it might suggest that
a learning agent wouldn't be able to solve the task with this set of actions. 
"""


from __future__ import print_function
import random
from math import copysign, pi


# Limited action space.
#ACTIONS = [-10, -5, -2, -1, -0.1, -0.001, 0, 0.001, 0.1, 1, 2, 5, 10]
ACTIONS = [-2, 0, 2]
NUM_ACTIONS = len(ACTIONS)


class Agent:
    def __init__(self):
        self.kp = 50
        self.ki = .3
        self.kd = -200

        self.error_integral = 0
        self.prev_error = 0
        print("initializing agent!")

        # Extra
        self.xs = []
        self.angles = []
        self.dxs = []
        self.dangles = []
        self.impulses = []

    def get_action(self, x, angle, dx, dangle, edge):
        error = -angle + x/10
        derivative = self.prev_error - error
        self.prev_error = error
        self.error_integral += error

        action = self.kp * error + \
                 self.ki * self.error_integral + \
                 self.kd * derivative


        self.xs.append(x)
        self.angles.append(angle)
        self.dxs.append(dx)
        self.dangles.append(dangle)
        self.impulses.append(action)

        l = [a for a in ACTIONS] + [action]
        l = sorted(l)
        index = l.index(action)
        # If the predicted action must exceed our ACTIONS, bound it on both
        # ends.
        if index == 0:
            action = ACTIONS[0]
        elif action >= ACTIONS[-1]:
            action = ACTIONS[-1]

        # Otherwise, pick the closest ACTION possible.
        else:
            t1 = (l[index - 1] - action)**2
            t2 = (l[index + 1] - action)**2
            if t1 >= t2:
                action = l[index + 1]
            else:
                action = l[index - 1]

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

        print("x       %f:%f" % (min(self.xs), max(self.xs)))
        print("dx      %f:%f" % (min(self.dxs), max(self.dxs)))
        print("angle   %f:%f" % (min(self.angles), max(self.angles)))
        print("dangle  %f:%f" % (min(self.dangles), max(self.dangles)))
        print("impulse %f:%f" % (min(self.impulses), max(self.impulses)))

    def end_trial(self, *meta):
        print(self.episode_rewards)
        return action

