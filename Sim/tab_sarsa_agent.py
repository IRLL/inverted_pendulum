#!/usr/bin/python
# -*- coding: utf-8 -*-
"""Tabular Sarsa(λ).

https://webdocs.cs.ualberta.ca/~sutton/book/ebook/node77.html

Value ranges when using impulses of -5 to 5.
x:0.000005-1.997147
dx:-9.145666-46.358980
angle:-4.658873-1.410222
dangle:-11.095294-12.640812
"""

import random
import math
import parameters


# [MIN, MAX, RESOLUTION]
X_RANGE = [0.0, 2.0, 200]
DX_RANGE = [-30.0, 30.0, 20]
ANGLE_RANGE = [-4.7, 1.5, 10]
DANGLE_RANGE = [-10, 10, 10]

# For 7 actions, 3 will be lefts(0, 1, 2), 1 will be neutral(3),
# 3 will be rights(4, 5, 6).
ACTIONS = [-5, -2, 2, 5]
NUM_ACTIONS = len(ACTIONS)


class Agent:
    def __init__(self):
        print("Initializing SARSA agent!")
        self.alpha = parameters.ALPHA
        self.epsilon = parameters.EPSILON
        self.lmbda = parameters.LAMBDA  # Because lambda is a keyword.

        self.gamma = 1.0

        self.action = None
        self.state = None
        self.q_values = dict()
        self.e_traces = dict()
        self.visited = set()
        self.last_visit_count = len(self.visited)

    def translate(self, x, angle, dx, dangle):
        """Translate real values into table index form."""
        def index(bounds, value):
            if value < bounds[0]:
                index = 0
            elif value >= bounds[1]:
                index = bounds[2] + 1
            else:
                value -= bounds[0]
                offset = (bounds[1] - bounds[0]) / bounds[2]
                index = int(value / offset)
            return index
        x = index(X_RANGE, x)
        angle = index(ANGLE_RANGE, angle)
        dx = index(DX_RANGE, dx)
        dangle = index(DANGLE_RANGE, dangle)

        return [x, angle, dx, dangle]

    def init_trial(self, *meta):
        exp_num = meta[0]
        print("Sarsa agent initializing for experiment %d" % (exp_num + 1))

        # Reset collected reward stats.
        self.episode_rewards = []
        self.cum_episode_reward = 0

        # Reset table and traces.
        self.q_values = dict()
        self.e_traces = dict()

    def init_episode(self, *meta):
        _, x, angle, dx, dangle, _ = meta

        self.state = [x, angle, dx, dangle]
        self.action = self.egreedy(self.state)
        self.cum_episode_reward = 0
        self.e_traces = dict()

        return ACTIONS[self.action]

    def egreedy(self, state):
        # ε-exploration.
        if random.random() < self.epsilon:
            return random.randint(0, NUM_ACTIONS - 1)

        # ε-exploitation.
        discretized = self.translate(*state)
        qs = [discretized + [i] for i in range(NUM_ACTIONS)]
        evaluated_qs = [self.q_values.get(tuple(i), 0) for i in qs]
        candidate_actions = [i for i, v in enumerate(evaluated_qs) if v ==
                             max(evaluated_qs)]

        return random.choice(candidate_actions)

    def get_action(self, x, angle, dx, dangle, edge):
        reward = -(math.pi - angle)**2
        self.cum_episode_reward += reward
        action_index = self.agent_step([x, angle, dx, dangle])

        return ACTIONS[action_index]

    def agent_step(self, state):
        # Took action self.action, observed reward and current state (s').
        x, angle, dx, dangle = state
        # Choose a' from s' using policy derived from Q (e.g., ε-greedy).
        ap = self.egreedy([x, angle, dx, dangle])

        # δ ← r + γQ(s', a') - Q(s, a)
        reward = -(math.pi - angle)**2
        disc_sp_ap = tuple(self.translate(*state) + [ap])
        self.visited.add(disc_sp_ap)
        disc_s_a = tuple(self.translate(*self.state) + [self.action])
        q_sp_ap = self.q_values.get(disc_sp_ap, 0)
        q_s_a = self.q_values.get(disc_s_a, 0)
        delta = reward + self.gamma * q_sp_ap - q_s_a

        # e(s, a) ← e(s, a) + 1
        self.e_traces[disc_s_a] = self.e_traces.get(disc_s_a, 0) + 1

        # For all s, a:
        for s_a in self.e_traces:
            # Q(s, a) ← Q(s, a) + αδe(s, a)
            self.q_values[s_a] = self.q_values.get(s_a, 0) + \
                self.alpha * delta * self.e_traces[s_a]
            # e(s, a) ← γλe(s, a)
            self.e_traces[s_a] = self.gamma * self.lmbda * \
                self.e_traces[s_a]

        self.state = state
        self.action = ap

        return ap

    def end_episode(self, *meta):
        self.episode_rewards.append(self.cum_episode_reward)
        print("ep_reward: %d, states visited: %d, diff: %d" %
              (self.cum_episode_reward,
               len(self.visited),
               len(self.visited) - self.last_visit_count))
        self.last_visit_count = len(self.visited)

    def end_trial(self, *meta):
        print(self.episode_rewards)
