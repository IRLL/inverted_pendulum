#!/usr/bin/python
# -*- coding: utf-8 -*-
"""Test agent with random behavior.

Uses the random behavior to explore then list the possible range of values.
"""


import random


class Agent:
    def __init__(self):
        print "initializing agent!"
        self.xs = []
        self.angles = []
        self.dxs = []
        self.dangles = []
        self.first = 0

    def get_action(self, x, angle, dx, dangle, edge):
        #action = random.gauss(0, 1)
        if self.first < 20:
            action = -5
            self.first += 1
        elif self.first < 100:
            action = 5
            self.first += 1
        else:
            action = -5

        self.xs.append(x)
        self.angles.append(angle)
        self.dxs.append(dx)
        self.dangles.append(dangle)
        return action

    def end_episode(self, *meta):
        print("x:%f-%f" % (min(self.xs), max(self.xs)))
        print("dx:%f-%f" % (min(self.dxs), max(self.dxs)))
        print("angle:%f-%f" % (min(self.angles), max(self.angles)))
        print("dangle:%f-%f" % (min(self.dangles), max(self.dangles)))
