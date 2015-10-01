#!/usr/bin/python

import random

class Agent:
	def __init__(self):
		print "initializing agent!"

	def get_action(self, x, angle, dx, dangle):
		action = random.gauss(0, 1)
		return action
