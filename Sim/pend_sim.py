#!/usr/bin/python
from math import *

dt = 0.01
g = 9.81
l = 1.0
m = 1.0


class Pendulum:
	def __init__(self, start_cartx=None, start_angle=pi+pi/10, track_length=1000):
		self.track_length = track_length
		self.reset(start_x, start_angle)

	def reset(self, start_x=None, start_angle=pi+pi/10):
		self.angle0 = start_angle
		self.angle = angle0
		self.velocity = 0

		if(start_cartx == None):
			start_cartx = self.track_length/2

		self.cartx = start_cartx
		self.carty = 0
		self.cartx_vel = 0
		self.massx = cartx + 250.0 * sin(angle0)
		self.massy = carty + 250.0 * cos(angle0)


	def update(self, control):
		self.angle = atan2(self.massx - self.cartx, self.massy - self.carty)
		d_velocity = -g * sin(self.angle) * dt / l
		self.velocity = self.velocity + d_velocity
		d_angle = dt * self.velocity
		self.angle = self.angle + d_angle
		self.massx = self.cartx + 250.0 * sin(self.angle)
		self.massy = self.carty + 250.0 * cos(self.angle)

		d_cartx_vel = dt * control
		self.cartx_vel = self.cartx_vel + d_cartx_vel
		dcartx = dt * self.cartx_vel
		self.cartx = self.cartx + dcartx

		if self.cartx > self.track_length or self.cartx < 0:
			self.cartx = self.cartx - dcartx

