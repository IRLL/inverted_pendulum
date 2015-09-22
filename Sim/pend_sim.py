#!/usr/bin/python
from math import *

dt = 0.01
g = 9.81
l = 1.0
m = 1.0


class Pendulum:
	def __init__(self, start_x=None, start_angle=pi+pi/10, track_length=1000):
		self.track_length = track_length
		self.reset(start_x, start_angle)

	def reset(self, start_x=None, start_angle=pi+pi/10):
		self.angle0 = start_angle
		self.angle = angle0
		self.velocity = 0

		if(start_x == None):
			start_x = self.track_length/2

		self.x0 = start_x
		self.y0 = 0
		self.x0_vel = 0
		self.x = x0 + 250.0 * sin(angle0)
		self.y = y0 + 250.0 * cos(angle0)


	def update(self, control):
		self.angle = atan2(self.x - self.x0, self.y - self.y0)
		d_velocity = -g * sin(self.angle) * dt / l
		self.velocity = self.velocity + d_velocity
		d_angle = dt * self.velocity
		self.angle = self.angle + d_angle
		self.x = self.x0 + 250.0 * sin(self.angle)
		self.y = self.y0 + 250.0 * cos(self.angle)

		d_x0_vel = dt * control
		self.x0_vel = self.x0_vel + d_x0_vel
		dx0 = dt * self.x0_vel
		self.x0 = self.x0 + dx0

		if self.x0 > self.track_length or self.x0 < 0:
			self.x0 = self.x0 - dx0


p = Pendulum()
while 1:
	screen.fill((255,255,255))
	for event in pygame.event.get():
		if event.type == pygame.QUIT:
			pygame.quit()
	pid.update()
	p.move(pid.output())
	p.draw()
	draw_text()
	(k1, k2, k3) = make_buttons(k1, k2, k3, pid, p)


	clock.tick(60)
	pygame.display.flip()
