#!/usr/bin/python
# -*- coding: utf-8 -*-


from math import *


class Pendulum:
    def __init__(self, start_cartx=None, start_angle=pi+pi/10,
                 track_length=1000, dt=0.01, g=9.81, l=1.0, m=1.0,
                 cfriction=.1, pfriction=0.1):

        self.track_length = track_length

        self.dt = dt
        self.g = g
        self.l = l
        self.m = m

        self.cfriction = cfriction
        self.pfriction = pfriction
        self.start_angle = start_angle

        self.reset(start_cartx, start_angle)

    def reset(self, start_cartx=None, start_angle=None):
        if start_angle is None:
            start_angle = self.start_angle
        self.angle = start_angle
        self.velocity = 0

        if start_cartx is None:
            start_cartx = self.track_length/2

        self.cartx = start_cartx
        self.carty = 0
        self.cartx_vel = 0
        self.massx = self.cartx + self.l * sin(self.angle)
        self.massy = self.carty + self.l * cos(self.angle)

    def update(self, control):
        ##################
        #calculate new pendulum angle
        ##################
        self.angle = atan2(self.massx - self.cartx, self.massy - self.carty)

        #calculate current acceleration
        acceleration = -self.g * self.m * sin(self.angle) / self.l  # gravity
        acceleration += (-self.velocity * self.pfriction) / self.m  # friction

        #update angle
        d_angle = self.velocity*self.dt + (0.5 * acceleration)*(self.dt ** 2)
        self.angle += d_angle

        #update velocity
        self.velocity += acceleration * self.dt

        self.massx = self.cartx + self.l * sin(self.angle)
        self.massy = self.carty + self.l * cos(self.angle)

        ##################
        #calculate new cart position
        ##################

        #calculate current acceleration
        acceleration = control  # input control
        acceleration += (-self.cartx_vel * self.cfriction)  # friction

        #update cart position
        dcartx = self.cartx_vel*self.dt + (0.5 * acceleration)*(self.dt ** 2)
        self.cartx += dcartx

        #update cart velocity
        self.cartx_vel += acceleration * self.dt

        #limit cart position
        if self.cartx > self.track_length or self.cartx < 0:
            self.cartx = self.cartx - dcartx

    def get_state(self):
        return self.cartx, self.angle-pi/2, self.cartx_vel, self.velocity


def tester():
    from visualizer import Visualizer
    import time
    sim = Pendulum()
    viz = Visualizer(sim.track_length)

    sim.update(1)
    x, angle, _, _ = sim.get_state()
    viz.draw(x, angle)
    while(1):
        x, angle, _, _ = sim.get_state()
        viz.draw(x, angle)
        sim.update(0)
        time.sleep(.01)


if __name__ == "__main__":
    tester()
