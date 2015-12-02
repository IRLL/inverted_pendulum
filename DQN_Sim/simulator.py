#!/usr/bin/python3
# -*- coding: utf-8 -*-
"""Inverted Pendulum Simulator.

This module implements the inverted pendulum simulator.
"""


import sys
import os
import random
import time
import argparse
import importlib
import zmq
import subprocess
from model import Pendulum
from visualizer import Visualizer

context = zmq.Context()
socket = context.socket(zmq.REP)
socket.bind("tcp://*:2355")


class Simulator:
    def __init__(self, model, num_trials, num_episodes, episode_length,
                 sim_timestep, gui_active, gui_freq, save_gui_dir, agent_freq, 
                 reset_angle, reset_angle_variance):

        self.num_trials = num_trials
        self.num_episodes = num_episodes
        self.num_timesteps = int(episode_length/sim_timestep)
        self.episode_length = episode_length
        self.sim_timestep = sim_timestep
        self.gui_active = gui_active
        self.gui_freq_mod = int(1.0/self.sim_timestep/gui_freq) + 1
        self.save_gui_dir = save_gui_dir
        self.agent_freq_mod = int(1.0/self.sim_timestep/agent_freq) + 1
        self.reset_angle = reset_angle
        self.reset_angle_variance = reset_angle_variance

        self.model = model
        if self.gui_active:
            self.gui = Visualizer(track_length=self.model.track_length)

        self.frame_num = 0
        self.frames = []
        self.step = 0
        self.action = 0
        self.t = 0

    def run_experiment(self):
        self.rand_reset()
        while True:
            request = socket.recv().decode()
            if request[:8] == "/action/":
                action = float(request[8:])
                self.action = action
                while self.step % self.agent_freq_mod != 0:
                    self.model.update(action)
                    self.step += 1
                self.step += 1

                x, angle, dx, dangle, edge = self.model.get_state()
                if edge:
                    terminal = 1
                else:
                    terminal = 0
                if angle**2 < 0.1:
                    reward = 1
                else:
                    reward = -1

                socket.send(("/status/%d/%d" % (reward, terminal)).encode())
            elif request == "/get/state":
                fn = "/home/ala/tmp/dump/%03d.png" % self.frame_num
                self.frame_num += 1
                x, angle, dx, dangle, edge = self.model.get_state()
                self.gui.update(x, angle, dx, dangle, self.action)
                self.gui.save_screen(fn)
                self.frames.append(fn)
                socket.send(fn.encode())
            elif request == "/newGame/":
                self.step = 0
                self.frame_num = 0
                self.rand_reset()
                socket.send(b"/ok/")
                for f in self.frames:
                    os.remove(f)
                self.t += 1
                self.frames = []
            elif request == "/quit/":
                socket.send(b"/ok/")
                return

    def run_episode(self, trial, episode):
        self.rand_reset()
        action = 0
        step = 0
        while step < self.num_timesteps:
            step += 1
            x, angle, dx, dangle, edge = self.model.get_state()

            #agent control signal update rate is separate from simulation rate
            if not (step % self.agent_freq_mod):
                action = self.try_exec("get_action", (x, angle, dx, dangle, edge))

            #update the model state
            self.model.update(action)

            #gui update is separate from simulation rate
            if (self.gui_active):
                time.sleep(self.sim_timestep)
                if not (step % self.gui_freq_mod):
                    if(self.save_gui_dir is not ""):
                        self.save_gui(step=step, episode=episode, trial=trial, state=None)
                    self.gui.draw(x, angle, dx, dangle, action)

    def save_gui(self, step, episode, trial, state):
        if self.save_gui != "":
            dirname = self.save_gui_dir
            dirname += "/t{}_e{}/".format(trial, episode)
            if not os.path.exists(dirname): os.makedirs(dirname)
            image_filename = dirname + "{}".format(step)
            self.gui.save_screen(image_filename)


    def rand_reset(self):
        new_angle = random.gauss(self.reset_angle, self.reset_angle_variance)
        self.model.reset(start_angle=new_angle)

def main(args):
    model = Pendulum(track_length=args.track_length,
                     l=args.pole_length,
                     dt=args.timestep,
                     start_angle=args.reset_angle)

    sim = Simulator(model,
                    num_trials=args.trials,
                    num_episodes=args.episodes,
                    episode_length=args.time,
                    sim_timestep=args.timestep,
                    gui_active=(not args.nogui),
                    gui_freq=args.gui_freq,
                    save_gui_dir=args.save_gui,
                    agent_freq=args.agent_freq,
                    reset_angle=args.reset_angle,
                    reset_angle_variance=args.reset_angle_variance)

    sim.run_experiment()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Pendulum simulator.')
    parser.add_argument('--trials', type=int, default=10,
                        help='number of trials to run (default: %(default)s)')
    parser.add_argument('--episodes', type=int, default=10,
                        help="number of episodes to run "
                        "(default: %(default)s)")
    parser.add_argument('--time', type=int, default=30,
                        help="time length (seconds) of an episode "
                        "(default: %(default)s)")
    parser.add_argument('--timestep', type=float, default=0.001,
                        help="delta timestep (seconds) for simulation "
                        "(default: %(default)s)")
    parser.add_argument('--nogui', action="store_true",
                        help="disables the gui")
    parser.add_argument('--gui_freq', type=int, default=25,
                        help="gui update frequency (default: %(default)s)")
    parser.add_argument('--save_gui', type=str, default="",
                        help="saves the screen output to a directory")
    parser.add_argument('--agent_freq', type=int, default=60,
                        help="agent update frequency (default: %(default)s)")
    parser.add_argument('--track_length', type=int, default=2,
                        help="length (meters) of the track (default: "
                        "%(default)s)")
    parser.add_argument('--pole_length', type=float, default=0.3,
                        help="length (meters) of the pole (default: "
                        "%(default)s)")
    parser.add_argument('--reset_angle', type=float, default=0.0,
                        help="average reset angle of the pendulum (default: "
                        "%(default)s)")
    parser.add_argument('--reset_angle_variance', type=float, default=0.1,
                        help="variance of reset angle of the pendulum "
                        "(default: %(default)s)")
    args = parser.parse_args()
    main(args)
