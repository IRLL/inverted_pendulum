#!/usr/bin/python
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
from model import Pendulum
from visualizer import Visualizer


class Simulator:
    def __init__(self, agent, model, num_trials, num_episodes, episode_length,
                 sim_timestep, gui_active, gui_freq, save_gui_dir, agent_freq, 
                 reset_angle, reset_angle_variance):
        self.agent = agent

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

    def run_experiment(self):
        for i in range(self.num_trials):
            print("starting trial {} of {}".format(i+1, self.num_trials))
            self.try_exec("init_trial", [i] + list(self.model.get_state()))
            for j in range(self.num_episodes):
                print("starting episode {} of {}".format(j+1,
                self.try_exec("init_episode", [j] + list(self.model.get_state()))
                self.run_episode(i,j)  # run the episode
                self.try_exec("end_episode", [j] + list(self.model.get_state()))
            self.try_exec("end_trial", [i] + list(self.model.get_state()))

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
                    self.gui.draw(x, angle)

    def save_gui(self, step, episode, trial, state):
        if self.save_gui != "":
            dirname = self.save_gui_dir
            dirname += "/t{}_e{}/".format(trial, episode)
            if not os.path.exists(dirname): os.makedirs(dirname)
            image_filename = dirname + "{}".format(step)
            self.gui.save_screen(image_filename)

    def try_exec(self, function_name, params):
        function = getattr(self.agent, function_name, None)
        if callable(function):
            result = function(*params)
        else:
            result = None

        return result

    def rand_reset(self):
        new_angle = random.gauss(self.reset_angle, self.reset_angle_variance)
        self.model.reset(start_angle=new_angle)


def function_exists(classname, function_name):
    function = getattr(classname, function_name, None)
    if callable(function):
        return True 
    else:
        return False 

def verify_agent(agent):
    critical_functions = ["get_action"]
    other_functions = ["init_trial", "init_episode", "end_episode",
                       "end_trial"]

    all_good = True
    for function in critical_functions:
        if(not function_exists(agent, function)):
            print "ERROR: agent does not have critical function:", function
            all_good = False

    for function in other_functions:
        if(not function_exists(agent, function)):
            print "WARNING: agent does not have function:", function

    if not all_good:
        print "one or more critical agent functions are missing, exiting!"
        sys.exit(1)


def main(args):
    # Dynamically load the agent.
    agent_filename = args.agent_file[:-3]
    agent_module = importlib.import_module(agent_filename)

    agent = agent_module.Agent()
    verify_agent(agent)

    model = Pendulum(track_length=args.track_length,
                     l=args.pole_length,
                     dt=args.timestep,
                     start_angle=args.reset_angle)

    sim = Simulator(agent, model,
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
    parser.add_argument('agent_file', type=str,
                        help="file name of agent to run")
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
    parser.add_argument('--agent_freq', type=int, default=10,
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
    parser.add_argument('--reset_angle_variance', type=float, default=0.0,
                        help="variance of reset angle of the pendulum "
                        "(default: %(default)s)")
    args = parser.parse_args()
    main(args)
