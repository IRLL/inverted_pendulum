#!/usr/bin/python
import sys
import time
from model import Pendulum
from visualizer import Visualizer

class Simulator:
	def __init__(self, agent, num_trials=10, num_episodes=1, episode_length=5, sim_timestep=0.001,
			gui_active=True, gui_freq=20, agent_freq=10):
		self.agent = agent

		self.num_trials = num_trials
		self.num_episodes = num_episodes
		self.num_timesteps = int(episode_length/sim_timestep)
		self.episode_length = episode_length
		self.sim_timestep = sim_timestep
		self.gui_active = gui_active
		self.gui_freq = gui_freq
		self.agent_freq = agent_freq

		self.model = Pendulum(self.sim_timestep)
		self.gui = Visualizer(self.model.track_length)

	def run_experiment(self):
		for i in range(self.num_trials):	
			self.try_exec("init_trial", i)
			for j in range(self.num_episodes):
				self.try_exec("init_episode", j)
				self.run_episode() #run the episode
				self.try_exec("end_episode", j)
			self.try_exec("end_trial", i)

	def run_episode(self):
		self.model.reset(start_angle = 0)
		for step in range(self.num_timesteps):
			x, angle, dx, dangle = self.model.get_state()

			#agent control rate is separate from simulation rate
			if not (step*self.sim_timestep*self.agent_freq % 1):
				action = self.try_exec("get_action", (x, angle, dx, dangle))
			else:
				action = 0

			#update the model state
			self.model.update(action)

			#gui update is separate from simulation rate
			if (self.gui_active):
				time.sleep(self.sim_timestep)
				if not (step*self.sim_timestep*self.gui_freq % 1):
					self.gui.draw(x, angle)
					
	def try_exec(self, function_name, params):
		function = getattr(self.agent, function_name, None)
		if callable(function):
			result = function(*params)
		else:
			result = None

		return result

def function_exists(classname, function_name):
	function = getattr(classname, function_name, None)
	if callable(function):
		return True
	return False	

def verify_agent(agent):
	critical_functions = ["get_action"]
	other_functions = ["init_trial", "init_episode", "end_episode", "end_trial"]

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
	#dynamically load the agent
	agent_filename, _ = args.agent_file.split('.')
	agent_module = __import__(agent_filename)

	agent = agent_module.Agent()
	verify_agent(agent)

	sim = Simulator(agent)

	sim.run_experiment()
	

if __name__ == "__main__":
	import argparse

	parser = argparse.ArgumentParser(description='pendulum simulator')
	parser.add_argument('agent_file', type=str, help="file name of agent to run")
	args = parser.parse_args()
	main(args)
