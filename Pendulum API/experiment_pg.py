# -*- coding: utf-8 -*-
"""
Created on Mon Jun 15 14:15:56 2015

@author: gabe
"""

from pendulum import *

import time
import numpy as np
import random
import matplotlib.pyplot as plt
from matplotlib import cm
from math import pi
import sys



np.set_printoptions(7, threshold=np.nan)

class Data():
    def __init__(self, n, traj_length):
        self.x = np.empty(shape=(n,traj_length+1))
        self.u = np.empty(shape=(1,traj_length))
        self.r = np.empty(shape=(1,traj_length))

class PolicyGradient():

	ACTION_DELAY = 0.001 # 0.001

	def __init__(self, world, threshold):

		self._world = world
		self._threshold = threshold

		# Parameter definition
		self._n = 4 # Number of states
		self._m = 1 # Number of inputs

		self._rate = .3 # Learning rate for gradient descent Original+0.75

		self._traj_length = 1000 # Number of time steps to simulate in the cart-pole system
		self._rollouts = 10 # Number of trajectories for training
		self._num_iterations = 1000 # Number of learning episodes/iterations

		self._rollouts_test = 2
		self._traj_length_test = 1000

		time.sleep(.1)
		#self._my0 = pi/6-2*pi/6*np.random.random((N,1)) # Initial state of the cart pole (between -60 and 50 deg)
		#self._my0 = np.array([[self._state]])
		self._s0 = 0.0001*np.eye(self._n)

		# Parameters for Gaussian policies
		self._theta = np.random.rand(self._n*self._m,1) # Remember that the mean of the normal dis. is theta'*xt
		for i in range(self._n*self._m):
			self._theta[:,0][i] = self._theta[:,0][i] if np.random.randint(2, size=1) == 1 else -self._theta[:,0][i]
		self._theta = np.array(
				[[ -0.3415466],
 				 [-10.8511862],
 				 [  2.4911943],
 				 [  0.5160055]]
			      )

		self._sigma = np.random.rand(1,self._m) # Variance of the Gaussian dist.
		#self._sigma = np.array([[ 0.764011]])

		self._data = [Data(self._n, self._traj_length) for i in range(self._rollouts)]

		self._mat = np.empty(shape=(self._rollouts,self._n+1))
		self._vec = np.empty(shape=(self._rollouts,1))
		self._r = np.empty(shape=(1,self._rollouts))
		self._reward_per_rates_per_iteration = np.empty(shape=(1,self._num_iterations))

		print "Initial Theta: ", self._theta
		print "Sigma: ", self._sigma
		print "Learning Rate: ", self._rate

	def startEpisode(self):
		#z = random.uniform(1.8, 4.2)
		z = 0 # TODO: random start
		self.reset(z)

	def swingup(self, x_pose=0.0):
		for i in range(200):
			self._world.moveRight(self._threshold, self._threshold)
			time.sleep(self.ACTION_DELAY)
		self._world.stop()
		for i in range(300):
			self._world.moveLeft(self._threshold, self._threshold)
			time.sleep(self.ACTION_DELAY)
		self._world.stop()
		for i in range(180):
			self._world.moveRight(self._threshold, self._threshold)
			time.sleep(self.ACTION_DELAY)
		self._world.stop()
		#for i in range(325):
		#	self._world.moveLeft(self._threshold, self._threshold)
		#	time.sleep(self.ACTION_DELAY)
		#self._world.stop()
		#for i in range(300):
		#	self._world.moveRight(self._threshold, self._threshold)
		#	time.sleep(self.ACTION_DELAY)
		#self._world.stop()

	def test(self, iteration_num, theta, rollouts, traj_length):
		data = [Data(self._n, traj_length) for i in range(rollouts)]
		r = np.empty(shape=(1,rollouts))
		# In this section we obtain our data by simulating the cart-pole system
		for trials in range(rollouts):
			isReset = False
			# reset
			print "____________@ Test Trial: ", (trials+1)
			self.startEpisode()
			self.swingup(x_pose=0.0)
			# Draw the initial state
			mPose, mSpeed, angle, radSpeed = self._world.getState()
			state = np.array([[
							mPose/1.25,
							mSpeed/100,
							angle/(180),
							radSpeed/(6*pi)
						    ]])
			last_action = 0.0
			print "Initial Angle: ", angle
			try:
				# Perform a trial of length L
				current_time = time.time()
				for steps in range(traj_length):
					#print "__________________@ Step #", steps+1
					# Draw an action from the policy
					action = np.dot(self._theta.conj().T, state.conj().T).conj().T[0]

					action = round(action, 0)
					# saturate action
					if action > 1.0:
						action = 1.0
					elif action < -1.0:
						action = -1.0
					# Execute action
					#print "Action: ", action
					if self._world.softStopFlag or isReset:
						if isReset:
							action = 0.0
						elif last_action > 0:
							action = -1.0
							#print "Action: Left"
						elif last_action < 0:
							action = 1.0
							#print "Action: Right"
						else:
							action = 0.0
							#print "Action: Stop"
					else:
						speed = 0
						if action == 0:
							self._world.stop()
							#print "Action: Stop"
						else:
							temp_action = abs(action)
							if temp_action >= .75:
								speed = self._threshold
							elif temp_action >= .5:
								speed = self._threshold - 10
							elif temp_action >= .25:
								speed = self._threshold - 20
							else:
								speed = self._threshold - 30
						if action > 0: #positive
							self._world.moveRight(speed, self._threshold)
							#print "Action: Right"
						elif action < 0:
							self._world.moveLeft(speed, self._threshold)
							#print "Action: Left"
						time.sleep(self.ACTION_DELAY)
						last_action = action

    					last_time = current_time
    					current_time = time.time()
    					print "time/freq for main testing loop: {}/{}".format(current_time - last_time, 1.0/(current_time-last_time))


					# Calculating the reward (Remember: First term is the reward for accuracy, second is for control cost)
					reward = self._world.getReward()
					if isReset or self._world.softStopFlag:
						reward = -2.0

					data[trials].r[:,steps] = [reward]

					# Draw next state from envrionment
					mPose, mSpeed, angle, radSpeed = self._world.getState()

					if self._world.softStopFlag or isReset: # went outside
						mSpeed = 0.0
						angle = 180.0
						radSpeed = 0.0
						if isReset:
							mPose = 0.0

					state = np.array([[
						    mPose/1.25,
						    mSpeed/100,
						    angle/(180),
						    radSpeed/(6*pi)
						]])

					if not self._world.softStopFlag and not isReset and (angle > 50.0 or angle < -50.0):
						print "Reset true ", angle, " Step ", steps
						isReset = True

				# end for steps...
			except KeyboardInterrupt:
				print "Program interrupted!"
				sys.exit(1)
		# end for trials

		# Calculation of the average reward
		for z in range(np.shape(data)[0]): #TODO:check
			r[0,z] = np.sum(data[z].r) #TODO:check
		# end for z...



		#############
		# Plotting the average reward to verify that the parameters of the
		# regulating controller are being learned
                file = open('theta_outputs_test.txt', 'a')
                print >>file, "Iteration: ", iteration_num
                print >>file, "Theta: ", self._theta
                mean = np.mean(r)
                print >>file, "Mean: ", mean
                file.close()

		print "Mean: ", mean
		return mean

	# end for k...

	def train(self):
		#self._my0 = np.array([[self._world.getState()]])
		#plt.ion()
		#plt.yscale("log")
		#plt.show()

		for k in range(self._num_iterations): # Loop for learning

			print "Test Theta: ", k
			m = self.test(k, self._theta, self._rollouts_test, self._traj_length_test)
			#plt.scatter(k, m, marker=u'x', c=np.random.random((2,3)), cmap=cm.jet)
			#plt.draw()

			print "______@ Iteration: ", k+1

			# In this section we obtain our data by simulating the cart-pole system
			for trials in range(self._rollouts):
				isReset = False
				# reset
				print "____________@ Trial: ", (trials+1)
				self.startEpisode()
				self.swingup(x_pose=0.0)
				# Draw the initial state
				#init_state = np.random.multivariate_normal(self._my0[:,0], self._s0, 1)
				#self._data[trials].x[:,0] = init_state[0,:]
				mPose, mSpeed, angle, radSpeed = self._world.getState()
				self._data[trials].x[:,0] = np.array([[
								mPose/1.25,
								mSpeed/100,
								angle/(180),
								radSpeed/(6*pi)
							    ]])
				last_action = 0.0
				print "Initial Angle: ", angle
				try:
					current_time = time.time()
					# Perform a trial of length L
					for steps in range(self._traj_length):
						#print "__________________@ Step #", steps+1
						# Draw an action from the policy
						action = np.random.multivariate_normal(
								np.dot(self._theta.conj().T,
								self._data[trials].x[:,steps]).conj().T,
								self._sigma
							 )

						last_time = current_time
						current_time = time.time()
						print "time/freq for main training loop: {}/{}".format(current_time - last_time, 1.0/(current_time-last_time))

						action = round(action, 0)
						# saturate action
						if action > 1.0:
							action = 1.0
						elif action < -1.0:
							action = -1.0
						# Execute action
						#print "Action: ", action
						if self._world.softStopFlag or isReset:
							if isReset:
								action = 0.0
							elif last_action > 0:
								action = -1.0
								#print "Action: Left"
							elif last_action < 0:
								action = 1.0
								#print "Action: Right"
							else:
								action = 0.0
								#print "Action: Stop"
						else:
							speed = 0
							if action == 0:
								self._world.stop()
								#print "Action: Stop"
							else:
								temp_action = abs(action)
								if temp_action >= .75:
									speed = self._threshold
								elif temp_action >= .5:
									speed = self._threshold - 10
								elif temp_action >= .25:
									speed = self._threshold - 20
								else:
									speed = self._threshold - 30
							if action > 0: #positive
								self._world.moveRight(speed, self._threshold)
								#print "Action: ", int(action*self._threshold)
								#self._world.moveRight(self._threshold, self._threshold)
								#print "Action: Right"
								#self._world.stop()
							elif action < 0:
								self._world.moveLeft(speed, self._threshold)
								#print "Action: ", int(-action*self._threshold)
								#self._world.moveLeft(self._threshold, self._threshold)
								#print "Action: Left"
								#self._world.stop()
							time.sleep(self.ACTION_DELAY) # 0.28
							last_action = action

						self._data[trials].u[:,steps] = action


						# Calculating the reward (Remember: First term is the reward for accuracy, second is for control cost)
						if isReset or self._world.softStopFlag:
							reward = -2.0
						else:
							reward = self._world.getReward()
						self._data[trials].r[:,steps] = [reward]

						# Draw next state from envrionment
						mPose, mSpeed, angle, radSpeed = self._world.getState()

						if self._world.softStopFlag or isReset: # went outside
							mSpeed = 0.0
							angle = 180.0
							radSpeed = 0.0
							if isReset:
								mPose = 0.0

						state = np.array([[
							    mPose/1.25,
							    mSpeed/100,
							    angle/(180),
							    radSpeed/(6*pi)
							]])
						self._data[trials].x[:,steps+1] = state

						if not self._world.softStopFlag and not isReset and (angle > 50.0 or angle < -50.0):
							print "Reset true ", angle, " Step ", steps
							isReset = True

						'''
						if not self._world.softStopFlag:
							print "State: ", state
							print "Reward: %f" %(reward)
						'''
					# end for steps...
				except KeyboardInterrupt:
					print "Program interrupted!"
					sys.exit(1)
			# end for trials

			self._gamma = 0.9


			# This section calculates the derivative of the lower bound of the expected average return
			# using Natural Actor Critic
			j = 0

			if self._gamma == 1:
				for Trials in range(np.max(np.shape(self._data))):
				    j = j + np.sum(self._data[Trials].r)/np.max(np.shape(self._data[Trials].r))
				#end for Trials...
				j = j / np.max(np.shape(self._data))
			# end if gamma...

			# OBTAIN GRADIENTS
			i = 0
			for Trials in range(np.max(np.shape(self._data))):
				self._mat[i,:] = np.append(np.zeros((1,self._n)),np.array([[1]]),axis=1)
				self._vec[i,0] = 0
				for Steps in range(np.max(np.shape(self._data[Trials].u))):
				    xx = self._data[Trials].x[:,Steps]
				    u  = self._data[Trials].u[:,Steps]

				    DlogPiDThetaNAC = np.dot((u-np.dot(self._theta.conj().T,xx)),np.array([xx]))/(self._sigma**2) # TODO:check
				    decayGamma = self._gamma**Steps
				    self._mat[i,:] = self._mat[i,:] + decayGamma*np.append(DlogPiDThetaNAC, np.array([[0]]), axis=1) # TODO:check
				    self._vec[i,0] = self._vec[i,0] + decayGamma*(self._data[Trials].r[:,Steps][0]-j) # TODO:check
				# end for Steps
				i = i+1
			#end for Trials

			# cond(Mat)
			nrm = np.diag(np.append(np.std(self._mat[:,0:self._n*self._m],ddof=1,axis=0),[1],axis=0))
			w = np.dot(nrm,np.dot(np.linalg.inv(np.dot(nrm,np.dot(self._mat.conj().T,np.dot(self._mat,nrm)))),np.dot(nrm,np.dot(self._mat.conj().T,self._vec)))) #TODO:check
			dJdtheta = w[0:(np.max(np.shape(w))-1)]; #TODO:check
			# the expected average return

			# Update of the parameter vector theta using gradient descent
			self._theta = self._theta + self._rate*dJdtheta;
			print "Theta: ", self._theta

			# Calculation of the average reward
			for z in range(np.shape(self._data)[0]): #TODO:check
				self._r[0,z] = np.sum(self._data[z].r) #TODO:check
			# end for z...

			if np.isnan(self._r.any()): #TODO:check
				print "System has NAN:", i
				print "..@ learning rate: ", self._rate #FIXME: DOUBLECHECK WITH MATLAB
				print "breaking this iteration ..."
				break
			# end if isnan...

			self._reward_per_rates_per_iteration[0,k] = np.mean(self._r) #TODO:check


			#############
			# Plotting the average reward to verify that the parameters of the
			# regulating controller are being learned
                        file = open('theta_outputs.txt', 'a')
                        print >>file, "Iteration: ", k+1
                        print >>file, "Theta: ", self._theta
                        mean = np.mean(self._r)
                        print >>file, "Mean: ", mean
                        file.close()

			print "Mean: ", mean
			time.sleep(0.05)
		# end for k...
		print "Final Test Theta: ", self._num_iterations
		m = self.test(self._num_iterations, self._theta, self._rollouts_test, self._traj_length_test)
		#plt.scatter(self._num_iterations, m, marker=u'x', c=np.random.random((2,3)), cmap=cm.jet)
		#plt.draw()
		#plt.show(block=True)

	def reset(self, start=0):
		self._world.Reset(start)


if __name__ == "__main__":
	world = Pendulum(
	          motorPort = '/dev/ttyACM0',
	          ucPort = '/dev/ttyUSB0',
	          config = "config"
	        )
	threshold = 60 # percent speed of pendulum #45 with delay of .1
	agent = PolicyGradient(world, threshold)
	agent.train()
