#!/usr/bin/python
'''
This is the class that controls the pendulum
'''
import serial
import threading
import time
import signal
import sys
import platform
import os
from motor import Motor
from uc_interface import ucEncoder
import math

p = None
LINUX = False
WINDOWS = False

class Pendulum():
	
	def __init__(self, motorPort, ucPort):
		self.status = "Booting..."
		self.motor = Motor(motorPort)
		self.uc = ucEncoder(ucPort)
		self.alive = 1
		self.status = "Booted"
	def __del__(self):
		self.motor.Stop()
	def Reset(self, start=0):
		self.status = "resetting pendulum!"
		speed = 19
		left_switch = 0
		right_switch = 0
		while(not right_switch):
			self.motor.MoveRight(speed)
			switches = self.uc.getSwitches()
			right_switch = switches[0]
			time.sleep(.1)
		while(not left_switch):
			self.motor.MoveLeft(speed)
			switches = self.uc.getSwitches()
			left_switch = switches[1]
		
		#wait for arm to settle
		current = 0
		previous = 1
		count = 3 #must be constant for this many seconds
		self.status = "waiting for arm to settle"
		while count > 0: 
			previous = current
			time.sleep(1)
			current = self.uc.getAngle()
			if (previous == current):
				count -= 1
		
		self.uc.send_reset()
		time.sleep(.5)

		self.status = "moving to zero position"
		while True:
			position = self.uc.getPosition()
			if (position > -12 + start):
				break;
			self.motor.MoveRight(speed)
		self.motor.Stop()
		
		self.status = "done resetting!"
		time.sleep(1)
	def getState(self):
		'''
		returns (x cm, degrees, L switch, R switch)
		'''
        mPose = self.uc.getXm() # meters
		angle = self.uc.getRadians()
		return (mPose, radians)
	def moveRight(self, percent, threshold=30):
		if (percent > threshold):
			percent = threshold
		self.motor.MoveRight(percent)
	def moveLeft(self, percent, threshold=30):
		if (percent > threshold):
			percent = threshold
		self.motor.MoveLeft(percent)
    def getActions(self):
        return [0,1] # (0)Left (1)Right
    def step(self, action):
        # TODO: change percent move
        if action == 0:
            self.moveLeft(15)
        elif action == 1:
            self.moveRight(15)
    def isTerminated(self):
        switches = self.uc.getSwitches()
        if switches[0] == 1 || switches[1] == 1:
            return True
        return False
    def getReward(self):
        # TODO: change angle range
        angle = self.uc.getAngle()
        reward = -1
        if angle >= 168 and angle <= 192:
            reward = 0
        return reward
	def stop(self):
		self.motor.Stop()
	def print_status():
		global LINUX
		global WINDOWS
		
		#Check to see what OS is running
		if (platform.system() == "Linux"):
			LINUX = True
		elif (platform.system() == "Windows"):
			WINDOWS = True
		
		print "Inverted Pendulum API by Brandon Kallaher and James Irwin"
		print "Running on ", platform.system()
		print "Python Version: ", platform.python_version()
		print "Clear the Path of the arm or suffer the consequences!"
		time.sleep(3)
		
		#Endlessly update the console
		while True:
			if (WINDOWS):
				os.system('cls')
			elif(LINUX):
				os.system('clear')
			self.console_update()
			time.sleep(.1)

	#Does all the printing for the print_status() thread		
	def console_update():
		stats = self.motor.getVariables()
		switches = self.uc.getSwitches()
		print "Pendulum Status: ", 			self.status
		print "uC Process Status: ", 		self.uc.status
		print "Motor Process Status: ", 	self.motor.status
		print "Motor Controller Status: ", 	hex(stats[0])
		print "Voltage: ", 					stats[1]
		print "Temperature: ", 				stats[2]
		print "Motor Speed: ", 				stats[3]
		print "Angle: %f%c"                 %(self.uc.getAngle(), u'\xb0')
        print "       %f rad", 			    %(self.uc.getRadians())
		print "Position: %f meters", 		%(self.uc.getXm())
		print "Left Switch: ", 				switches[1]
		print "Right Switch: ", 			switches[0]
		print ""



def exit_handler(signum, frame):
	global p
	p.motor.Stop()
	p.status = "Exiting, Goodbye!"
	sys.exit()

#Tests the Pendulum Reset function
def temp():
	global p
	p = Pendulum('/dev/ttyACM0', '/dev/ttyUSB0')
	p.status = "Running..."
	status_thread = threading.Thread(target=print_status)
	status_thread.daemon = True
	status_thread.start()
	time.sleep(5)
	p.Reset(-20)
	
#Tests Motor movement
def tester():
	global p
	p = Pendulum('/dev/ttyACM0', '/dev/ttyUSB0')
	
	status_thread = threading.Thread(target=print_status)
	status_thread.daemon = True
	status_thread.start()	
	
	while 1:	
		p.motor.MoveRight(25)
		time.sleep(2)
		p.motor.MoveLeft(25)
		time.sleep(2)
#p.motor.Stop()
#		time.sleep(.5)

#Continuously print the status of the Pendulum	
def print_status():
	global p
	global LINUX
	global WINDOWS
	
	#Check to see what OS is running
	if (platform.system() == "Linux"):
		LINUX = True
	elif (platform.system() == "Windows"):
		WINDOWS = True
	
	print "Inverted Pendulum API by Brandon Kallaher and James Irwin"
	print "Running on ", platform.system()
	print "Python Version: ", platform.python_version()
	print "Clear the Path of the arm or suffer the consequences!"
	time.sleep(3)
	
	#Endlessly update the console
	while True:
		if (WINDOWS):
			os.system('cls')
		elif(LINUX):
			os.system('clear')
		console_update()
		time.sleep(.1)

#Does all the printing for the print_status() thread		
def console_update():
	global p
	stats = p.motor.getVariables()
	switches = p.uc.getSwitches()
	print "Pendulum Status: ", 			p.status
	print "uC Process Status: ", 		p.uc.status
	print "Motor Process Status: ", 	p.motor.status
	print "Motor Controller Status: ", 	hex(stats[0])
	print "Voltage: ", 					stats[1]
	print "Temperature: ", 				stats[2]
	print "Motor Speed: ", 				stats[3]
	print "Angle: ", 					p.uc.getAngle()
	print "Position: ", 				p.uc.getPosition()
	print "Left Switch: ", 				switches[1]
	print "Right Switch: ", 			switches[0]
	print ""
	
if __name__ == "__main__":
	signal.signal(signal.SIGINT, exit_handler)
	#tester()
	temp()
