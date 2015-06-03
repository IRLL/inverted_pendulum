#!/usr/bin/python
'''
This is the class that controls the pendulum
'''
import threading
import time
import signal
import sys
import platform
import os
from motor import Motor
from uc_interface import ucEncoder
import setup
import pickle

p = None
LINUX = False
WINDOWS = False

class Setup():
	pass

class Pendulum():
	def __init__(self, motorPort, ucPort):
		self.status = "Booting..."
		self.motor = Motor(motorPort)
		self.uc = ucEncoder(ucPort)
		self.resetFlag = False
		self.softStopFlag = False
		
		#Load the Setup configuration
		file = open("config", 'r')
		up = pickle.Unpickler(file)
		self.su = up.load()
		#self.su = self.su.unpack("config")
		
		self.watchdogThread = threading.Thread(target=self.watchdog)
		self.watchdogThread.daemon = True
		self.watchdogThread.start()
				
		self.status = "Booted"
	def __del__(self):
		self.motor.Stop()
	def Reset(self, start=0):
		self.status = "resetting pendulum!"
		self.resetFlag = True
		speed = 20
		left_switch = 0
		right_switch = 0
		while(not left_switch):
			self.motor.MoveLeft(speed)
			switches = self.uc.getSwitches()
			left_switch = switches[1]
			time.sleep(.1)
			
		while(not right_switch):
			self.motor.MoveRight(speed+5)
			switches = self.uc.getSwitches()
			right_switch = switches[0]
			time.sleep(.1)
		
		self.motor.Stop()
		
		#wait for arm to settle
		current = 0
		previous = 1
		wait = 3
		count = wait #must be constant for this many seconds
		self.status = "waiting for arm to settle"
		while count > 0: 
			previous = current
			time.sleep(1)
			current = self.uc.getAngle()
			if (previous == current):
				count -= 1
			else:
				count = wait
		
		self.uc.send_reset()
		time.sleep(2)

		self.status = "moving to center position"
		while True:
			position = self.uc.motor_count
			if (position <= self.su.encoderCenter):
				self.motor.Stop()
				break;
			self.motor.MoveLeft(speed)
		self.motor.Stop()
		time.sleep(1)		
		self.resetFlag = False
		
		self.status = "done resetting!"
		time.sleep(2)
        
	def getState(self):
		'''
		returns (meters, radians)
		'''
		mPose = self.uc.getXm() # meters
		radians = self.uc.getRadians()
		return (mPose, radians)
	def moveRight(self, percent, threshold=30):
		if (percent > threshold):
			percent = threshold
		if not self.softStopFlag:
			self.motor.MoveRight(percent)
	def moveLeft(self, percent, threshold=30):
		if (percent > threshold):
			percent = threshold
		if not self.softStopFlag:
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
		if switches[0] == 1 or switches[1] == 1:
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
	
	#TODO: Add in the Camera positioning
	def watchdog(self):
		while(True):
			#TODO Check
			pos = self.uc.getMotorCount()
			if (not self.resetFlag):
				if (pos >= self.su.rightBrakeEncoderPos) or (pos <= self.su.leftBrakeEncoderPos):
					self.softStopFlag = True
					self.uc.status = "Watchdog tripped!"
					self.stop()
					time.sleep(2)
					self.Reset()
					self.softStopFlag = False
			time.sleep(.05)
		
	def print_status(self):
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
	def console_update(self):
         stats = self.motor.getVariables()
         switches = self.uc.getSwitches()
         print "Pendulum Status: ",				self.status
         print "uC Process Status: ",			self.uc.status
         print "Motor Process Status: ",		self.motor.status
         print "Motor Controller Status: ",		hex(stats[0])
         print "Voltage: ",						stats[1]
         print "Temperature: ",					stats[2]
         print "Motor Speed: ",					stats[3]
         print "Angle: %f%c"					%(self.uc.getAngle(), u'\xb0')
         print "       %f rad"					%(self.uc.getRadians())
         print "Position: %f meters"			%(self.uc.getXm())
         print "Left Switch: ",					switches[1]
         print "Right Switch: ",				switches[0]
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
#	p.Reset()
	
	while 1:	
		p.moveRight(30)
		time.sleep(1)
		p.moveLeft(30)
		time.sleep(1)
#		p.motor.Stop()
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
	#p.Reset()	
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
	print "right Brake: ", 				p.su.rightBrakeEncoderPos
	
if __name__ == "__main__":
	signal.signal(signal.SIGINT, exit_handler)
	tester()
	#temp()
