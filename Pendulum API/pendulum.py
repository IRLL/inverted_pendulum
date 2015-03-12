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

p = None
LINUX = False
WINDOWS = False

class Pendulum():
	
	def __init__(self, motorPort, ucPort):
		self.motor = Motor(motorPort)
		self.uc = ucEncoder(ucPort)
		self.alive = 1
	def __del__(self):
		self.motor.Stop()
	def Reset(self):
		print "resetting pendulum!"
		left_switch = 0
		right_switch = 0
		while(not right_switch):
			self.motor.MoveRight(20)
			switches = self.uc.getSwitches()
			right_switch = switches[0]
			time.sleep(1)
		while(not left_switch):
			self.motor.MoveLeft(20)
			switches = self.uc.getSwitches()
			left_switch = switches[1]
			time.sleep(1)


		time.sleep(5)
		self.uc.send_reset()
		print "done!"
			
def exit_handler(signum, frame):
	global p
	p.motor.Stop()
	print "exiting!"
	sys.exit()

def temp():
	global p
	p = Pendulum('/dev/ttyACM0', '/dev/ttyUSB0')
	
	status_thread = threading.Thread(target=print_status)
	status_thread.daemon = True
	#status_thread.start()	
	p.Reset()
	

def tester():
	global p
	p = Pendulum('/dev/ttyACM0', '/dev/ttyUSB0')
	
	status_thread = threading.Thread(target=print_status)
	status_thread.daemon = True
	#status_thread.start()	
	
	while 1:	
		p.motor.MoveRight(25)
		time.sleep(2)
		p.motor.MoveLeft(25)
		time.sleep(2)
#p.motor.Stop()
#		time.sleep(.5)
	
def print_status():
	global p
	global LINUX
	global WINDOWS
	
	#Check to see what OS is running
	if (platform.system() == "Linux"):
		LINUX = True
	elif (platform.system() == "Windows"):
		WINDOWS = True
		
	#Endlessly update the console
	while True:
		if (WINDOWS):
			os.system('cls')
		elif(LINUX):
			os.system('clear')
		console_update()
		time.sleep(.1)
	
def console_update():
	global p
	stats = p.motor.getVariables()
	angle = p.uc.getAngle()
	position = p.uc.getPosition()
	switches = p.uc.getSwitches()
	print "Status: ", hex(stats[0])
	print "Voltage: ", stats[1]
	print "Temperature: ", stats[2]
	print "Motor Speed: ", stats[3]
	print "Angle: ", angle
	print "Position: ", position
	print "left switch: ", switches[1], "right switch: ", switches[0]
	print ""
	
if __name__ == "__main__":
	signal.signal(signal.SIGINT, exit_handler)
	#tester()
	temp()
