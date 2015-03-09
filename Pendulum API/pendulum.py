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
		self.uc = ucEncoder(ucPort, 2000000)
		self.alive = 1
		self.position = 0
	def __del__(self):
		Reset()
		self.motor.Stop()
	def Reset(self):
		while(position != 0):
			self.motor.MoveLeft(self, getValueFromPercent(10))
			
def exit_handler(signum, frame):
	global p
	p.Reset()
	p.motor.Stop()
	print "exiting!"
	sys.exit()

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
	
def console_update():
	global p
	stats = p.motor.getVariables()
	stats1 = p.uc.getVariables()
	print "Status: ", hex(stats[0])
	print "Voltage: ", stats[1]
	print "Temperature: ", stats[2]
	print "Motor Speed: ", stats[3]
	print "Angle: ", stats1[1]
	print "Arm Ticks: ", stats[2]
	print "Position: ", stats1[3]
	print "Motor Ticks: ", stats1[4]
	print ""
	
if __name__ == "__main__":
	signal.signal(signal.SIGINT, exit_handler)
	tester()
