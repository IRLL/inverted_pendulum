#!/usr/bin/python
'''
This is the class that controls the pendulum
'''
import serial
import threading
import time
import signal
import sys
from motor import Motor


p = None

class Pendulum():
	
	def __init__(self, Port):
		self.motor = Motor(Port)
		self.alive = 1
		self.position = 0
	def __del__(self):
		self.motor.Stop()
	
	def Reset(self):
		while(position != 0):
			MoveLeft(self, getValueFromPercent(10))


def exit_handler(signum, frame):
	global p
	p.motor.Stop()
	print "exiting!"
	sys.exit()

def tester():
	global p
	p = Pendulum('/dev/ttyACM0')
	
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
	while 1:
		stats = p.motor.getVariables()
		print "status", hex(stats[0])
		print "voltage", stats[1]
		print "temperature", stats[2]
		print ""
		time.sleep(1)


if __name__ == "__main__":
	signal.signal(signal.SIGINT, exit_handler)
	tester()
