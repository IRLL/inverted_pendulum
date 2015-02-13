'''
This is the class that reads the encoder data from the microcontroller
'''
import serial
import threading
import time
import signal
import sys
import argparse

class ucEncoder():
	
	def __init__(self, Port, baudrate=115200):
		
		##Serial port setup	
		self.ser = serial.Serial()
		self.ser.baudrate = baudrate
		if (type(Port) is int):
			self.ser.port = Port - 1 #windows numerical port
		elif(type(Port) is str):
			self.ser.port = Port #linux string port
		print "opening port", Port
		self.ser.open()
		
		#Threading setup	
		self.alive = 1 #tell the thread to stay alive
		self.thread = threading.Thread(target=self.uc_process)
		self.thread.daemon = True #make daemon thread so it exits when program ends
		self.data_lock = threading.Lock() #mutex for variables
		self.ser_lock = threading.Lock() #mutex for serial port
		self.thread.start()
		
		self.uc_data = 0;
		
	def __del__(self):
		self.ser.close()
	def getVariables(self):
		variables = []
		self.data_lock.acquire()
		try:
			#return a tuple containing all the values
			variables.append(self.uc_data)
		finally:
			self.data_lock.release()
		return variables	

	def uc_process(self):
		print "microcontroller process started"
		while 1:
			#read serial byte (this call is blocking)
			print "reading data"
			data = self.ser.read(1)
			print "data read"
			#aquire mutex lock
			self.data_lock.acquire()
			try: #update the motor variable
				self.uc_data = data
			finally: #release the lock
				self.data_lock.release()
		print "microcontroller process exiting"
		
		
		
def exit_handler(signum, frame):
	print "exiting!"
	sys.exit()
	
	
def tester():
	while 1:
		uc = ucEncoder(28)
		data = uc.getVariables()
		
		print data[0]
	
	
if __name__ == "__main__":
	signal.signal(signal.SIGINT, exit_handler)
	tester()

	
