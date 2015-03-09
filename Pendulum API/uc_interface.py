'''
This is the class that reads the encoder data from the microcontroller
'''
import serial
import threading
import time
import signal
import sys
import argparse
from encoder import Encoder

class ucEncoder():
	
	def __init__(self, Port, baudrate=2000000):
		##Serial port setup	
		self.ser = serial.Serial()
		self.ser.baudrate = baudrate
		if (type(Port) is int):
			self.ser.port = Port - 1 #windows numerical port
		elif(type(Port) is str):
			self.ser.port = Port #linux string port
		print "opening port", Port
		self.ser.open()

		self.uc_data = 0;
		self.mEncoder = Encoder(90, 10)
		self.aEncoder = Encoder(500, 1)
		
		#Threading setup	
		self.alive = 1 #tell the thread to stay alive
		self.thread = threading.Thread(target=self.uc_process)
		self.thread.daemon = True #make daemon thread so it exits when program ends
		self.data_lock = threading.Lock() #mutex for variables
		self.ser_lock = threading.Lock() #mutex for serial port
		self.thread.start()
		
		
	def __del__(self):
		self.ser.close()

	def getVariables(self):
		variables = []
		self.data_lock.acquire()
		try:
			#return a tuple containing all the values
			variables.append(self.uc_data)
			#get the encoder values
			mE = self.mEncoder.getVariables()
			aE = self.aEncoder.getVariables()
		finally:
			self.data_lock.release()
		#1 is the angle of the encoder
		variables.append (aE[1])
		#2 is the counts seen by the arm
		variables.append (aE[2])
		#0 is the linear position of the Cart
		variables.append (mE[0])
		#2 is the number of counts seen by motor
		variables.append (mE[2])
		
		return variables
	
	def encoder_process(self, byte):
		#Encoder processing
		#pass new encoder channel values to the appropriate encoder
		self.mEncoder.setNextState((byte & 0b1100) >> 2)
		self.aEncoder.setNextState(byte >> 4)
		
	def uc_process(self):
		print "microcontroller process started"
		while 1:
			#read serial byte (this call is blocking)
			#print "reading data..."
			data = self.ser.read(1)
			#print "data read, pushing to variables..."
			#aquire mutex lock
			self.data_lock.acquire()
			try: #update the motor variable
				self.uc_data = ord(data)
			finally: #release the lock
				self.data_lock.release()
			#start encoder process
			self.encoder_process(ord(data))
		print "microcontroller process exiting"

def exit_handler(signum, frame):
	print "exiting!"
	sys.exit()
	
	
def tester():
	uc = ucEncoder('/dev/ttyUSB0')
	while 1:
		data = uc.getVariables()
		print hex(data[0]), data[0] & 0b1, ((data[0] & 0b10) >> 1), data[3]
		time.sleep(.2)
	
	
if __name__ == "__main__":
	signal.signal(signal.SIGINT, exit_handler)
	tester()

	
