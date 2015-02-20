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
	
	def encoder_process(self, byte):
		#Encoder processing
		'''
			Theory (in c State is (CHA, CHB) (bit1 = CHA, bit0 = CHB)):
				switch (currentState)
				{
					case 0: //current state = 00
						//if LastState=01, increment Count (CW)
						//else LastState=10, decrement Count (CCW)
						LastState==1 ? Count++ : Count--;
						break;
					case 1: //current state = 01
						//if LastState=11, increment Count (CW)
						//else LastState=00, decrement Count (CCW)
						LastState==3 ? Count++ : Count--;
						break;
					case 2: //current state = 10
						//if LastState=00, increment Count (CW)
						//else LastState=11, decrement Count (CCW)
						LastState==0 ? Count++ : Count--;
						break;
					case 3: //current state = 11
						//if LastState=10, increment Count (CW)
						//else LastState=01, decrement Count (CCW)
						LastState==2 ? Count++ : Count--;
						break;
				}
				LastState = currentState;
		'''
		
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
			encoder_process(self.getVariables())
		print "microcontroller process exiting"


def exit_handler(signum, frame):
	print "exiting!"
	sys.exit()
	
	
def tester():
	uc = ucEncoder('/dev/ttyUSB1')
	while 1:
		data = uc.getVariables()
		print hex(data[0]), data[0] & 0b1, ((data[0] & 0b10) >> 1)
		time.sleep(.2)
	
	
if __name__ == "__main__":
	signal.signal(signal.SIGINT, exit_handler)
	tester()

	
