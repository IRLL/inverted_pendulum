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
		self.ser.open()
		
		#Threading setup	
		self.alive = 1 #tell the thread to stay alive
		self.thread = threading.Thread(target=self.uc_process)
		self.thread.daemon = True #make daemon thread so it exits when program ends
		self.data_lock = threading.Lock() #mutex for variables
		self.ser_lock = threading.Lock() #mutex for serial port
		self.thread.start()
		

	def __del__(self):
		self.Stop()
		self.ser.close()
	def getVariables(self):
		variables = []
		self.data_lock.acquire()
		try:
			#return a tuple containing all the values
			pass
		finally:
			self.data_lock.release()
		return variables	

	def uc_process(self):
		print "microcontroller process started"
		while 1:
			#aquire mutex locks
			self.ser_lock.acquire()
			self.data_lock.acquire()
			try: #try to read the motor variables
				#get error codes
				self.error_codes = self.ReadVar(1)
				#get supply voltage
				tmp = self.ReadVar(23)
				self.supply_voltage = float(tmp)/1000
				#get motor temperature
				tmp = self.ReadVar(24)
				self.temperature = float(tmp)/10
			finally: #release the locks
				self.ser_lock.release()
				self.data_lock.release()
			time.sleep(self.update_period)
		print "motor variable process exiting"
	
	def ReadVar(self, addr):
		#read variable at motor address
		self.ser.flushInput()
		self.ser.write(chr(0xA1) + chr(addr))
		[lbyte, hbyte] = self.ser.read(2)
		result = (ord(hbyte) << 8) | ord(lbyte)
		return result

	
