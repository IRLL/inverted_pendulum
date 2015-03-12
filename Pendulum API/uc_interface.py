'''
This is the class that reads the encoder data from the microcontroller
'''
import serial
import threading
import time
import signal
import sys
import argparse
#from encoder import Encoder

class ucEncoder():
	
	def __init__(self, Port, baudrate=115200):

		self.start_byte = 0x0A
		self.ppc = 10
		self.acpr = 500
		self.mcpr = 90


		self.arm_count = 0
		self.motor_count = 0
		self.switches = 0
		self.status = "Idle"

		##Serial port setup	
		self.ser = serial.Serial()
		self.ser.baudrate = baudrate
		if (type(Port) is int):
			self.ser.port = Port - 1 #windows numerical port
		elif(type(Port) is str):
			self.ser.port = Port #linux string port
		self.status = "opening port", Port
		self.ser.open()
		
		#Threading setup	
		self.alive = 1 #tell the thread to stay alive
		self.thread = threading.Thread(target=self.uc_process)
		self.thread.daemon = True #make daemon thread so it exits when program ends
		self.data_lock = threading.Lock() #mutex for variables
		self.ser_lock = threading.Lock() #mutex for serial port
		self.thread.start()
		self.status = "Spawned Process"
		time.sleep(.1)
		
	def __del__(self):
		self.ser.close()

	def getAngle(self):
		angle = float(self.arm_count) * 360 / (4*self.acpr)
		return self.arm_count
		return angle
	
	def getPosition(self):
		position = float(self.motor_count) / (4 * self.mcpr * self.ppc)
		return self.motor_count
		return position

	def getSwitches(self):
		temp = []
		temp.append(self.switches & 0b01)
		temp.append( (self.switches & 0b10) >> 1 )
		return temp
		
	'''
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
	'''
	
	def send_reset(self):
		self.status = "Zeroing Counters"
		self.ser.write(chr(0x0A))
	
	'''
	def encoder_process(self, byte):
		#Encoder processing
		#pass new encoder channel values to the appropriate encoder
		self.mEncoder.setNextState((byte & 0b1100) >> 2)
		self.aEncoder.setNextState(byte >> 4)
	'''	
	def uc_process(self):
		self.status = "Process Started"
		self.get_lock()
		while 1:
			#read serial byte (this call is blocking)
			#print "reading data..."
			packet = self.get_packet()
			#print "data read, pushing to variables..."
			#aquire mutex lock
			
			self.data_lock.acquire()
			try: #update the motor variable
				self.arm_count = (packet[1] << 8) | packet[2]
				self.motor_count = (packet[3] << 8) | packet[4]
				self.switches = packet[5]
			finally: #release the lock
				self.data_lock.release()

		self.status = "Process Exiting"

	def get_lock(self):
		#variables for the sync loop
		current_byte = '\0'
		packet_array = []
		in_sync = False

		#reset the serial port
		self.ser.close()
		self.ser.open()

		self.status = "Acquiring stream sync"

		while in_sync == False:
			#read a packet from the serial port
			current_byte = ord(self.ser.read())

			#if the byte is the control_byte, then receive several packets
			#otherwise, we will jump back to the top of the loop and get another byte
			if current_byte == self.start_byte:
				packet_array = [] # clear out the array
				packet_array.append(current_byte)  # add the byte to the array

				#receive several packets
				while len(packet_array) != 7:
					packet_array.append(ord(self.ser.read()))
				
				#x000000x000000x000000
				#check to see if the control byte is in the proper location in the received packets
				if  packet_array[0] == self.start_byte and \
					packet_array[6] == self.start_byte:

					#throw away rest of last packet
					self.ser.read(5)

					#say we are in sync so we can break out of the loop
					in_sync = True
					self.status = "sync locked"
	#end get_lock()

	def get_packet(self):

		success = False

		while success == False:

			#read 6 bytes from the serial port
			packet = self.ser.read(6)

			#ensure we are in sync by checking that the control byte is in the correct place
			if ord(packet[0]) != self.start_byte:
				self.status = "Error: lost sync"
				self.ser.flushInput() #flushes the serial rx buffer
				self.get_lock() #get back into sync
			else : #if we are in sync, break out of loop
				success = True

		num_packet = []
		for i in packet:
			num_packet.append(ord(i))

		return num_packet
	#end get_packet()


def exit_handler(signum, frame):
	print "exiting!"
	sys.exit()
	
	
def tester():
	uc = ucEncoder('/dev/ttyUSB0')	
	uc.send_reset()
	while 1:
		angle= uc.getAngle()
		position = uc.getPosition()
		switches = uc.getSwitches()
		print "buffer: ", uc.ser.inWaiting()
		print "angle: ", angle
		print "position: ", position
		print "left switch: ", switches[1], "right switch: ", switches[0]
		print ""
		time.sleep(.1)
	
	
if __name__ == "__main__":
	signal.signal(signal.SIGINT, exit_handler)
	tester()

	
