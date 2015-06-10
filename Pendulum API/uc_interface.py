'''
This is the class that reads the encoder data from the microcontroller
'''
import serial
import threading
import time
import math

class ucEncoder():
	
	def __init__(self, Port, baudrate=115200):

		self.start_byte = 0x0A
		self.acpr = 500
		self.mcpr = 540
		self.mcircumference = math.pi * 5 #pi * diameter
		self.mconst = float(self.mcpr * 4) / self.mcircumference #Pulses per centimeter

		self.arm_count = 0
		self.arm_vel_dps = 0
		self.motor_count = 0
		self.motor_vel = 0
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
		self.thread.start()
		self.status = "Spawned Process"
		time.sleep(.1)
		
	def __del__(self):
		self.ser.close()

	def getAngle(self):
		angle = float(self.arm_count) * 360 / (4*self.acpr)
		return angle

	def getRadians(self):
		radians = float(self.arm_count) * 2 * math.pi / (4*self.acpr)
		return radians
	
	def getPosition(self):
		position = float(self.motor_count-6244) / (self.mconst)
		return position

	def getArmVelDegPS(self):
		return self.arm_vel_dps

	def getArmVelRadPS(self):
		return (float(self.arm_vel_dps) / 90) * (math.pi / 2)

	def getMotorVelCMPS(self):
		return float(self.motor_vel) / self.mconst
	
	def getMotorVelMPS(self):
		return float(self.motor_vel) / (100 * self.mconst)

	def getXcm(self):
		return self.getPosition()

	def getXm(self):
		return self.getPosition() / 100.0

	def getSwitches(self):
		temp = []
		temp.append(self.switches & 0b01)
		temp.append( (self.switches & 0b10) >> 1 )
		return temp

	def getMotorCount(self):
		return self.motor_count
		
	def send_reset(self):
		self.status = "Zeroing Counters"
		self.ser.write(chr(0x0A))
	
	def uc_process(self):
		self.status = "Process Started"
		prev_arm = 0
		prev_motor = 0
		delta = 500
		start = time.time()
		end = 0
		self.get_lock()
		while 1:
			#read serial byte (this call is blocking)
			#print "reading data..."
			start = time.time()
			#print "start: ", start
			packet = self.get_packet()
			#print "data read, pushing to variables..."
			#aquire mutex lock
			end = time.time()
			#print "End: ", end
			#print "end-start: ", end - start
			self.data_lock.acquire()
			try: #update the motor variable
				self.arm_count = (packet[1] << 8) | packet[2]
				if prev_arm > 2000 - delta and self.arm_count < delta:
					prev_arm = prev_arm - 2000
				elif prev_arm < delta and self.arm_count > 2000 - delta:
					prev_arm = 2000 + prev_arm
				self.arm_vel_dps = float( (self.arm_count - prev_arm) * 360 / (4 * self.acpr)) / (float(end) - float(start))
				self.motor_count = (packet[3] << 8) | packet[4]
				self.motor_vel = float (self.motor_count - prev_motor) / (float(end) - float(start))
				self.switches = packet[5]
				prev_arm = self.arm_count
				prev_motor = self.motor_count
			#except ZeroDivisionError:
				#pass
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

		self.status = "Acquiring Stream Sync..."

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
					self.status = "Sync Locked"
	#end get_lock()

	def get_packet(self):

		success = False

		while success == False:

			#read 6 bytes from the serial port
			packet = self.ser.read(6)

			#ensure we are in sync by checking that the control byte is in the correct place
			if ord(packet[0]) != self.start_byte:
				self.status = "Error: Lost Sync"
				self.ser.flushInput() #flushes the serial rx buffer
				self.get_lock() #get back into sync
			else : #if we are in sync, break out of loop
				success = True

		num_packet = []
		for i in packet:
			num_packet.append(ord(i))

		return num_packet
	#end get_packet()	