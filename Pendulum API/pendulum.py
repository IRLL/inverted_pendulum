'''
This is the class that controls the pendulum
'''
import serial
import threading
import time

class Pendulum(threading.Thread):
	
	def __init__(self, Port):
		threading.Thread.__init__(self)
		self.ser = serial.Serial()
		self.data_lock = threading.Lock()
		self.position = 0
		self.ser.baudrate = 9600
		if (type(Port) is int):
			self.ser.port = Port - 1
		elif(type(Port) is str):
			self.ser.port = Port
		self.ser.open()
		print(self.ser.name)
		self.Enable()
	def __del__(self):
		self.Stop()
		self.ser.close()
	def MoveRight(self, percent):
		speed = self.getValueFromPercent(percent)
		speed1 = chr(speed & 0x1F)
		speed2 = chr(speed >> 5)
		self.ser.write(chr(0x85) + speed1 + speed2)
	def MoveLeft(self, percent):
		speed = self.getValueFromPercent(percent)
		speed1 = chr(speed & 0x1F)
		speed2 = chr(speed >> 5)
		self.ser.write(chr(0x86) + speed1 + speed2)
	def Stop(self):
		self.ser.write(chr(0xE0))
	def getVariables():
		self.data_lock.acquire()
		try:
			#return a tuple containing all the values
			pass
		finally:
			self.data_lock.release()
	def run():
		self.data_lock.acquire()
		try:
			#run the data acquisition from the microcontroller
			pass
		finally:
			self.data_lock.release()
	def Reset(self):
		while(position != 0):
			MoveLeft(self, getValueFromPercent(10))

	def Enable(self):	
		self.ser.write(chr(0x83))
	@staticmethod
	def getValueFromPercent(percent):
		return 3200 * percent / 100

def tester():
	p = Pendulum('/dev/ttyACM0')
	while 1:
		p.MoveRight(10)
#		time.sleep(2)
#		p.MoveLeft(10)
#		time.sleep(2)
#		p.Stop()
#		time.sleep(1)
#		p.Enable()		
	
	
if __name__ == "__main__":
	tester()
