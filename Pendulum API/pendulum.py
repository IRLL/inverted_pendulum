'''
This is the class that controls the pendulum
'''
import serial
import threading

class Pendulum(threading.Thread):
	
	def __init__(self, Port):
		threading.Thread.__init__(self)
		self.ser = serial.Serial()
		self.position = 0
		if (type(Port) is int):
			self.ser.port = Port - 1
		elif(type(Port) is str):
			self.ser.port = Port
		self.ser.open()
		print(self.ser.name)
		self.ser.write("0x83")
	def __del__(self):
		self.Stop()
		self.ser.close()
	def MoveRight(self, percent):
		speed = self.getValueFromPercent(percent)
		self.ser.write(0x85 + ' ' + speed)
	def MoveLeft(self, percent):
		speed = self.getValueFromPercent(percent)
		self.ser.write(0x86 + ' ' + speed)
	def Stop(self):
		self.ser.write("0xE0")
	def Reset(self):
		while(position != 0):
			MoveLeft(self, getValueFromPercent(10))
	@staticmethod
	def getValueFromPercent(percent):
		return 3200 * percent / 100
p = Pendulum(6)
p.MoveRight(50)