'''
	This program is used to set the desired configuration values for
		the pendulum

'''

#TODO add camera Calibration

import sys
import pickle
from uc_interface import ucEncoder

class Setup():
	def __init__(self):
		self.rightBrakeEncoderPos = 500
		self.rightBrakeCameraPos = 500
		self.leftBrakeEncoderPos = 0
		self.leftBrakeCameraPos = 0
		
	def pack(self):
		print "Used to setup the pendulum, this will overwrite the configuration file!"
		answer = raw_input("Is this your intended action (Y/n)?: ")
		
		#Exit if answer is no
		if (answer[0] != 'y' and answer[0] != 'Y'):
			print "Exiting..."
			sys.exit()
		
		#Continue the program
		
		#Get user input
		port = raw_input("Enter the micro-controller's port: ")
		file_name = raw_input("Enter the file for saving: ")
		
		#Open the micro-controller port for reading
		print "Initializing with port: " + port
		uc = ucEncoder(port)
		
		#Open the config file for writing
		file = open(file_name, 'w')
		print "Opened File " + file.name + " for writing."
		print file
		
		#Do the setup
		raw_input("Move the cart to the far right and press enter.")
		uc.send_reset()
		
		raw_input("Move the cart to the right brake position and press enter.")
		self.rightBrakeEncoderPos = uc.motor_count
		print self.rightBrakeEncoderPos
		#TODO add camera Calibration
		
		raw_input("Move the cart to the left brake position and press enter.")
		self.leftBrakeEncoderPos = uc.motor_count
		print self.leftBrakeEncoderPos
		#TODO add camera Calibration
		
		#write the setup object to the file in pickle
		pickle.dump(self, file)
		
		#Close the file
		file.close()
	def unpack(self, file_name):
		file = open(file_name, 'r')
		data = pickle.load(file)
		#Remember to close the file!
		file.close()
		return data
		
if __name__ == "__main__":
	setup = Setup()
	setup.pack()

