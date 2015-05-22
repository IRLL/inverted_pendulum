'''
	This program is used to set the desired configuration values for
		the pendulum

'''

#TODO add camera Calibration

import sys
import pickle
import time
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
		if port == '':
			print "Defaulting to port: COM4!"
			port = "COM4"
		file_name = raw_input("Enter the file for saving: ")
		if file_name == '':
			print "Defaulting to file name: test!"
			file_name = "test"
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
		time.sleep(1)
		base = uc.motor_count
		
		raw_input("Move the cart to the right brake position and press enter.")
		self.rightBrakeEncoderPos = uc.motor_count
		print "Right Brake: ", self.rightBrakeEncoderPos
		#TODO add camera Calibration
		
		raw_input("Move the cart to the left brake position and press enter.")
		self.leftBrakeEncoderPos = uc.motor_count
		print "Left Brake: ", self.leftBrakeEncoderPos
		#TODO add camera Calibration
		
		#Test for Arm angle Error
		raw_input("Let the arm come to a complete stop with the cart moved to the right. Press enter.")
		uc.send_reset()
		time.sleep(1)
		print "Arm Count: ", uc.arm_count
		print "Arm Angle: "
		print "----------degrees: ", uc.getAngle()
		print "----------radians: ", uc.getRadians()
		
		raw_input("Move the arm around a bit then let it come to rest. Press Enter.")
		print "Arm Count: ", uc.arm_count
		print "Arm Angle: "
		print "----------degrees: ", uc.getAngle()
		print "----------radians: ", uc.getRadians()
		
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

