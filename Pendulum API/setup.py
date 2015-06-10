'''
	This program is used to set the desired configuration values for
		the pendulum

'''

import sys
import pickle
import time
from uc_interface import ucEncoder

class Setup():
	def __init__(self):
		self.rightBrakeEncoderPos = 500
		self.rightBrakeCameraPos = 500
		self.encoderCenter = 0
		self.leftBrakeEncoderPos = 0
		self.leftBrakeCameraPos = 0
		
def pack():
	print "Used to setup the pendulum, this will overwrite the configuration file!"
	answer = raw_input("Is this your intended action (Y/n)?: ")
	
	#Exit if answer is no
	if (answer[0] != 'y' and answer[0] != 'Y'):
		print "Exiting..."
		sys.exit()
	
	#Continue the program
	setup = Setup()
	
	#Get user input
	port = raw_input("Enter the micro-controller's port: ")
	if port == '':
		print "Defaulting to port: /dev/ttyUSB0!"
		port = "/dev/ttyUSB0"
	file_name = raw_input("Enter the file for saving: ")
	if file_name == '':
		print "Defaulting to file name: config!"
		file_name = "config"
	#Open the micro-controller port for reading
	print "Initializing with port: " + port
	uc = ucEncoder(port)
	
	#Open the config file for writing
	file = open(file_name, 'w')
	print "Opened File " + file.name + " for writing."
	print file
	
	#Do the setup
	raw_input("Move the cart to the far right, wait for the arm to\nsettle and press enter.")
	uc.send_reset()
	time.sleep(1)
	
	raw_input("Move the cart to the right brake position wait for the\narm to settle and press enter.")
	setup.rightBrakeEncoderPos = uc.motor_count
	print "Right Brake: ", setup.rightBrakeEncoderPos
	print "Arm Count: ", uc.arm_count
	print "Arm Angle: "
	print "----------degrees: ", uc.getAngle()
	print "----------radians: ", uc.getRadians()
	
	raw_input("Move the cart to the left brake position and press enter.")
	setup.leftBrakeEncoderPos = uc.motor_count
	print "Left Brake: ", setup.leftBrakeEncoderPos

	#Calculate the center
	setup.encoderCenter = (setup.leftBrakeEncoderPos + setup.rightBrakeEncoderPos)/2
	print "Center: ", setup.encoderCenter
	
	#write the setup object to the file in pickle
	p = pickle.Pickler(file)
	p.dump(setup)
	
	#Close the file
	file.close()
def unpack(file_name):
	file = open(file_name, 'r')
	data = pickle.load(file)
	#Remember to close the file!
	file.close()
	return data
		
if __name__ == "__main__":
	pack()

