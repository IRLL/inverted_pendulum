'''
	This program is used to set the desired configuration values for
		the pendulum

'''

import sys
import pickle
from uc_interface import ucEncoder

class Setup():
	def __init__(self):
		self.rightBrakePos = 500
		self.leftBrakePos = 0
		
def config():
	print "Used to setup the pendulum, this will overwrite the configuration file!"
	answer = raw_input("Is this your intended action (Y/n)?: ")
	
	#Exit if answer is no
	if (answer != 'y' and answer != 'Y'):
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
	
	#initialize setup object
	setup = Setup()
	
	#Do the setup
	raw_input("Move the cart to the right brake position and press enter.")
	setup.rightBrakePos = uc.getPosition()
	
	raw_input("Move the cart to the left brake position and press enter.")
	setup.leftBrakePos = uc.getPosition()
	
	#write the setup object to the file in pickle
	pickle.dump(setup, file)
	
	#Close the file
	file.close()

if __name__ == "__main__":
	config()
	
	
#Example for loading the config object
'''
file = open(file_name, 'r')
test = pickle.load(file)
print test.rightBrakePos
#Remember to close the file!
file.close()
'''