#!/usr/bin/python
'''
This is the class that controls the pendulum
'''
import threading
import time
import platform
import os
from motor import Motor
from uc_interface import ucEncoder
import pickle
import sys 
from math import floor, ceil,sqrt

p = None
LINUX = False
WINDOWS = False

class Setup():
    pass

class Pendulum():
    def __init__(self, motorPort = '/dev/ttyACM0', ucPort = '/dev/ttyUSB0', config = "config"):
        self.status = "Booting..."
        self.motor = Motor(motorPort)
        self.uc = ucEncoder(ucPort)
        self.resetFlag = False
        self.softStopFlag = False
        
        #Load the Setup configuration
        file = open(config, 'r')
        up = pickle.Unpickler(file)
        self.su = up.load()
        
        self.watchdogThread = threading.Thread(target=self.watchdog)
        self.watchdogThread.daemon = True
        self.watchdogThread.start()
                
        self.status = "Booted"
    def __del__(self):
        self.motor.Stop()
    def Reset(self, start=0):
        self.status = "resetting pendulum!"
        self.resetFlag = True
        speed = 25
        left_switch = 0
        right_switch = 0
        while(not left_switch):
            self.motor.MoveLeft(speed)
            switches = self.uc.getSwitches()
            left_switch = switches[1]
            time.sleep(.1)
            
        while(not right_switch):
            self.motor.MoveRight(speed)
            switches = self.uc.getSwitches()
            right_switch = switches[0]
            time.sleep(.1)
        
        self.motor.Stop()
        
        #wait for arm to settle
        current = 0
        previous = 1
        wait = 3
        count = wait #must be constant for this many seconds
        self.status = "waiting for arm to settle"
        while count > 0: 
            previous = current
            time.sleep(1)
            current = self.uc.getAngle()
            if (previous == current):
                count -= 1
            else:
                count = wait
        
        self.uc.send_reset()
        time.sleep(2)

        self.status = "moving to center position"
        position = self.uc.motor_count
        self.motor.MoveLeft(speed)
        while position > self.su.encoderCenter:
            position = self.uc.motor_count
            self.motor.MoveLeft(speed)
        self.motor.Stop()
        time.sleep(1)       

	# GABE start
	#wait for arm to settle at the center 
        current = 0
        previous = 1
        wait = 3
        count = wait #must be constant for this many seconds
        self.status = "waiting for arm to settle"
        while count > 0: 
            previous = current
            time.sleep(1)
            current = self.uc.getAngle()
            if (previous == current):
                count -= 1
            else:
                count = wait
	# GABE end
 
        self.resetFlag = False
        self.softStopFlag = False
  
        self.status = "done resetting!"
        time.sleep(2)

    def plotting_process(self):
        print "starting plotting thread"        
        self.myplot = MyPlot(100)
        fig = plt.figure()
        ax = plt.axes(xlim=(0,100), ylim=(-360, 360))
        angle_data, = ax.plot([], [])
        pos_data, = ax.plot([], [])
        anim = animation.FuncAnimation(fig, self.update_plot, fargs=(angle_data, pos_data), interval = 100)
        
        plt.show()
        print "plotting windows closed"

        #os._exit(0)

    def update_plot(self, frameNum, angle_data, pos_data):
        data = [self.uc.getAngle(), 0]
        self.myplot.add(data)
        angle_data.set_data(range(self.myplot.maxLen), self.myplot.ax)
        pos_data.set_data(range(self.myplot.maxLen), self.myplot.ay)

        return angle_data,
        
        

        
    def getState(self):
        '''
        returns (meters, radians)
        '''
        mSpeed = self.uc.getMotorVelMPS()
        #radians = self.uc.getRadians()
	radps = self.uc.getArmVelRadPS()
	mPose = self.uc.getXm() # meters
	angle = self.uc.getAngle() 
	#print "Angle from State ", angle
	if angle >= 0 and angle <= 180:
		angle = 180 - angle
	elif angle > 180:
		angle = -(angle - 180)
  	if angle == -0:
		angle = 0.0
        
        return (mPose, mSpeed, angle, radps)
    def getDegrees(self):
        return self.uc.getAngle()
    def moveRight(self, percent, threshold=30):
        if not self.softStopFlag:
            if (percent > threshold):
                percent = threshold            
            self.motor.MoveRight(percent)
    def moveLeft(self, percent, threshold=30):
        if not self.softStopFlag:
            if (percent > threshold):
                percent = threshold
            self.motor.MoveLeft(percent)
    def getActions(self):
        return [0,1] # (0)Left (1)Right
    def step(self, action):
        # TODO: change percent move
        if action == 0:
            self.moveLeft(15)
        elif action == 1:
            self.moveRight(15)
    def isTerminated(self):
        return self.softStopFlag
    def getReward(self):
        # TODO: change angle range
        angle = self.uc.getAngle()   
	#print "Angle from Reward ", angle,

	f_angle = floor(angle)
	c_angle = ceil(angle)

	if f_angle == 180 or c_angle == 180:
		print "Perfect!"
	if f_angle == 90 or c_angle == 90:
		print "90 degrees"
	if f_angle == 270 or c_angle == 270:
		print "270 degrees"


	if angle >= 0 and angle <= 180:
		reward = (angle - 180) / 180.0
	elif angle > 180:
		reward = -(angle - 180) / 180.0
	else:
		reward = -10000 # should NEVER get this values
		print angle
		print "Should never get a reward of -1000"
		sys.exit(1)

	if reward == -0.0:
		reward = 0.0

	if self.softStopFlag: # went outside
		reward = -2
        return reward

    def stop(self):
        self.motor.Stop()
    
    #TODO: Add in the Camera positioning
    def watchdog(self):
        while(True):
            #TODO Check
            pos = self.uc.getMotorCount()
            if (not self.resetFlag):
                if (pos >= self.su.rightBrakeEncoderPos) or (pos <= self.su.leftBrakeEncoderPos):
                    self.softStopFlag = True
                    self.uc.status = "Watchdog tripped!"
                    self.motor.Stop()
                    time.sleep(2)
                    self.uc.status = "Watchdog idle"
                    while(self.softStopFlag):
                        pass
            time.sleep(.05)
        
    def print_status(self):
        global LINUX
        global WINDOWS
        
        #Check to see what OS is running
        if (platform.system() == "Linux"):
            LINUX = True
        elif (platform.system() == "Windows"):
            WINDOWS = True
        
        print "Inverted Pendulum API by Brandon Kallaher and James Irwin"
        print "Running on ", platform.system()
        print "Python Version: ", platform.python_version()
        print "Clear the Path of the arm or suffer the consequences!"
        time.sleep(3)
        
        #Endlessly update the console
        while True:
            if (WINDOWS):
                os.system('cls')
            elif(LINUX):
                os.system('clear')
            self.console_update()
            time.sleep(.1)

    #Does all the printing for the print_status() thread        
    def console_update(self):
         stats = self.motor.getVariables()
         switches = self.uc.getSwitches()
         print "Pendulum Status: ",                self.status
         print "uC Process Status: ",            self.uc.status
         print "Motor Process Status: ",        self.motor.status
         print "Motor Controller Status: ",        hex(stats[0])
         print "Voltage: ",                        stats[1]
         print "Temperature: ",                    stats[2]
         print "Motor Speed: ",                    stats[3]
         print "Angle: %f%c"                    %(self.uc.getAngle(), u'\xb0')
         print "       %f rad"                    %(self.uc.getRadians())
         print "Position: %f meters"            %(self.uc.getXm())
         print "Left Switch: ",                    switches[1]
         print "Right Switch: ",                switches[0]
         print ""
