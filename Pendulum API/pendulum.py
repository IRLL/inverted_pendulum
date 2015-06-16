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
        speed = 28
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
        while True:
            position = self.uc.motor_count
            if (position <= self.su.encoderCenter):
                self.motor.Stop()
                break;
            self.motor.MoveLeft(speed)
        self.motor.Stop()
        time.sleep(1)        
        self.resetFlag = False
        self.softStopFlag = False
  
        self.status = "done resetting!"
        time.sleep(2)
        
    def getState(self):
        '''
        returns (meters, radians)
        '''
        mPose = self.uc.getXm() # meters
        mSpeed = self.uc.getMotorVelMPS()
        radians = self.uc.getRadians()
        radps = self.uc.getArmVelRadPS()
        return (mPose, mSpeed, radians, radps)
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
        reward = -1
        if angle >= 168 and angle <= 192:
            reward = 0
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