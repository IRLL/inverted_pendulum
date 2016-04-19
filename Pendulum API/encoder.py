'''
    This class evaluates new encoder data and saves the new position/angle
    
    Position is in centimeters per Gabe's request
    angle is in degrees
'''

import threading
import time
import sys
import argparse
import signal

class Encoder():
    def __init__(self, cpr = 256, ppc = 10):
        #class variable setup
        self.lastState = 0
        self.currState = 0
        self.position = 0 #In centimeters
        self.angle = 0 #In degrees
        self.tickCount = 0
        self.cpr = cpr #CONST Don't change!!!!!
        self.ppc = ppc #CONST Don't change!!!!!
        
        #threading setup
        self.alive = 1 #tell the thread to stay alive
        self.thread = threading.Thread(target=self.encoder_process)
        self.thread.daemon = True #make daemon thread so it exits when program ends
        self.data_lock = threading.Lock() #mutex for variables
        self.thread.start()
    
    def getVariables(self):
        variables = []
        self.data_lock.acquire()
        try:
            #return a tuple containing all of the values
            variables.append(self.position)
            variables.append(self.angle)
            variables.append(self.tickCount)
        finally:
            self.data_lock.release()
        return variables
    def encoder_process(self):
        print "Encoder initialized"
        while 1:
            if self.currState == self.lastState:
                '''
                    Theory (in c State is (CHA, CHB) (bit1 = CHA, bit0 = CHB)):
                        switch (currentState)
                        {
                            case 0: //current state = 00
                                //if LastState=01, increment Count (CW)
                                //else LastState=10, decrement Count (CCW)
                                LastState==1 ? Count++ : Count--;
                                break;
                            case 1: //current state = 01
                                //if LastState=11, increment Count (CW)
                                //else LastState=00, decrement Count (CCW)
                                LastState==3 ? Count++ : Count--;
                                break;
                            case 2: //current state = 10
                                //if LastState=00, increment Count (CW)
                                //else LastState=11, decrement Count (CCW)
                                LastState==0 ? Count++ : Count--;
                                break;
                            case 3: //current state = 11
                                //if LastState=10, increment Count (CW)
                                //else LastState=01, decrement Count (CCW)
                                LastState==2 ? Count++ : Count--;
                                break;
                        }
                        LastState = currentState;
                '''
                if self.currState == 0:
                    self.data_lock.acquire()
                    try:
                        self.tickCount = (self.tickCount + 1 if self.lastState == 1 else self.tickCount - 1)
                    finally:
                        self.data_lock.release()
                elif self.currState == 1:
                    self.data_lock.acquire()
                    try:
                        self.tickCount = (self.tickCount + 1 if self.lastState == 3 else self.tickCount - 1)
                    finally:
                        self.data_lock.release()
                elif self.currState == 2:
                    self.data_lock.acquire()
                    try:
                        self.tickCount = (self.tickCount + 1 if self.lastState == 0 else self.tickCount - 1)
                    finally:
                        self.data_lock.release()
                elif self.currState == 3:
                    self.data_lock.acquire()
                    try:
                        self.tickCount = (self.tickCount + 1 if self.lastState == 2 else self.tickCount - 1)
                    finally:
                        self.data_lock.release()
            
                self.lastState = self.currState
        print "Encoder Process Died"
        
    def setNextState(self, state):
        self.data_lock.acquire()
        try:
            self.currState = state
        finally:
            self.data_lock.release()
        
    def zero(self):
        self.data_lock.acquire()
        try:
            self.position = 0
            self.angle = 0
        finally:
            self.data_lock.release()
    
    def updateCalculations(self):
        #first calculate position
        #tickCount / (4CPR * PPC)
        pos = self.tickCount / (4 * self.cpr * self.ppc)
        
        #then calculate angle
        #tickCount * 360 / (4CPR)
        ang = self.tickCount * 360 / (4 * self.cpr)
        
        #Update variables
        self.data_lock.acquire()
        try:
            self.position = pos
            self.angle = ang
        finally:
            self.data_lock.release()
            
