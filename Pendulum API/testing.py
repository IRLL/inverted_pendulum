# -*- coding: utf-8 -*-
"""
Created on Tue Jun 09 22:03:11 2015

@author: Brandon
"""

from pendulum import *
import sys
import signal

'''****************************************************************************
                            Global Variables
****************************************************************************'''
p = None
LINUX = False
WINDOWS = False

'''****************************************************************************
                        Testing for pendulum.py
****************************************************************************'''
def pexit_handler(signum, frame):
    global p
    p.motor.Stop()
    p.status = "Exiting, Goodbye!"
    menu()

#Tests the Pendulum Reset function
def ptest_reset(r_val=0):
    global p
    p = Pendulum('/dev/ttyACM0', '/dev/ttyUSB0')
    p.status = "Running..."
    status_thread = threading.Thread(target=print_status)
    status_thread.daemon = True
    status_thread.start()
    time.sleep(5)
    p.Reset(r_val)
    
#Tests Motor movement
def ptester():
    global p
    move_speed = 35
    p = Pendulum('/dev/ttyACM0', '/dev/ttyUSB0')
    
    status_thread = threading.Thread(target=print_status)
    status_thread.daemon = True
    status_thread.start()
 
    p.Reset()

    while 1:
        p.moveRight(move_speed, move_speed)
        time.sleep(.25)
        p.moveLeft(move_speed, move_speed)
        time.sleep(.2)

#Continuously print the status of the Pendulum    
def print_status():
    global p
    global LINUX
    global WINDOWS
    
    print "Inverted Pendulum API by Brandon Kallaher and James Irwin"
    print "Running on ", platform.system()
    print "Python Version: ", platform.python_version()
    print "Clear the Path of the arm or suffer the consequences!"
    time.sleep(3)
    #p.Reset()
    #Endlessly update the console
    while True:
        if (WINDOWS):
            os.system('cls')
        elif(LINUX):
            os.system('clear')
        console_update()
        time.sleep(.1)

#Does all the printing for the print_status() thread        
def console_update():
    global p
    stats = p.motor.getVariables()
    switches = p.uc.getSwitches()
    print "Pendulum Status: ",             p.status
    print "uC Process Status: ",         p.uc.status
    print "Motor Process Status: ",     p.motor.status
    print "Motor Controller Status: ",     hex(stats[0])
    print "Voltage: ",                     stats[1]
    print "Temperature: ",                 stats[2]
    print "Motor Speed: ",                 stats[3]
    print "Angle: ",                     p.uc.getAngle()
    print "Position: ",                 p.uc.getPosition()
    print "Left Switch: ",                 switches[1]
    print "Right Switch: ",             switches[0]
    print "right Brake: ",                 p.su.rightBrakeEncoderPos
    
def psubmenu():
    signal.signal(signal.SIGINT, pexit_handler)
    if LINUX:
        os.system("clear")
    elif WINDOWS:
        os.system("cls")
    print u'\u2588' * 52    
    
    print "System Sub-menu:"
    print "1. Test the reset function"
    print "2. Test the Motor with status updates"
    print "3. Return to Main Menu"
    choice = raw_input("Enter your decision: ")
    if choice == '1':
        print u'\u2588' * 52
        reset = raw_input("Enter the offset from zero to reset to: ")
        ptest_reset(reset)
    elif choice == '2':
        ptester()
    elif choice == '3':
        menu()
    else:
        print "Invalid Input... Try Again."
        psubmenu()

'''****************************************************************************
                        Testing for uc_interface.py
****************************************************************************'''
def ucexit_handler(signum, frame):
    print "Returning to Main Menu..."
    menu()
    
    
def uctester():
    signal.signal(signal.SIGINT, ucexit_handler)
    uc = ucEncoder('/dev/ttyUSB0')    
    time.sleep(1)
    uc.send_reset()
    while 1:
        angle= uc.getAngle()
        position = uc.getPosition()
        switches = uc.getSwitches()
        arm_spd = uc.getArmVelDegPS()
        rps = uc.getArmVelRadPS()
        m_spd = uc.getMotorVelMPS()
        cmps = uc.getMotorVelCMPS()
        #print "buffer: ", uc.ser.inWaiting()
        print "Angle: ", angle
        print "Position: ", position
        print "Arm Speed (Deg/S): ", arm_spd
        print "Arm Speed (Rad/S): ", rps
        print "Motor Speed (M/S): ", m_spd
        print "Motor Speed (CM/S): ", cmps
        print "Left Switch: ", switches[1], "Right Switch: ", switches[0]
        print ""
        time.sleep(.1)

'''****************************************************************************
                            Testing for motor.py
****************************************************************************'''
def mtester():
    signal.signal(signal.SIGINT, mexit_handler)
    motor = Motor()
    flop = 0
    while True:
        motor_stats = motor.getVariables()
        if flop:
            motor.MoveLeft(25)
        else:
            motor.MoveRight(25)
        flop = not flop
        print "Temperature:   ", motor_stats[2]
        print "Input Voltage: ", motor_stats[1]
        time.sleep(2)    

def mexit_handler(signal):
    menu()

'''****************************************************************************
                                Menu Logic
****************************************************************************'''
def exit_handler(signal):
    sys.exit()

def menu():    
    signal.signal(signal.SIGINT, exit_handler)
    if LINUX:
        os.system("clear")
    elif WINDOWS:
        os.system("cls")
    print u'\u2588' * 52

    print "Main Menu:"
    print "Use this file for testing an aspect of the pendulum."
    print "1. Test the Motor"
    print "2. Test the Micro-controller"
    print "3. Test the System as a Whole"
    print "4. Exit"
    choice = raw_input("Enter your decision: ")
    
    if choice == '1':
        mtester()
    elif choice == '2':
        uctester()
    elif choice == '3':
        psubmenu()
    elif choice == '4':
        sys.exit()
    else:
        print "Invalid Input... Try Again."
        menu()

if __name__ == "__main__":
    #Check to see what OS is running
    if (platform.system() == "Linux"):
        LINUX = True
    elif (platform.system() == "Windows"):
        WINDOWS = True
    menu()
