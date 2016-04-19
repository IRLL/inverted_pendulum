#!/usr/bin/env python

"""
Author: Brandon Kallaher (brandon.kallaher@wsu.edu)
Description:
    Subcribes to /cmd for controlling the motor
    Publishes to /motor/info to send out Debug information from the motor
"""

import rospy
import serial
import sys
import os.path

#import our messages
from inverted_pendulum.msg import Cmd
from inverted_pendulum.msg import MotorInfo

#define variables for changing later
port = '/dev/ttyACM0'


def callback(data):
    pass

class Motor():
    def __init__(self, Port='/dev/ttyACM0', baudrate=115200):
        self.ser = serial.Serial()
        self.ser.timeout = 1
        self.ser.baudrate = baudrate
        self.ser.port = Port

        self.update_period = .5

        self.error_codes = 0
        self.supply_voltage = 0
        self.temperature = 0
        self.speed = 0

    def __del__(self):
        self.Stop()
        self.ser.close()

    def MoveRight(self, percent):
        self.Enable()
        speed = self.getValueFromPercent(percent)
        speed1 = chr(speed & 0x1F)
        speed2 = chr(speed >> 5)
        self.ser.write(chr(0x85) + speed1 + speed2)

    def MoveLeft(self, percent):
        self.Enable()
        speed = self.getValueFromPercent(percent)
        speed1 = chr(speed & 0x1F)
        speed2 = chr(speed >> 5)
        self.ser.write(chr(0x86) + speed1 + speed2)

    def Stop(self):
        self.ser.write(chr(0x92))
        self.ser.write(chr(32))

    def Enable(self):
        self.ser.write(chr(0x83))

    @staticmethod
    def getValueFromPercent(percent):
        return 3200 * percent / 100

    def ReadVar(self, addr):
        self.ser.flushInput()
        self.ser.write(chr(0xA1) + chr(addr))
        [lbyte, hbyte] = self.ser.read()
        result = (ord(hbyte) << 8) | ord(lbyte)
        return result

if __name__ == '__main__':
    #initialize the ros node
    rospy.init_node('MotorSerial')

    #Setup the publisher and subscriber
    sub = rospy.Subscriber('/cmd', Cmd, callback)
    pub = rospy.Publisher('/motor/info', MotorInfo, queue_size=1)

    #check to make sure the port actually exists
    if not os.path.isfile(port):
        print "The serial port {} could not be opened!\n".format(port)
        sys.exit(1)

    #open the serial port
    mc = serial.Serial()
    mc.timeout = 1 #max blocking time for read is 1
    mc.baudrate = 115200
    mc.port = port
    mc.open()

    #Send the Safe Start command upon boot
    mc.write(0x83)

    while not rospy.is_shutdown():
        pass

    #stop the motor
    mc.write(chr(0x92))
    mc.write(chr(32))

    #close the serial port
    mc.close()
