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

    #upon shutdown close the serial port
    mc.close()
