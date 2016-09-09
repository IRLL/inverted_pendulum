#!/usr/bin/env python

"""
Author: Brandon Kallaher (brandon.kallaher@wsu.edu)
Description:
    Subcribes to all topics and prints them in a readable format
"""

#TODO: Wrapup into a class for readability

import rospy
import os
import time
import curses
import signal

from inverted_pendulum.msg import PendulumPose
from inverted_pendulum.msg import MotorInfo
from inverted_pendulum.msg import MotorError
from inverted_pendulum.msg import SerialError
from inverted_pendulum.msg import LimitStatus
from inverted_pendulum.msg import Cmd

global stdscr

class Status():
    def __init__(self):
        self.fps = 20

        self.cmd = 0.0
        self.x = 0.0
        self.theta = 0.0
        self.vel = 0.0
        self.tDot = 0.0
        self.right = False
        self.left = False
        self.errorStatus = MotorError()
        self.serialError = SerialError()
        self.limitStatus = LimitStatus()
        self.targetSpeed = 0
        self.speed = 0
        self.brakeAmt = 0
        self.vin = 0
        self.temp = 0

    def redraw(self, time):
        global stdscr
        os.system('clear')
        stdscr.addstr(0,6,"{}".format(self.cmd))
        stdscr.addstr(1,0,"Sensors:")
        stdscr.addstr(2,3,"X: {}".format(self.x))
        stdscr.addstr(3,3, u"x\u0307: {}".format(self.vel).encode('utf-8'))
        stdscr.addstr(4,3, u"\u03B8: {}".format(self.theta).encode('utf-8'))
        stdscr.addstr(5,3, u"\u03B8'".encode('utf-8') + " Velocity: {}".format(self.tDot))
        stdscr.addstr(6,3, "Limits:")
        stdscr.addstr(7,5, "Right: {}".format(self.right))
        stdscr.addstr(8,5,"Left: {}".format(self.left))
        stdscr.addstr(9,0,"Motor Status:")
        stdscr.addstr(10,3, "Error Status:")
        stdscr.addstr(11,6, "Safe Start: {}".format(self.errorStatus.safeStart))
        stdscr.addstr(12,6, "Serial Error: {}".format(self.errorStatus.serialError))
        stdscr.addstr(13,6, "Command Timeout: {}".format(self.errorStatus.cmdTimeout))
        stdscr.addstr(14,6, "Limit Switch: {}".format(self.errorStatus.limitSwitch))
        stdscr.addstr(15,6, "Low Vin: {}".format(self.errorStatus.lowVin))
        stdscr.addstr(16,6, "High Vin: {}".format(self.errorStatus.highVin))
        stdscr.addstr(17,6, "Over Temp: {}".format(self.errorStatus.overTemp))
        stdscr.addstr(18,6, "Driver Error: {}".format(self.errorStatus.driverError))
        stdscr.addstr(19,6, "Error Line High: {}".format(self.errorStatus.errorLineHigh))
        stdscr.addstr(20,3, "Serial Error:")
        stdscr.addstr(21,6, "Framing Error: {}".format(self.serialError.framing))
        stdscr.addstr(22,6, "Noise: {}".format(self.serialError.noise))
        stdscr.addstr(23,6, "RX Overrun: {}".format(self.serialError.rxOverrun))
        stdscr.addstr(24,6, "Format Error: {}".format(self.serialError.format))
        stdscr.addstr(25,6, "CRC Error: {}".format(self.serialError.crc))
        stdscr.addstr(26,3, "Limit Status:")
        stdscr.addstr(27,6, "Error Line or Safe Start: {}".format(self.limitStatus.errorOrSafeStart))
        stdscr.addstr(28,6, "Temperature Limiter: {}".format(self.limitStatus.tempLimiter))
        stdscr.addstr(29,6, "High Target Speed: {}".format(self.limitStatus.highTargetSpeed))
        stdscr.addstr(30,6, "Low Target Speed: {}".format(self.limitStatus.lowTargetSpeed))
        stdscr.addstr(31,6, "Acceleration/Decceleration/Brake limiter: {}".format(self.limitStatus.accelDeccelLimiter))
        stdscr.addstr(32,6, "AN1 Limit Switch: {}".format(self.limitStatus.an1Limit))
        stdscr.addstr(33,6, "AN2 Limit Switch: {}".format(self.limitStatus.an2Limit))
        stdscr.addstr(34,6, "USB Kill Switch: {}".format(self.limitStatus.usbKill))
        stdscr.addstr(35,3, "Target Speed: {}".format(self.targetSpeed))
        stdscr.addstr(36,3, "Speed: {}".format(self.speed))
        stdscr.addstr(37,3, "Brake Amount: {}".format(self.brakeAmt))
        stdscr.addstr(38,3, "Voltage: {}".format(self.vin))
        stdscr.addstr(39,3, "Temperature: {}".format(self.temp))
        stdscr.refresh()

    def info_callback(self, data):
        self.errorStatus = data.errorStatus
        self.serialError = data.SerialError
        self.limitStatus = data.limitStatus
        self.targetSpeed = data.targetSpeed
        self.speed = data.speed
        self.brakeAmt = data.brakeAmt
        self.vin = data.vin
        self.temp = data.temp

    def cmd_callback(self, data):
        self.cmd = data.cmd


    def sensor_callback(self, data):
        self.x = data.x
        self.vel = data.xDot
        self.theta = data.theta
        self.tDot = data.thetaDot
        self.right = data.rightLim
        self.left = data.leftLim

#initialize the ros node
status = Status()

#Signal handler for ctrl-c
def handler ():
        curses.nocbreak()
        curses.echo()
        curses.endwin()
        print "Goodbye"

if __name__ == "__main__":
    global stdscr
    rospy.init_node('StatusDisplay')

    rospy.on_shutdown(handler)

    #initialize the curses screen
    stdscr = curses.initscr()
    curses.noecho()
    curses.cbreak()

    stdscr.addstr(0,0, "CMD: ")
    motor_info_sub = rospy.Subscriber('motor/info', MotorInfo, status.info_callback)
    motor_cmd_sub = rospy.Subscriber('cmd', Cmd, status.cmd_callback)
    sensor_sub = rospy.Subscriber('sensors', PendulumPose, status.sensor_callback)

    rospy.Timer(rospy.Duration(1.0/status.fps), status.redraw)

    rospy.spin()
