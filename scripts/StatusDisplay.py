#!/usr/bin/env python

"""
Author: Brandon Kallaher (brandon.kallaher@wsu.edu)
Description:
    Subcribes to all topics and prints them in a readable format
"""

import rospy
import os
import time
import curses
import signal
import atexit

from inverted_pendulum.msg import PendulumPose
from inverted_pendulum.msg import MotorInfo
from inverted_pendulum.msg import MotorError
from inverted_pendulum.msg import SerialError
from inverted_pendulum.msg import LimitStatus
from inverted_pendulum.msg import Cmd

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
        self.stdscr = curses.initscr()
        curses.noecho()
        curses.cbreak()
        self.print_headers()

    # def __del__(self):
    #     curses.nocbreak()
    #     curses.echo()
    #     curses.endwin()
    #     rospy.loginfo("Deleting Status Node")

    def redraw(self, time):
        self.stdscr.addstr(0,6,  "{}".format(self.cmd))
        self.stdscr.addstr(2,17,  "{}".format(self.x))
        self.stdscr.addstr(3,17,  "{}".format(self.vel))
        self.stdscr.addstr(4,17,  "{}".format(self.theta))
        self.stdscr.addstr(5,18,  "{}".format(self.tDot))
        self.stdscr.addstr(7,12,  "{}".format(self.right))
        self.stdscr.addstr(8,12,  "{}".format(self.left))
        self.stdscr.addstr(11,23, "{}".format(self.errorStatus.safeStart))
        self.stdscr.addstr(12,23, "{}".format(self.errorStatus.serialError))
        self.stdscr.addstr(13,23, "{}".format(self.errorStatus.cmdTimeout))
        self.stdscr.addstr(14,23, "{}".format(self.errorStatus.limitSwitch))
        self.stdscr.addstr(15,23, "{}".format(self.errorStatus.lowVin))
        self.stdscr.addstr(16,23, "{}".format(self.errorStatus.highVin))
        self.stdscr.addstr(17,23, "{}".format(self.errorStatus.overTemp))
        self.stdscr.addstr(18,23, "{}".format(self.errorStatus.driverError))
        self.stdscr.addstr(19,23, "{}".format(self.errorStatus.errorLineHigh))
        self.stdscr.addstr(21,21, "{}".format(self.serialError.framing))
        self.stdscr.addstr(22,21, "{}".format(self.serialError.noise))
        self.stdscr.addstr(23,21, "{}".format(self.serialError.rxOverrun))
        self.stdscr.addstr(24,21, "{}".format(self.serialError.format))
        self.stdscr.addstr(25,21, "{}".format(self.serialError.crc))
        self.stdscr.addstr(27,40, "{}".format(self.limitStatus.errorOrSafeStart))
        self.stdscr.addstr(28,40, "{}".format(self.limitStatus.tempLimiter))
        self.stdscr.addstr(29,40, "{}".format(self.limitStatus.highTargetSpeed))
        self.stdscr.addstr(30,40, "{}".format(self.limitStatus.lowTargetSpeed))
        self.stdscr.addstr(31,40, "{}".format(self.limitStatus.accelDeccelLimiter))
        self.stdscr.addstr(32,40, "{}".format(self.limitStatus.an1Limit))
        self.stdscr.addstr(33,40, "{}".format(self.limitStatus.an2Limit))
        self.stdscr.addstr(34,40, "{}".format(self.limitStatus.usbKill))
        self.stdscr.addstr(35,3, "Target Speed: {}".format(self.targetSpeed))
        self.stdscr.addstr(36,3, "Speed: {}".format(self.speed))
        self.stdscr.addstr(37,3, "Brake Amount: {}".format(self.brakeAmt))
        self.stdscr.addstr(38,3, "Voltage: {}".format(self.vin))
        self.stdscr.addstr(39,3, "Temperature: {}".format(self.temp))
        self.stdscr.refresh()

    def info_callback(self, data):
        self.errorStatus = data.errorStatus
        self.serialError = data.serialError
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

    def print_headers(self):
        self.stdscr.addstr(0,0,  "CMD:")
        self.stdscr.addstr(1,0,  "Sensors:")
        self.stdscr.addstr(2,3,  "X:")
        self.stdscr.addstr(3,3,  u"x\u0307:".encode('utf-8'))
        self.stdscr.addstr(4,3,  u"\u03B8:".encode('utf-8'))
        self.stdscr.addstr(5,3,  u"\u03B8'".encode('utf-8') + " Velocity: ")
        self.stdscr.addstr(6,3,  "Limits:")
        self.stdscr.addstr(7,5,  "Right:")
        self.stdscr.addstr(8,5,  "Left:")
        self.stdscr.addstr(9,0,  "Motor Status:")
        self.stdscr.addstr(10,3, "Error Status:")
        self.stdscr.addstr(11,6, "Safe Start:")
        self.stdscr.addstr(12,6, "Serial Error:")
        self.stdscr.addstr(13,6, "Command Timeout:")
        self.stdscr.addstr(14,3, "Limit Switch:")
        self.stdscr.addstr(15,6, "Low Vin:")
        self.stdscr.addstr(16,6, "High Vin:")
        self.stdscr.addstr(17,6, "Over Temp:")
        self.stdscr.addstr(18,6, "Driver Error:")
        self.stdscr.addstr(19,6, "Error Line High:")
        self.stdscr.addstr(20,3, "Serial Error:")
        self.stdscr.addstr(21,6, "Framing Error:")
        self.stdscr.addstr(22,6, "Noise:")
        self.stdscr.addstr(23,6, "RX Overrun:")
        self.stdscr.addstr(24,6, "Format Error:")
        self.stdscr.addstr(25,6, "CRC Error:")
        self.stdscr.addstr(26,3, "Limit Status:")
        self.stdscr.addstr(27,6, "Error Line or Safe Start:")
        self.stdscr.addstr(28,6, "Temperature Limiter:")
        self.stdscr.addstr(29,3, "High Target Speed:")
        self.stdscr.addstr(30,6, "Low Target Speed:")
        self.stdscr.addstr(31,6, "Accel/Deccel/Brake limiter:")
        self.stdscr.addstr(32,6, "AN1 Limit Switch:")
        self.stdscr.addstr(33,6, "AN2 Limit Switch:")
        self.stdscr.addstr(34,6, "USB Kill Switch:")

#initialize the ros node
status = Status()

#Signal handler for ctrl-c
def handler ():
    curses.nocbreak()
    curses.echo()
    curses.endwin()
    rospy.loginfo("Goodbye")

if __name__ == "__main__":
    rospy.init_node('StatusDisplay')

    atexit.register(handler)

    motor_info_sub = rospy.Subscriber('motor/info', MotorInfo, status.info_callback)
    motor_cmd_sub = rospy.Subscriber('cmd', Cmd, status.cmd_callback)
    sensor_sub = rospy.Subscriber('sensors', PendulumPose, status.sensor_callback)

    rospy.Timer(rospy.Duration(1.0/status.fps), status.redraw)

    rospy.spin()
