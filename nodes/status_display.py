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
        self.edge_hit = False
        self.errorStatus = MotorError()
        self.serialError = SerialError()
        self.limitStatus = LimitStatus()
        self.targetSpeed = 0
        self.speed = 0
        self.brakeAmt = 0
        self.vin = 0
        self.temp = 0

        self.colors = {}
        self.col2Start = 36
        self.stdscr = curses.initscr()
        curses.noecho()
        curses.cbreak()
        curses.start_color()
        curses.use_default_colors()
        curses.init_pair(1,curses.COLOR_CYAN, -1)
        curses.init_pair(3,curses.COLOR_WHITE,curses.COLOR_RED)
        curses.init_pair(2,curses.COLOR_GREEN, -1)
        self.header_color = curses.color_pair(1) | curses.A_BOLD

        self.init_colors()
        self.print_headers()

    def init_colors(self):
        self.colors['edgeHit']  = curses.color_pair(self.edge_hit + 2)
        self.colors['safeStart'] = curses.color_pair(self.errorStatus.safeStart + 2)
        self.colors['serialError'] = curses.color_pair(self.errorStatus.serialError + 2)
        self.colors['cmdTimeout'] = curses.color_pair(self.errorStatus.cmdTimeout + 2)
        self.colors['limitSwitch'] = curses.color_pair(self.errorStatus.limitSwitch + 2)
        self.colors['lowVin'] = curses.color_pair(self.errorStatus.lowVin + 2)
        self.colors['highVin'] = curses.color_pair(self.errorStatus.highVin + 2)
        self.colors['overTemp'] = curses.color_pair(self.errorStatus.overTemp + 2)
        self.colors['driverError'] = curses.color_pair(self.errorStatus.driverError + 2)
        self.colors['errorLineHigh'] = curses.color_pair(self.errorStatus.errorLineHigh + 2)
        self.colors['framing'] = curses.color_pair(self.serialError.framing + 2)
        self.colors['noise'] = curses.color_pair(self.serialError.noise + 2)
        self.colors['rxOverrun'] = curses.color_pair(self.serialError.rxOverrun + 2)
        self.colors['format'] = curses.color_pair(self.serialError.format + 2)
        self.colors['crc'] = curses.color_pair(self.serialError.crc + 2)
        self.colors['errorOrSafeStart'] = curses.color_pair(self.limitStatus.errorOrSafeStart + 2)
        self.colors['tempLimiter'] = curses.color_pair(self.limitStatus.tempLimiter + 2)
        self.colors['highTargetSpeed'] = curses.color_pair(self.limitStatus.highTargetSpeed + 2)
        self.colors['lowTargetSpeed'] = curses.color_pair(self.limitStatus.lowTargetSpeed + 2)
        self.colors['accelDeccelLimiter'] = curses.color_pair(self.limitStatus.accelDeccelLimiter + 2)
        self.colors['an1Limit'] = curses.color_pair(self.limitStatus.an1Limit + 2)
        self.colors['an2Limit'] = curses.color_pair(self.limitStatus.an2Limit + 2)
        self.colors['usbKill'] = curses.color_pair(self.limitStatus.usbKill + 2)

    def redraw(self, time):
        self.stdscr.addstr(0,23,  "{: 10.3f}".format(self.cmd))
        self.stdscr.addstr(2,23,  "{: 10.3f}".format(self.x))
        self.stdscr.addstr(3,23,  "{: 10.3f}".format(self.vel))
        self.stdscr.addstr(4,23,  "{: 10.3f}".format(self.theta))
        self.stdscr.addstr(5,23,  "{: 10.3f}".format(self.tDot))
        self.stdscr.addstr(7,23,  "{}   ".format(self.edge_hit), self.colors['edgeHit'])
        self.stdscr.addstr(11,23, "{}   ".format(self.errorStatus.safeStart), self.colors['safeStart'])
        self.stdscr.addstr(12,23, "{}   ".format(self.errorStatus.serialError), self.colors['serialError'])
        self.stdscr.addstr(13,23, "{}   ".format(self.errorStatus.cmdTimeout), self.colors['cmdTimeout'])
        self.stdscr.addstr(14,23, "{}   ".format(self.errorStatus.limitSwitch), self.colors['limitSwitch'])
        self.stdscr.addstr(15,23, "{}   ".format(self.errorStatus.lowVin), self.colors['lowVin'])
        self.stdscr.addstr(16,23, "{}   ".format(self.errorStatus.highVin), self.colors['highVin'])
        self.stdscr.addstr(17,23, "{}   ".format(self.errorStatus.overTemp), self.colors['overTemp'])
        self.stdscr.addstr(18,23, "{}   ".format(self.errorStatus.driverError), self.colors['driverError'])
        self.stdscr.addstr(19,23, "{}   ".format(self.errorStatus.errorLineHigh), self.colors['errorLineHigh'])
        self.stdscr.addstr(1, self.col2Start + 23, "{}   ".format(self.serialError.framing), self.colors['framing'])
        self.stdscr.addstr(2, self.col2Start + 23, "{}   ".format(self.serialError.noise), self.colors['noise'])
        self.stdscr.addstr(3, self.col2Start + 23, "{}   ".format(self.serialError.rxOverrun), self.colors['rxOverrun'])
        self.stdscr.addstr(4, self.col2Start + 21, "{}   ".format(self.serialError.format), self.colors['format'])
        self.stdscr.addstr(5, self.col2Start + 21, "{}   ".format(self.serialError.crc), self.colors['crc'])
        self.stdscr.addstr(7, self.col2Start + 34, "{}   ".format(self.limitStatus.errorOrSafeStart), self.colors['errorOrSafeStart'])
        self.stdscr.addstr(8, self.col2Start + 34, "{}   ".format(self.limitStatus.tempLimiter), self.colors['tempLimiter'])
        self.stdscr.addstr(9, self.col2Start + 34, "{}   ".format(self.limitStatus.highTargetSpeed), self.colors['highTargetSpeed'])
        self.stdscr.addstr(10, self.col2Start + 34, "{}   ".format(self.limitStatus.lowTargetSpeed), self.colors['lowTargetSpeed'])
        self.stdscr.addstr(11, self.col2Start + 34, "{}   ".format(self.limitStatus.accelDeccelLimiter), self.colors['accelDeccelLimiter'])
        self.stdscr.addstr(12, self.col2Start + 34, "{}   ".format(self.limitStatus.an1Limit), self.colors['an1Limit'])
        self.stdscr.addstr(13, self.col2Start + 34, "{}   ".format(self.limitStatus.an2Limit), self.colors['an2Limit'])
        self.stdscr.addstr(14, self.col2Start + 34, "{}   ".format(self.limitStatus.usbKill), self.colors['usbKill'])
        self.stdscr.addstr(15, self.col2Start + 18, "{}   ".format(self.targetSpeed))
        self.stdscr.addstr(16, self.col2Start + 18, "{}   ".format(self.speed))
        self.stdscr.addstr(17, self.col2Start + 18, "{}   ".format(self.brakeAmt))
        self.stdscr.addstr(18, self.col2Start + 18, "{}   ".format(self.vin))
        self.stdscr.addstr(19, self.col2Start + 18, "{}   ".format(self.temp))
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
        self.colors['safeStart'] = curses.color_pair(self.errorStatus.safeStart + 2)
        self.colors['serialError'] = curses.color_pair(self.errorStatus.serialError + 2)
        self.colors['cmdTimeout'] = curses.color_pair(self.errorStatus.cmdTimeout + 2)
        self.colors['limitSwitch'] = curses.color_pair(self.errorStatus.limitSwitch + 2)
        self.colors['lowVin'] = curses.color_pair(self.errorStatus.lowVin + 2)
        self.colors['highVin'] = curses.color_pair(self.errorStatus.highVin + 2)
        self.colors['overTemp'] = curses.color_pair(self.errorStatus.overTemp + 2)
        self.colors['driverError'] = curses.color_pair(self.errorStatus.driverError + 2)
        self.colors['errorLineHigh'] = curses.color_pair(self.errorStatus.errorLineHigh + 2)
        self.colors['framing'] = curses.color_pair(self.serialError.framing + 2)
        self.colors['noise'] = curses.color_pair(self.serialError.noise + 2)
        self.colors['rxOverrun'] = curses.color_pair(self.serialError.rxOverrun + 2)
        self.colors['format'] = curses.color_pair(self.serialError.format + 2)
        self.colors['crc'] = curses.color_pair(self.serialError.crc + 2)
        self.colors['errorOrSafeStart'] = curses.color_pair(self.limitStatus.errorOrSafeStart + 2)
        self.colors['tempLimiter'] = curses.color_pair(self.limitStatus.tempLimiter + 2)
        self.colors['highTargetSpeed'] = curses.color_pair(self.limitStatus.highTargetSpeed + 2)
        self.colors['lowTargetSpeed'] = curses.color_pair(self.limitStatus.lowTargetSpeed + 2)
        self.colors['accelDeccelLimiter'] = curses.color_pair(self.limitStatus.accelDeccelLimiter + 2)
        self.colors['an1Limit'] = curses.color_pair(self.limitStatus.an1Limit + 2)
        self.colors['an2Limit'] = curses.color_pair(self.limitStatus.an2Limit + 2)
        self.colors['usbKill'] = curses.color_pair(self.limitStatus.usbKill + 2)

    def cmd_callback(self, data):
        self.cmd = data.cmd

    def sensor_callback(self, data):
        self.x = data.x
        self.vel = data.xDot
        self.theta = data.theta
        self.tDot = data.thetaDot
        self.edge_hit = data.edge
        self.colors['edgeHit'] = curses.color_pair(self.edge_hit + 2)

    def print_headers(self):
        self.stdscr.addstr(0,0,  "CMD:")
        self.stdscr.addstr(1,0,  "Sensors:", self.header_color)
        self.stdscr.addstr(2,3,  "X:")
        self.stdscr.addstr(3,3,  "dx/dt:")
        self.stdscr.addstr(4,3,  u"\u03B8:".encode('utf-8'))
        self.stdscr.addstr(5,3,  u"d\u03B8/dt:".encode('utf-8'))
        self.stdscr.addstr(6,3,  "Limits:", self.header_color)
        self.stdscr.addstr(7,5,  "Edge Hit:")
        self.stdscr.addstr(9,0,  "Motor Status:", self.header_color)
        self.stdscr.addstr(10,3, "Error Status:", self.header_color)
        self.stdscr.addstr(11,6, "Safe Start:")
        self.stdscr.addstr(12,6, "Serial Error:")
        self.stdscr.addstr(13,6, "Command Timeout:")
        self.stdscr.addstr(14,6, "Limit Switch:")
        self.stdscr.addstr(15,6, "Low Vin:")
        self.stdscr.addstr(16,6, "High Vin:")
        self.stdscr.addstr(17,6, "Over Temp:")
        self.stdscr.addstr(18,6, "Driver Error:")
        self.stdscr.addstr(19,6, "Error Line High:")
        self.stdscr.addstr(0, self.col2Start + 3, "Serial Error:", self.header_color)
        self.stdscr.addstr(1, self.col2Start + 6, "Framing Error:")
        self.stdscr.addstr(2, self.col2Start + 6, "Noise:")
        self.stdscr.addstr(3, self.col2Start + 6, "RX Overrun:")
        self.stdscr.addstr(4, self.col2Start + 6, "Format Error:")
        self.stdscr.addstr(5, self.col2Start + 6, "CRC Error:")
        self.stdscr.addstr(6, self.col2Start + 3, "Limit Status:", self.header_color)
        self.stdscr.addstr(7, self.col2Start + 6, "Error Line or Safe Start:")
        self.stdscr.addstr(8, self.col2Start + 6, "Temperature Limiter:")
        self.stdscr.addstr(9, self.col2Start + 6, "High Target Speed:")
        self.stdscr.addstr(10, self.col2Start + 6, "Low Target Speed:")
        self.stdscr.addstr(11, self.col2Start + 6, "Accel/Deccel/Brake limiter:")
        self.stdscr.addstr(12, self.col2Start + 6, "AN1 Limit Switch:")
        self.stdscr.addstr(13, self.col2Start + 6, "AN2 Limit Switch:")
        self.stdscr.addstr(14, self.col2Start + 6, "USB Kill Switch:")
        self.stdscr.addstr(15, self.col2Start + 3, "Target Speed:")
        self.stdscr.addstr(16, self.col2Start + 3, "Speed:")
        self.stdscr.addstr(17, self.col2Start + 3, "Brake Amount:")
        self.stdscr.addstr(18, self.col2Start + 3, "Voltage:")
        self.stdscr.addstr(19, self.col2Start + 3, "Temperature:")

        for i in range(0,24):
            self.stdscr.addch(i,self.col2Start-1,curses.ACS_VLINE)

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
