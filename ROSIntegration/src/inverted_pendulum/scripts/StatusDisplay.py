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

from inverted_pendulum.msg import PendulumPose
from inverted_pendulum.msg import MotorInfo
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
        self.errorStatus = 0x0
        self.SerialError = 0x0
        self.limitStatus = 0
        self.targetSpeed = 0
        self.speed = 0
        self.brakeAmt = 0
        self.vin = 0
        self.temp = 0

    def redraw(self, time):
        os.system('clear')
        print "CMD: ", self.cmd
        print "Sensors:"
        print "  X: ", self.x
        print u"  x\u0307: ", self.vel
        print u"  \u03B8: ", self.theta
        print u"  \u03B8'", "Velocity: ", self.tDot
        print "  Limits:"
        print "    Right: ", self.right
        print "    Left: ", self.left
        print "Motor Status:"
        print "  Error Status: ", hex(self.errorStatus)
        print "  Serial Error: ", hex(self.SerialError)
        print "  Limit Status: ", self.limitStatus
        print "  Target Speed: ", self.targetSpeed
        print "  Speed: ", self.speed
        print "  Brake Amount: ", self.brakeAmt
        print "  Voltage: ", self.vin
        print "  Temperature: ", self.temp

    def info_callback(self, data):
        self.errorStatus = data.errorStatus
        self.SerialError = data.SerialError
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


if __name__ == "__main__":
    rospy.init_node('StatusDisplay')

    motor_info_sub = rospy.Subscriber('/motor/info', MotorInfo, status.info_callback)
    motor_cmd_sub = rospy.Subscriber('/cmd', Cmd, status.cmd_callback)
    sensor_sub = rospy.Subscriber('/sensors', PendulumPose, status.sensor_callback)

    rospy.Timer(rospy.Duration(1.0/status.fps), status.redraw)

    rospy.spin()

# while not rospy.is_shutdown():
#     os.system('clear')
#     redraw()
#     time.sleep(1.0/fps)
