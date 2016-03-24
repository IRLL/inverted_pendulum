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

fps = 20

cmd = 0.0
x = 0.0
theta = 0.0
vel = 0.0
right = False
left = False
errorStatus = 0x0
SerialError = 0x0
limitStatus = 0
targetSpeed = 0
speed = 0
brakeAmt = 0
vin = 0
temp = 0

#define the callbacks
def info_callback(data):
    global errorStatus, SerialError, limitStatus, targetSpeed, speed, \
            brakeAmt, vin, temp
    errorStatus = data.errorStatus
    SerialError = data.SerialError
    limitStatus = data.limitStatus
    targetSpeed = data.targetSpeed
    speed = data.speed
    brakeAmt = data.brakeAmt
    vin = data.vin
    temp = data.temp

def cmd_callback(data):
    global cmd
    cmd = data.cmd

def sensor_callback(data):
    global x, theta, vel, right, left
    x = data.x
    theta = data.theta
    vel = data.vel
    right = data.right
    left = data.left

def redraw():
    print "CMD: ", cmd
    print "Sensors:"
    print "  X: ", x
    print "  Theta: ", theta
    print "  Velocity: ", vel
    print "  Limits:"
    print "    Right: ", right
    print "    Left: ", left
    print "Motor Status:"
    print "  Error Status: ", hex(errorStatus)
    print "  Serial Error: ", hex(SerialError)
    print "  Limit Status: ", limitStatus
    print "  Target Speed: ", targetSpeed
    print "  Speed: ", speed
    print "  Brake Amount: ", brakeAmt
    print "  Voltage: ", vin
    print "  Temperature: ", temp

#initialize the ros node
rospy.init_node('StatusDisplay')

motor_info_sub = rospy.Subscriber('/motor/info', MotorInfo, info_callback)
motor_cmd_sub = rospy.Subscriber('/cmd', Cmd, cmd_callback)
sensor_sub = rospy.Subscriber('/sensors', PendulumPose)


while not rospy.is_shutdown():
    os.system('clear')
    redraw()
    time.sleep(1.0/fps)
