#!/usr/bin/env python

"""
Author: Brandon Kallaher (brandon.kallaher@wsu.edu)
Description:
    Subcribes to all topics and prints them in a readable format
"""

import rospy
import curses

from inverted_pendulum.msg import PendulumPose
from inverted_pendulum.msg import MotorInfo
from inverted_pendulum.msg import Cmd

#initialize the curses screen
stdscr = curses.initscr()

#define the callbacks
def info_callback:
    pass

def cmd_callback:
    pass

def sensor_callback:
    pass

#initialize the ros node
rospy.init_node('StatusDisplay')

motor_info_sub = rospy.Subscriber('/motor/info', MotorInfo, info_callback)
motor_cmd_sub = rospy.Subscriber('/cmd', Cmd)
sensor_sub = rospy.Subscriber('/sensors', PendulumPose)

rospy.spin()
