#!/usr/bin/env python

import rospy
from inverted_pendulum.msg import PendulumPose, Cmd
from math import pi
from visualizer import Visualizer

class Node():
    def __init__(self):
        self.sensor_sub = rospy.Subscriber('/sensors', PendulumPose, self.pose_callback)
        self.cmd_sub = rospy.Subscriber('/cmd', Cmd, self.cmd_callback)
        self.rate = rospy.Rate(20)
        self.cmd = Cmd()
        self.pose = PendulumPose()
        self.visualizer = Visualizer(2)

    def cmd_callback(self, cmd):
        self.cmd = cmd
    def pose_callback(self, pose):
        self.pose = pose


    def update(self):
        self.visualizer.draw(self.pose.x, self.pose.theta*pi/180,
                self.pose.xDot, self.pose.thetaDot*pi/180, self.cmd.cmd)	
        self.cmd = Cmd()

if __name__ == "__main__":
    rospy.init_node('Visualizer')
    node = Node()

    while not rospy.is_shutdown():
        node.update()

        node.rate.sleep()	
