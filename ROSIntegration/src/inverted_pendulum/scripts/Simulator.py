#!/usr/bin/env python

import rospy
from inverted_pendulum.msg import PendulumPose, Cmd
import threading
from math import pi

from model import Pendulum as PendulumModel

class Node():
    def __init__(self):
        self.sensor_pub = rospy.Publisher('/sensors', PendulumPose, queue_size=1)
        self.cmd_sub = rospy.Subscriber('/cmd', Cmd, self.cmd_callback)
        self.cmd_lock = threading.Lock()
        self.cmds = list()
        self.rate = rospy.Rate(100)

    def cmd_callback(self, cmd):
        self.cmd_lock.acquire()
        try:
            self.cmds.append(cmd)
        finally:
            self.cmd_lock.release()


    def update(self, model):
        self.cmd_lock.acquire()
        try:
            if self.cmds: #check if list has an item
                cmd = self.cmds.pop()
                self.cmds = list()
            else: #empty list, use default value
                cmd = Cmd()
        finally:
            self.cmd_lock.release()

        model.update(cmd.cmd)
        state = model.get_state()
        pose = PendulumPose()

        pose.x, pose.theta, pose.xDot, pose.thetaDot, _ = state
        pose.theta = pose.theta * 180.0/pi
        pose.thetaDot = pose.thetaDot * 180.0/pi
        pose.header.stamp = rospy.Time.now()
        return pose



if __name__ == "__main__":
    rospy.init_node('Simulator')
    node = Node()
    model = PendulumModel()

    while not rospy.is_shutdown():
        status = node.update(model)
        node.sensor_pub.publish(status)

        node.rate.sleep()	
