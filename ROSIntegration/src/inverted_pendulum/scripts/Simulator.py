#!/usr/bin/env python

import rospy
from inverted_pendulum.msg import PendulumPose, Cmd
import threading
from math import pi

from model import Pendulum as PendulumModel

class Node():
    def __init__(self, sim_parameters):
        self.sensor_pub = rospy.Publisher('/inverted_pendulum/sensors', PendulumPose, queue_size=1)
        self.cmd_sub = rospy.Subscriber('/inverted_pendulum/cmd', Cmd, self.cmd_callback)
        self.cmd_lock = threading.Lock()
        self.cmds = list()

        self.rate = rospy.Rate(sim_parameters['realtime_multiplier'] * 1.0/sim_parameters['delta_time'])

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
    parameters = rospy.get_param('pendulum')
    sim_parameters = parameters['simulation']
    node = Node(sim_parameters)
    model = PendulumModel(
            start_cartx = parameters['start_cartx'],
            start_angle = parameters['start_angle'],
            track_length = parameters['track_length'],
            dt = sim_parameters['delta_time'],
            g = sim_parameters['gravity'],
            l = sim_parameters['pole_length'],
            m = sim_parameters['mass'],
            cfriction = sim_parameters['cart_friction'],
            pfriction = sim_parameters['pole_friction']
            )

    while not rospy.is_shutdown():
        status = node.update(model)
        node.sensor_pub.publish(status)

        node.rate.sleep()
