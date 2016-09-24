#!/usr/bin/env python

import rospy
import actionlib
from inverted_pendulum.msg import PendulumPose, Cmd
from inverted_pendulum.msg import ResetAction, ResetResult
import threading
from math import pi

from inverted_pendulum.model import Pendulum as PendulumModel

class Node():
    def __init__(self, sim_parameters, model):
        self.sensor_pub = rospy.Publisher('/sensors', PendulumPose, queue_size=1)
        self.cmd_sub = rospy.Subscriber('/cmd', Cmd, self.cmd_callback)
        self.reset_as = actionlib.SimpleActionServer(
            'inverted_pendulum/reset',
            ResetAction, execute_cb=self.reset_callback, auto_start=False
        )
        self.reset_as.start()
        self.cmd_lock = threading.Lock()
        self.cmds = list()
        self.model = model

        self.rate = rospy.Rate(sim_parameters['realtime_multiplier'] * 1.0/sim_parameters['delta_time'])

    def cmd_callback(self, cmd):
        self.cmd_lock.acquire()
        try:
            self.cmds.append(cmd)
        finally:
            self.cmd_lock.release()


    def update(self):
        self.cmd_lock.acquire()
        try:
            if self.cmds: #check if list has an item
                cmd = self.cmds.pop()
                self.cmds = list()
            else: #empty list, use default value
                cmd = Cmd()
        finally:
            self.cmd_lock.release()

        self.model.update(cmd.cmd)
        state = self.model.get_state()
        pose = PendulumPose()

        pose.x, pose.theta, pose.xDot, pose.thetaDot, _ = state
        pose.theta = pose.theta * 180.0/pi
        pose.thetaDot = pose.thetaDot * 180.0/pi
        pose.header.stamp = rospy.Time.now()
        return pose

    def get_state(self):
        state = self.model.get_state()
        pose = PendulumPose()

        pose.x, pose.theta, pose.xDot, pose.thetaDot, _ = state
        pose.theta = pose.theta * 180.0/pi
        pose.thetaDot = pose.thetaDot * 180.0/pi
        pose.header.stamp = rospy.Time.now()
        return pose

    def reset_callback(self, goal):
        rospy.loginfo("resetting angle=%f x=%f", goal.angle, goal.position)
        result = ResetResult(goal.angle, goal.position)
        self.model.reset(start_cartx=goal.position, start_angle=goal.angle)
        self.sensor_pub.publish(self.get_state())
        self.reset_as.set_succeeded(result)


if __name__ == "__main__":
    rospy.init_node('Simulator')
    parameters = rospy.get_param('pendulum')
    sim_parameters = parameters['simulation']
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

    node = Node(sim_parameters, model)

    while not rospy.is_shutdown():
        status = node.update()
        node.sensor_pub.publish(status)

        node.rate.sleep()
