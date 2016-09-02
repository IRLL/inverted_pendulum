#!/usr/bin/env python

import rospy
from inverted_pendulum.msg import PendulumPose, Cmd, PID_state
from std_msgs.msg import Header
import threading
import math

def in_threshold(value, goal, tolerance):
    delta = abs(goal - value)

    if delta < tolerance:
        return True
    else:
        return False


class Node():
    def __init__(self):
        self.sensor_sub = rospy.Subscriber('sensors', PendulumPose, self.sensor_callback) 
        self.cmd_pub = rospy.Publisher('cmd', Cmd, queue_size=1)
        pid_parameters = rospy.get_param('pendulum/pid/')
        self.rate = rospy.Rate(pid_parameters['update_frequency'])
        self.sensor_lock = threading.Lock()
        self.sensor_data = list()
        self.state = "INIT"

    def sensor_callback(self, data):
        self.sensor_lock.acquire()
        try:
            self.sensor_data.append(data)
        finally:
            self.sensor_lock.release()


    def update(self):
        rospy.loginfo("in state: %s", self.state)
        self.sensor_lock.acquire()
        try:
            if self.sensor_data: #check if list has an item
                current_sensors = self.sensor_data.pop()
                self.sensor_data = list()
            else: #empty list, use default value
                current_sensors = PendulumPose()
        finally:
            self.sensor_lock.release()

        cmd = Cmd()
        time = rospy.Time.now()
        cmd.header.stamp = time 

        if self.state == "INIT":
            p=1
            goal = -0.74
            error = goal - current_sensors.x
            cmd.cmd = error*p
            if(in_threshold(goal, current_sensors.x, 0.05) and
                in_threshold(0, current_sensors.xDot, 0.1) and
                in_threshold(0, current_sensors.thetaDot, 0.1) and
                in_threshold(180, abs(current_sensors.theta), 1)):
                self.state = "RIGHT"
        elif self.state == "RIGHT":
            cmd.cmd = 150
            if(current_sensors.xDot > 1.5):
                self.state = "LEFT"
        elif self.state == "LEFT":
            cmd.cmd = -150



        self.cmd_pub.publish(cmd)

        

if __name__ == "__main__":
    rospy.init_node('agent')
    node = Node()

    while not rospy.is_shutdown():
        node.update()
        node.rate.sleep()	
