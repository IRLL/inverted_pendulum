#!/usr/bin/env python

import rospy
from inverted_pendulum.msg import PendulumPose, Cmd
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
        self.cmd_safe_pub = rospy.Publisher('cmd_safe', Cmd, queue_size=1)
        pid_parameters = rospy.get_param('pendulum/pid/')
        self.rate = rospy.Rate(100)
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

        if self.state == "IDLE":
            #don't want to publish anything, just return
            return;
        elif self.state == "INIT":
            if(current_sensors.x > -0.45):
                cmd.cmd = -15
                self.cmd_pub.publish(cmd)
            else:
                self.next_state_time = rospy.Time.now() + rospy.Duration(0.3)
                self.state = "RIGHT"
        elif self.state == "RIGHT":
            if(rospy.Time.now() > self.next_state_time):
                self.next_state_time = rospy.Time.now() + rospy.Duration(0.3)
                self.state = "LEFT"
            else:
                cmd.cmd = 70
                self.cmd_safe_pub.publish(cmd)
        elif self.state == "LEFT":
            if(rospy.Time.now() > self.next_state_time):
                self.state = "DONE"
            else:
                cmd.cmd = -50
                self.cmd_safe_pub.publish(cmd)

        elif self.state == "DONE":
            cmd.cmd = 0
            self.cmd_safe_pub.publish(cmd)
            rospy.signal_shutdown("done!")







if __name__ == "__main__":
    rospy.init_node('agent')
    node = Node()

    while not rospy.is_shutdown():
        rospy.loginfo("state: {}".format(node.state))
        node.update()
        node.rate.sleep()
