#!/usr/bin/env python

import rospy
from inverted_pendulum.msg import PendulumPose, Cmd, PID_state
import threading
import math

class Node():
    def __init__(self):
        self.sensor_sub = rospy.Subscriber('sensors', PendulumPose, self.sensor_callback) 
        self.cmd_pub = rospy.Publisher('cmd', Cmd, queue_size=1)
        self.pid_pub = rospy.Publisher('pid', PID_state, queue_size=1)
        self.sensor_lock = threading.Lock()
        self.sensor_data = list()

        pid_parameters = rospy.get_param('pendulum/pid')
        
        self.rate = rospy.Rate(pid_parameters['update_frequency'])
        self.kp = pid_parameters['p']
        self.ki = pid_parameters['i']
        self.kd = -pid_parameters['d']
        self.xpos_weight = pid_parameters['xpos_weight']
        self.angle_weight = pid_parameters['angle_weight']
        self.error_integral = 0
        self.prev_error = 0

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
        pid_state = PID_state()
        cmd.header.stamp = rospy.Time.now()
        pid_state.header.stamp = cmd.header.stamp
        angle = current_sensors.theta
        
        error = -angle*self.angle_weight + -current_sensors.x*self.xpos_weight
        derivative = self.prev_error - error
        self.prev_error = error
        self.error_integral += error

        cmd.cmd = self.kp * error + \
                  self.ki * self.error_integral + \
                  self.kd * derivative

        self.cmd_pub.publish(cmd)

        pid_state.error = error
        pid_state.error_integral = self.error_integral
        pid_state.error_derivative = derivative
        self.pid_pub.publish(pid_state)

        
        
        

if __name__ == "__main__":
    rospy.init_node('agent')
    node = Node()

    while not rospy.is_shutdown():
        node.update()
        node.rate.sleep()	
