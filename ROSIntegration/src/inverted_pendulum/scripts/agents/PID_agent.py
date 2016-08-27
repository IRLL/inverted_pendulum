#!/usr/bin/env python

import rospy
from inverted_pendulum.msg import PendulumPose, Cmd, PID_state
from std_msgs.msg import Header
import threading
import math

class PID():
    def __init__(self, parameter_dict):
        self.kp = parameter_dict['p']
        self.ki = parameter_dict['i']
        self.kd = parameter_dict['d']
        self.error_integral = 0
        self.prev_error = 0
        self.derivative = 0

    def update(self, error):
        self.derivative =  error - self.prev_error
        self.prev_error = error
        self.error_integral += error

        return  self.kp * error + \
                  self.ki * self.error_integral + \
                  self.kd * self.derivative

    def reset(self):
        self.error_integral = 0
        self.prev_error = 0
        self.derivative = 0

    def get_state(self):
        return self.prev_error, self.error_integral, self.derivative



class Node():
    def __init__(self):
        self.sensor_sub = rospy.Subscriber('sensors', PendulumPose, self.sensor_callback) 
        self.cmd_pub = rospy.Publisher('cmd', Cmd, queue_size=1)
        self.angle_pid_state_pub = rospy.Publisher('pid/angle', PID_state, queue_size=1)
        self.cart_pid_state_pub = rospy.Publisher('pid/cart', PID_state, queue_size=1)
        self.sensor_lock = threading.Lock()
        self.sensor_data = list()


        pid_parameters = rospy.get_param('pendulum/pid/')
        self.rate = rospy.Rate(pid_parameters['update_frequency'])
        angle_pid = pid_parameters['angle']
        cart_pid = pid_parameters['cart']

        self.angle_pid = PID(pid_parameters['angle'])
        self.cart_pid = PID(pid_parameters['cart'])



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
        time = rospy.Time.now()
        cmd.header.stamp = time 
        

        angle_correction = self.angle_pid.update(-current_sensors.theta)
        cart_correction = self.cart_pid.update(-current_sensors.x)

        cmd.cmd = angle_correction + cart_correction

        self.cmd_pub.publish(cmd)

        pid_state.header = Header()
        pid_state.header.stamp = time
        pid_state.error, pid_state.error_integral, pid_state.error_derivative = self.angle_pid.get_state()	
        self.angle_pid_state_pub.publish(pid_state)

        pid_state.header = Header()
        pid_state.header.stamp = time
        pid_state.error, pid_state.error_integral, pid_state.error_derivative = self.cart_pid.get_state()	
        self.cart_pid_state_pub.publish(pid_state)
        
        
        

if __name__ == "__main__":
    rospy.init_node('agent')
    node = Node()

    while not rospy.is_shutdown():
        node.update()
        node.rate.sleep()	
