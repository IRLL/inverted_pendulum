#!/usr/bin/env python

import rospy
from inverted_pendulum.msg import PendulumPose, Cmd
from std_msgs.msg import Header
import threading
import math
from inverted_pendulum.timeout import Timeout

class Node():
    def __init__(self):
        self.sensor_sub = rospy.Subscriber('sensors', PendulumPose, self.sensor_callback)
        self.cmd_sub = rospy.Subscriber('cmd', Cmd, self.cmd_callback)
        self.cmd_pub = rospy.Publisher('cmd_safe', Cmd, queue_size=1)
        self.sensor_lock = threading.Lock()
        self.sensor_data = list()
        self.last_sensor = PendulumPose()
        self.sensor_timeout = Timeout(0.1)

        self.cmd_data = list()
        self.cmd_lock = threading.Lock()

        #pid_parameters = rospy.get_param('pendulum/safety/')
        self.max_cmd = 30
        self.brake_location = 0.5

    def sensor_callback(self, data):
        self.sensor_lock.acquire()
        try:
            self.sensor_data.append(data)
        finally:
            self.sensor_lock.release()
        self.sensor_timeout.reset()

    def cmd_callback(self, data):
        self.cmd_lock.acquire()
        try:
            self.cmd_data.append(data)
        finally:
            self.cmd_lock.release()



    def update(self):
        self.sensor_lock.acquire()
        try:
            if self.sensor_data: #check if list has an item
                current_sensors = self.sensor_data.pop()
                self.last_sensor = current_sensors
                self.sensor_data = list()
            else: #empty list, use default value
                current_sensors = self.last_sensor
        finally:
            self.sensor_lock.release()

        self.cmd_lock.acquire()
        try:
            if self.cmd_data: #check if list has an item
                current_cmd = self.cmd_data.pop()
                new_cmd = True
                self.cmd_data = list()
            else: #empty list, use default value
                new_cmd = False
                current_cmd = Cmd()
        finally:
            self.cmd_lock.release()


        #perform safety_checks, halt cart if necessary
        halt = False

        #limit max position of the cart
        if(current_sensors.x > self.brake_location):
            if(current_cmd.cmd > 0):
                halt = True
        if(current_sensors.x < -self.brake_location):
            if(current_cmd.cmd < 0):
                halt = True

        #if we aren't getting sensor data, we definitely need to not move
        if(self.sensor_timeout.isExpired()):
            rospy.logerr("stale sensor data!")
            halt = True

        if halt == True:
            safe_msg = Cmd()
            safe_msg.header = Header()
            safe_msg.header.stamp = rospy.Time.now()
            reverse_factor = -40

            #something better than brake
            if(current_sensors.xDot > 0.1) and (current_sensors.x > self.brake_location):
                safe_msg.cmd = current_sensors.xDot * reverse_factor
            elif(current_sensors.xDot < -0.1) and (current_sensors.x < -self.brake_location):
                safe_msg.cmd = current_sensors.xDot * reverse_factor
            else:
                safe_msg.cmd  = 0

            self.cmd_pub.publish(safe_msg)
            return



        #if we get here, all safety checks passed and we can send new commands
        if(new_cmd == True):
            #limit the maximum command of the cart
            if(abs(current_cmd.cmd) > self.max_cmd):
                current_cmd.cmd = math.copysign(self.max_cmd, current_cmd.cmd)
            self.cmd_pub.publish(current_cmd)




if __name__ == "__main__":
    rospy.init_node('safety_controller')
    node = Node()
    rate = rospy.Rate(100)
    while not rospy.is_shutdown():
        node.update()
        rate.sleep()
