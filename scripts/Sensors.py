#!/usr/bin/env python

import rospy

from inverted_pendulum.msg import PendulumPose, MotorInfo
from std_msgs.msg import Int16MultiArray
from std_msgs.msg import Header
import std_srvs.srv
from collections import deque

class avg_filter:
    def __init__(self, window_size):
        self.window_size = window_size
        self.queue = deque()
        self.total = 0

    def run(self, new_item):
        self.queue.append(new_item)
        if(len(self.queue) > self.window_size):
            old_item = self.queue.popleft()
        else:
            old_item = 0

        self.total -= old_item
        self.total += new_item

        return self.total/len(self.queue)


class Node():
    def __init__(self):
        self.theta_conv = 360.0/4096
        
        self.sensor_pub = rospy.Publisher('sensors', PendulumPose, queue_size=1)
        self.raw_sensor_sub = rospy.Subscriber('raw_sensors', Int16MultiArray, self.sensor_callback)
        self.motor_info_sub = rospy.Subscriber('motor/info', MotorInfo, self.motor_callback)
        self.calibrate_server = rospy.Service('calibrate', std_srvs.srv.Empty, self.calibrate)
                

        #position calculations
        self.pot_settings = rospy.get_param('pendulum/potentiometer')
        self.track_length = rospy.get_param('pendulum/track_length')
        self.resistance_high = self.pot_settings['high']
        self.resistance_low = self.pot_settings['low']
        resistance_mid = (self.resistance_high + self.resistance_low)/2
        self.pos_scalar = 1/(2*(self.resistance_high-resistance_mid)/self.track_length)
        self.pos_offset = resistance_mid
        
        self.current_unrotated_theta = 0
        self.angle_calibration = 0
        self.prev_theta = 0
        self.prev_x = 0
        self.prev_time = 0
        self.rightLim = False
        self.leftLim = False
        self.pos_filter = avg_filter(10)

    def motor_callback(self, data):
        self.rightLim = data.limitStatus.an2Limit
        self.leftLim = data.limitStatus.an1Limit

    def sensor_callback(self, data):
        status = PendulumPose()
        status.header = Header()
        status.header.stamp = rospy.Time.now()

        now = rospy.get_time()
        delta = now - self.prev_time

        status.x = self.pos_filter.run(self.get_position(data.data[1]))

        status.leftLim = self.leftLim
        status.rightLim = self.rightLim
    
        if status.leftLim:
            status.x = -self.track_length/2
        if status.rightLim:
            status.x = self.track_length/2
    
        
        status_bits = data.data[2] >> 8;

        md = status_bits & (1 << 5);
        ml = status_bits & (1 << 4);
        mh = status_bits & (1 << 3);


        if md:
            self.current_unrotated_theta = data.data[0] * self.theta_conv
            status.theta = self.rotate(self.current_unrotated_theta)
            
        else:
            status.theta = 0
            rospy.logerr("magnet not detected!")

        if ml: rospy.loginfo("magnet strength low")
        if mh: rospy.loginfo("magnet strength high")
            

        try:
            status.thetaDot = (status.theta - self.prev_theta)/delta
            status.xDot = (status.x - self.prev_x)/delta
        except ZeroDivisionError:
            rospy.logwarning("delta is 0, no time passed?")


        self.sensor_pub.publish(status)

        self.prev_time = now
        self.prev_theta = status.theta
        self.prev_x = status.x

    def calibrate(self, request):
        self.angle_calibration = self.current_unrotated_theta
        rospy.loginfo("calibrating with bottom as %d", self.current_unrotated_theta)
        return std_srvs.srv.EmptyResponse()
        
    def rotate(self, theta):
        theta = theta - self.angle_calibration + 180
        theta = -theta #+ 180 
        if theta > 180:
            theta = theta - 360
        elif theta < -180:
            theta = theta + 360
        return theta 
    
    def get_position(self, raw_position):
        return (raw_position - self.pos_offset) * self.pos_scalar


if __name__ == "__main__":
    rospy.init_node('sensors')
    node = Node()

    rospy.spin()

