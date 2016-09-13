#!/usr/bin/env python

import rospy

from inverted_pendulum.msg import PendulumPose
from std_msgs.msg import Int16MultiArray
from std_msgs.msg import Header
import std_srvs.srv



class Node():
    def __init__(self):
        self.theta_conv = 360.0/4096
        
        self.sensor_pub = rospy.Publisher('sensors', PendulumPose, queue_size=1)
        self.raw_sensor_sub = rospy.Subscriber('raw_sensors', Int16MultiArray, self.my_callback)
        self.calibrate_server = rospy.Service('calibrate', std_srvs.srv.Empty, self.calibrate)
        
        self.pot_settings = rospy.get_param('pendulum/potentiometer')

        self.resistance_high = self.pot_settings['high']
        self.resistance_low = self.pot_settings['low']
        self.resistance_mid = (self.resistance_high + self.resistance_low)/2
        
        self.current_unrotated_theta = 0
        self.angle_calibration = 0
        self.prev_theta = 0
        self.prev_x = 0
        self.prev_time = 0

    def my_callback(self, data):
        status = PendulumPose()
        status.header = Header()
        status.header.stamp = rospy.Time.now()

        now = rospy.get_time()
        delta = now - self.prev_time

        status.x = data.data[1]
        
        status.leftLim = True if data.data[2] & 0x02 else False
        status.rightLim = True if data.data[2] & 0x01 else False
        
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



if __name__ == "__main__":
    rospy.init_node('sensors')
    node = Node()

    rospy.spin()

