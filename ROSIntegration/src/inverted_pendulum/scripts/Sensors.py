#!/usr/bin/env python

import rospy

from inverted_pendulum.msg import PendulumPose
from std_msgs.msg import Int16MultiArray
from std_msgs.msg import Header

class Node():
    def __init__(self):
        self.theta_conv = 360.0/4096
        
        self.sensor_pub = rospy.Publisher('/sensors', PendulumPose, queue_size=1)
        self.raw_sensor_sub = rospy.Subscriber('/raw_sensors', Int16MultiArray, self.my_callback)
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
            status.theta = data.data[0] * self.theta_conv
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
        



if __name__ == "__main__":
    rospy.init_node('sensors')
    node = Node()

    rospy.spin()

