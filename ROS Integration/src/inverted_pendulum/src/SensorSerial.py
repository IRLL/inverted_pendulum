#!/usr/bin/env python

import rospy

from std_msgs.msg import Int32


rospy.init_node('SensorSerial')

pub = rospy.Publisher('/sensors', Int32)

while not rospy.is_shutdown():
    pass
