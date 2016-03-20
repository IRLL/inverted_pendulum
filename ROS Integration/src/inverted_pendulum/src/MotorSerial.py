#!/usr/bin/env python

import rospy

from std_msgs.msg import Int32


rospy.init_node('MotorSerial')

sub = rospy.Subscriber('/motor/cmd', Int32)
pub = rospy.Publisher('/motor/info', Int32)

while not rospy.is_shutdown():
    pass
