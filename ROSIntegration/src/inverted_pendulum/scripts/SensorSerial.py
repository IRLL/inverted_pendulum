#!/usr/bin/env python

import rospy

from inverted_pendulum.msg import PendulumPose


rospy.init_node('SensorSerial')

pub = rospy.Publisher('/sensors', PendulumPose, queue_size=1)

while not rospy.is_shutdown():
    pass
