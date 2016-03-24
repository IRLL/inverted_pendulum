#!/usr/bin/env python

import rospy

from inverted_pendulum.msg import PendulumPose
from inverted_pendulum.msg import MotorInfo
from inverted_pendulum.msg import Cmd


rospy.init_node('StatusDisplay')

motor_info_sub = rospy.Subscriber('/motor/info', MotorInfo)
motor_cmd_sub = rospy.Subscriber('/cmd', Cmd)
sensor_sub = rospy.Subscriber('/sensors', PendulumPose)

while not rospy.is_shutdown():
    pass
