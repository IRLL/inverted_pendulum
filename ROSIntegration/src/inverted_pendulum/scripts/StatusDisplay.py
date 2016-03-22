#!/usr/bin/env python

import rospy

from inverted_pendulum.msg import PendulumPose


rospy.init_node('StatusDisplay')

motor_info_sub = rospy.Subscriber('/motor/info', Int32)
motor_cmd_sub = rospy.Subscriber('/motor/cmd', Int32)
agent_sub = rospy.Subscriber('/agent/cmd', Int32)
sensor_sub = rospy.Subscriber('/sensors', Int32)

while not rospy.is_shutdown():
    pass
