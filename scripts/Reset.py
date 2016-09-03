#!/usr/bin/python

import rospy
import actionlib
from inverted_pendulum.msg import ResetAction, ResetGoal
import time


if __name__ == '__main__':
    rospy.init_node('reset_client', anonymous=True)
    client = actionlib.SimpleActionClient('inverted_pendulum/reset', ResetAction)
    client.wait_for_server()

    goal = ResetGoal
    goal.angle = 0.0
    goal.position = 0.0

    client.send_goal(goal)
    client.wait_for_result()
    print client.get_result()
    time.sleep(1)
