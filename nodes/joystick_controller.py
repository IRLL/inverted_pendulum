#!/usr/bin/env python

import rospy
from inverted_pendulum.msg import Cmd
from std_msgs.msg import Header
from robosub.msg import joystick
magnitude = 40


class Node:
    def __init__(self):
        self.cmd_pub = rospy.Publisher('/cmd', Cmd, queue_size=1)
        self.joy_sub = rospy.Subscriber('/joystick_driver/joystick_driver', joystick, self.callback);

    def callback(self, joystick):
        cmd = Cmd()
        cmd.header = Header()
        cmd.header.stamp = rospy.Time.now()
        cmd.cmd = joystick.axisZ * magnitude
        self.cmd_pub.publish(cmd)

if __name__ == "__main__":
    rospy.init_node('JoystickController')
    node = Node()
    rospy.spin()
