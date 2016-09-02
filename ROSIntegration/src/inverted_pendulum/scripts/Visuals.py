#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from inverted_pendulum.msg import PendulumPose, Cmd
from math import pi
from visualizer import Visualizer
from cv_bridge import CvBridge, CvBridgeError

class Node():
    def __init__(self):
        self.image_pub = rospy.Publisher('/inverted_pendulum/image_raw', Image, queue_size=1)
        self.bridge = CvBridge()
        self.sensor_sub = rospy.Subscriber('/inverted_pendulum/sensors', PendulumPose, self.pose_callback)
        self.cmd_sub = rospy.Subscriber('/inverted_pendulum/cmd', Cmd, self.cmd_callback)
        self.rate = rospy.Rate(rospy.get_param('pendulum/gui_update_frequency'))
        self.cmd = Cmd()
        self.pose = PendulumPose()

        parameters = rospy.get_param('pendulum')
        self.visualizer = Visualizer(
                track_length = parameters['track_length']
                )

    def cmd_callback(self, cmd):
        self.cmd = cmd
    def pose_callback(self, pose):
        self.pose = pose

    def update(self):
        try:
            image = self.visualizer.draw(self.pose.x, self.pose.theta*pi/180,
                                         self.pose.xDot, self.pose.thetaDot*pi/180, self.cmd.cmd)
            ros_image = self.bridge.cv2_to_imgmsg(image, "rgb8")
            self.cmd = Cmd()
            self.image_pub.publish(ros_image)
        except CvBridgeError as e:
            print(e)

if __name__ == "__main__":
    rospy.init_node('Visualizer')
    node = Node()

    while not rospy.is_shutdown():
        node.update()
        node.rate.sleep()
