#!/usr/bin/env python

import rospy

from inverted_pendulum.msg import PendulumPose

class Node():
    def __init__(self):
        self.sensor_pub = rospy.Publisher('/sensors', PendulumPose, queue_size=1)



if __name__ == "__main__":
    rospy.init_node('SensorSerial')
    node = Node()

    while not rospy.is_shutdown():
        status = PendulumPose()
        #read sensors
        #status = read sensors
        node.sensor_pub.publish(status)
        
