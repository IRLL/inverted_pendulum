#!/usr/bin/env python
import rospy

class Timeout:
    def __init__(self, timeout_period):
        self.timeout_duration = rospy.Duration.from_sec(timeout_period)
        self.time_expire = rospy.Time.now() + self.timeout_duration

    def reset(self):
        self.time_expire = rospy.Time.now() + self.timeout_duration

    def isExpired(self):
        return rospy.Time.now() > self.time_expire
