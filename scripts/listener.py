#!/usr/bin/env python
# -*- coding: utf-8 -*-


import rospy
from std_msgs.msg import String


class listen():
    def __init__(self):
        # initiliaze
        rospy.init_node('listener', anonymous=False)
        rospy.Subscriber('chatter', String, self.callback)
        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()

    def callback(self,talk_data):
        rospy.loginfo('I heard : %s', talk_data.data)


if __name__ == '__main__':
    try:
        listen()
    except:
        rospy.loginfo("listener node terminated.")
