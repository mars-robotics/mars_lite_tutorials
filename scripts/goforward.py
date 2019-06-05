#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import Twist


class GoForward():
    def __init__(self):
        # initiliaze
        rospy.init_node('GoForward', anonymous=False)
        # tell user how to stop Robot
        rospy.loginfo("To stop Robot CTRL + C")
        # What function to call when you ctrl + c
        rospy.on_shutdown(self.shutdown)
        # Create a publisher which can "talk" to Robot and tell it to move
        # Tip: You may need to change /cmd_vel
        self.cmd_vel = rospy.Publisher('/mob_plat/cmd_vel', Twist, queue_size=10)
#       self.cmd_vel = rospy.Publisher('cmd_vel_mux/input/navi', Twist, queue_size=10)

        # How fast will we update the robot's movement?
        r = rospy.Rate(10) # 10hz
        # Initialize the movement command
        move_cmd = Twist()
        # let's go forward at 0.2 m/s
        move_cmd.linear.x = 0.2
        print("設定 前進速度為%5.2f m/s" % move_cmd.linear.x)

        # let's turn at 0 radians/s
        move_cmd.angular.z = 0
        print("設定 旋轉速度為%5.2f rad/s" % move_cmd.angular.z)

        # as long as you haven't ctrl + c keeping doing...
        while not rospy.is_shutdown():
            # publish the velocity
            self.cmd_vel.publish(move_cmd)
            # wait for 0.1 seconds (10 HZ) and publish again
            r.sleep()

    def shutdown(self):
        # stop Robot
        rospy.loginfo("Stopping the robot...")
        # a default Twist has linear.x of 0 and angular.z of 0.  So it'll stop Robot
        self.cmd_vel.publish(Twist())
        # sleep just makes sure Robot receives the stop command prior to shutting down the script
        rospy.sleep(1)


if __name__ == '__main__':
    try:
        GoForward()
    except:
        rospy.loginfo("GoForward node terminated.")

