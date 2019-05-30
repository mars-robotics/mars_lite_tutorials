#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import Twist
from math import pi


class OutAndBack():
    def __init__(self):
        # initiliaze
        rospy.init_node('out_and_back', anonymous=False)
        # tell user how to stop Robot
        rospy.loginfo("To stop Robot CTRL + C")
        # What function to call when you ctrl + c
        rospy.on_shutdown(self.shutdown)
        # Create a publisher which can "talk" to Robot and tell it to move
        # Tip: You may need to change /cmd_vel
        self.cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size=10)
#       self.cmd_vel = rospy.Publisher('cmd_vel_mux/input/navi', Twist, queue_size=10)

        # How fast will we update the robot's movement?
        rate = 50
        r = rospy.Rate(rate)  # 50hz

    #   設定 out_and_back 參數

        # Set the travel distance to 1.0 meters
        goal_distance = 1.0
        # Set the forward linear speed to 0.2 meters per second
        linear_speed = 0.2
        print("設定 前進距離為%5.2f meters，速度為%5.2f m/s" % (goal_distance,linear_speed))
        # How long should it take us to get there?
        linear_duration = goal_distance / linear_speed
        print("執行 前進動作 花費時間=%5.2f sec" % linear_duration)

        # Set the rotation angle to Pi radians (180 degrees)
        goal_angle = pi
        # Set the rotation speed to 1.0 radians per second
        angular_speed = 1.0
        print("設定 旋轉角度為%5.2f radians，旋轉速度為%5.2f rad/s" % (goal_angle,angular_speed))

        # How long should it take to rotate
        angular_duration = goal_angle / angular_speed
        print("執行 旋轉動作 花費時間=%5.2f sec" % angular_duration)

        # 設定 out_and_back 執行程式
        for i in range(2):
            # Initialize the movement command
            move_cmd = Twist()
            move_cmd.linear.x = linear_speed
            # Move forward for a time to go the desired distance
            print(move_cmd.linear.x)

            ticks = int(linear_duration * rate)
            for t in range(ticks):
                self.cmd_vel.publish(move_cmd)
                r.sleep()
            # Stop the robot before the rotation
            move_cmd = Twist()
            self.cmd_vel.publish(move_cmd)
            rospy.sleep(1)

            # Now rotate left roughly 180 degrees

            # Set the angular speed
            move_cmd.angular.z = angular_speed
            # Rotate for a time to go 180 degrees
            ticks = int(goal_angle * rate)
            for t in range(ticks):
                self.cmd_vel.publish(move_cmd)
                r.sleep()

            # Stop the robot before the next leg
            move_cmd = Twist()
            self.cmd_vel.publish(move_cmd)
            rospy.sleep(1)

        # Stop the robot
        self.cmd_vel.publish(Twist())

    def shutdown(self):
        # stop Robot
        rospy.loginfo("Stopping the robot...")
        # a default Twist has linear.x of 0 and angular.z of 0.  So it'll stop Robot
        self.cmd_vel.publish(Twist())
        # sleep just makes sure Robot receives the stop command prior to shutting down the script
        rospy.sleep(1)


if __name__ == '__main__':
    try:
        OutAndBack()
    except:
        rospy.loginfo("Out-and-Back node terminated.")
