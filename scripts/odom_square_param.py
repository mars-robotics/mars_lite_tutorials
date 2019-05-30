#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import Twist, Point, Quaternion
from math import radians, copysign, sqrt, pow, pi, degrees
import tf
from transform_utils import quat_to_angle, normalize_angle


class OdomSquare():
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
        rate = 20
        r = rospy.Rate(rate)  # 20hz

    #   設定 out_and_back 參數

        # Set the travel distance to 1.0 meters
        goal_distance = rospy.get_param("~goal_distance", 1.0)

        # Set the forward linear speed to 0.4 meters per second
        linear_speed = rospy.get_param("~linear_speed", 0.4)        # meters per second
        print("設定 前進距離為%5.2f meters，速度為%5.2f m/s" % (goal_distance,linear_speed))

        # Set the rotation angle to 180 degrees
        goal_angle = radians(rospy.get_param("~goal_angle", 90))    # degrees converted to radians


        # Set the rotation speed in radians per second
        angular_speed = rospy.get_param("~angular_speed", 0.8)      # radians per second
        print("設定 旋轉角度為%5.2f radians，旋轉速度為%5.2f rad/s" % (goal_angle,angular_speed))

        # Set the angular tolerance in degrees converted to radians
        angular_tolerance = radians(rospy.get_param("~angular_tolerance", 1.0)) # degrees to radians

        # Initialize the tf listener
        self.tf_listener = tf.TransformListener()
        # Give tf some time to fill its buffer
        rospy.sleep(2)

        # Set the odom frame
        self.odom_frame = rospy.get_param('~odom_frame', '/odom')

        # Set the base frame for Robot
        self.base_frame = rospy.get_param('~base_frame', '/base_link')

        # Initialize the position variable as a Point type
        position = Point()

        for i in range(4):
            # Initialize the movement command
            move_cmd = Twist()
            # Set the movement command to forward motion
            move_cmd.linear.x = linear_speed
            (position,rotation) = self.get_odom()

            x_start = position.x
            y_start = position.y
            # Keep track of the distance traveled
            distance = 0
            print(" distance = %5.2f meters " % distance)

            # Enter the loop to move along a side

            while distance < goal_distance and not rospy.is_shutdown():
                # Publish the Twist message and sleep 1 cycle
                self.cmd_vel.publish(move_cmd)
                r.sleep()
                # Get the current position
                (position, rotation) = self.get_odom()
                # Compute the Euclidean distance from the start
                distance = sqrt(pow((position.x - x_start), 2) +
                                pow((position.y - y_start), 2))
                print(" distance = %5.2f meters " % distance)

            # Stop the robot before the rotation
            move_cmd = Twist()
            self.cmd_vel.publish(move_cmd)
            rospy.sleep(1)

            # Set the movement command to a rotation
            move_cmd.angular.z = angular_speed
            # Track the last angle measured
            last_angle = rotation
            # Track how far we have turned
            turn_angle = 0
            print(" turn_angle = %5.2f degrees " % degrees(turn_angle))
            # Begin the rotation
            while abs(turn_angle + angular_tolerance) < abs(goal_angle) and not rospy.is_shutdown():
                # Publish the Twist message and sleep 1 cycle
                self.cmd_vel.publish(move_cmd)
                r.sleep()
                # Get the current rotation
                (position, rotation) = self.get_odom()
                # Compute the amount of rotation since the last loop
                delta_angle = normalize_angle(rotation - last_angle)
                # Add to the running total
                turn_angle += delta_angle
                last_angle = rotation
                print(" turn_angle = %5.2f degrees " % degrees(turn_angle))

            # Stop the robot before the next leg
            move_cmd = Twist()
            self.cmd_vel.publish(move_cmd)
            rospy.sleep(1)

        # Stop the robot for good
        self.cmd_vel.publish(Twist())

    def get_odom(self):
        try:
            (trans, rot) = self.tf_listener.lookupTransform(self.odom_frame, self.base_frame, rospy.Time(0))
        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            rospy.loginfo("TF Exception")
            return
        return (Point(*trans), quat_to_angle(Quaternion(*rot)))

    def shutdown(self):
        # stop Robot
        rospy.loginfo("Stopping the robot...")
        # a default Twist has linear.x of 0 and angular.z of 0.  So it'll stop Robot
        self.cmd_vel.publish(Twist())
        # sleep just makes sure Robot receives the stop command prior to shutting down the script
        rospy.sleep(1)


if __name__ == '__main__':
    try:
        OdomSquare()
    except:
        rospy.loginfo(" Odom Square node terminated.")

